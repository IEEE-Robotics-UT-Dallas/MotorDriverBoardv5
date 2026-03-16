/*
 * control_task.c
 *
 *  Created on: Mar 13, 2026
 *      Author: John Ratke
 */

#include "control_task.h"
#include "shared_resources.h"

#include "FreeRTOS.h"
#include "task.h"

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rosidl_runtime_c/string_functions.h>

extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

extern osMessageQueueId_t odomQueueHandle;
extern osMessageQueueId_t twistQueueHandle;
extern osMessageQueueId_t pidQueueHandle;
extern osMessageQueueId_t telemetryQueueHandle;
extern osMessageQueueId_t acc1QueueHandle;
extern osMessageQueueId_t acc2QueueHandle;
extern volatile float imuHeading;

static volatile float odom_x = 0.0f;
static volatile float odom_y = 0.0f;

#define NUM_MOTORS 6
static const MotorInstance motors[] = {
	{{&htim8, TIM_CHANNEL_1}, {GPIOC, GPIO_PIN_4},  {&hadc1, ADC_CHANNEL_14}, &htim5},
	{{&htim8, TIM_CHANNEL_2}, {GPIOC, GPIO_PIN_5},  {&hadc1, ADC_CHANNEL_15}, &htim3},
	{{&htim8, TIM_CHANNEL_3}, {GPIOD, GPIO_PIN_2},  {&hadc1, ADC_CHANNEL_11}, &htim2},
	{{&htim8, TIM_CHANNEL_4}, {GPIOC, GPIO_PIN_12}, {&hadc1, ADC_CHANNEL_10}, &htim4},
	{{&htim1, TIM_CHANNEL_1}, {GPIOC, GPIO_PIN_13}, {&hadc1, ADC_CHANNEL_12}, NULL},
	{{&htim1, TIM_CHANNEL_2}, {GPIOC, GPIO_PIN_14}, {&hadc1, ADC_CHANNEL_13}, NULL},
};

#define NUM_ENCODERS 4
static const MecanumConfig mecanumConfig = {
	65 * 0.5 * 0.001, //65mm wheel diameter
	20 * 0.01 * 0.5, //20cm wheelbase front/rear
	20 * 0.01 * 0.5, //20cm wheelbase left/right
	330 * 0.1047, //333 RPM max speed
	11, //11 PPR encoder
	30, //1:30 Gear reduction
};

static const float dt = 0.001f;

static const float PI = 3.1415927f;
static const float ENCODER_VEL_CONSTANT = 2.0f * PI / (4.0f * mecanumConfig.encoder_ppr * mecanumConfig.gear_reduction * dt);
static const float ENCODER_ALPHA = 0.2;

static const MotorDACChannel drvVref = {&hdac1, DAC_CHANNEL_1};
static const MotorDACChannel accVref = {&hdac1, DAC_CHANNEL_2};

static const MotorDigitalPin nSleep = {GPIOC, GPIO_PIN_11};
static const MotorDigitalPin drvFault = {GPIOC, GPIO_PIN_10};
static const MotorDigitalPin accFault = {GPIOB, GPIO_PIN_0};

static PIDConstants pidConstants = {
	0.15,  // kP
	0.0,  // kI
	0.0,  // kD
	0.022, // Feedforward
	1.0   // Integration limit
};

volatile EncoderValues encoderValues[4] = {0};
volatile PIDController pidControllers[4] = {0};

volatile uint16_t rawADCValues[7] = {0};
volatile float velocityTarget[3] = {0, 0, 0};

void updateEncoder(const MotorInstance *motor, volatile EncoderValues *encoder) {
	if (motor->encoder_htim == NULL) return;

	uint16_t timer_counter = __HAL_TIM_GET_COUNTER(motor->encoder_htim);

	int32_t delta = (int32_t)(timer_counter - encoder->last_counter_value);
	if (delta > 32767) {
		delta -= 65536;
	} else if (delta < -32768) {
		delta += 65536;
	}

	encoder->delta_ticks = delta;
	encoder->position += delta;
	encoder->last_counter_value = timer_counter;

	float raw_vel_rad_s = (float)delta * ENCODER_VEL_CONSTANT;

	encoder->velocity_rad_s = (ENCODER_ALPHA * raw_vel_rad_s) + ((1.0f - ENCODER_ALPHA) * encoder->velocity_rad_s);
}

float updatePID(volatile PIDController *controller, PIDConstants *constants, float setpoint, float measurement) {
	float error = setpoint - measurement;

	controller->integral += error * dt;

	if (controller->integral > constants->limit) controller->integral = constants->limit;
	else if (controller->integral < -constants->limit) controller->integral = -constants->limit;

	float derivative = -(measurement - controller->prevMeasurement);
	controller->prevMeasurement = measurement;

	return (constants->kP * error)
		+ (constants->kI * controller->integral)
		+ (constants->kD * derivative)
		+ (constants->kF * setpoint);
}

void updateForwardKinematics(nav_msgs__msg__Odometry *odom) {
    // Wheel velocities in rad/s (FL, FR, BL, BR)
    float w_fl = encoderValues[0].velocity_rad_s;
    float w_fr = encoderValues[1].velocity_rad_s;
    float w_bl = encoderValues[2].velocity_rad_s;
    float w_br = encoderValues[3].velocity_rad_s;

    float r = mecanumConfig.wheel_radius_m;
    float k = mecanumConfig.lx_m + mecanumConfig.ly_m;

    // Mecanum forward kinematics (body-frame velocities)
    float v_x =  r * 0.25f * ( w_fl + w_fr + w_bl + w_br) * -1.0f;
    float v_y =  r * 0.25f * (-w_fl + w_fr + w_bl - w_br) * -1.0f;
    float v_z =  r * 0.25f * (-w_fl + w_fr - w_bl + w_br) / k;

    // Rotate body-frame linear velocity into odom frame using IMU heading
    float cos_h = cosf(imuHeading);
    float sin_h = sinf(imuHeading);
    float v_x_odom = v_x * cos_h - v_y * sin_h;
    float v_y_odom = v_x * sin_h + v_y * cos_h;

    // Integrate position
    odom_x += v_x_odom * dt;
    odom_y += v_y_odom * dt;

    // Position
    odom->pose.pose.position.x = odom_x;
    odom->pose.pose.position.y = odom_y;
    odom->pose.pose.orientation.z = sinf(imuHeading / 2.0f);
    odom->pose.pose.orientation.w = cosf(imuHeading / 2.0f);

    // Velocity (body frame, as is convention for twist in odom message)
    odom->twist.twist.linear.x = v_x;
    odom->twist.twist.linear.y = v_y;
    odom->twist.twist.angular.z = v_z;
}

void setMotorDutyCycle(const MotorInstance *motor, float dutyCycle)
{
	if (dutyCycle > 1.0f) dutyCycle = 1.0f;
	else if (dutyCycle < -1.0f) dutyCycle = -1.0f;

	uint32_t max = motor->en.htim->Init.Period;
	uint32_t duty = (uint32_t)(fabsf(dutyCycle) * max);

	if (dutyCycle > 0) {
		HAL_GPIO_WritePin(motor->ph.port, motor->ph.pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(motor->ph.port, motor->ph.pin, GPIO_PIN_RESET);
	}
	__HAL_TIM_SET_COMPARE(motor->en.htim, motor->en.channel, duty);
}

void setMotorsEnabled(uint8_t enabled) {
	HAL_GPIO_WritePin(nSleep.port, nSleep.pin, enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void setDrvVref(uint16_t value) {
	HAL_DAC_SetValue(drvVref.hdac, drvVref.channel, DAC_ALIGN_12B_R, value);
}

void setAccVref(uint16_t value) {
	HAL_DAC_SetValue(accVref.hdac, accVref.channel, DAC_ALIGN_12B_R, value);
}

void StartControlTask(void *argument) {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)rawADCValues, 7);

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

	setDrvVref(4095); //No hardware current limiting
	setAccVref(4095); //No hardware current limiting

	setMotorsEnabled(1);

	const TickType_t xFrequency = pdMS_TO_TICKS(1);
	TickType_t xLastWakeTime = xTaskGetTickCount();

	static geometry_msgs__msg__Twist twist_msg;

	static float pid_msg[5];
	static float acc1_dutyCycle;
	static float acc2_dutyCycle;

	static nav_msgs__msg__Odometry odom_msg;
	nav_msgs__msg__Odometry__init(&odom_msg);
	rosidl_runtime_c__String__assign(
		&odom_msg.header.frame_id,
		"odom");
	rosidl_runtime_c__String__assign(
		&odom_msg.child_frame_id,
		"base_link");
	odom_msg.pose.pose.position.z = 0.0;
	odom_msg.pose.pose.orientation.x = 0.0;
	odom_msg.pose.pose.orientation.y = 0.0;
	odom_msg.twist.twist.linear.z = 0.0;
	odom_msg.twist.twist.angular.x = 0.0;
	odom_msg.twist.twist.angular.y = 0.0;

	static float telemetry_data[TELEMETRY_SIZE] = {0};
	static float velocitySetpoints[4] = {0}; //FL, FR, BL, BR

	for(;;) {
	    vTaskDelayUntil(&xLastWakeTime, xFrequency);

		// Update encoders
		for (int i=0; i<NUM_ENCODERS; i++) {
			updateEncoder(&motors[i], &encoderValues[i]);
		}

		// Update motor velocities
		// Check for new Twist message
		osStatus_t status = osMessageQueueGet(twistQueueHandle, &twist_msg, NULL, 0);
		if (status == osOK) {
			float inv_r = 1.0f / mecanumConfig.wheel_radius_m;
			float k = mecanumConfig.lx_m + mecanumConfig.ly_m;

			float v_x = twist_msg.linear.x, v_y = twist_msg.linear.y, v_z = twist_msg.angular.z;

			velocitySetpoints[0] = (inv_r * (v_x - v_y - k * v_z)) * -1.0f; // FL
			velocitySetpoints[1] = (inv_r * (v_x + v_y + k * v_z)); // FR
			velocitySetpoints[2] = (inv_r * (v_x + v_y - k * v_z)) * -1.0f; // BL
			velocitySetpoints[3] = (inv_r * (v_x - v_y + k * v_z)); // BR
		}

		// Check for PID values
		status = osMessageQueueGet(pidQueueHandle, &pid_msg, NULL, 0);
		if (status == osOK) {
			pidConstants.kP = pid_msg[0];
			pidConstants.kI = pid_msg[1];
			pidConstants.kD = pid_msg[2];
			pidConstants.kF = pid_msg[3];
			pidConstants.limit = pid_msg[4];
		}

		// Check for accessory motor values
		status = osMessageQueueGet(acc1QueueHandle, &acc1_dutyCycle, NULL, 0);
		if (status == osOK) {
			if (acc1_dutyCycle > 1.0) acc1_dutyCycle = 1.0;
			if (acc1_dutyCycle < -1.0) acc1_dutyCycle = -1.0;
		}

		status = osMessageQueueGet(acc2QueueHandle, &acc2_dutyCycle, NULL, 0);
		if (status == osOK) {
			if (acc2_dutyCycle > 1.0) acc2_dutyCycle = 1.0;
			if (acc2_dutyCycle < -1.0) acc2_dutyCycle = -1.0;
		}

		// Update motor setpoints
	    // Note: motor duty cycles should not exceed voltageScaler to prevent damage
	    float vBatt = (rawADCValues[6] / 4095.0f) * 3.3f * ((22.0f + 100.0f)/22.0f); // Scaled based on voltage divider
		float voltageScaler = 12.0f / vBatt; //Multiply by this value to get effective 12V

		if (voltageScaler > 0.75f) voltageScaler = 0.75f; // Maximum safe value 

	    float dutyCycles[NUM_MOTORS] = {0};

	    for (int i=0; i<4; i++) {
	    	dutyCycles[i] = updatePID(&pidControllers[i], &pidConstants, velocitySetpoints[i], encoderValues[i].velocity_rad_s);
	    }
	    dutyCycles[4] = acc1_dutyCycle;
		dutyCycles[5] = acc2_dutyCycle;

		// If any duty cycle exceeds rated motor voltage, scale them all down proportionally to prevent damage
		// and maintain commanded velocity directionality. (Drive motors only)
		float maxDuty = 0.0f;
		for (int i = 0; i < 4; i++) {
			float absDuty = fabsf(dutyCycles[i]);
			if (absDuty > maxDuty) maxDuty = absDuty;
		}
		if (maxDuty > voltageScaler) {
			float scale = voltageScaler / maxDuty;
			for (int i = 0; i < 4; i++) {
				dutyCycles[i] *= scale;
			}
		}

		// Apply voltage scaling to accessory motors
		dutyCycles[4] *= voltageScaler;
		dutyCycles[5] *= voltageScaler;

	    // Check current limits
		float motorCurrents[NUM_MOTORS] = {0};
		for (int i=0; i<NUM_MOTORS; i++) {
			// Current feedback: 450uA/A, 1kOhm resistor
			motorCurrents[i] = (rawADCValues[i] / 4095.0f) * 3.3f / (0.000450f * 1000.0f);
		}
	    // TODO: implement active current limiting


		// Update PWM outputs
	    for (int i=0; i<NUM_MOTORS; i++) {
	    	setMotorDutyCycle(&motors[i], dutyCycles[i]);
	    }

	    // Update forward kinematics
		uint64_t now = (uint64_t)xTaskGetTickCount();
		odom_msg.header.stamp.sec = now / 1000;
		odom_msg.header.stamp.nanosec = (now % 1000) * 1000000;

		updateForwardKinematics(&odom_msg);

	    osMessageQueueReset(odomQueueHandle);
	    osMessageQueuePut(odomQueueHandle, &odom_msg, 0, 0);

	    // Update telemetry values
	    telemetry_data[IDX_BATTERY] = vBatt;
	    for (int i=0; i<NUM_ENCODERS; i++) {
	    	telemetry_data[IDX_M1_SPEED + i] = encoderValues[i].velocity_rad_s;
	    	telemetry_data[IDX_M1_SETPOINT + i] = velocitySetpoints[i];

	    }
	    for (int i=0; i<NUM_MOTORS; i++) {
	    	telemetry_data[IDX_M1_OUTPUT + i] = dutyCycles[i];
	    	telemetry_data[IDX_M1_CURRENT + i] = motorCurrents[i];
	    }
	    osMessageQueueReset(telemetryQueueHandle);
	    osMessageQueuePut(telemetryQueueHandle, &telemetry_data, 0, 0);

	}
}
