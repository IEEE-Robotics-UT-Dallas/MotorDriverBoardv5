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

extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

static const int16_t NUM_MOTORS = 6;
static const MotorInstance motors[] = {
	{{&htim8, TIM_CHANNEL_1}, {GPIOC, GPIO_PIN_4},  {&hadc1, ADC_CHANNEL_14}, &htim5},
	{{&htim8, TIM_CHANNEL_2}, {GPIOC, GPIO_PIN_5},  {&hadc1, ADC_CHANNEL_15}, &htim3},
	{{&htim8, TIM_CHANNEL_3}, {GPIOD, GPIO_PIN_2},  {&hadc1, ADC_CHANNEL_11}, &htim2},
	{{&htim8, TIM_CHANNEL_4}, {GPIOC, GPIO_PIN_12}, {&hadc1, ADC_CHANNEL_10}, &htim4},
	{{&htim1, TIM_CHANNEL_1}, {GPIOC, GPIO_PIN_13}, {&hadc1, ADC_CHANNEL_12}, NULL},
	{{&htim1, TIM_CHANNEL_2}, {GPIOC, GPIO_PIN_14}, {&hadc1, ADC_CHANNEL_13}, NULL},
};

static const int32_t NUM_ENCODERS = 4;
static const MecanumConfig mecanumConfig = {
	65 * 0.5 * 0.001, //65mm wheel diameter
	20 * 0.01 * 0.5, //20cm wheelbase front/rear
	20 * 0.01 * 0.5, //20cm wheelbase left/right
	200 * 0.1047, //205 RPM max speed
	11, //11 PPR encoder
	56, //1:30 Gear reduction
};

static const float PI = 3.1415927f;
static const float ENCODER_VEL_CONSTANT = 2.0f * PI / (4.0f * mecanumConfig.encoder_ppr * mecanumConfig.gear_reduction * 0.001f);
static const float ENCODER_ALPHA = 0.2;

static const MotorDACChannel drvVref = {&hdac1, DAC_CHANNEL_1};
static const MotorDACChannel accVref = {&hdac1, DAC_CHANNEL_2};

static const MotorDigitalPin nSleep = {GPIOC, GPIO_PIN_11};
static const MotorDigitalPin drvFault = {GPIOC, GPIO_PIN_10};
static const MotorDigitalPin accFault = {GPIOB, GPIO_PIN_0};

static const PIDConstants pidConstants = {
	0.0,
	0.0,
	0.0,
	0.05
};

volatile EncoderValues encoderValues[4] = {0};
volatile PIDController pidControllers[4] = {0};

volatile uint16_t rawADCValues[7] = {0};
volatile float velocityTarget[3] = {0, 0, 0};

void updateEncoder(MotorInstance *motor, EncoderValues *encoder) {
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

float updatePID(PIDController *controller, PIDConstants *constants, float setpoint, float error, float limit) {
	controller->integral += error * 0.001f;

	if (controller->integral > limit) controller->integral = limit;
	else if (controller->integral < -limit) controller->integral = -limit;

	float derivative = (error - controller->prevError);
	controller->prevError = error;

	float output = (constants->kP * error) + (constants->kI * controller->integral) + (constants->kD * derivative) + (constants->kF * setpoint);

	if (output > 1.0f) output = 1.0f;
	else if (output < -1.0f) output = -1.0f;

	return output;
}

void setMotorDutyCycle(MotorInstance *motor, float dutyCycle)
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
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value);
}

void setAccVref(uint16_t value) {
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, value);
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

	for(;;) {
	    vTaskDelayUntil(&xLastWakeTime, xFrequency);

		// Update encoders
		for (int i=0; i<NUM_ENCODERS; i++) {
			updateEncoder(&motors[i], &encoderValues[i]);
		}

		// Update velocity targets
		float velocitySetpoints[4] = {0}; //FL, FR, BL, BR

	    float inv_r = 1.0f / mecanumConfig.wheel_radius_m;
	    float k = mecanumConfig.lx_m + mecanumConfig.ly_m;

	    float v_x = velocityTarget[0], v_y = velocityTarget[1], v_z = velocityTarget[2];

	    velocitySetpoints[0] = (inv_r * (v_x - v_y - k * v_z)) * -1.0f;
	    velocitySetpoints[1] = (inv_r * (v_x + v_y + k * v_z));
	    velocitySetpoints[2] = (inv_r * (v_x + v_y - k * v_z)) * -1.0f;
	    velocitySetpoints[3] = (inv_r * (v_x - v_y + k * v_z));

		// Update motor setpoints
	    float vBatt = (rawADCValues[6] / 4095.0f) * 3.3f * ((22.0f + 100.0f)/22.0f); // Scaled based on voltage divider
	    float voltageScaler = 12.0f / vBatt; //Multiply by this value to get effective 12V
	    float dutyCycles[4] = {0};

	    for (int i=0; i<4; i++) {
	    	float error = velocitySetpoints[i] - encoderValues[i].velocity_rad_s;
	    	dutyCycles[i] = updatePID(&pidControllers[i], &pidConstants, velocitySetpoints[i], error, 1.0);

	    	dutyCycles[i] *= voltageScaler;
	    }

	    // Check current limits
	    // TODO: implement this

		// Update PWM outputs
	    for (int i=0; i<4; i++) {
	    	setMotorDutyCycle(&motors[i], dutyCycles[i]);
	    }
	}
}
