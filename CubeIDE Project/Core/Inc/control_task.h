/*
 * control_task.h
 *
 *  Created on: Mar 13, 2026
 *      Author: John Ratke
 */

#ifndef __CONTROL_TASK_H__
#define __CONTROL_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "stm32h5xx_hal.h"

typedef struct{
	GPIO_TypeDef *port;
	uint16_t pin;
} MotorDigitalPin;

typedef struct{
	TIM_HandleTypeDef *htim;
    uint32_t channel;
} MotorTimerChannel;

typedef struct{
    ADC_HandleTypeDef *hadc;
    uint32_t channel;
} MotorADCChannel;

typedef struct{
    DAC_HandleTypeDef *hdac;
    uint32_t channel;
} MotorDACChannel;

typedef struct{
	MotorTimerChannel en;
	MotorDigitalPin ph;
	MotorADCChannel ipropi;
	TIM_HandleTypeDef *encoder_htim;
} MotorInstance;

typedef struct {
    float wheel_radius_m;
    float lx_m;
    float ly_m;
    float max_wheel_rad_s;
    float encoder_ppr;
    float gear_reduction;
} MecanumConfig;

typedef struct {
    float kP;
    float kI;
    float kD;
    float kF;
} PIDConstants;

typedef struct {
    float integral;
    float prevError;
    float limit;
} PIDController;

// Data structures

typedef struct{
	int32_t delta_ticks;
	int64_t position;
	uint16_t last_counter_value;
	float velocity_rad_s;
} EncoderValues;

void StartControlTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif
