/*
 * BNO085.h
 *
 * Driver for the Hillcrest/CEVA BNO085 IMU over SPI on STM32H5
 * using the CEVA sh2 sensor hub driver and FreeRTOS.
 *
 * Currently, DMA is not implemented interrupt is polled every 1ms
 *
 * Library written by Claude Sonnet 4.6
 */

#pragma once
#include "stm32h5xx_hal.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include <stdint.h>

// ── Pins (from your CubeMX) ──────────────────────────────────────────────────
#define BNO085_CS_PORT      GPIOB
#define BNO085_CS_PIN       GPIO_PIN_2
#define BNO085_RST_PORT     GPIOB
#define BNO085_RST_PIN      GPIO_PIN_12
#define BNO085_INT_PORT     GPIOB
#define BNO085_INT_PIN      GPIO_PIN_10
// ────────────────────────────────────────────────────────────────────────────

typedef struct {
    SPI_HandleTypeDef *hspi;
    sh2_Hal_t          hal;
    sh2_SensorValue_t  sensorValue;
    volatile bool      intFired;
    bool               resetOccurred;
} BNO085_t;

// Call from HAL_GPIO_EXTI_Callback
void BNO085_INT_Callback(BNO085_t *dev);

bool BNO085_Init(BNO085_t *dev, SPI_HandleTypeDef *hspi);
bool BNO085_EnableRotationVector(BNO085_t *dev, uint32_t interval_ms);
bool BNO085_EnableAccelerometer(BNO085_t *dev, uint32_t interval_ms);
bool BNO085_EnableGyro(BNO085_t *dev, uint32_t interval_ms);
bool BNO085_EnableGameRotationVector(BNO085_t *dev, uint32_t interval_ms);
bool BNO085_GetSensorEvent(BNO085_t *dev);
bool BNO085_WasReset(BNO085_t *dev);
