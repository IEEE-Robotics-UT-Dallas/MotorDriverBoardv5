/*
 * imu_task.c
 *
 *  Created on: Mar 13, 2026
 *      Author: John Ratke
 */

#include "imu_task.h"
#include "BNO085.h"
#include "app_freertos.h"

BNO085_t bno085;
extern volatile float imuHeading;
extern SPI_HandleTypeDef hspi2;

void StartIMUTask(void *argument)
{
  osDelay(100);

  if (!BNO085_Init(&bno085, &hspi2))
  {
    vTaskDelete(NULL);
  }

  BNO085_EnableGameRotationVector(&bno085, 10);

  for (;;)
  {
    if (BNO085_GetSensorEvent(&bno085))
    {

      if (BNO085_WasReset(&bno085))
      {
        BNO085_EnableGameRotationVector(&bno085, 10);
      }

      switch (bno085.sensorValue.sensorId)
      {
      case SH2_GAME_ROTATION_VECTOR:
      {
        float qi = bno085.sensorValue.un.gameRotationVector.i;
        float qj = bno085.sensorValue.un.gameRotationVector.j;
        float qk = bno085.sensorValue.un.gameRotationVector.k;
        float qr = bno085.sensorValue.un.gameRotationVector.real;

        // Convert quaternion to yaw (rotation around Z axis)
        // Result is in radians, -PI to +PI
        imuHeading = atan2f(2.0f * (qr * qk + qi * qj),
                         1.0f - 2.0f * (qj * qj + qk * qk));
        break;
      }
      }
    }
    osDelay(1);
  }
}
