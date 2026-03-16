/*
 * shared_resources.h
 *
 *  Created on: Mar 13, 2026s
 *      Author: John Ratke
 */

#ifndef INC_SHARED_RESOURCES_H_
#define INC_SHARED_RESOURCES_H_

#include "cmsis_os.h"

extern osMessageQueueId_t odomQueueHandle;
extern osMessageQueueId_t twistQueueHandle;
extern osMessageQueueId_t telemetryQueueHandle;
extern volatile float imuHeading;

// Telemetry
#define IDX_BATTERY     0   // 1
#define IDX_M1_OUTPUT   1   // 6
#define IDX_M1_CURRENT  7   // 6
#define IDX_M1_SPEED    13  // 4
#define IDX_M1_SETPOINT 17  // 4

#define TELEMETRY_SIZE 21

#endif /* INC_SHARED_RESOURCES_H_ */
