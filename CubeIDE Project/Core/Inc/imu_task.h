/*
 * imu_task.h
 *
 *  Created on: Mar 13, 2026
 *      Author: John Ratke
 */

#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"

void StartIMUTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif
