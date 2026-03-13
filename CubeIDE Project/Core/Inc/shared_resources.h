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

#endif /* INC_SHARED_RESOURCES_H_ */
