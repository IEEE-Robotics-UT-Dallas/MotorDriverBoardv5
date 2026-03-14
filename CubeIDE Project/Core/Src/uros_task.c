/*
 * uros_task.c
 *
 *  Created on: Mar 13, 2026
 *      Author: John Ratke
 */

#include "uros_task.h"
#include "shared_resources.h"

#include "stm32h5xx_hal.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/publisher.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

extern UART_HandleTypeDef huart4;

extern osMessageQueueId_t odomQueueHandle;
extern osMessageQueueId_t twistQueueHandle;

rcl_timer_t odom_timer;
rcl_publisher_t odom_pub;
rcl_subscription_t twist_sub;

void twist_sub_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg =
        (const geometry_msgs__msg__Twist *)msgin;

	osMessageQueueReset(twistQueueHandle);
	osMessageQueuePut(twistQueueHandle, msg, 0, 0);
}

void odom_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) timer;
	(void) last_call_time;

	nav_msgs__msg__Odometry msg;
	osStatus_t status = osMessageQueueGet(odomQueueHandle, &msg, NULL, 0);

	if (status == osOK) {
		rcl_publish(&odom_pub, &msg, NULL);
	}
}

void StartuROSTask(void *argument) {
	rmw_uros_set_custom_transport(
		true,
		(void *) &huart4,
		cubemx_transport_open,
		cubemx_transport_close,
		cubemx_transport_write,
		cubemx_transport_read
	);

    //while (rmw_uros_ping_agent(100, 10) != RMW_RET_OK) {
    //    osDelay(100);
    //}

	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    	Error_Handler();
	}

	rclc_executor_t executor;
	rclc_support_t support;
	rcl_allocator_t allocator;
	rcl_node_t node;

	allocator = rcl_get_default_allocator();

	rclc_support_init(&support, 0, NULL, &allocator);

	rclc_node_init_default(&node, "drivetrain_board", "", &support);

	rclc_publisher_init_default(
		&odom_pub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
		"odometry_publisher"
	);

	rclc_timer_init_default2(
      &odom_timer,
      &support,
      RCL_MS_TO_NS(100),
      odom_timer_callback,
	  true
   );

	rclc_subscription_init_default(
		&twist_sub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"velocity_subscriber"
	);

	rclc_executor_init(&executor, &support.context, 2, &allocator);
	rclc_executor_add_timer(&executor, &odom_timer);
	static geometry_msgs__msg__Twist twist_msg;
	rclc_executor_add_subscription(
		&executor,
		&twist_sub,
		&twist_msg,
		&twist_sub_callback,
		ON_NEW_DATA
	);

	for(;;) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		osDelay(10);
	}
}
