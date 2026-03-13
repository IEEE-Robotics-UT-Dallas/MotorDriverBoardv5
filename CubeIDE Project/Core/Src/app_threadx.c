/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
    * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

#include "ux_device_cdc_acm.h"
#include "app_usbx_device.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TX_THREAD uros_thread;
TX_THREAD imu_thread;
TX_THREAD control_thread;

extern UX_SLAVE_CLASS_CDC_ACM *cdc_acm;
ULONG actual_length;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void uros_thread_entry(ULONG arg)
{
    while(1)
    {
        tx_thread_sleep(100);

        char* msg = "daaa";

        if(cdc_acm != UX_NULL)
        {
			ux_device_class_cdc_acm_write(
				cdc_acm,
				(UCHAR *)msg,
				strlen(msg),
				&actual_length
			);
        }
    }
}

void imu_thread_entry(ULONG arg)
{
    while(1)
    {
        tx_thread_sleep(50);
    }
}

void control_thread_entry(ULONG arg)
{
    while(1)
    {
        tx_thread_sleep(100);
    }
}
/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;

  /* USER CODE BEGIN App_ThreadX_MEM_POOL */
  CHAR *uros_stack;
  CHAR *imu_stack;
  CHAR *control_stack;

  tx_byte_allocate((TX_BYTE_POOL*)memory_ptr, (VOID**)&uros_stack, 4096, TX_NO_WAIT);
  tx_byte_allocate((TX_BYTE_POOL*)memory_ptr, (VOID**)&imu_stack, 1024, TX_NO_WAIT);
  tx_byte_allocate((TX_BYTE_POOL*)memory_ptr, (VOID**)&control_stack, 1024, TX_NO_WAIT);

  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */
  MX_USBX_Device_Init(memory_ptr);

  tx_thread_create(&uros_thread, "uros",
                   uros_thread_entry, 0,
                   uros_stack, 4096,
                   15,15,TX_NO_TIME_SLICE,TX_AUTO_START);

  tx_thread_create(&imu_thread, "imu",
                   imu_thread_entry, 0,
                   imu_stack, 1024,
                   10,10,TX_NO_TIME_SLICE,TX_AUTO_START);

  tx_thread_create(&control_thread, "control",
                   control_thread_entry, 0,
                   control_stack, 1024,
                   5,5,TX_NO_TIME_SLICE,TX_AUTO_START);
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  Function that implements the kernel's initialization.
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN Before_Kernel_Start */

  /* USER CODE END Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN Kernel_Start_Error */

  /* USER CODE END Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
