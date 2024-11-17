/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MahonyAHRS.h"
#include "BMI088driver.h"
#include "pid.h"
#include "odrive_can.h"
#include "unitreeA1_cmd.h"
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
/* USER CODE BEGIN Variables */
float Gyro[3], Accel[3], temp;
AHRSIMU_t IMU;

float vel_pid_output_value; 
float Angle_pid_output_value;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId balance_TasHandle;
osThreadId leg_TaskHandle;
osThreadId wheel_taskHandle;
osThreadId blink_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void balance_ctrl(void const * argument);
void leg_ctrl(void const * argument);
void get_wheel_speed(void const * argument);
void blink_status(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of balance_Tas */
  osThreadDef(balance_Tas, balance_ctrl, osPriorityIdle, 0, 128);
  balance_TasHandle = osThreadCreate(osThread(balance_Tas), NULL);

  /* definition and creation of leg_Task */
  osThreadDef(leg_Task, leg_ctrl, osPriorityIdle, 0, 128);
  leg_TaskHandle = osThreadCreate(osThread(leg_Task), NULL);

  /* definition and creation of wheel_task */
  osThreadDef(wheel_task, get_wheel_speed, osPriorityIdle, 0, 128);
  wheel_taskHandle = osThreadCreate(osThread(wheel_task), NULL);

  /* definition and creation of blink_task */
  osThreadDef(blink_task, blink_status, osPriorityIdle, 0, 128);
  blink_taskHandle = osThreadCreate(osThread(blink_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_balance_ctrl */
/**
* @brief Function implementing the balance_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_balance_ctrl */
void balance_ctrl(void const * argument)
{
  /* USER CODE BEGIN balance_ctrl */
  /* Infinite loop */
	
	TickType_t xlastwakeTime;
	xlastwakeTime =xTaskGetTickCount();
	
  for(;;)
  {
		BMI088_read(Gyro, Accel, &temp);
		MahonyAHRSupdateIMU(&IMU,Gyro[0],Gyro[1],Gyro[2],Accel[0],Accel[1],Accel[2]);
		
		vel_pid_output_value = Velocity_PID(-left_wheel_vel,right_wheel_vel,0);		
		Angle_pid_output_value = Angle_PID_Output(vel_pid_output_value,IMU.Pitch);
		
//		send_wheel_torque(0,0);
		
		vTaskDelayUntil(&xlastwakeTime,10); //ms
  }
  /* USER CODE END balance_ctrl */
}

/* USER CODE BEGIN Header_leg_ctrl */
/**
* @brief Function implementing the leg_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_leg_ctrl */
void leg_ctrl(void const * argument)
{
  /* USER CODE BEGIN leg_ctrl */
  /* Infinite loop */
  for(;;)
  {
		unitreeA1_sendCMD(&cmd_left,2, 0.15, 0, 0, 0, 0);
		osDelay(200);
//		unitreeA1_sendCMD(&cmd_left,1, 0.2, 0, 0, 0, 0);
//		osDelay(200);
  }
  /* USER CODE END leg_ctrl */
}

/* USER CODE BEGIN Header_get_wheel_speed */
/**
* @brief Function implementing the wheel_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_get_wheel_speed */
void get_wheel_speed(void const * argument)
{
  /* USER CODE BEGIN get_wheel_speed */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END get_wheel_speed */
}

/* USER CODE BEGIN Header_blink_status */
/**
* @brief Function implementing the blink_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_blink_status */
void blink_status(void const * argument)
{
  /* USER CODE BEGIN blink_status */
  /* Infinite loop */
	static uint8_t motor_power_en;
  for(;;)
  {
		if(HAL_GPIO_ReadPin(key_c_GPIO_Port,key_c_Pin) == GPIO_PIN_RESET){ //motor power disable	
			HAL_GPIO_TogglePin(led_r_GPIO_Port,led_r_Pin); //blink Red led
			HAL_GPIO_WritePin(led_g_GPIO_Port,led_g_Pin,GPIO_PIN_SET); 
			HAL_GPIO_WritePin(led_b_GPIO_Port,led_b_Pin,GPIO_PIN_SET); 
			motor_power_en = 0;
			osDelay(200);
		}
		else if(HAL_GPIO_ReadPin(key_c_GPIO_Port,key_c_Pin) == GPIO_PIN_SET && motor_power_en == 0){ //if All motor power enable
			HAL_GPIO_WritePin(led_r_GPIO_Port,led_r_Pin,GPIO_PIN_RESET); //turn on red led;
			HAL_GPIO_WritePin(led_g_GPIO_Port,led_g_Pin,GPIO_PIN_RESET); //turn on green led;	 mean yellow light
			HAL_GPIO_WritePin(led_b_GPIO_Port,led_b_Pin,GPIO_PIN_SET); //turn off blue led;
			osDelay(2000); //waiting motor startup
			motor_power_en = 1;
		}	

		if(motor_power_en == 1){
			HAL_GPIO_WritePin(led_r_GPIO_Port,led_r_Pin,GPIO_PIN_SET); 
			HAL_GPIO_WritePin(led_b_GPIO_Port,led_b_Pin,GPIO_PIN_SET); 
			HAL_GPIO_TogglePin(led_g_GPIO_Port,led_g_Pin); //blink green led
			osDelay(35);
		}
  }
  /* USER CODE END blink_status */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
