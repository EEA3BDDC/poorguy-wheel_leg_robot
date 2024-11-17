#include "BMI088driver.h"

#include "bsp_can.h"
#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"
#include "beep.h"
#include "odrive_can.h"
#include "ros_main.h"
#include "usart.h"
//#define EN_DM_motor
//#define EN_ros

uint8_t Left_leg_RevData[78];
uint8_t Right_leg_RevData[78];

void bsp_device_init(){
	
	while(BMI088_init());
	
	bsp_can_init();	
	
	HAL_GPIO_WritePin(dir1_485_GPIO_Port,dir1_485_Pin,GPIO_PIN_SET);//TX mode
	HAL_GPIO_WritePin(dir2_485_GPIO_Port,dir2_485_Pin,GPIO_PIN_SET);
	
	HAL_UART_Receive_DMA(&huart1,(uint8_t *)Left_leg_RevData,78); // ????DMA?? 
	HAL_UART_Receive_DMA(&huart6,(uint8_t *)Right_leg_RevData,78); // ????DMA?? 

#ifdef EN_ros
		setup();//ros communicate sys
#endif

#ifdef EN_DM_motor
	dm_motor_enable(&hcan1, &motor[Motor1]);
	dm_motor_enable(&hcan2, &motor[Motor1]);
	dm_motor_init();
#endif
	
	
	startup_sing();	//init ALL ok
}

