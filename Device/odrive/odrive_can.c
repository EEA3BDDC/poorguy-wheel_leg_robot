#include "odrive_can.h"
#include "can.h"
#include "bsp_can.h"
#include <string.h> 

//can-protocol refer   https://docs.odriverobotics.com/v/0.5.6/can-protocol.html
//                     https://blog.csdn.net/gjy_skyblue/article/details/115031160

float left_wheel_odom,right_wheel_odom,left_wheel_vel,right_wheel_vel;


void send_wheel_torque(float left_wheel_torque,float right_wheel_torque){
	
	uint8_t left_data[8],right_data[8];
	uint16_t id = 0x0E; 

	memcpy(left_data, &left_wheel_torque, sizeof(float));
	memcpy(right_data, &right_wheel_torque, sizeof(float));
	
	canx_send_data(&hcan1, id, left_data, 8);
	canx_send_data(&hcan2, id, right_data, 8);
}

void read_wheel_encoder(uint8_t *rxdata,float *wheel_odom,float *wheel_vel){
	uint32_t temp_data;
	float Encoder_Pos,Encoder_Vel;
	
	temp_data = rxdata[0];
	temp_data |= rxdata[1] << 8;
	temp_data |= rxdata[2] << 16;
	temp_data |= rxdata[3] << 24;
	memcpy(&Encoder_Pos,&temp_data,4);
	
	temp_data = rxdata[4];
	temp_data |= rxdata[5] << 8;
	temp_data |= rxdata[6] << 16;
	temp_data |= rxdata[7] << 24;
	memcpy(&Encoder_Vel,&temp_data,4);
	
    *wheel_odom = Encoder_Pos;  
    *wheel_vel = Encoder_Vel;  
}



