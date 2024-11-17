#ifndef __ODRIVE_CAN_H__
#define __ODRIVE_CAN_H__

#include "main.h"

extern float left_wheel_odom,right_wheel_odom,left_wheel_vel,right_wheel_vel;

void send_wheel_torque(float left_wheel_torque,float right_wheel_torque);
void read_wheel_encoder(uint8_t *rxdata,float *wheel_odom,float *wheel_vel);
#endif 
