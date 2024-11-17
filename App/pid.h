#ifndef pid_h
#define pid_h

#include "main.h"



typedef struct _PID_PARAM_t {
	
    float kp;          
    float ki;           
    float kd;             
    float IntegralMAX;

	
} PID_PARAM_t;

extern PID_PARAM_t angle_pid;

float Angle_PID_Output(float med,float angle);
float Velocity_PID(float Encoder_Left,float Encoder_Right,float target);


#endif





