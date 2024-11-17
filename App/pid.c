#include <pid.h>

uint8_t stop_flag;
PID_PARAM_t angle_pid ={0.012 , 0 , 0.4 , 2};
PID_PARAM_t vel_pid ={0.6 , 0.01 , 0 , 10};

float Vel_integral;
extern float Gyro[3];
float Angle_PID_Output(float med,float angle)
{
	float error;  
	static float Angle_integral;
	float output; 	
	
	if(stop_flag == 1){
		Angle_integral = 0;
		return 0;
	}
		
	error = (float)(med - angle);	
	
	Angle_integral = Angle_integral + error * angle_pid.ki;
	if(Angle_integral > angle_pid.IntegralMAX)Angle_integral = angle_pid.IntegralMAX;
	if(Angle_integral < -angle_pid.IntegralMAX)Angle_integral = angle_pid.IntegralMAX;
	
	output = angle_pid.kp * error + Angle_integral + angle_pid.kd * Gyro[0];

	return output; 
}

float Velocity_PID(float Encoder_Left,float Encoder_Right,float target)
{
	//static float Vel_integral;
	float output,error,max_output = 15.0;
	
	if(stop_flag == 1){
		Vel_integral = 0;
		return 0;
	}

	error=(Encoder_Right + Encoder_Left) - target;
	Vel_integral += vel_pid.ki * error;
		
	if(Vel_integral < -vel_pid.IntegralMAX) Vel_integral = -vel_pid.IntegralMAX;
	if(Vel_integral > vel_pid.IntegralMAX) Vel_integral = vel_pid.IntegralMAX;	
	
	output = vel_pid.kp * error + Vel_integral;
	
	if(output < -max_output)output = -max_output;
	if(output > max_output)output = max_output;		
	
	return output;
}

