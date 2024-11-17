#include <beep.h>
#include <tim.h>


void startup_sing(){
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
	for(char i = 0;i < 4;i++){
		TIM12->PSC = 2000 - i*200;
		HAL_Delay(100);
	}
	HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_1);
}

