#include "encoder.h"

void Encoder_Init(void){
	HAL_TIM_Encoder_Start(&ENCODER_TIM,ENCODER_CHANNEL1);
	HAL_TIM_Encoder_Start(&ENCODER_TIM,ENCODER_CHANNEL2);
	__HAL_TIM_SET_COUNTER(&ENCODER_TIM,100);
}

int Get_Encoder_Count(void){
	int count = __HAL_TIM_GET_COUNTER(&ENCODER_TIM);
	if (count > 6000){
		count=0;
		__HAL_TIM_SET_COUNTER(&ENCODER_TIM,0);
	}else if(count > 200){
		count=200;
		__HAL_TIM_SET_COUNTER(&ENCODER_TIM,200);
	}
	return count-100;
}

void Set_Encoder_Count(int count){
	__HAL_TIM_SET_COUNTER(&ENCODER_TIM,count+100);
}

















