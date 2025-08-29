#ifndef __ENCODER_H
#define __ENCODER_H

#include "tim.h"

#define ENCODER_TIM htim1
#define ENCODER_CHANNEL1 TIM_CHANNEL_1 //PA8
#define ENCODER_CHANNEL2 TIM_CHANNEL_2 //PA9

void Encoder_Init(void);
int Get_Encoder_Count(void);
void Set_Encoder_Count(int count);

#endif
