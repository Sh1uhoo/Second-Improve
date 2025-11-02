#include "stm32f10x.h"                  // Device header
#include "LED.h"

void Timer_Init(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_InternalClockConfig(TIM2);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Down;
	TIM_TimeBaseInitStructure.TIM_Period = 20000-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update , ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitSructure;
	NVIC_InitSructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitSructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitSructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitSructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitSructure);
	
	TIM_Cmd(TIM2,ENABLE);
	
}

