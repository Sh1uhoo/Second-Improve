#include "stm32f10x.h"                  // Device header
#include "Motor.h"
#include "PWM.h"

extern uint8_t State;

 void Motor_Init(void)
 {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	 
	 PWM_Init();
	 
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0xF;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM3,&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0xF;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM3,&TIM_ICInitStructure);
	
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising ,  TIM_ICPolarity_Rising );
	
	TIM_Cmd(TIM3, ENABLE);
 }
 
 void Motor_Setspeed(int8_t Speed){
	 if (Speed>=0)
	 {
		 GPIO_SetBits(GPIOA,GPIO_Pin_4);
		 GPIO_ResetBits(GPIOA,GPIO_Pin_5);
		 PWM_SetCompare1(Speed);
	 }
	 else
	 {
		  GPIO_SetBits(GPIOA,GPIO_Pin_5);
		 GPIO_ResetBits(GPIOA,GPIO_Pin_4);
		 PWM_SetCompare1(-Speed);
	 }
 }
 
 int16_t Motor_Get(void)
{
	int16_t Temp;
	Temp = TIM_GetCounter(TIM3);
	if (State == 0)
		TIM_SetCounter(TIM3 , 0);
	return Temp;
}