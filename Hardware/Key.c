#include "stm32f10x.h"                  // Device header
#include "Delay.h"

void Key_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructureA;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructureA.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructureA.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStructureA.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructureA);
	
}

uint8_t Key=0;

uint8_t Key_GetNum(void)
{
	
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==0)
	{
		Delay_ms(5);
		while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==0);
		Delay_ms(5);
		Key=1-Key;
		if (Key == 0)
			TIM_SetCounter(TIM3 , 0);
		if (Key == 1)
			TIM_SetCounter(TIM4 , 0);
	}

	return Key;
}

