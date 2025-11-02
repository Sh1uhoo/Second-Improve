#include "stm32f10x.h"                  // Device header

void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

void LED_ON(uint16_t Pin)
{
	GPIO_ResetBits(GPIOB, Pin);
}

void LED_OFF(uint16_t Pin)
{
	GPIO_SetBits(GPIOB, Pin);
}
