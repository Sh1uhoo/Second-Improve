#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <string.h>

char Serial_RxData[6];

uint8_t Serial_RxFlag;


void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStrycture;
	USART_InitStrycture.USART_BaudRate = 9600;
	USART_InitStrycture.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStrycture.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStrycture.USART_Parity = USART_Parity_No;
	USART_InitStrycture.USART_StopBits = USART_StopBits_1;
	USART_InitStrycture.USART_WordLength = USART_WordLength_8b;
	
	USART_Init(USART1, &USART_InitStrycture);
	
	
	USART_ITConfig(USART1, USART_IT_RXNE,ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructrue;
	NVIC_InitStructrue.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructrue.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructrue.NVIC_IRQChannelPreemptionPriority =1;
	NVIC_InitStructrue.NVIC_IRQChannelSubPriority = 1;
	
	NVIC_Init(&NVIC_InitStructrue);
	
	USART_Cmd(USART1, ENABLE);
}

void Serial_sendByte(uint8_t Byte)
{
	USART_SendData(USART1,Byte);
	while (USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
}

int fputc(int ch , FILE *f)
{
	Serial_sendByte(ch);
	return ch;
}

uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)
	{
		Serial_RxFlag=0;
		return 1;
	}
	else
		return 0;
}



void USART1_IRQHandler(void)
{
	static uint8_t RxState = 0;
	static uint8_t RxP = 0;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		
		
		char RxData = USART_ReceiveData(USART1);
		if (RxState == 0)
		{
			if (RxData == '@')
			{
				RxState = 1;
				RxP = 0;
			}
		}
		
		else if (RxState == 1)
		{
			if (RxData == '%')
			{
				Serial_RxData[RxP] = '\0';
				Serial_RxFlag = 1;
				RxState = 0;
			}
			else if (RxP < 5)
			{
				Serial_RxData[RxP++] = RxData;
			}
			else
			{
				RxState = 0;
			}
		}
		
		else if (RxState == 2)
		{
			if (RxData == '%')
			{
				RxState = 0;
				Serial_RxData[RxP++]='\0';
				Serial_RxFlag = 1;
			}
		}
		
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
}

