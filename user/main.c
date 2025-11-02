#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "Encoder.h"
#include "Delay.h"
#include "Motor.h"
#include "Serial.h"
#include "Key.h"
#include <string.h>

float Target,Actual,Out;
float kp[2]={0.3,0.07},ki[2]={0.1,0.01},kd[2]={0.05,0.001};
float err0,err1,errint;
uint8_t State=0;
static int8_t Speed=0;
static int8_t loc=0;

int main(void)
{
	OLED_Init();
	Key_Init();
	Motor_Init();
	Encoder_Init();
	Serial_Init();
	
	
	while(1)
	{
		
		
		State = Key_GetNum() ? 1 - State : State;
			
		if (Serial_GetRxFlag() == 1)
		{
			OLED_ShowString(1,5," ");
			OLED_ShowString(2,1,"      ");
			Speed = 0;
			int8_t k=1;
			
			uint8_t p=0;
			
			if (strcmp(Serial_RxData,"fun1") == 0)
			{
				State = 0;
				Serial_RxFlag = 0;
				
			}
			else if (strcmp(Serial_RxData,"fun2") == 0)
			{	
				State = 1;
				Serial_RxFlag = 0;
			}
			else
			{
				if (Serial_RxData[0] == '-') 
				{
					p++;
					k=-1;
				}
			
				while (Serial_RxData[p] != '\0')
				{
					Speed=Speed*10+(Serial_RxData[p++]-'0');
				}
			
			Speed*=k;
			}
			
			
			
			Serial_RxFlag = 0;
		}
		
		
		OLED_ShowString(1,1,"Func");
		OLED_ShowNum(1,5,State+1,1);
		OLED_ShowString(2,1,Serial_RxData);
		
	}
	
}

void TIM2_IRQHandler(void){
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		if (State == 0)
				Target = Speed;
		else if (State == 1)
		{
				Target = Encoder_Get();
		}
		
		
		Actual = Motor_Get();
		
		
		err1=err0;
		err0=Target - Actual;
		errint += err0;
		
		Out = kp[State]*err0 + ki[State]*errint + kd[State] * (err0-err1);
		
		if (Out > 100 ) Out = 100;
		if (Out < -100 ) Out = -100;
		if ((Actual >=-2 && Actual<=2 && Target == 0) || (Actual-Target<=3 && Actual-Target>=-3 && State == 1))
		{
			Out = 0;
			errint = 0;
		}
		else if (err1 == err0 && State == 1) Out*=2.2;

		Motor_Setspeed(Out);
		
		
		printf("%f,%f,%f,%d\n",Actual,Out,Target,Encoder_Get());
		TIM_ClearITPendingBit(TIM2 , TIM_IT_Update);
	}
}
