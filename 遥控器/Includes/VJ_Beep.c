#include "VJ_Beep.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//void Beep_GPIOInit(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
//	
//	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOE,&GPIO_InitStructure);
//	
//}

//void BEEP1(u32 BTIME)
//{
//	u32 i = 0;
//	for(i = 0;i < 4;i++) 
//	{
//		BEEP_ON;
//		vTaskDelay(BTIME*50);
//		BEEP_OFF;
//		vTaskDelay(BTIME*50);
//		BTIME = BTIME - 2;
//		
//	}
//}
