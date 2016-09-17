/**
  ******************************************************************************
  * @file    Rsing_NVIC.c
  * @author  Rsing
  * @version V1.0
  * @date    22-July-2013
  * @brief   Initializate the NVIC
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Rsing_NVIC.h"

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  R_NVIC_Init
  * @param  None					
  * @retval None
  */
void NVIC_Configuration(void) 
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
	
	/*    
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
	*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;			//TIM2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级：0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//响应优先级：1级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;			//TIM3中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级：0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//响应优先级：1级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);
	/*
	NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	*/
}

/*********************************END OF FILE**********************************/
