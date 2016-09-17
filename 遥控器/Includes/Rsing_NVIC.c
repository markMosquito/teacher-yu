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
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;			//TIM2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ���0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//��Ӧ���ȼ���1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;			//TIM3�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ���0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//��Ӧ���ȼ���1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);
	/*
	NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	*/
}

/*********************************END OF FILE**********************************/
