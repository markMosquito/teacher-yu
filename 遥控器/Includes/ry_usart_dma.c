/**
  ******************************************************************************
  * @file    ry_usart_dma.c
  * @author  ry
  * @version V1.0
  * @date    16-Feb-2016
  * @brief   Initializate the Simple Periph.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ry_usart_dma.h"

/* Private variables ---------------------------------------------------------*/
u8 DmaUsartTxBuf[100];
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  
  * @param  None					
  * @retval None
  */
void usart1_dma_config(void)
{
	//重映射步骤：打开IO时钟和重映射时钟，重映射开启，配置重映射引脚
	//初始化串口
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);

        /* Configure USART Tx () */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init( GPIOB, &GPIO_InitStructure );

        /* Configure USART Rx () */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init( GPIOB, &GPIO_InitStructure );
        
		GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);
		
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        USART_DeInit(USART1);
        USART_InitStructure.USART_BaudRate = 115200;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        USART_Init(USART1, &USART_InitStructure);
        
        USART_Cmd(USART1, ENABLE);
    }

    //配置NVIC
    {
//        NVIC_InitTypeDef  NVIC_InitStructure;
//        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
//        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);        //开串口中断
//        
//        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;          
//        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 
//        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        
//        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           
//        NVIC_Init(&NVIC_InitStructure);
    }
    
    //DMA发送
    {
		NVIC_InitTypeDef NVIC_InitStructure;
        DMA_InitTypeDef DMA_InitStructure;
        
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
        USART_DMACmd(USART1,USART_DMAReq_Tx ,ENABLE);
        
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	
        
        DMA_DeInit(DMA1_Channel4);
        DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(USART1->DR));
        DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(&(DmaUsartTxBuf));
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//从内存到外设
        DMA_InitStructure.DMA_BufferSize = 0;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
        
        DMA_Init(DMA1_Channel4, &DMA_InitStructure);
        DMA_ClearITPendingBit( DMA_IT_TC);
        DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
		
		DMA_Cmd (DMA1_Channel4, ENABLE);
    }
}

void DMA1_Channel4_IRQHandler(void)
{
	DMA_ClearITPendingBit( DMA1_IT_TC4);
}
extern vu16 After_filter[5];
void DmaSendPos(void)
{  
	u16 sum = After_filter[0]+After_filter[1]+After_filter[2]+After_filter[3];
    DmaUsartTxBuf[0] = 0xa5;
	DmaUsartTxBuf[1] = 0xff;
	DmaUsartTxBuf[2] = After_filter[0];
	DmaUsartTxBuf[3] = After_filter[0]>>8;
	DmaUsartTxBuf[4] = After_filter[1];
	DmaUsartTxBuf[5] = After_filter[1]>>8;
	DmaUsartTxBuf[6] = After_filter[2];
	DmaUsartTxBuf[7] = After_filter[2]>>8;
	DmaUsartTxBuf[8] = After_filter[3];
	DmaUsartTxBuf[9] = After_filter[3]>>8;
	DmaUsartTxBuf[10] = sum;
	DmaUsartTxBuf[11] = sum>>8;
    if(DMA1_Channel4->CNDTR == 0)
    {
		DMA_Cmd(DMA1_Channel4, DISABLE);
        DMA1_Channel4->CNDTR = 12;
        DMA_Cmd(DMA1_Channel4, ENABLE);
    }
}
/*********************************END OF FILE**********************************/
