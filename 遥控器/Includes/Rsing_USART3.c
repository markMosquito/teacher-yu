/**
  ******************************************************************************
  * @file    Rsing_USART3.c
  * @author  Rsing
  * @version V1.0
  * @date    22-July-2013
  * @brief   Initializate the USART3
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "Rsing_USART3.h"

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  USART3 Config
  * @param  None					
  * @retval None
  */
void USART3_Configuration(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
//	/* config USART3 clock */
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);//初始化USART3和GPIOB的时钟
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
//	/* USART3 GPIO config */								//GPIO初始化
//	/* Configure USART3 Tx (PB.10) as alternate function push-pull */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			//GPIO推挽复用输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	/* Configure USART3 Rx (PB.11) as input floating */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//浮空输入模式
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* USART3 mode config */									//串口的模式，波特率初始化
	USART_InitStructure.USART_BaudRate = 9600;					//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//配置串口传输的字长，8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		//配置停止位，1位
	USART_InitStructure.USART_Parity = USART_Parity_No ;		//配置奇偶效验位，不设置
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//配置硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//配置串口的工作模式
	USART_Init(USART3, &USART_InitStructure);					//向寄存器写入配置参数

//	串口多机通信
//	USART_WakeUpConfig(USART_Numb, USART_WakeUp_IdleLine);		//USART的唤醒方式-空闲总线唤醒
//	USART_WakeUpConfig(USART_Numb, USART_WakeUp_AddressMark);	//USART的唤醒方式-地址标记唤醒
//	USART_SetAddress(USART_Numb, USART_Address);				//地址唤醒中，设置USART节点的地址
//	USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState Newstate);//是否进入静默模式

	USART_ClearFlag(USART3,USART_FLAG_RXNE);
	USART_ClearITPendingBit(USART3,USART_IT_RXNE);
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);				//Enable "Receive Data register not empty" interrupt,\
																  also need "Rsing_NVIC.c","stm32f10x_it.c"
	
	USART_Cmd(USART3, ENABLE);									//Enable USART3
//	USART_DMACmd(USART_Numb, USART_DMAReq_Tx, ENABLE);			//Enable USART DMA channel
}

/**
  * @brief  USART3 Send String
  * @param  String address					
  * @retval None
  */
void USART3_SendString(u8* str)		
{
	while(*str)
	{
		USART_SendData(USART3, *str++);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	}
}

/**
  * @brief  重定向c库函数printf到USART3
  * @param  None					
  * @retval None
  */
int fputc(int ch, FILE *f)
{
	/* 将Printf内容发往串口 */
	USART_SendData(USART3, (u16) ch);
	while( USART_GetFlagStatus(USART3,USART_FLAG_TC) != SET );	
	return (ch);
}
/********************************END OF FILE**********************************/
