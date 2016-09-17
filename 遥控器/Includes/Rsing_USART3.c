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
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);//��ʼ��USART3��GPIOB��ʱ��
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
//	/* USART3 GPIO config */								//GPIO��ʼ��
//	/* Configure USART3 Tx (PB.10) as alternate function push-pull */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			//GPIO���츴�����
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	/* Configure USART3 Rx (PB.11) as input floating */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//��������ģʽ
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* USART3 mode config */									//���ڵ�ģʽ�������ʳ�ʼ��
	USART_InitStructure.USART_BaudRate = 9600;					//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//���ô��ڴ�����ֳ���8λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		//����ֹͣλ��1λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;		//������żЧ��λ��������
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//����Ӳ��������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//���ô��ڵĹ���ģʽ
	USART_Init(USART3, &USART_InitStructure);					//��Ĵ���д�����ò���

//	���ڶ��ͨ��
//	USART_WakeUpConfig(USART_Numb, USART_WakeUp_IdleLine);		//USART�Ļ��ѷ�ʽ-�������߻���
//	USART_WakeUpConfig(USART_Numb, USART_WakeUp_AddressMark);	//USART�Ļ��ѷ�ʽ-��ַ��ǻ���
//	USART_SetAddress(USART_Numb, USART_Address);				//��ַ�����У�����USART�ڵ�ĵ�ַ
//	USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState Newstate);//�Ƿ���뾲Ĭģʽ

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
  * @brief  �ض���c�⺯��printf��USART3
  * @param  None					
  * @retval None
  */
int fputc(int ch, FILE *f)
{
	/* ��Printf���ݷ������� */
	USART_SendData(USART3, (u16) ch);
	while( USART_GetFlagStatus(USART3,USART_FLAG_TC) != SET );	
	return (ch);
}
/********************************END OF FILE**********************************/
