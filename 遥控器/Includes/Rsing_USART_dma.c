/**
  ******************************************************************************
  * @file    USART_DMA.c
  * @author  
  * @version V1.0
  * @date    2-September-2013
  * @brief   Initializate the USART&DMA.
  *
  * @Configuration: USART3_Configuration();
  *                 DMA_Configuration();
  * @printf:        DMA_printf("...");
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Rsing_USART_dma.h"
#include <stdarg.h>
/* Private defines -----------------------------------------------------------*/
#define Buff_SendData(Data) USART_DMA_BUF[Buff_i++]=((uint16_t)Data)

/* Private variables ---------------------------------------------------------*/
uint16_t Buff_i=0;
uint16_t USART_DMA_BUF[150];//DMA�������飬printf�ϳ�����ʱ��������������

/* Functions ---------------------------------------------------------*/

/**
  * @brief  USART3 GPIO ����,����ģʽ���á�9600 8-N-1
  * @param  None					
  * @retval None
  */
void USART3_Configuration(void)
{
// 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
// 	/* config USART3 clock */
// 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);//��ʼ��USART3��GPIOB��ʱ��
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
// 	
// 	/* USART3 GPIO config */								//GPIO��ʼ��
// 	/* Configure USART3 Tx (PB.10) as alternate function push-pull */
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			//GPIO���츴�����
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_Init(GPIOB, &GPIO_InitStructure);
// 	/* Configure USART3 Rx (PB.11) as input floating */
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
// 	GPIO_InitStructure.GPIO_M ode = GPIO_Mode_IN_FLOATING;	//��������ģʽ
// 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	  
	/* USART3 mode config */									//���ڵ�ģʽ�������ʳ�ʼ��
	USART_InitStructure.USART_BaudRate = 9600;					//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//���ô��ڴ�����ֳ���8λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		//����ֹͣλ��1λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;		//������żЧ��λ��������
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//����Ӳ��������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//���ô��ڵĹ���ģʽ
	USART_Init(USART3, &USART_InitStructure);					//��Ĵ���д�����ò��� 
	
	
	USART_ClearFlag(USART3,USART_FLAG_RXNE);
	USART_ClearITPendingBit(USART3,USART_IT_RXNE);
//	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);				//Enable "Receive Data register not empty" interrupt,\
																  also need "Rsing_NVIC.c","stm32f10x_it.c"
	USART_Cmd(USART3, ENABLE); 
	
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
}

/**
  * @brief  DMA_Config
  * @param  None					
  * @retval None
  */
void DMA_Configuration(void)
{
    DMA_InitTypeDef DMA_InitStructure;

	DMA_DeInit(DMA1_Channel2);
// 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//����DMAʱ��

 	/*����DMAԴ���ڴ��ַ&�������ݼĴ�����ַ*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&((USART_TypeDef *)USART3)->DR);   

	/*�ڴ��ַ(Ҫ����ı�����ָ��)*/
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART_DMA_BUF;
	
	/*���򣺴��ڴ浽����*/		
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	
	
	/*�����СDMA_BufferSize=SENDBUFF_SIZE*/	
    DMA_InitStructure.DMA_BufferSize = Buff_i;
	
	/*�����ַ����*/	    
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	
	/*�ڴ��ַ����*/
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
	
	/*�������ݵ�λ*/	
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	
	/*�ڴ����ݵ�λ 16bit*/
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	 
	
	/*DMAģʽ��һ�δ��䣬ѭ��*/
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;	 
	
	/*���ȼ�����*/	
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  
	
	/*��ֹ�ڴ浽�ڴ�Ĵ���	*/
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	
	/*����DMA1��4ͨ��*/		   
    DMA_Init(DMA1_Channel2, &DMA_InitStructure); 	   

	DMA_Cmd (DMA1_Channel2, ENABLE);					//ʹ��DMA
}

/*
 * ��������itoa
 * ����  ������������ת�����ַ���
 * ����  ��-radix =10 ��ʾ10���ƣ��������Ϊ0
 *         -value Ҫת����������
 *         -buf ת������ַ���
 *         -radix = 10
 * ���  ����
 * ����  ����
 * ����  ����DMA_printf()����
 */
static char *itoa(int value, char *string, int radix)
{
	int     i, d;
	int     flag = 0;
	char    *ptr = string;
	
	/* This implementation only works for decimal numbers. */
	if (radix != 10)
	{
	    *ptr = 0;
	    return string;
	}
	
	if (!value)
	{
	    *ptr++ = 0x30;
	    *ptr = 0;
	    return string;
	}
	
	/* if this is a negative value insert the minus sign. */
	if (value < 0)
	{
	    *ptr++ = '-';
	
	    /* Make the value positive. */
	    value *= -1;
	}
	
	for (i = 10000; i > 0; i /= 10)
	{
	    d = value / i;
	
	    if (d || flag)
	    {
	        *ptr++ = (char)(d + 0x30);
	        value -= (d * i);
	        flag = 1;
	    }
	}
	
	/* Null terminate the string. */
	*ptr = 0;
	
	return string;

} /* NCL_Itoa */

/*
 * ��������DMA_printf
 * ����  ����ʽ�������������C���е�printf��������û���õ�C��
 * ����  �� -Data   Ҫ���͵����ڵ����ݵ�ָ��
 *			   -...    ��������
 * ���  ����
 * ����  ���� 
 * ����  ���ⲿ����
 *         ����Ӧ��  DMA_printf("\r\n this is a demo \r\n");
 *            		 DMA_printf("\r\n %d \r\n", i);
 *            		 DMA_printf("\r\n %s \r\n", j);
 */

void DMA_printf(uint8_t *Data,...)
{
	const char *s;
	int d;   
	char buf[16];
	
	va_list ap;
	va_start(ap, Data);
	Buff_i=0;
	
	while( DMA1_Channel2->CNDTR !=0 );
	DMA_Cmd (DMA1_Channel2, DISABLE);
	while ( *Data != 0)     // �ж��Ƿ񵽴��ַ���������
	{
		if ( *Data == 0x5c )  //'\'
		{
			switch ( *++Data )
			{
				case 'r':							          //�س���
					Buff_SendData(0x0d);
					Data ++;
				break;
				
				case 'n':							          //���з�
					Buff_SendData(0x0a);	
					Data ++;
				break;
				
				default:
					Data ++;
				break;
			}
		}
		else if ( *Data == '%')
		{									  //
			switch ( *++Data )
			{				
				case 's':										  //�ַ���
					s = va_arg(ap, const char *);
					for ( ; *s; s++) 
					{
						Buff_SendData(*s);
					}
					Data++;
				break;
				case 'd':										//ʮ����
					d = va_arg(ap, int);
					itoa(d, buf, 10);
					for (s = buf; *s; s++) 
					{
						Buff_SendData(*s);
					}
					Data++;
					break;
				default:
					Data++;
					break;
			}		 
		} /* end of else if */
		else Buff_SendData(*Data++);
	}
	DMA1_Channel2->CNDTR=Buff_i;
	DMA_Cmd (DMA1_Channel2, ENABLE);
}
/*********************************END OF FILE**********************************/
