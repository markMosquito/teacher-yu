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
uint16_t USART_DMA_BUF[150];//DMA缓冲数组，printf较长数据时，此数组需增大

/* Functions ---------------------------------------------------------*/

/**
  * @brief  USART3 GPIO 配置,工作模式配置。9600 8-N-1
  * @param  None					
  * @retval None
  */
void USART3_Configuration(void)
{
// 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
// 	/* config USART3 clock */
// 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);//初始化USART3和GPIOB的时钟
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
// 	
// 	/* USART3 GPIO config */								//GPIO初始化
// 	/* Configure USART3 Tx (PB.10) as alternate function push-pull */
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			//GPIO推挽复用输出
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_Init(GPIOB, &GPIO_InitStructure);
// 	/* Configure USART3 Rx (PB.11) as input floating */
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
// 	GPIO_InitStructure.GPIO_M ode = GPIO_Mode_IN_FLOATING;	//浮空输入模式
// 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	  
	/* USART3 mode config */									//串口的模式，波特率初始化
	USART_InitStructure.USART_BaudRate = 9600;					//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//配置串口传输的字长，8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		//配置停止位，1位
	USART_InitStructure.USART_Parity = USART_Parity_No ;		//配置奇偶效验位，不设置
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//配置硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//配置串口的工作模式
	USART_Init(USART3, &USART_InitStructure);					//向寄存器写入配置参数 
	
	
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
// 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//开启DMA时钟

 	/*设置DMA源：内存地址&串口数据寄存器地址*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&((USART_TypeDef *)USART3)->DR);   

	/*内存地址(要传输的变量的指针)*/
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART_DMA_BUF;
	
	/*方向：从内存到外设*/		
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	
	
	/*传输大小DMA_BufferSize=SENDBUFF_SIZE*/	
    DMA_InitStructure.DMA_BufferSize = Buff_i;
	
	/*外设地址不增*/	    
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	
	/*内存地址自增*/
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
	
	/*外设数据单位*/	
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	
	/*内存数据单位 16bit*/
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	 
	
	/*DMA模式：一次传输，循环*/
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;	 
	
	/*优先级：中*/	
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  
	
	/*禁止内存到内存的传输	*/
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	
	/*配置DMA1的4通道*/		   
    DMA_Init(DMA1_Channel2, &DMA_InitStructure); 	   

	DMA_Cmd (DMA1_Channel2, ENABLE);					//使能DMA
}

/*
 * 函数名：itoa
 * 描述  ：将整形数据转换成字符串
 * 输入  ：-radix =10 表示10进制，其他结果为0
 *         -value 要转换的整形数
 *         -buf 转换后的字符串
 *         -radix = 10
 * 输出  ：无
 * 返回  ：无
 * 调用  ：被DMA_printf()调用
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
 * 函数名：DMA_printf
 * 描述  ：格式化输出，类似于C库中的printf，但这里没有用到C库
 * 输入  ： -Data   要发送到串口的内容的指针
 *			   -...    其他参数
 * 输出  ：无
 * 返回  ：无 
 * 调用  ：外部调用
 *         典型应用  DMA_printf("\r\n this is a demo \r\n");
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
	while ( *Data != 0)     // 判断是否到达字符串结束符
	{
		if ( *Data == 0x5c )  //'\'
		{
			switch ( *++Data )
			{
				case 'r':							          //回车符
					Buff_SendData(0x0d);
					Data ++;
				break;
				
				case 'n':							          //换行符
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
				case 's':										  //字符串
					s = va_arg(ap, const char *);
					for ( ; *s; s++) 
					{
						Buff_SendData(*s);
					}
					Data++;
				break;
				case 'd':										//十进制
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
