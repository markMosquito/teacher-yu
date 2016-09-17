/**
  ******************************************************************************
  * @file    ry_adc.c
  * @author  Ry
  * @version V1.0
  * @date    16-Feb-2013
  * @brief   Initializate the Simple Periph.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ry_adc.h"
#include <stm32f10x_adc.h>
/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

#define N 50 //采样50次
#define M 4 //4个通道

vu16 AD_Value[N][M]; //存储数据
vu16 After_filter[M]; //
u8 adc_begin_flag = 0;
/**
  * @brief  
  * @param  None					
  * @retval None
  */
void adc_dma_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//所有的时钟使能GPIO   ADC    DMA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1 , ENABLE ); 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/*step 1 : GPIO初始化*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*step 2 : ADC初始化*/
	ADC_DeInit(ADC1); //将adc1设为缺省值

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //独立模式
	ADC_InitStructure.ADC_ScanConvMode =ENABLE; //扫描模式开
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //连续模式开
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //无外部触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = M; //adc通道数
	ADC_Init(ADC1, &ADC_InitStructure); //
	
	//ADC的通道的初始化
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 4, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 3, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_239Cycles5 );
	
	ADC_DMACmd(ADC1, ENABLE);// ADC1的DMA使能
	ADC_Cmd(ADC1, ENABLE); //使能ADC1

	ADC_ResetCalibration(ADC1); //
	while(ADC_GetResetCalibrationStatus(ADC1)); //矫正
	ADC_StartCalibration(ADC1); //
	while(ADC_GetCalibrationStatus(ADC1)); //
	
	
	/*step3 : DMA初始化及中断配置*/
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
		
	DMA_DeInit(DMA1_Channel1); //DMA1缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR; //外设地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Value; //内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //内存作为目的地
	DMA_InitStructure.DMA_BufferSize = N*M; //DMA的缓存大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //内存递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //数据16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //
	DMA_Init(DMA1_Channel1, &DMA_InitStructure); //
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	
	DMA_ClearITPendingBit(DMA_IT_TC);
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA1_Channel1, ENABLE); //??DMA??
}
void DMA1_Channel1_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA1_IT_TC1);
	filter();
	adc_begin_flag = 1;
}





void filter(void)
{
	int i = 0;
	int sum = 0;
	u8 count;
	for(i=0;i<M;i++)
	{
		for ( count=0;count<N;count++)
		{
			sum += AD_Value[count][i];
		}
		After_filter[i]=sum/N;
		sum=0;
	}
}














/*********************************END OF FILE**********************************/
