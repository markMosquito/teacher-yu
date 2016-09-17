/**
  ******************************************************************************
  * @file    Rsing_RCC.c
  * @author  Rsing
  * @version V1.0
  * @date    22-July-2013
  * @brief   Initializate the RCC
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Rsing_RCC.h"

/* Private variables ---------------------------------------------------------*/
RCC_ClocksTypeDef RCC_Sampling_Freq;//clock sampling for sysclk,hclk,pclk1,pclk2,adcclk

uint32_t RCC_AHB_PeriphClockCmd=0x00000000,RCC_APB1_PeriphClockCmd=0x00000000,RCC_APB2_PeriphClockCmd=0x00000000;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Calculate & open periphclock
  * @param  None					
  * @retval None
  */
void RCC_PeriphClockCmd()
{
	#ifdef RCC_AHB_DMA1
	RCC_AHB_PeriphClockCmd |= RCC_AHBPeriph_DMA1;
	#endif
	#ifdef RCC_AHB_DMA2
	RCC_AHB_PeriphClockCmd |= RCC_AHBPeriph_DMA2;
	#endif 
	#ifdef RCC_AHB_SRAM
	RCC_AHB_PeriphClockCmd |= RCC_AHBPeriph_SRAM;
	#endif  
	#ifdef RCC_AHB_FLITF
	RCC_AHB_PeriphClockCmd |= RCC_AHBPeriph_FLITF;
	#endif
	//--------------------------------------------------------------------------
	#ifdef RCC_APB1_TIM2
	RCC_APB1_PeriphClockCmd |= RCC_APB1Periph_TIM2;
	#endif  
	#ifdef RCC_APB1_TIM3
	RCC_APB1_PeriphClockCmd |= RCC_APB1Periph_TIM3;
	#endif  
	#ifdef RCC_APB1_TIM4
	RCC_APB1_PeriphClockCmd |= RCC_APB1Periph_TIM4;
	#endif 
	#ifdef RCC_APB1_WWDG
	RCC_APB1_PeriphClockCmd |= RCC_APB1Periph_WWDG;
	#endif  
	#ifdef RCC_APB1_SPI2
	RCC_APB1_PeriphClockCmd |= RCC_APB1Periph_SPI2;
	#endif  
	#ifdef RCC_APB1_USART2
	RCC_APB1_PeriphClockCmd |= RCC_APB1Periph_USART2;
	#endif  
	#ifdef RCC_APB1_USART3
	RCC_APB1_PeriphClockCmd |= RCC_APB1Periph_USART3;
	#endif  
	#ifdef RCC_APB1_I2C1
	RCC_APB1_PeriphClockCmd |= RCC_APB1Periph_I2C1;
	#endif  
	#ifdef RCC_APB1_I2C2
	RCC_APB1_PeriphClockCmd |= RCC_APB1Periph_I2C2;
	#endif 
	#ifdef RCC_APB1_USB
	RCC_APB1_PeriphClockCmd |= RCC_APB1Periph_USB;
	#endif  
	#ifdef RCC_APB1_CAN1
	RCC_APB1_PeriphClockCmd |= RCC_APB1Periph_CAN1;
	#endif   
	#ifdef RCC_APB1_BKP
	RCC_APB1_PeriphClockCmd |= RCC_APB1Periph_BKP;
	#endif  
	#ifdef RCC_APB1_PWR
	RCC_APB1_PeriphClockCmd |= RCC_APB1Periph_PWR;
	#endif   
	//--------------------------------------------------------------------------
	#ifdef RCC_APB2_AFIO
	RCC_APB2_PeriphClockCmd |= RCC_APB2Periph_AFIO;
	#endif  
	#ifdef RCC_APB2_GPIOA
	RCC_APB2_PeriphClockCmd |= RCC_APB2Periph_GPIOA;
	#endif  
	#ifdef RCC_APB2_GPIOB
	RCC_APB2_PeriphClockCmd |= RCC_APB2Periph_GPIOB;
	#endif 
	#ifdef RCC_APB2_GPIOC
	RCC_APB2_PeriphClockCmd |= RCC_APB2Periph_GPIOC;
	#endif 
	#ifdef RCC_APB2_GPIOD
	RCC_APB2_PeriphClockCmd |= RCC_APB2Periph_GPIOD;
	#endif  
	#ifdef RCC_APB2_GPIOE
	RCC_APB2_PeriphClockCmd |= RCC_APB2Periph_GPIOE;
	#endif
	#ifdef RCC_APB2_GPIOF
	RCC_APB2_PeriphClockCmd |= RCC_APB2Periph_GPIOF;
	#endif
	#ifdef RCC_APB2_GPIOG
	RCC_APB2_PeriphClockCmd |= RCC_APB2Periph_GPIOG;
	#endif  
	#ifdef RCC_APB2_ADC1
	RCC_APB2_PeriphClockCmd |= RCC_APB2Periph_ADC1;
	#endif  
	#ifdef RCC_APB2_ADC2
	RCC_APB2_PeriphClockCmd |= RCC_APB2Periph_ADC2;
	#endif  
	#ifdef RCC_APB2_TIM1
	RCC_APB2_PeriphClockCmd |= RCC_APB2Periph_TIM1;
	#endif  
	#ifdef RCC_APB2_SPI1
	RCC_APB2_PeriphClockCmd |= RCC_APB2Periph_SPI1;
	#endif  
	#ifdef RCC_APB2_USART1
	RCC_APB2_PeriphClockCmd |= RCC_APB2Periph_USART1;
	#endif 
}

/**
  * @brief  RCC initialization
  * @param  None					
  * @retval None
  */
void RCC_Configuration()
{
	/* 定义枚举类型变量 HSEStartUpStatus */
	ErrorStatus HSEStartUpStatus;

	/* 复位系统时钟设置*/
	RCC_DeInit();

	/* 开启HSE（高速外部时钟）*/
	RCC_HSEConfig(RCC_HSE_ON);

	/* 等待HSE起振并稳定*/
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	
	/* 判断HSE起是否振成功，是则进入if()内部 */
	if(HSEStartUpStatus == SUCCESS)
	{
		/* 选择HCLK（AHB）时钟源为SYSCLK 1分频 */
		RCC_HCLKConfig(RCC_SYSCLK_Div1); 

		/* 选择PCLK1时钟源为 HCLK（AHB） 2分频，外设时钟，由APB1得到，MAX=36MHz,NOW=36MHz */
		RCC_PCLK1Config(RCC_HCLK_Div2);
		
		/* 选择PCLK2时钟源为 HCLK（AHB） 1分频，外设时钟，由APB2得到，MAX=72MHz,NOW=72MHz */
		RCC_PCLK2Config(RCC_HCLK_Div1); 

		/* STM32工作在较高频率如72MHz，FLASH工作在较低频率如24MHz，STM32读取FLASH需要几个等待周期 */
		/* 设置FLASH延时周期数为2，设置代码延时值 */
		FLASH_SetLatency(FLASH_Latency_2);
		
		/* 使能FLASH预取缓存 */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* 选择锁相环（PLL）时钟源为HSE 1分频，倍频数为9，则PLL输出频率为 8MHz * 9 = 72MHz */
    	RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    	/* 使能PLL */ 
    	RCC_PLLCmd(ENABLE);

    	/* 等待指定的RCC标志位设置成功 等待PLL输出稳定 */
    	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

    	/* 选择SYSCLK时钟源为PLL */
    	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    	/* 等待PLL成为SYSCLK时钟源 */
    	while(RCC_GetSYSCLKSource() != 0x08);
  	}
/*	else
	{
		// 选择锁相环（PLL）时钟源为HSI 2分频，倍频数为16，则PLL输出频率为 8MHz / 2 * 16 = 64MHz
		RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);
		
		// 使能PLL
    	RCC_PLLCmd(ENABLE);
		
    	// 等待指定的RCC标志位设置成功 等待PLL输出稳定
    	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
		
		// 选择SYSCLK时钟源为PLL
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		
    	// 等待PLL成为SYSCLK时钟源
		while(RCC_GetSYSCLKSource() != 0x08);
	}*/

	RCC_PeriphClockCmd();									  /*set	PeriphClock	*/ 
	RCC_AHBPeriphClockCmd(RCC_AHB_PeriphClockCmd,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1_PeriphClockCmd,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2_PeriphClockCmd,ENABLE); 
	  
	RCC_ClockSecuritySystemCmd(ENABLE);
	RCC_GetClocksFreq(&RCC_Sampling_Freq);
}

/*********************************END OF FILE**********************************/
