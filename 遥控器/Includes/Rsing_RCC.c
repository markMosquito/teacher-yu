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
	/* ����ö�����ͱ��� HSEStartUpStatus */
	ErrorStatus HSEStartUpStatus;

	/* ��λϵͳʱ������*/
	RCC_DeInit();

	/* ����HSE�������ⲿʱ�ӣ�*/
	RCC_HSEConfig(RCC_HSE_ON);

	/* �ȴ�HSE�����ȶ�*/
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	
	/* �ж�HSE���Ƿ���ɹ����������if()�ڲ� */
	if(HSEStartUpStatus == SUCCESS)
	{
		/* ѡ��HCLK��AHB��ʱ��ԴΪSYSCLK 1��Ƶ */
		RCC_HCLKConfig(RCC_SYSCLK_Div1); 

		/* ѡ��PCLK1ʱ��ԴΪ HCLK��AHB�� 2��Ƶ������ʱ�ӣ���APB1�õ���MAX=36MHz,NOW=36MHz */
		RCC_PCLK1Config(RCC_HCLK_Div2);
		
		/* ѡ��PCLK2ʱ��ԴΪ HCLK��AHB�� 1��Ƶ������ʱ�ӣ���APB2�õ���MAX=72MHz,NOW=72MHz */
		RCC_PCLK2Config(RCC_HCLK_Div1); 

		/* STM32�����ڽϸ�Ƶ����72MHz��FLASH�����ڽϵ�Ƶ����24MHz��STM32��ȡFLASH��Ҫ�����ȴ����� */
		/* ����FLASH��ʱ������Ϊ2�����ô�����ʱֵ */
		FLASH_SetLatency(FLASH_Latency_2);
		
		/* ʹ��FLASHԤȡ���� */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* ѡ�����໷��PLL��ʱ��ԴΪHSE 1��Ƶ����Ƶ��Ϊ9����PLL���Ƶ��Ϊ 8MHz * 9 = 72MHz */
    	RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    	/* ʹ��PLL */ 
    	RCC_PLLCmd(ENABLE);

    	/* �ȴ�ָ����RCC��־λ���óɹ� �ȴ�PLL����ȶ� */
    	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

    	/* ѡ��SYSCLKʱ��ԴΪPLL */
    	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    	/* �ȴ�PLL��ΪSYSCLKʱ��Դ */
    	while(RCC_GetSYSCLKSource() != 0x08);
  	}
/*	else
	{
		// ѡ�����໷��PLL��ʱ��ԴΪHSI 2��Ƶ����Ƶ��Ϊ16����PLL���Ƶ��Ϊ 8MHz / 2 * 16 = 64MHz
		RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);
		
		// ʹ��PLL
    	RCC_PLLCmd(ENABLE);
		
    	// �ȴ�ָ����RCC��־λ���óɹ� �ȴ�PLL����ȶ�
    	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
		
		// ѡ��SYSCLKʱ��ԴΪPLL
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		
    	// �ȴ�PLL��ΪSYSCLKʱ��Դ
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
