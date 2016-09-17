/**
  ******************************************************************************
  * @file    Rsing_SysTick.c
  * @author  Rsing
  * @version V1.0
  * @date    18-August-2013
  * @brief   Initializate the SysTick
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Rsing_SysTick.h"
__IO u16 TimingDelay;

/**
  * @brief  Init and start SysTick
  * @param  MsCount	
  * @retval None
  */
void SysTick_Configuration(u32 MsCount)
{
	if(MsCount > 0xFFFFFF) MsCount=0xFFFFFF;
	if ( SysTick_Config(MsCount*9000) )	//Set systick for 1ms,MsCount*9000_Max=0xFFFFFF,0x1000000
	{ 
		/* Capture error */ 
		while (1);
	}
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
					//ATTEATION : this must be set after SysTick_Config(),or it will reflash the maxcount
}

/**
  * @brief  Delay_ms
  * @param  ms_delay
  * @retval None
  */
void Delay_ms(__IO u16 ms_delay)
{
	TimingDelay = ms_delay;
	while(TimingDelay != 0);
}
/**********************************END OF FILE*********************************/
