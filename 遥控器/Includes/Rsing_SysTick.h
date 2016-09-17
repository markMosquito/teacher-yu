/**
  ******************************************************************************
  * @file    Rsing_SysTick.h
  * @author  Rsing
  * @version V1.0
  * @date    18-August-2013
  * @brief   Header file for Rsing_SysTick.c module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RSING_SYSTICK_H
#define __RSING_SYSTICK_H

/* Includes ------------------------------------------------------------------*/
#include "Rsing_Conf.h"

/* Exported Function----------------------------------------------------------*/
extern __IO u16 TimingDelay;
void SysTick_Configuration(u32 MsCount);
void Delay_ms(__IO u16 ms_delay);
#endif /* __RSING_SYSTICK_H */
