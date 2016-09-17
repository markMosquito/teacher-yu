/**
  ******************************************************************************
  * @file    Rsing_Led.h
  * @author  Rsing
  * @version V1.0
  * @date    29-November-2014
  * @brief   Header file for Rsing_Led.c module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RSING_LED_H
#define __RSING_LED_H

/* Includes ------------------------------------------------------------------*/
#if defined (STM32F10X_HD)
 #include "stm32f10x.h"
#elif defined (STM32F40_41xxx)
 #include "stm32f4xx.h"
#endif
/* Private functions ---------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum 
{
  LED0 = 0,//B1
  LED1 = 1,//G1
  LED2 = 2,//R1
  LED3 = 3,//B2
  LED4 = 4,//G2
  LED5 = 5,//R2
} Led_TypeDef;
/* Exported constants --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void LEDInit(Led_TypeDef Led);
void LEDOn(Led_TypeDef Led);
void LEDOff(Led_TypeDef Led);
void LEDToggle(Led_TypeDef Led);
#endif //Rsing_Led.h
