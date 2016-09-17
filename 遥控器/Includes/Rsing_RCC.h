/**
  ******************************************************************************
  * @file    Rsing_RCC.h
  * @author  Rsing
  * @version V1.0
  * @date    22-July-2013
  * @brief   Header file for Rsing_RCC.c module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RSING_RCC_H
#define __RSING_RCC_H

/* Includes ------------------------------------------------------------------*/
#include "Rsing_Conf.h"

/* -------------------------------------------------------------------------- */
/*     Uncomment the line below to enable peripheral header file inclusion    */
/* -------------------------------------------------------------------------- */

/* --------------------------- AHB_periphclockcmd --------------------------- */
#define RCC_AHB_DMA1
//#define RCC_AHB_DMA2
//#define RCC_AHB_SRAM
//#define RCC_AHB_FLITF

/* -------------------------- APB1_periphclockcmd --------------------------- */
#define RCC_APB1_TIM2
#define RCC_APB1_TIM3
#define RCC_APB1_TIM4
//#define RCC_APB1_WWDG
//#define RCC_APB1_SPI2
//#define RCC_APB1_USART2
#define RCC_APB1_USART3
//#define RCC_APB1_I2C1
//#define RCC_APB1_I2C2
//#define RCC_APB1_USB
//#define RCC_APB1_CAN1
//#define RCC_APB1_BKP
//#define RCC_APB1_PWR
//#define RCC_APB1_ALL

/* -------------------------- APB2_periphclockcmd --------------------------- */
#define RCC_APB2_AFIO
#define RCC_APB2_GPIOA
#define RCC_APB2_GPIOB
#define RCC_APB2_GPIOC
#define RCC_APB2_GPIOD
#define RCC_APB2_GPIOE
//#define RCC_APB2_GPIOF
//#define RCC_APB2_GPIOG
//#define RCC_APB2_ADC1
//#define RCC_APB2_ADC2
#define RCC_APB2_TIM1
//#define RCC_APB2_SPI1
//#define RCC_APB2_USART1
//#define RCC_APB2_ALL

/* Exported Function----------------------------------------------------------*/
void RCC_Configuration(void);

#endif //Rsing_GPIO.h
