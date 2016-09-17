/**
  ******************************************************************************
  * @file    Rsing_GPIO.h
  * @author  Rsing
  * @version V1.0
  * @date    28-July-2014
  * @brief   Header file for Rsing_GPIO.c module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RSING_GPIO_H
#define __RSING_GPIO_H

/* Includes ------------------------------------------------------------------*/
#if defined (STM32F10X_HD)
 #include "stm32f10x.h"
#elif defined (STM32F40_41xxx)
 #include "stm32f4xx.h"
#endif

/* Define --------------------------------------------------------------------*/
#define PA(x)       (16*0 + x)
#define PB(x)       (16*1 + x)
#define PC(x)       (16*2 + x)
#define PD(x)       (16*3 + x)
#define PE(x)       (16*4 + x)
#define PF(x)       (16*5 + x)
#define PG(x)       (16*6 + x)
#define PH(x)       (16*7 + x)

//GPIO_Port
#define GPIO_PORT(n) \
        (\
            ((uint16_t)n /16 == 0)?GPIOA:\
          ( ((uint16_t)n /16 == 1)?GPIOB:\
          ( ((uint16_t)n /16 == 2)?GPIOC:\
          ( ((uint16_t)n /16 == 3)?GPIOD:\
          ( ((uint16_t)n /16 == 4)?GPIOE:\
          ( ((uint16_t)n /16 == 5)?GPIOF:GPIOG\
          )))))\
        )

//GPIO_Pin
#define GPIO_PIN(n)                 (uint16_t)(1 << (n %16) )

//GPIO_PinSource
#define GPIO_PINSOURCE(n)           (uint8_t) ( n %16 )

//RCC_AHB1_Periph_GPIO
#define RCC_AHB1_PERIPH_GPIO(n)     (uint32_t)(1 << (n /16) )

//RCC_AHB1_Periph_GPIO
#define RCC_APB2_PERIPH_GPIO(n)     (uint32_t)(1 << (2 + (n /16)) )

//EXTI_PORTSOURCE
#define EXTI_PORTSOURCE(n)          (uint8_t) ( n /16 )

//EXTI_LINE
#define EXTI_LINE(n)                (uint32_t)(1 << (n %16) )

//EXTI_IRQn
#define EXTI_IRQN(n)                \
        (\
            (n %16 < 5)?(n %16 + EXTI0_IRQn):\
          ( (n %16 < 10)?EXTI9_5_IRQn:EXTI15_10_IRQn\
          )\
        )
/* Private functions ---------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#endif //Rsing_Periph_Handle.h
