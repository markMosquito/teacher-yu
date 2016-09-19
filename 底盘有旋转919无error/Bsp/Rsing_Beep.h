/**
  ******************************************************************************
  * @file    Rsing_Beep.h
  * @author  Rsing
  * @version V1.0
  * @date    14-November-2014
  * @brief   Header file for Rsing_Beep.c module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RSING_BEEP_H
#define __RSING_BEEP_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "Redef_GPIO.h"
#include "bsp.h"

/* Private functions ---------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#ifndef IO_BEEP
 #error "IO_BEEPδ����"
//#define IO_BEEP         PE(12)
#endif
#define BEEP_ON         (IO_GPIO_PORT(IO_BEEP)->BSRRL = IO_GPIO_PIN(IO_BEEP))
#define BEEP_OFF        (IO_GPIO_PORT(IO_BEEP)->BSRRH = IO_GPIO_PIN(IO_BEEP))

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void Beep_Configuration(void);
void Buzzer(uint8_t time, uint8_t OnTime, uint8_t OffTime);
void BuzzerFromISR(uint8_t time, uint8_t OnTime, uint8_t OffTime);
#endif //__RSING_BEEP_H.h
