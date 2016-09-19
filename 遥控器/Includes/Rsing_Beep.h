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
#include "stm32f10x.h"
#include "Redef_GPIO.h"

/* Private functions ---------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define IO_BEEP PA(3)

#ifndef IO_BEEP
 #define IO_BEEP         PB(12)
 #warning "IO_BEEP not define, so defined PB(12)."
#endif

#define BEEP_ON         (IO_GPIO_PORT(IO_BEEP)->BSRR = IO_GPIO_PIN(IO_BEEP))
#define BEEP_OFF        (IO_GPIO_PORT(IO_BEEP)->BRR  = IO_GPIO_PIN(IO_BEEP))

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void Beep_Configuration(void);
void Buzzer(uint8_t time, uint8_t OnTime, uint8_t OffTime);
void BuzzerFromISR(uint8_t time, uint8_t OnTime, uint8_t OffTime);
#endif //__RSING_BEEP_H.h
