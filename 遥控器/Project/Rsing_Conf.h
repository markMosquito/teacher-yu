/**
  ******************************************************************************
  * @file    Rsing_Conf.h
  * @author  Rsing
  * @version V1.0
  * @date    22-July-2013
  * @brief   Library configuration file. 
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RSING_CONF_H
#define __RSING_CONF_H

/* Defines -------------------------------------------------------------------*/
#ifndef TRUE
#define TRUE	1
#endif

#ifndef	FALSE
#define	FALSE	0
#endif


/* Includes ------------------------------------------------------------------*/
/* -------------------------------------------------------------------------- */
/*     Uncomment the line below to enable peripheral header file inclusion    */
/* -------------------------------------------------------------------------- */
#include "stm32f10x.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "serial.h"
#include "comtest.h"

#include "BSP.h"
#include "APP.h"


//#include "VJ_Beep.h"
#include "VJ_Key.h"
// #include "Rsing_SysTick.h"
// #include "Rsing_RCC.h"
// #include "Rsing_GPIO.h"
// #include "Rsing_USART_dma.h"
// #include "Rsing_NVIC.h"
#include "Rsing_TIM.h"
#include "Rsing_Led.h"
#include "Rsing_12864.h"

#define UART_Put_Char(cByteToSend)  xSerialPutChar(0, cByteToSend,( TickType_t ) 0)
#define UART_Put_String(pcString, usStringLength)  vSerialPutString(0, pcString, usStringLength )
#endif  //Rsing_Conf.h
