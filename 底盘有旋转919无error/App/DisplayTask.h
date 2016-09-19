/**
  ******************************************************************************
  * @file    DisplayTask.h
  * @author  
  * @version V1.0
  * @date    8-May-2015
  * @brief   Header file for DisplayTask.c module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DISPLAYTASK_H
#define __DISPLAYTASK_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"

/* Defines -------------------------------------------------------------------*/
#define DISPLAY_CHAR_BUF    (128)   //每个字符串的最大长度
#define DISPLAY_QUEUE_NUM   (32)    //

#define DISPLAY_MODE_ERRSHOW        (1)
#define DISPLAY_MODE_RACKET_ERR     (2)
#define DISPLAY_MODE_CORRECT_ERR    (3)
#define DISPLAY_MODE_CAR_MOVE       (4)
#define DISPLAY_MODE_CORRECT_OK     (5)
#define DISPLAY_MODE_RESET     		(6)
#define DISPLAY_MODE_AVOID     		(7)
#define DISPLAY_MODE_TIM250         (8)
#define DISPLAY_PRINTF              (9)
#define DISPLAY_MODE_TIMMAX         (10)
/* Exported variables --------------------------------------------------------*/

/* Exported Function----------------------------------------------------------*/
void LCD_QueuePrintf(uint16_t Xpos, uint16_t Ypos, uint8_t width, const char *fmt,...);
void LCD_QueuePrintfErr(uint16_t Xpos, uint16_t Ypos, uint8_t width, const char *fmt,...);
void LCDShowErrFromISR(uint32_t ErrType);
#endif //__DISPLAYTASK_H
