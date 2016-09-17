/**
  ******************************************************************************
  * @file    Rsing_USART3.h
  * @author  Rsing
  * @version V1.0
  * @date    22-July-2013
  * @brief   Header file for Rsing_USART3.c module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RSING_USART3_H
#define __RSING_USART3_H

/* Includes ------------------------------------------------------------------*/
#include "Rsing_Conf.h"
#include "stdio.h"

/* Exported Function----------------------------------------------------------*/
void USART3_Configuration(void);
int fputc(int ch, FILE *f);
//extern void USART3_SendString(u8* str);
#endif /* __RSING_USART3_H */
