/**
  ******************************************************************************
  * @file    USART_DMA.h
  * @author  
  * @version V1.0
  * @date    2-September-2013
  * @brief   Header file for USART_DMA.c module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_DMA_H
#define __USART_DMA_H

/* Includes ------------------------------------------------------------------*/
#include "Rsing_Conf.h"

/* Private functions ---------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void USART3_Configuration(void);
void DMA_Configuration(void);
void DMA_printf(uint8_t *Data,...);

#endif //USART_DMA.h

