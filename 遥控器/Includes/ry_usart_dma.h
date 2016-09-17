/**
  ******************************************************************************
  * @file    ry_usart_dma.h
  * @author  Ry
  * @version V1.0
  * @date    16-Feb-2016
  * @brief   Header file for usart_dma module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RY_USART_DMA_H__
#define __RY_USART_DMA_H__

/* Includes ------------------------------------------------------------------*/
#include "Rsing_Conf.h"

/* Private functions ---------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void DmaSendPos(void);
void usart1_dma_config(void);

#endif //Rsing_Periph_Handle.h
