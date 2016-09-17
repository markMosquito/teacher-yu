/**
  ******************************************************************************
  * @file    ry_adc.h
  * @author  Ry
  * @version V1.0
  * @date    16-Feb-2016
  * @brief   Header file for Rsing_Periph_Handle.c module. 前几天写的没保存，日了狗
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RY_ADC_H
#define __RY_ADC_H

/* Includes ------------------------------------------------------------------*/
#include "Rsing_Conf.h"

/* Private functions ---------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void adc_dma_config(void);
void filter(void);
#endif //Rsing_Periph_Handle.h
