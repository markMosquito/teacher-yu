/**
  ******************************************************************************
  * @file    Rsing_TIM.h
  * @author  Rsing
  * @version V1.0
  * @date    20-July-2013
  * @brief   Header file for Rsing_TIM.c module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RSING_TIM_H
#define __RSING_TIM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
/* Exported Function----------------------------------------------------------*/
//void PWM_PeriphInit(TIM_TypeDef* PWM_TIMx ,u16 PWM_Frequence,u16 PWM_Prescaler);
//void PWM_Configuration(TIM_TypeDef* PWM_TIMx, u16 PWM_Frequence, u16 PWM_Prescaler);
void TIM_Configuration(void);
void PWM_DutyChange(TIM_TypeDef* PWM_TIMx ,u8 PWM_Channel ,u16 PWM_Duty);
void TimFreChange(uint16_t fre);
#endif //Rsing_TIM.h
