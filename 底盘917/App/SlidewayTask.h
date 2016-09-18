/**
  ******************************************************************************
  * @file    SlidewayTask.h
  * @author  
  * @version V1.0
  * @date    7-May-2015
  * @brief   Header file for SlidewayTask.c module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SLIDEWAYTASK_H
#define __SLIDEWAYTASK_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "007_elmo.h"
#include "pid.h"
#include "Manage.h"

/* Exported variables --------------------------------------------------------*/
extern pid_t PidSlideX, PidSlideY;
extern uint32_t SlidewayStop;      //仅在SlidewayStatus == RobotRun时有效
extern uint32_t SlidewayStatus;

#define X_Elmo_PVM(v)       Elmo_PVM(MOTOR_ID_SLIDEWAY_X, MOTOR_DIR_SLIDEWAY_X * MOTOR_MM2CNT_SLIDEWAY_X*(v))
#define Y_Elmo_PVM(v)       Elmo_PVM(MOTOR_ID_SLIDEWAY_Y, MOTOR_DIR_SLIDEWAY_Y * MOTOR_MM2CNT_SLIDEWAY_Y*(v))
#define X_Elmo_Stop()       Elmo_Stop(MOTOR_ID_SLIDEWAY_X)
#define Y_Elmo_Stop()       Elmo_Stop(MOTOR_ID_SLIDEWAY_Y)

/* Exported Function----------------------------------------------------------*/
void XY_Elmo_PVM(float speed1,float speed2);
void SlidewayHoming(void);
#endif //SlidewayTask.h
