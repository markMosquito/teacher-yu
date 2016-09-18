/**
  ******************************************************************************
  * @file    ChassisTask.h
  * @author  
  * @version V1.0
  * @date    7-May-2015
  * @brief   Header file for ChassisTask.c module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CHASSISTASK_H
#define __CHASSISTASK_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "007_elmo.h"
#include "pid.h"

/* Defines -------------------------------------------------------------------*/
#define DEF_pi          (3.1415926535897932384626433832795)
#define DEF_sqrt2       (1.4142135623730950488016887242097)
#define DEF_sqrt2_div2  (0.70710678118654752440084436210485f)

/*����볤����Ϊ�����Σ�*/
#define CHASSIS_L (750/2)
/*�����ķ�ְ뾶 mm*/
#define WHEEL_RADIUS    (151.0/2.0)
/*�����ķ���ܳ� mm*/
#define WHEEL_PERIMETER (151 * DEF_pi)

/*��ʵ���ٶȺͿ����ٶȵ�ת����x��mm/s */
#define SPEED(x)        (((float)(x) / (float)WHEEL_PERIMETER) * REDUCTION_RATIO*2000)
/*������*/
#define VECT(a) (a).e.x - (a).s.x,(a).e.y - (a).s.y /*�����������ԭ��Ϊ���ı���*/

/*�ٶ�ת�����ضԽ��߷�����ٶ�ת��Ϊ����ת�٣� mm/s -> cnt/s*/
#define LINE_SPEED_CORRECT          (1)
#define LINE_SPEED_CONVERT_RATIO    ((float)(LINE_SPEED_CORRECT * DEF_sqrt2 * CHASSIS_REDUCTION_RATIO*CHASSIS_ENCODER_CIRCLE_CNT / WHEEL_PERIMETER))
/*���ٶ�ת�� 1��/s -> cnt/s */
#define ROTATE_SPEED_CORRECT        (1)
#define ROTATE_SPEED_CONVERT_RATIO  ((float)(2*CHASSIS_L / WHEEL_RADIUS * CHASSIS_REDUCTION_RATIO*CHASSIS_ENCODER_CIRCLE_CNT * ROTATE_SPEED_CORRECT / 360))

#define SetChassisSpeed(v1, v2, v3, v4)     Chassis_Elmo_PVM(v1,v2,v3,v4)

/* Exported variables --------------------------------------------------------*/
extern pid_t PidX, PidY, PidAng;
extern uint32_t ChassisStatus;
extern uint32_t ChassisStop;
/* Exported Function----------------------------------------------------------*/

#endif //ChassisTask.h
