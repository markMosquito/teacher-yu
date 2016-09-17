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

/*车身半长（车为正方形）*/
#define CHASSIS_L (750/2)
/*麦克纳姆轮半径 mm*/
#define WHEEL_RADIUS    (151.0/2.0)
/*麦克纳姆轮周长 mm*/
#define WHEEL_PERIMETER (151 * DEF_pi)

/*车实际速度和控制速度的转化，x是mm/s */
#define SPEED(x)        (((float)(x) / (float)WHEEL_PERIMETER) * REDUCTION_RATIO*2000)
/*其他宏*/
#define VECT(a) (a).e.x - (a).s.x,(a).e.y - (a).s.y /*将向量变成以原点为起点的变量*/

/*速度转换（沿对角线方向的速度转换为轮子转速） mm/s -> cnt/s*/
#define LINE_SPEED_CORRECT          (1)
#define LINE_SPEED_CONVERT_RATIO    ((float)(LINE_SPEED_CORRECT * DEF_sqrt2 * CHASSIS_REDUCTION_RATIO*CHASSIS_ENCODER_CIRCLE_CNT / WHEEL_PERIMETER))
/*角速度转换 1°/s -> cnt/s */
#define ROTATE_SPEED_CORRECT        (1)
#define ROTATE_SPEED_CONVERT_RATIO  ((float)(2*CHASSIS_L / WHEEL_RADIUS * CHASSIS_REDUCTION_RATIO*CHASSIS_ENCODER_CIRCLE_CNT * ROTATE_SPEED_CORRECT / 360))

#define SetChassisSpeed(v1, v2, v3, v4)     Chassis_Elmo_PVM(v1,v2,v3,v4)

/* Exported variables --------------------------------------------------------*/
extern pid_t PidX, PidY, PidAng;
extern uint32_t ChassisStatus;
extern uint32_t ChassisStop;
/* Exported Function----------------------------------------------------------*/

#endif //ChassisTask.h
