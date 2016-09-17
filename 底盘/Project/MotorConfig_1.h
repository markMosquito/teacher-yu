/**
  ******************************************************************************
  * @file    MotorConfig_1.h
  * @author  
  * @version V1.0
  * @date    7-May-2015
  * @brief   
  ******************************************************************************
  * 以麦克纳姆轮为例，其余的底盘类型是一样的
  * 默认电机编号是逆时针排列如下图，电机正方向转动的时候底盘向前运动
  * 左上角第一个电机为1号，电机正方向如箭头
  * 
  * Y正方向
  * ^
  * |   ___车头方向___
  * |   | 1 ↑    4 ↑ |
  * |   |            |
  * |   | 2 ↑    3 ↑ |
  * |   --------------
  * |-------------------------> X正方向
  * Without angle:
  * Motor1 = ( Vx + Vy)/R;
  * Motor2 = (-Vx + Vy)/R;
  * Motor3 = Motor1;
  * Motor4 = Motor2;
  * With angle:  此处为角度
  * Motor1 = ( Vx*cos(45 + alpha) + Vy*sin(45 + alpha))*sqrt(2)/R; 
  * Motor2 = (-Vx*sin(45 + alpha) + Vy*cos(45 + alpha))*sqrt(2)/R;
  * Motor3 = Motor1;
  * Motor4 = Motor2;
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTORCONFIG_SELECT_H
#define __MOTORCONFIG_SELECT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"


/* Exported define -----------------------------------------------------------*/
/**
  * @note
   此处定义的ID及方向
   底盘与滑轨的在 007_elmo.c: Chassis_Elmo_PVM, Slideway_Elmo_PVM函数内部进行修改
   
   5~8 & 1~2采用组ID形式，在Elmo_Init时进行组ID分配
   若ID非5~8 & 1~2，修改时应注意，此处定义ID仅为方便切换顺序及方向
   
   挥拍电机ID及方向在宏定义函数内修改
*/

/* 滑轨编码器一圈计数值 */
#define ENCODER_CIRCLE_CNT      (10000)

/* EC30最大速度， 单位：cnt/s */
#define EC30_MAX_SPEED          ((int32_t)(16000*2000/60 - 10))
#define EC45_MAX_SPEED          ((int32_t)(10000*2000/60 - 10))
#define ST8N60P40V4E_MAX_SPEED  ((int32_t)(3000 *10000/60 - 10))
#define EC4270_MAX_SPEED        ((int32_t)(12949*10000/60 - 10))

/* 底盘 Motor Can ID & Dir (其余在 ChassisTask.h)-----------------------------*/
#define MOTOR_ID_CHASSIS_1      (5)
#define MOTOR_ID_CHASSIS_2      (8)
#define MOTOR_ID_CHASSIS_3      (7)
#define MOTOR_ID_CHASSIS_4      (6)

#define MOTOR_DIR_CHASSIS_1     (-1)
#define MOTOR_DIR_CHASSIS_2     (-1)
#define MOTOR_DIR_CHASSIS_3     (1)
#define MOTOR_DIR_CHASSIS_4     (1)

/* 底盘电机减速比 */
#define CHASSIS_REDUCTION_RATIO (3)
/* 底盘编码器一圈计数值 */
#define CHASSIS_ENCODER_CIRCLE_CNT      (10000)
/* 底盘车轮的最大速度(cnt/s) */
#define CHASSIS_MAX_SPEED               ST8N60P40V4E_MAX_SPEED
    
/* 滑轨 Motor Can ID & Dir----------------------------------------------------*/
#define MOTOR_ID_SLIDEWAY_X                 (1)
#define MOTOR_DIR_SLIDEWAY_X                (1)
#define MOTOR_REDUCTION_RATIO_SLIDEWAY_X    (16.2f)
#define MOTOR_WHEEL_PERIMETER_SLIDEWAY_X    (200)    //mm
#define MOTOR_MM2CNT_SLIDEWAY_X             ((float)(MOTOR_REDUCTION_RATIO_SLIDEWAY_X * ENCODER_CIRCLE_CNT / MOTOR_WHEEL_PERIMETER_SLIDEWAY_X))
#define MOTOR_MAXSPEED_SLIDEWAY_X           EC4270_MAX_SPEED

#define MOTOR_ID_SLIDEWAY_Y                 (2)
#define MOTOR_DIR_SLIDEWAY_Y                (-1)
#define MOTOR_REDUCTION_RATIO_SLIDEWAY_Y    (16.2f)
#define MOTOR_WHEEL_PERIMETER_SLIDEWAY_Y    (200)    //mm
#define MOTOR_MM2CNT_SLIDEWAY_Y             ((float)(MOTOR_REDUCTION_RATIO_SLIDEWAY_Y * ENCODER_CIRCLE_CNT / MOTOR_WHEEL_PERIMETER_SLIDEWAY_Y))
#define MOTOR_MAXSPEED_SLIDEWAY_Y           EC4270_MAX_SPEED

/* 挥拍 Motor Can ID & Dir----------------------------------------------------*/
#define MOTOR_ID_HIT_MAIN                   (3)
#define MOTOR_DIR_HIT_MAIN                  (-1)
#define MOTOR_REDUCTION_RATIO_HIT_MAIN      (27.6f)
#define MOTOR_R2CNT_HIT_MAIN                (MOTOR_REDUCTION_RATIO_HIT_MAIN * ENCODER_CIRCLE_CNT)  /* 转/秒->cnt/s */
#define MOTOR_CNT2ANG_HIT_MAIN              (360.0f/MOTOR_R2CNT_HIT_MAIN)
#define MOTOR_MAXSPEED_HIT_MAIN             EC4270_MAX_SPEED

#define MOTOR_ID_HIT_SERVE                  (4)
#define MOTOR_DIR_HIT_SERVE                 (-1)         //左侧角度为正，右侧为负
#define MOTOR_REDUCTION_RATIO_HIT_SERVE     (27.6f)
#define MOTOR_R2CNT_HIT_SERVE               (MOTOR_REDUCTION_RATIO_HIT_SERVE * ENCODER_CIRCLE_CNT)
#define MOTOR_CNT2ANG_HIT_SERVE             (360.0f/MOTOR_R2CNT_HIT_SERVE)
#define MOTOR_MAXSPEED_HIT_SERVE            EC4270_MAX_SPEED

/* 加速度 */                                //减速后转速rps^2 -->> cnt/s^2
#define CHASSIS_ACC_MAX                     ((int32_t)(33*CHASSIS_REDUCTION_RATIO*CHASSIS_ENCODER_CIRCLE_CNT))
#define SLIDEWAY_X_ACC_MAX                  ((int32_t)(100000000   *MOTOR_REDUCTION_RATIO_SLIDEWAY_X*ENCODER_CIRCLE_CNT/(19*2000)))//((int32_t)(100000000  ))	//1m/s2 190000 60000 * 190
#define SLIDEWAY_Y_ACC_MAX                  ((int32_t)(35000 * 190 *MOTOR_REDUCTION_RATIO_SLIDEWAY_Y*ENCODER_CIRCLE_CNT/(19*2000)))//((int32_t)(35000 * 190))	//1m/s2 190000

/* PID */
                                /*  Kp,     Ki,     Kd,         out_max,    out_min,    iteg_max,   dead_zone */
#define DEF_PID_PidXNormal      {   4,      0,      200.0,   	8000,       -8000,      0,          25}
#define DEF_PID_PidXReset       {   5,      0,      100.0,      1000,       -1000,      0,          0 }
#define DEF_PID_PidYNormal      {   4,      0,      210.0,   	8000,       -8000,      0,          25}
#define DEF_PID_PidYReset       {   5,      0,      100.0,      1000,       -1000,      0,          0 }

#define DEF_PID_PidAngNormal    {   6,      0,      0,          100,        -100,       0,          0.05 }
#define DEF_PID_PidAngReset     {   4,      0,      0,     		100,        -100,       0,          0.05 }

                                /* Kp,      Ki,     Kd,         out_max,    out_min,    iteg_max,   dead_zone */
#define DEF_PID_PidSlideXNormal {  26,      0,      0,          2800,       -2800,      0,          2};
#define DEF_PID_PidSlideXReset  {  26*0.9,  0,      0,          2800*0.25,  -2800*0.25, 0,          0};
#define DEF_PID_PidSlideYNormal {  18.5,    0,      0,          2800,       -2800,      0,          2};
#define DEF_PID_PidSlideYReset  {  18.5*0.9,0,      0,          2800*0.25,  -2800*0.25, 0,          0};

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#else
#error "__MOTORCONFIG_SELECT_H has defined!"
#endif //__MOTORCONFIG_SELECT_H
