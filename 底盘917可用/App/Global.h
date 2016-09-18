/**
  ******************************************************************************
  * @file    Global.h
  * @author  
  * @version V1.0
  * @date    8-May-2015
  * @brief   位置信息
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GLOBAL_H_
#define __GLOBAL_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "vect.h"

typedef struct 
{
    /*系统信息*/
    point_f speed;       /* mm/s   °/s */
    
    /*当前坐标，陀螺仪的原始数据*/
    point_t pos_ori;            /*原始x,y坐标cnt & 原始姿态°*/
    point_t correctPos;
    
    /*实际坐标 mm */
    point_f cur_pos;            /*当前坐标点 */
    point_f pos_old;            /*上一个周期的坐标点*/
    point_f pos_off;            /*起点坐标*/
    
    point_cam_off_f pos_cam_off;         /*摄像头矫正坐标与当前坐标的偏移量*/
    
    /*滑轨位置*/
    float slideway_x_mm;
    float slideway_y_mm;
}GlobalParamTypeDef;

extern GlobalParamTypeDef G_Param;

/* 陀螺仪 --------------------------------------------------------------------*/
#define CAN_ID_GYRO          (0x11)
#define CAN_ID_GYRO_SET      (0x12)
#define CAN_ID_SWITCH        (0x15)

/* 码盘的周长 单位：mm */
#define GYRO_WHEEL_PERIMETER     (3.1415926 * 50.7)
#define GYRO_CNT2M_RATIO        ((float)(GYRO_WHEEL_PERIMETER/2000.0))   //cnt转换为mm
#define GYRO_M2CNT_RATIO        ((float)(2000.0/GYRO_WHEEL_PERIMETER))   //mm转换为cnt

/* Slideway Encoder ----------------------------------------------------------*/
//滑轨编码器单位转换 同步轮周长200mm，编码器一圈2000cnt
#define SLIDEWAY_X_RATIO        ((float)(200.0 / 2000.0))
#define SLIDEWAY_Y_RATIO        ((float)(200.0 / 2000.0))

//此处单位为cnt
#define ReadSlidewayX_cnt()     ((int16_t)TIM3->CNT)
#define ReadSlidewayY_cnt()     ((int16_t)TIM4->CNT)

void SetSlidewayX_mm(float mm);
void SetSlidewayY_mm(float mm);
float ReadSlidewayX_mmFromCan(void);
float ReadSlidewayY_mmFromCan(void);
float ReadSlidewayX_mm(void);
float ReadSlidewayY_mm(void);
/* Exported functions --------------------------------------------------------*/
void GyroReset(void);
#endif //__GLOBAL_H_
