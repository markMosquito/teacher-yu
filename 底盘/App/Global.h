/**
  ******************************************************************************
  * @file    Global.h
  * @author  
  * @version V1.0
  * @date    8-May-2015
  * @brief   λ����Ϣ
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
    /*ϵͳ��Ϣ*/
    point_f speed;       /* mm/s   ��/s */
    
    /*��ǰ���꣬�����ǵ�ԭʼ����*/
    point_t pos_ori;            /*ԭʼx,y����cnt & ԭʼ��̬��*/
    point_t correctPos;
    
    /*ʵ������ mm */
    point_f cur_pos;            /*��ǰ����� */
    point_f pos_old;            /*��һ�����ڵ������*/
    point_f pos_off;            /*�������*/
    
    point_cam_off_f pos_cam_off;         /*����ͷ���������뵱ǰ�����ƫ����*/
    
    /*����λ��*/
    float slideway_x_mm;
    float slideway_y_mm;
}GlobalParamTypeDef;

extern GlobalParamTypeDef G_Param;

/* ������ --------------------------------------------------------------------*/
#define CAN_ID_GYRO          (0x11)
#define CAN_ID_GYRO_SET      (0x12)
#define CAN_ID_SWITCH        (0x15)

/* ���̵��ܳ� ��λ��mm */
#define GYRO_WHEEL_PERIMETER     (3.1415926 * 50.7)
#define GYRO_CNT2M_RATIO        ((float)(GYRO_WHEEL_PERIMETER/2000.0))   //cntת��Ϊmm
#define GYRO_M2CNT_RATIO        ((float)(2000.0/GYRO_WHEEL_PERIMETER))   //mmת��Ϊcnt

/* Slideway Encoder ----------------------------------------------------------*/
//�����������λת�� ͬ�����ܳ�200mm��������һȦ2000cnt
#define SLIDEWAY_X_RATIO        ((float)(200.0 / 2000.0))
#define SLIDEWAY_Y_RATIO        ((float)(200.0 / 2000.0))

//�˴���λΪcnt
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
