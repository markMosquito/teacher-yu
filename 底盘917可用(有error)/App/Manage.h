/**
  ******************************************************************************
  * @file    Manage.h
  * @author  
  * @version V1.0
  * @date    7-May-2015
  * @brief   Header file for Manage.c module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MANAGE_H
#define __MANAGE_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "vect.h"
#include "ManageConfig.h"

/* Private functions ---------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	int32_t x;
	int32_t y;
	int32_t z;
	int32_t t;
  uint32_t absEndT;   //TickΪ��λ
    
	uint32_t running_flag;
}xyzt_TypeDef;


/** 
  * @brief  ����/��������״̬ 
  */ 
enum RobotStatus
{
    RobotReady   = 0,     //�����Ƶ��״̬
    RobotRun     = 1,     //���л����״̬
    RobotReset   = 2,     //��λ״̬
    RobotStop    = 3,     //ֹͣ
    RobotFreeRun = 4,     //�����������ChassisRunPoint, SlidewayRunPoint�������״̬���л�������ϵ
	RobotAvoid   = 5,     //���̱���
    RobotTele    = 6,     //ң�ص���
    RobotTest    = 7,
};

/** 
  * @brief  RobotFreeRunģʽ������
  */
typedef struct
{
	int32_t x;
	int32_t y;
	float ang;
    uint32_t tim;   //��ʱʱ�䣨��ԣ�
}FreeRun_TypeDef;

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern uint8_t ChassisStatusChange, SlidewayStatusChange;
extern FreeRun_TypeDef ChassisRunPoint, SlidewayRunPoint;
extern xyzt_TypeDef xyzt_structure;

/* Exported functions --------------------------------------------------------*/
#endif //__MANAGE_H
