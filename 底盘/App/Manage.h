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
  uint32_t absEndT;   //Tick为单位
    
	uint32_t running_flag;
}xyzt_TypeDef;


/** 
  * @brief  底盘/滑轨运行状态 
  */ 
enum RobotStatus
{
    RobotReady   = 0,     //不控制电机状态
    RobotRun     = 1,     //运行击球点状态
    RobotReset   = 2,     //复位状态
    RobotStop    = 3,     //停止
    RobotFreeRun = 4,     //运行至任意点ChassisRunPoint, SlidewayRunPoint，滑轨此状态运行滑轨坐标系
	RobotAvoid   = 5,     //底盘避让
    RobotTele    = 6,     //遥控底盘
    RobotTest    = 7,
};

/** 
  * @brief  RobotFreeRun模式用数据
  */
typedef struct
{
	int32_t x;
	int32_t y;
	float ang;
    uint32_t tim;   //超时时间（相对）
}FreeRun_TypeDef;

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern uint8_t ChassisStatusChange, SlidewayStatusChange;
extern FreeRun_TypeDef ChassisRunPoint, SlidewayRunPoint;
extern xyzt_TypeDef xyzt_structure;

/* Exported functions --------------------------------------------------------*/
#endif //__MANAGE_H
