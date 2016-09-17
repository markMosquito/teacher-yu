/**
  ******************************************************************************
  * @file    Manage.c
  * @author  
  * @version V1.0
  * @date    7-May-2015
  * @brief   ¿ØÖÆµ×ÅÌ&»¬¹ì
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Manage.h"
#include "ChassisTask.h"
#include "SlidewayTask.h"
#include "HitTask.h"

#include "FreeRTOS.h"
#include "task.h"

/* Private variables ---------------------------------------------------------*/
uint8_t ChassisStatusChange = RobotReady;
uint8_t SlidewayStatusChange = RobotReady;
FreeRun_TypeDef ChassisRunPoint, SlidewayRunPoint;
xyzt_TypeDef xyzt_structure;


/* Private define ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  
  * @param  None
  * @retval None
  */
/*********************************END OF FILE**********************************/
