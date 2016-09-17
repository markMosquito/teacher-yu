/**
  ******************************************************************************
  * @file    APP.h
  * @author  Rsing
  * @version V1.0
  * @date    22-July-2014
  * @brief   Header file for APP.c module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_H
#define __APP_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Defines -------------------------------------------------------------------*/
//堆栈大小定义
#define AppTaskStartStk                 (configMINIMAL_STACK_SIZE)

//任务优先级定义
#define APP_CFG_TASK_START_PRIO         ( tskIDLE_PRIORITY + 1UL )
#define APP_CFG_TASK_TX_PRIO            ( tskIDLE_PRIORITY + 1UL )
#define APP_CFG_TASK_RX_PRIO            ( tskIDLE_PRIORITY + 1UL )

/*蜂鸣器*/
#define AppTaskBeepStk                  (configMINIMAL_STACK_SIZE)
#define APP_CFG_TASK_BEEP_PRIO          ( tskIDLE_PRIORITY + 2UL )
void vBeepTask( void *p_arg );
extern TaskHandle_t BeepTaskHandle;


/*位置计算任务*/
#define AppTaskMainStk                  (512)
#define APP_CFG_TASK_MAIN_PRIO          ( tskIDLE_PRIORITY + 4UL )
void vMainTask( void *p_arg );
extern TaskHandle_t MainTaskHandle;



/* Exported xSemaphoreHandle ----------------------------------------------------------*/
//extern xSemaphoreHandle PosSem;

/* Exported Function----------------------------------------------------------*/
void StartTaskCreate(void);
void AppTaskCreate (void);

#endif //APP.h
