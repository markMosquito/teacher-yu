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
//��ջ��С����
#define AppTaskStartStk                 (configMINIMAL_STACK_SIZE)

//�������ȼ�����
#define APP_CFG_TASK_START_PRIO         ( tskIDLE_PRIORITY + 1UL )
#define APP_CFG_TASK_TX_PRIO            ( tskIDLE_PRIORITY + 1UL )
#define APP_CFG_TASK_RX_PRIO            ( tskIDLE_PRIORITY + 1UL )

/*������*/
#define AppTaskBeepStk                  (configMINIMAL_STACK_SIZE)
#define APP_CFG_TASK_BEEP_PRIO          ( tskIDLE_PRIORITY + 2UL )
void vBeepTask( void *p_arg );
extern TaskHandle_t BeepTaskHandle;


/*λ�ü�������*/
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
