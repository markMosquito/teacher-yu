/**
  ******************************************************************************
  * @file    APP.h
  * @author  
  * @version V1.0
  * @date    8-May-2015
  * @brief   Header file for APP.c module. 
  * @Note    优先级数字越大优先级越高
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_H
#define __APP_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "Rsing_Beep.h"
#include "manage.h"

#include "Global.h"
#include "ChassisTask.h"
#include "SlidewayTask.h"
#include "HitTask.h"
#include "DisplayTask.h"

/* Defines -------------------------------------------------------------------*/
//堆栈大小定义
#define AppTaskStartStk                 (512)

//任务优先级定义
#define APP_CFG_TASK_START_PRIO         ( tskIDLE_PRIORITY + 1UL )
#define APP_CFG_TASK_TX_PRIO            ( tskIDLE_PRIORITY + 1UL )
#define APP_CFG_TASK_RX_PRIO            ( tskIDLE_PRIORITY + 1UL )

/*蜂鸣器*/
#define AppTaskBeepStk                  (configMINIMAL_STACK_SIZE)
#define APP_CFG_TASK_BEEP_PRIO          ( tskIDLE_PRIORITY + 2UL )
void vBeepTask( void *p_arg );
extern TaskHandle_t BeepTaskHandle;

/*12864显示任务*/
#define DISPLAY_TASK_STK                (1024)
#define DISPLAY_TASK_PRIO               ( tskIDLE_PRIORITY + 2UL )
void vDisplayTask( void *p_arg );
extern TaskHandle_t DisplayTaskHandle;

/*ELMO检测*/
#define CHECK_TASK_STK                  (configMINIMAL_STACK_SIZE)
#define CHECK_TASK_PRIO                 ( tskIDLE_PRIORITY + 2UL )
void vCheckTask(void *p_arg);
extern TaskHandle_t CheckTaskHandle;

/*位置计算任务*/
#define POSCAL_TASK_STK                 (configMINIMAL_STACK_SIZE)
#define POSCAL_TASK_PRIO                ( tskIDLE_PRIORITY + 4UL )
void vPosCalTask( void *p_arg );
extern TaskHandle_t PosCalTaskHandle;

/*底盘任务*/
#define CHASSIS_TASK_STK                (1024)
#define CHASSIS_TASK_PRIO               ( tskIDLE_PRIORITY + 2UL )
void vChassisTask( void *p_arg );
extern TaskHandle_t ChassisTaskHandle;

/*底盘任务*/
#define AMC_MOTION_TASK_STK                (1024)
#define AMC_MOTION_TASK_PRIO               ( tskIDLE_PRIORITY + 3UL )
void vAmcMotionTask( void *p_arg );
extern TaskHandle_t AmcMotionTaskHandle;


/*滑轨任务*/
#define SLIDEWAY_TASK_STK               (1024)
#define SLIDEWAY_TASK_PRIO              ( tskIDLE_PRIORITY + 3UL )
void vSlidewayTask( void *p_arg );
extern TaskHandle_t SlidewayTaskHandle;

/*击球任务*/
#define HIT_TASK_STK                    (512)
#define HIT_TASK_PRIO                   ( tskIDLE_PRIORITY + 3UL )
void vHitTask( void *p_arg );
extern TaskHandle_t HitTaskHandle;

/*发球任务*/
#define SERVE_TASK_STK                  (512)
#define SERVE_TASK_PRIO                 ( tskIDLE_PRIORITY + 3UL )
void vServeTask( void *p_arg );
extern TaskHandle_t ServeTaskHandle;

/*重启任务*/
#define RESTART_TASK_STK                (512)
#define RESTART_TASK_PRIO               ( tskIDLE_PRIORITY + 4UL )
void vRestartTask( void *p_arg );
extern TaskHandle_t RestartTaskHandle;

/* xSemaphoreHandle ----------------------------------------------------------*/
extern xSemaphoreHandle PosSem;         //收到码盘+陀螺仪数据 (中断送出)
extern xSemaphoreHandle PosChassisSem;  //发送信号量给底盘
extern xSemaphoreHandle PosSlideSem;    //发送信号量给滑轨
extern QueueHandle_t HitInQueue;        //挥拍任务接收用邮箱
extern QueueHandle_t UsartTxQueue;      //串口DMA发送队列

/* Exported Function----------------------------------------------------------*/
void StartTaskCreate(void);
void AppTaskCreate (void);
void GlobalVariableInit(void);
void ElmoErrBeep(uint32_t ElmoFlag);

typedef enum
{
    TEST_NO,
    TEST_HIT_MAIN,
    TEST_HIT_LEFT,
    TEST_HIT_RIGHT,
    TEST_SLIDEWAY,
    TEST_HITTIME
}Test_Typedef;
#endif //APP.h
