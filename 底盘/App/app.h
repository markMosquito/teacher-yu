/**
  ******************************************************************************
  * @file    APP.h
  * @author  
  * @version V1.0
  * @date    8-May-2015
  * @brief   Header file for APP.c module. 
  * @Note    ���ȼ�����Խ�����ȼ�Խ��
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
//��ջ��С����
#define AppTaskStartStk                 (512)

//�������ȼ�����
#define APP_CFG_TASK_START_PRIO         ( tskIDLE_PRIORITY + 1UL )
#define APP_CFG_TASK_TX_PRIO            ( tskIDLE_PRIORITY + 1UL )
#define APP_CFG_TASK_RX_PRIO            ( tskIDLE_PRIORITY + 1UL )

/*������*/
#define AppTaskBeepStk                  (configMINIMAL_STACK_SIZE)
#define APP_CFG_TASK_BEEP_PRIO          ( tskIDLE_PRIORITY + 2UL )
void vBeepTask( void *p_arg );
extern TaskHandle_t BeepTaskHandle;

/*12864��ʾ����*/
#define DISPLAY_TASK_STK                (1024)
#define DISPLAY_TASK_PRIO               ( tskIDLE_PRIORITY + 2UL )
void vDisplayTask( void *p_arg );
extern TaskHandle_t DisplayTaskHandle;

/*ELMO���*/
#define CHECK_TASK_STK                  (configMINIMAL_STACK_SIZE)
#define CHECK_TASK_PRIO                 ( tskIDLE_PRIORITY + 2UL )
void vCheckTask(void *p_arg);
extern TaskHandle_t CheckTaskHandle;

/*λ�ü�������*/
#define POSCAL_TASK_STK                 (configMINIMAL_STACK_SIZE)
#define POSCAL_TASK_PRIO                ( tskIDLE_PRIORITY + 4UL )
void vPosCalTask( void *p_arg );
extern TaskHandle_t PosCalTaskHandle;

/*��������*/
#define CHASSIS_TASK_STK                (1024)
#define CHASSIS_TASK_PRIO               ( tskIDLE_PRIORITY + 2UL )
void vChassisTask( void *p_arg );
extern TaskHandle_t ChassisTaskHandle;

/*��������*/
#define AMC_MOTION_TASK_STK                (1024)
#define AMC_MOTION_TASK_PRIO               ( tskIDLE_PRIORITY + 3UL )
void vAmcMotionTask( void *p_arg );
extern TaskHandle_t AmcMotionTaskHandle;


/*��������*/
#define SLIDEWAY_TASK_STK               (1024)
#define SLIDEWAY_TASK_PRIO              ( tskIDLE_PRIORITY + 3UL )
void vSlidewayTask( void *p_arg );
extern TaskHandle_t SlidewayTaskHandle;

/*��������*/
#define HIT_TASK_STK                    (512)
#define HIT_TASK_PRIO                   ( tskIDLE_PRIORITY + 3UL )
void vHitTask( void *p_arg );
extern TaskHandle_t HitTaskHandle;

/*��������*/
#define SERVE_TASK_STK                  (512)
#define SERVE_TASK_PRIO                 ( tskIDLE_PRIORITY + 3UL )
void vServeTask( void *p_arg );
extern TaskHandle_t ServeTaskHandle;

/*��������*/
#define RESTART_TASK_STK                (512)
#define RESTART_TASK_PRIO               ( tskIDLE_PRIORITY + 4UL )
void vRestartTask( void *p_arg );
extern TaskHandle_t RestartTaskHandle;

/* xSemaphoreHandle ----------------------------------------------------------*/
extern xSemaphoreHandle PosSem;         //�յ�����+���������� (�ж��ͳ�)
extern xSemaphoreHandle PosChassisSem;  //�����ź���������
extern xSemaphoreHandle PosSlideSem;    //�����ź���������
extern QueueHandle_t HitInQueue;        //�����������������
extern QueueHandle_t UsartTxQueue;      //����DMA���Ͷ���

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
