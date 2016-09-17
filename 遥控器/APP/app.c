/**
  ******************************************************************************
  * @file    APP.c
  * @author  Rsing
  * @version V1.0
  * @date    22-July-2014
  * @brief   Initializate the APP
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "app.h"
#include "bsp.h"
#include "Rsing_Led.h"
#include "ry_usart_dma.h"
#include "ry_adc.h"
/* Private variables ---------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* TaskHandles ---------------------------------------------------------------*/
TaskHandle_t MainTaskHandle;

/* Semaphores ----------------------------------------------------------------*/
//xSemaphoreHandle ElmoSem = 0;

/* QueueHandles --------------------------------------------------------------*/
//QueueHandle_t HitInQueue = 0;

/* Private function ----------------------------------------------------------*/
/**
  * @brief  vStartTask
  * @param  None					
  * @retval None
  */
static void vStartTask( void *p_arg )
{
    (void)p_arg;
    
    //等待上电完全
    vTaskDelay( 300 * portTICK_MS );
    
    BSP_Init();
    
    //AppTaskCreate();----------------------------------------------------------
    vTaskSuspendAll();
    
    /* Initialise the com port then spawn the Rx and Tx tasks. */
	//vAltStartComTestTasks(APP_CFG_TASK_TX_PRIO, APP_CFG_TASK_RX_PRIO, 115200);
  
    /* Create one task. */
    xTaskCreate(    vMainTask,               /* Pointer to the function that implements the task.              */
                    "Main",                  /* Text name for the task.  This is to facilitate debugging only. */
                    AppTaskMainStk,          /* Stack depth in words.                                          */
                    NULL,                    /* We are not using the task parameter.                           */
                    APP_CFG_TASK_MAIN_PRIO,  /* This task will run at priority x.                              */
                    &MainTaskHandle );  /* We are not using the task handle.                              */
  
    xTaskResumeAll();
    //End-AppTaskCreate();------------------------------------------------------
    
    vTaskDelete(NULL);
}

//--------------------------------------------------------------
/**
  * @brief  vMainTask
  * @param  None					
  * @retval None
  */
extern u8 adc_begin_flag;
portTASK_FUNCTION( vMainTask, pvParameters )
{
    TickType_t xLastFlashTime;
    ( void ) pvParameters;
    usart1_dma_config();
	adc_dma_config();
	while(adc_begin_flag == 0)
	{
		vTaskDelay(1*portTICK_MS);
	}
    xLastFlashTime = xTaskGetTickCount();
    for(;;)
    {
		DmaSendPos();
        vTaskDelayUntil( &xLastFlashTime, 500 * portTICK_MS );
    }
}
//--------------------------------------------------------------


//--------------------------------------------------------------
/**
  * @brief  vDisTask
  * @param  None					
  * @retval None
  */
//static  portTASK_FUNCTION( vDisTask, pvParameters )
//{
//    uint32_t i = 0;
//    (void)pvParameters;
//    
//    while (1)
//    {
//        vTaskDelay(10 * portTICK_MS );
//    }
//}
//



//--------------------------------------------------------------
void StartTaskCreate(void)
{
    /* Create one task. */
    xTaskCreate(    vStartTask,               /* Pointer to the function that implements the task.              */
                    "Start",                  /* Text name for the task.  This is to facilitate debugging only. */
                    AppTaskStartStk,          /* Stack depth in words.                                          */
                    NULL,                    /* We are not using the task parameter.                            */
                    APP_CFG_TASK_START_PRIO,  /* This task will run at priority x.                              */
                    NULL );       /* We are not using the task handle.                              */
    
    /* Create one task. */
    xTaskCreate(    vBeepTask,               /* Pointer to the function that implements the task.              */
                    "Beep",                  /* Text name for the task.  This is to facilitate debugging only. */
                    AppTaskBeepStk,          /* Stack depth in words.                                          */
                    NULL,                    /* We are not using the task parameter.                           */
                    APP_CFG_TASK_BEEP_PRIO,  /* This task will run at priority x.                              */
                    &BeepTaskHandle );     
}

/******************************** END OF FILE *********************************/
