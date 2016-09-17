/**
  ******************************************************************************
  * @file    Rsing_Beep.c
  * @author  Rsing
  * @version V1.0
  * @date    14-November-2014
  * @brief   Initializate the Beep.
  * @note    FreeRTOS中断中仅可调用BuzzerFromISR(uint8_t time, uint8_t OnTime, uint8_t OffTime);
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Rsing_Beep.h"
#include "Redef_GPIO.h"
#include "FreeRTOS.h"
#include "task.h"

/* Private variables ---------------------------------------------------------*/
TaskHandle_t BeepTaskHandle;

/* Private define ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Beep_Configuration
  * @param  None					
  * @retval None
  */
void Beep_Configuration(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* GPIO clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2_PERIPH_GPIO(IO_BEEP), ENABLE);

    GPIO_InitStructure.GPIO_Pin =  IO_GPIO_PIN(IO_BEEP);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IO_GPIO_PORT(IO_BEEP), &GPIO_InitStructure);

    BEEP_OFF;
}


//蜂鸣器任务-----------------------------------------------------
static uint8_t BuzzerTime = 0;
static uint8_t BuzzerOnTime = 0;
static uint8_t BuzzerOffTime = 0;

/**
  * @brief  一般调用函数
  * @param  None					
  * @retval None
  */
void Buzzer(uint8_t time, uint8_t OnTime, uint8_t OffTime)
{
    BuzzerTime = time;
    BuzzerOnTime = OnTime;
    BuzzerOffTime = OffTime;
    vTaskResume(BeepTaskHandle);
}

/**
  * @brief  中断调用函数
  * @param  None					
  * @retval None
  */
void BuzzerFromISR(uint8_t time, uint8_t OnTime, uint8_t OffTime)
{
    BuzzerTime = time;
    BuzzerOnTime = OnTime;
    BuzzerOffTime = OffTime;
    xTaskResumeFromISR(BeepTaskHandle);
}

/**
  * @brief  vBeepTask
  * @param  None					
  * @retval None
  */
void vBeepTask( void *p_arg )
{
    uint32_t i = 0;
    (void)p_arg;

    vTaskSuspend(BeepTaskHandle);
    while (1)
    {
        for (i = 0; i < BuzzerTime; i++)
        {
            BEEP_ON;
            vTaskDelay(BuzzerOnTime * portTICK_MS );  
            BEEP_OFF;
            vTaskDelay(BuzzerOffTime * portTICK_MS );  
        }
        vTaskSuspend(BeepTaskHandle);
    }
}

/*********************************END OF FILE**********************************/
