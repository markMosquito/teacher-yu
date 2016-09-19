/**
  ******************************************************************************
  * @file    BSP.c
  * @author  Rsing
  * @version V1.0
  * @date    22-July-2014
  * @brief   
  ******************************************************************************
  */
#include "BSP.h"
#include "Rsing_beep.h"
void TIM2_Init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 

  //��ʱ����T=(Period+1)*(Prescaler+1)/TCLK=200us------------100*4us=400us
  TIM_TimeBaseStructure.TIM_Period = 65535; 
  TIM_TimeBaseStructure.TIM_Prescaler = 1 - 1; //36-1
  TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  TIM_Cmd(TIM2, ENABLE); 
}


/*
 * ��������BSP_Init
 * ����  ��ʱ�ӳ�ʼ����Ӳ����ʼ��
 * ����  ����
 * ���  ����
 */
void BSP_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //Led -----------------------------------------
    LEDInit(LED0);

	Beep_Configuration();
    vTaskDelay(50 * portTICK_MS );
    //LCD -----------------------------------------
    //LCD_Config();             //��ʼ��12864
}

/********************************* END OF FILE ********************************/
