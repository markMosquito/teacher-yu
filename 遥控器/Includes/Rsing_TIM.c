/**
  ******************************************************************************
  * @file    Rsing_TIM.c
  * @author  Rsing
  * @version V1.0
  * @date    20-July-2014
  * @brief   Initializate the TIM
  ******************************************************************************
  */
  
 /* -----------------------------------------------------------------------
    Example：
	TIMx Configuration: generate 4 PWM signals with 4 different duty cycles:
    TIMx CLK = 72 MHz, Prescaler = 0x0, TIMx counter clock = 72 MHz
    TIMx ARR Register = 999 => TIMx Frequency = TIM3 counter clock/(ARR + 1)
    TIMx Frequency = 72 KHz.
    TIMx Channelx duty cycle = (TIMx_CCRx/ TIMx_ARR)* 100 = XX%
  ----------------------------------------------------------------------- */

/* Includes ------------------------------------------------------------------*/
#include "Rsing_TIM.h"

/* Private defines -----------------------------------------------------------*/
uint16_t PWM_Motor_Frequence = 100;
uint16_t PWM_Motor_Prescaler = 1;
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  TIM_GPIO_Config
  * @param  None					
  * @retval None
  */
static void TIM_GPIO_Config()
{
	GPIO_InitTypeDef GPIO_PinDefine;
	
	/* TIM1 ------------------------------------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	
	//PWM输出=TIM1_CH1~4, PA8~PA9~PA11
	GPIO_PinDefine.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_PinDefine.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_PinDefine.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_PinDefine);
}


/**
  * @brief  TIM_MODE_Config
  * @param  None					
  * @retval None
  */
static void TIM_MODE_Config()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_ClocksTypeDef RCC_Clocks; 
 	TIM_OCInitTypeDef  PWM_OCInitStructure;
	RCC_GetClocksFreq(&RCC_Clocks);
	
	/* Motor -----------------------------------------------------------------*/
	/* Motor time base configuration */
	TIM_DeInit(TIM1);
	TIM_TimeBaseStructure.TIM_Period = RCC_Clocks.SYSCLK_Frequency/PWM_Motor_Frequence/PWM_Motor_Prescaler-1;	//TIMx_ARR,当定时器从0计数到PWM_Frequence，即为PWM_Frequence+1次，为一个定时周期
	TIM_TimeBaseStructure.TIM_Prescaler = PWM_Motor_Prescaler-1;		//设置TIM时钟频率的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;			//设置时钟分频系数：不分频
 	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//选择计数器模式，向上计数模式
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; 
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	/* Motor mode configuration: Channel1, Channel2, Channel3, Channel4 */
	PWM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			//配置为PWM模式1
	PWM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
	PWM_OCInitStructure.TIM_Pulse = 0;							//设置跳变值，当计数器计数到这个值时，电平发生跳变
	PWM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	//当定时器计数值小于CCR1_Val时为低电平
	
	TIM_OC1Init(TIM1, &PWM_OCInitStructure);					//使能TIM1_Channel1
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM1, &PWM_OCInitStructure);					//使能TIM1_Channel2
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	

	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


/**
  * @brief  TIM Configuration
  * @param  None					
  * @retval None
  */
void TIM_Configuration()
{
	TIM_GPIO_Config();
	TIM_MODE_Config();
    TimFreChange(100);
    TIM_Cmd(TIM1, ENABLE);
}
/**
  * @brief  PWM TIMx Channelx duty cycle change.
  * @param  PWM_TIMx, PWM_Channel, PWM_Duty		
  * @retval None
  */
//void PWM_DutyChange(TIM_TypeDef* PWM_TIMx, u8 PWM_Channel, u16 PWM_Duty)
//{
//	u16 PWM_Pulse = 0;
//	
//	PWM_Pulse = PWM_TIMx->ARR;
//	if(PWM_Duty > 10000)
//		PWM_Pulse = PWM_Pulse;
//	else if(PWM_Duty < 1)
//		PWM_Pulse = 0;
//	else 
//		PWM_Pulse = PWM_Pulse*PWM_Duty/1000;
//	
//	switch(PWM_Channel)
//	{	
//		case 1:
//			TIM_SetCompare1(PWM_TIMx, (u16)PWM_Pulse);
//			break;
//		case 2:
//			TIM_SetCompare2(PWM_TIMx, (u16)PWM_Pulse);
//			break;
//		case 3:
//			TIM_SetCompare3(PWM_TIMx, (u16)PWM_Pulse);
//			break;
//		case 4:
//			TIM_SetCompare4(PWM_TIMx, (u16)PWM_Pulse);
//			break;
//	}
//}

void TimFreChange(uint16_t fre)
{
    if(fre == 0)
    {
        TIM_Cmd(TIM1, DISABLE);
    }
    else
    {
        RCC_ClocksTypeDef RCC_Clocks;
        uint16_t TIM_Prescaler = 0;
        uint16_t TIM_Period = 0;
        RCC_GetClocksFreq(&RCC_Clocks);
        
        TIM_Cmd(TIM1, DISABLE);
        TIM_Prescaler = RCC_Clocks.SYSCLK_Frequency/fre/65536 + 1;
        TIM_Period = RCC_Clocks.SYSCLK_Frequency/fre/TIM_Prescaler;
        
        if(TIM_Period>1)
            TIM1->ARR = TIM_Period - 1;
        else
            TIM1->ARR = 0;
        
        if(TIM_Prescaler>1)
            TIM1->PSC = TIM_Prescaler - 1;
        else
            TIM1->PSC = 0;
        
        if(TIM_Period/2>1)
            TIM1->CCR1 = TIM_Period/2 - 1;
        else
            TIM1->CCR1 = 0;
        
        if(TIM_Period/2>1)
            TIM1->CCR2 = TIM_Period/2 - 1;
        else
            TIM1->CCR2 = 0;

//            TIM1->ARR = TIM_Period - 1;

//            TIM1->PSC = TIM_Prescaler - 1;

//            TIM1->CCR1 = TIM_Period/2 - 1;

//            TIM1->CCR2 = TIM_Period/2 - 1;

        TIM_Cmd(TIM1, ENABLE);
    }
}

/*********************************END OF FILE**********************************/
