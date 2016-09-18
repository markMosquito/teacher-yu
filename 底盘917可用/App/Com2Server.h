/**
  ******************************************************************************
  * @file    Com2Server.h
  * @author  
  * @version V1.0
  * @date    7-May-2015
  * @brief   Header file for Com2Server.c module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COM2SERVER_H
#define __COM2SERVER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "Redef_GPIO.h"
#include "Rsing_led.h"
#include "Manage.h"

#include "ChassisTask.h"
#include "SlidewayTask.h"
#include "HitTask.h"

/* Private functions ---------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define DEF_RecievePosOn()      {CarReadyFlag = 1;LEDOn(LED1G);}
#define DEF_RecievePosOff()     {CarReadyFlag = 0;LEDOff(LED1G);}
#define DEF_SendPosOn()         {TIM7->CNT = 0; TIM_Cmd(TIM7, ENABLE);}//±ÿ—°œ»ENABLE UASRT1
#define DEF_SendPosOff()        TIM_Cmd(TIM7, DISABLE)

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    unsigned char DLC;
    unsigned char Data[32];
}UsartTxMsg;

typedef struct
{
    unsigned char DLC;
    unsigned char Data[32];
} UsartRxMsg;

typedef struct
{
	int32_t x;
	int32_t y;
	int32_t z;
	uint32_t t;
	uint32_t spd;
    float spd_real;
	uint32_t ang;
	int32_t instruct_mode;
}Receive_Typedef;

/* Exported variables --------------------------------------------------------*/

extern uint32_t DbgChoose;
extern int16_t hitting;
extern uint32_t CarReadyFlag;

/* Exported functions --------------------------------------------------------*/
uint8_t SumCheck(uint8_t *pData, uint8_t length);
void DmaSendWord(uint32_t ComX, uint8_t command, uint32_t data);
void USART1_Configuration(void);
void USART2_Configuration(void);
void TIM7_Configuration(uint8_t PPr, uint8_t SPr, uint16_t period_ms);
void DmaSendHitInfo(uint16_t dir, float spd, uint16_t ang, uint16_t tim);
void DmaSendWordQueue(uint8_t command, uint32_t data, uint32_t ifISR);
void DmaSendSpeed(int32_t speed);
void DMA_SendCommand(u8 addr,u8 control,u8 index,u8 offset,u8 dataWords, int32_t dataField);
int32_t DMA_GetPos(u8 addr);
#endif //Com2Server.h
