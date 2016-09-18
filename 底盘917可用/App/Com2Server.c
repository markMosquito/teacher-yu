/**
  ******************************************************************************
  * @file    Com2Server.c
  * @author  
  * @version V1.0
  * @date    7-May-2015
  * @brief   发送底盘位置至电脑&接收电脑指令.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Com2Server.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app.h"
#include "mymath.h"
#define  GetRealTimeTick()      (TIM2->CNT)
#include <string.h>
#include "amc.h"

/* Private variables ---------------------------------------------------------*/
uint8_t DmaUsartTxBuf[14], DmaUsartTimBuf[14];
UsartRxMsg UsartRx1,UsartRx2, UsartRx3;
uint32_t DbgChoose = COM_DBG_ALL;
uint32_t SyncFlag = 0;
uint32_t SyncTimOut = 0;

u8 read_amc_over_flag = 0;

QueueHandle_t xCharsForTx3;
/* Extern variable prototypes-------------------------------------------------*/
extern uint32_t SendCorrectFlag, AngCorrectFlag;
extern uint32_t tFirst;
extern point_t ChassisResetPoint;

extern uint32_t CarReadyFlag;
extern uint32_t SelfTestFlag;
/* Private define ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void DmaSendPos(void);

void soundMsgRepeat(uint8_t id)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = 0x13;
    TxMessage.DLC = 1;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.Data[0] = id;
    CAN_Transmit(CAN2, &TxMessage);
}



extern uint32_t test_state;
extern uint32_t test_value;
/* USART1 RX -----------------------------------------------------------------*/
extern int16_t correctDis;
Receive_Typedef RecTemp;

u16 remote_data[4];
extern int32_t speed;
extern motion_target amc_motion_target;
extern float dir_angle_wheel[2];
void Remote_IRQHandler(uint8_t *pData)
{    
	u16 tmp[5];
	tmp[0] = pData[2] + (pData[3]<<8);
	tmp[1] = pData[4] + (pData[5]<<8);
	tmp[2] = pData[6] + (pData[7]<<8);
	tmp[3] = pData[8] + (pData[9]<<8);
	tmp[4] = pData[10] + (pData[11]<<8);
	if((tmp[0]+tmp[1]+tmp[2]+tmp[3]) == tmp[4])
	{
		memcpy(remote_data,tmp,8);
		if(remote_data[0] < 1900)
		{
			amc_motion_target.wheel_angle = (dir_angle_wheel[0]+dir_angle_wheel[1])/2 -20;
		}
		else if(remote_data[0] > 2150)
		{
			amc_motion_target.wheel_angle = (dir_angle_wheel[0]+dir_angle_wheel[1])/2 +20;
		}
		else
		{
			amc_motion_target.wheel_angle = (dir_angle_wheel[0]+dir_angle_wheel[1])/2;
		}
		if(amc_motion_target.wheel_angle> 110)
		{
			amc_motion_target.wheel_angle = 110;
		}
		else if(amc_motion_target.wheel_angle<-20)
		{
			amc_motion_target.wheel_angle = -20;
		}
		
		if(remote_data[1] < 1900)
		{
			speed = (remote_data[1] - 1900)*100;
		}
		else if(remote_data[1] > 2150)
		{
			speed = (remote_data[1] - 2150)*100;
		}
		else
		{
			speed = 0;
		}
		
	}
}
/**
  * @brief  USART1 接收中断，接收羽毛球落点
  * @param  None					
  * @retval None
  */
void USART1_IRQHandler(void)
{
    if((USART1->SR & USART_FLAG_RXNE) != (uint16_t)RESET)
	{
        static unsigned char come_data;
         
		come_data = (uint16_t)(USART1->DR & (uint16_t)0x01FF);
        if (come_data == 0xa5)  /* 如果遇到起始字节 */
        {
			UsartRx1.Data[0] = 0xa5;
            UsartRx1.DLC = 1;
        }
        else if(come_data == 0xff&&UsartRx1.DLC == 1)
		{
			UsartRx1.Data[1] = 0xff;
            UsartRx1.DLC = 2;
		}
		else
        {
            UsartRx1.Data[UsartRx1.DLC++] = come_data;
            if (UsartRx1.DLC == 12) //包含0xfx及校验位共UsartRx.DLC个字节
            {
				Remote_IRQHandler(UsartRx1.Data);
            }
            else if(UsartRx1.DLC > 31)
			{
				UsartRx1.DLC = 31;
			}
        }
    }
}
#if COM3_DBG == COM_RELEASE
void USART3_IRQHandler(void)
{
    if((USART3->SR & USART_FLAG_RXNE) != (uint16_t)RESET)
	{
        static unsigned char come_data;
        
		come_data = (uint16_t)(USART3->DR & (uint16_t)0x01FF);
        if (come_data == 0xff)  /* 如果遇到起始字节 */
        {
            UsartRx3.DLC = 0;
        }
        else
        {
            UsartRx3.Data[UsartRx3.DLC++] = come_data;
            if (UsartRx3.DLC == 13) //包含0xfx及校验位共UsartRx.DLC个字节
            {
                if (SumCheck(&UsartRx3.Data[1], UsartRx3.DLC-2) == UsartRx3.Data[UsartRx3.DLC-1])
                {
                    uint8_t *pData = UsartRx3.Data;
                    //COM_IRQHandler(pData);
                }
            }
            else if(UsartRx3.DLC > 31)
			{
				UsartRx3.DLC = 31;
			 }
        }
    }

    #if 0
    if((USART3->SR & USART_FLAG_TXE) != (uint16_t)RESET)
	{
        portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
        char cChar;
		/* The interrupt was caused by the THR becoming empty.  Are there any
		more characters to transmit? */
		if( xQueueReceiveFromISR( xCharsForTx3, &cChar, &xHigherPriorityTaskWoken ) == pdTRUE )
		{
            USART3->DR = (cChar & (uint16_t)0x01FF);
		}
		else
		{
			USART_ITConfig( USART3, USART_IT_TXE, DISABLE );		
		}
	}
    #endif
}
#endif

/* USART1 TX 使用TIM7 -----------------------------------------------------------------*/
/**
  * @brief  采用定时器调用串口DMA向电脑发送当前位置
  * @param  uint8_t PPr, uint8_t SPr, uint16_t period_ms
  * @retval None
  * @note   period_ms 应小于6553ms 且不宜太快
  */
void TIM7_Configuration(uint8_t PPr, uint8_t SPr, uint16_t period_ms)
{
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	NVIC_InitTypeDef          NVIC_InitStructure;

    assert_param(period_ms <= 6553);
    
	/* TIM7 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , ENABLE);

	/* Time base configuration (TIM7 clocked at 84 MHz)*/
	TIM_TimeBaseStructure.TIM_Period = period_ms*10;//
	TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;// 100us
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); 

	/* TIM7 IT enable */
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

    /* Enable the TIM7 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PPr;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SPr;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    
	/* TIM7 enable counter */
	TIM_Cmd(TIM7, DISABLE);
    TIM7->CNT = 0;
}

#include "Global.h"
void TIM7_IRQHandler()
{
    if(TIM_GetITStatus(TIM7, TIM_IT_Update))
    {
        TIM7->SR = (uint16_t)~TIM_IT_Update;
        if(SyncTimOut > 2)
        {
            SyncTimOut = 0;
            SyncFlag = 0;
            LEDOff(LED1B);
        }
        if(!SyncFlag)
            DmaSendPos();
        else
            SyncTimOut++;
    }
}

/* USART1&DMA 初始化----------------------------------------------------------*/
/** @note DMA中断优先级应高于USART，否则会出错 */
void USART1_Configuration(void)
{
	//初始化串口
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
        RCC_AHB1PeriphClockCmd(IO_RCC_AHB1_PERIPH(IO_USART1_TX), ENABLE);

        /* Configure USART Tx () */
        GPIO_InitStructure.GPIO_Pin = IO_GPIO_PIN(IO_USART1_TX);
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

        GPIO_Init( IO_GPIO_PORT(IO_USART1_TX), &GPIO_InitStructure );

        /* Configure USART Rx () */
        GPIO_InitStructure.GPIO_Pin = IO_GPIO_PIN(IO_USART1_RX);
        GPIO_Init( IO_GPIO_PORT(IO_USART1_RX), &GPIO_InitStructure );
        
        GPIO_PinAFConfig(IO_GPIO_PORT(IO_USART1_TX), IO_GPIO_PINSOURCE(IO_USART1_TX), GPIO_AF_USART1);
        GPIO_PinAFConfig(IO_GPIO_PORT(IO_USART1_RX), IO_GPIO_PINSOURCE(IO_USART1_RX), GPIO_AF_USART1);
        
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        USART_DeInit(USART1);
        USART_InitStructure.USART_BaudRate = 115200;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        USART_Init(USART1, &USART_InitStructure);
        
        USART_Cmd(USART1, ENABLE);
    }

    //配置NVIC
    {
        NVIC_InitTypeDef  NVIC_InitStructure;
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);        //开串口中断
        
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;          
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9; 
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           
        NVIC_Init(&NVIC_InitStructure);
    }
    
    //DMA发送
    {
		NVIC_InitTypeDef NVIC_InitStructure;
        DMA_InitTypeDef DMA_InitStructure;
        
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
        USART_DMACmd(USART1,USART_DMAReq_Tx ,ENABLE);
        
		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	
        
        DMA_DeInit(DMA2_Stream7);
        DMA_InitStructure.DMA_Channel= DMA_Channel_4;
        DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(USART1->DR));
        DMA_InitStructure.DMA_Memory0BaseAddr = (u32)(&(DmaUsartTxBuf));
        DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        DMA_InitStructure.DMA_BufferSize = 0;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA2_Stream7, &DMA_InitStructure);
        DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TC);
        DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
    }
}
//485
void USART2_Configuration(void)
{
	//
	//初始化串口
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
        RCC_AHB1PeriphClockCmd(IO_RCC_AHB1_PERIPH(IO_USART2_TX), ENABLE);

        /* Configure USART Tx () */
        GPIO_InitStructure.GPIO_Pin = IO_GPIO_PIN(IO_USART2_TX);
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

        GPIO_Init( IO_GPIO_PORT(IO_USART2_TX), &GPIO_InitStructure );

        /* Configure USART Rx () */
        GPIO_InitStructure.GPIO_Pin = IO_GPIO_PIN(IO_USART2_RX);
        GPIO_Init( IO_GPIO_PORT(IO_USART2_RX), &GPIO_InitStructure );
        
        GPIO_PinAFConfig(IO_GPIO_PORT(IO_USART2_TX), IO_GPIO_PINSOURCE(IO_USART2_TX), GPIO_AF_USART2);
        GPIO_PinAFConfig(IO_GPIO_PORT(IO_USART2_RX), IO_GPIO_PINSOURCE(IO_USART2_RX), GPIO_AF_USART2);
        //RE口，控制T   R
		GPIO_InitStructure.GPIO_Pin = IO_GPIO_PIN(IO_USART2_485_RE);;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init( IO_GPIO_PORT(IO_USART2_485_RE), &GPIO_InitStructure );
		
		RS485_Rx;
		
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        USART_DeInit(USART2);
        USART_InitStructure.USART_BaudRate = 115200;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        USART_Init(USART2, &USART_InitStructure);
        
        USART_Cmd(USART2, ENABLE);
    }

    //配置NVIC
    {
        NVIC_InitTypeDef  NVIC_InitStructure;
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);        //开串口中断
        
        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;          
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9; 
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           
        NVIC_Init(&NVIC_InitStructure);
    }
    
    //DMA发送
    {
		NVIC_InitTypeDef NVIC_InitStructure;
        DMA_InitTypeDef DMA_InitStructure;
        
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
        USART_DMACmd(USART2,USART_DMAReq_Tx ,ENABLE);
        
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	
        
        DMA_DeInit(DMA1_Stream6);
        DMA_InitStructure.DMA_Channel= DMA_Channel_4;
        DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(USART2->DR));
        DMA_InitStructure.DMA_Memory0BaseAddr = (u32)(&(DmaUsartTxBuf));
        DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        DMA_InitStructure.DMA_BufferSize = 0;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream6, &DMA_InitStructure);
        DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TC);
        DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
    }
}
/**
  * @brief  SumCheck 校验和
  * @param  *pData：数据起始地址指针
            length：数据长度
  * @retval (uint8_t)校验和
  */
uint8_t SumCheck(uint8_t *pData, uint8_t length)
{
	register uint32_t sum = 0, i;
	for (i = 0; i < length; i++)
	{
		sum += *pData++;
	}
	return (uint8_t)(sum & 0x7f);
}


/**
  * @brief  DMA串口中断
  * @param  int32_t x, int32_t y
  * @retval None
  */
void DMA2_Stream7_IRQHandler(void)
{
    static UsartTxMsg txMsg;
	DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
	IO_GPIO_ResetBits(IO_USART2_485_RE);
    if( xQueueReceiveFromISR( UsartTxQueue, &txMsg, 0 ) )
    {
        DMA2_Stream7->NDTR = txMsg.DLC;
        DMA2_Stream7->M0AR = (uint32_t)(&txMsg.Data[0]);
        DMA2_Stream7->CR |= (uint32_t)DMA_SxCR_EN;//DMA_Cmd(DMA2_Stream7, ENABLE);
    }
}

//usart2

void DMA1_Stream6_IRQHandler(void)
{
    static UsartTxMsg txMsg;
	//int i = 18000;
	DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
	
	//while(i--);
	RS485_Rx;//拉低  接收 
    if( xQueueReceiveFromISR( UsartTxQueue, &txMsg, 0 ) )
    {
        DMA1_Stream6->NDTR = txMsg.DLC;
        DMA1_Stream6->M0AR = (uint32_t)(&txMsg.Data[0]);
        DMA1_Stream6->CR |= (uint32_t)DMA_SxCR_EN;//DMA_Cmd(DMA1_Stream6, ENABLE);
    }
}

int32_t motor_spd[4];
int32_t motor_pos[4];
extern u8 init_success;
extern int motor1_value_begining;
extern int motor2_value_begining;
float dir_angle_wheel[2];
int count = 0;
u8 amc_reply_ok = 0;
int timeout_err = 0;
int timeout_noerr = 0;
#include "amc.h"
void AMC_COM_IRQHandler(uint8_t *pData)
{    
	u16 crcData = 0;
	u8 node = 0,is_pos = 0;
    node = ((pData[2]>>2)&0x03)+1;
	is_pos = pData[2]>>4&0x01;
    switch(pData[5])
	{
		case 0:
			crcData = crcCal(pData,6);
			if(pData[6] == (u8)(crcData>>8) && pData[7] == (u8)(crcData))
			{
				amc_reply_ok = 1;
				timeout_noerr++;
				RS485_Tx;
			}
		break;
		case 1:break;
		case 2:
			if(is_pos)
			{
				count ++;
				motor_pos[node-1] = pData[8] + (pData[9]<<8)+(pData[10]<<16)+(pData[11]<<24);
				read_amc_over_flag = 1;
				if(node == 3 && init_success == 1)
				{
					dir_angle_wheel[0] = -(motor_pos[2] - motor1_value_begining)*90.0/4000.0 + 0.5;
				}
				else if(node == 4 && init_success == 1)
				{
					dir_angle_wheel[1] = -(motor_pos[3] - motor2_value_begining)*90.0/4000.0 - 2.8;
				}
			}
			else
			{
				motor_spd[node-1] = pData[8] + (pData[9]<<8)+(pData[10]<<16)+(pData[11]<<24);
				read_amc_over_flag = 1;
			}
		break;
				
	}
	
}

char start_bytes[2];
u8 start_length = 0;
void USART2_IRQHandler(void)
{
    if((USART2->SR & USART_FLAG_RXNE) != (uint16_t)RESET)
	{
        static unsigned char come_data;
         
		come_data = (uint16_t)(USART2->DR & (uint16_t)0x01FF);
        if (come_data == 0xa5)  /* 如果遇到起始字节 */
        {
			UsartRx2.Data[0] = 0xa5;
            UsartRx2.DLC = 1;
        }
        else if(come_data == 0xff&&UsartRx2.DLC == 1)
		{
			UsartRx2.Data[1] = 0xff;
            UsartRx2.DLC = 2;
		}
		else
        {
            UsartRx2.Data[UsartRx2.DLC++] = come_data;
            if (UsartRx2.DLC >= 8 && UsartRx2.DLC <= 16) //包含0xfx及校验位共UsartRx.DLC个字节
            {
				switch(UsartRx2.Data[5])
				{
					case 0:
						if(UsartRx2.DLC == 8)
						{
							UsartRx2.DLC = 0;
							AMC_COM_IRQHandler(UsartRx2.Data);
						}
					break;
					case 1:
						if(UsartRx2.DLC == 12)
						{
							UsartRx2.DLC = 0;
						}
					break;
					case 2:
						if(UsartRx2.DLC == 14)
						{
							UsartRx2.DLC = 0;
							AMC_COM_IRQHandler(UsartRx2.Data);
						}
					break;
					default:break;
				}
            }
            else if(UsartRx2.DLC > 31)
			{
				UsartRx2.DLC = 31;
			}
        }
    }
}
#define CHANGE_SIGN(x)  ((x)<0?(-(x) + 0x2000):(x))
/**
  * @brief  DMA发送位置
  * @param  int32_t x, int32_t y, int32_t vx, int32_t vy
  * @retval None
  * @note   发送间隔
  */
static void DmaSendPos(void)
{
    const uint32_t num = 0;
    int32_t x, y, vx, vy;
    
    x = G_Param.cur_pos.x;
    y = G_Param.cur_pos.y;
    vx = G_Param.speed.x;
    vy = G_Param.speed.y;
    
    if(x < -8191) x = -8191;
    else if(x > 8191) x = 8191;
    if(y < -8191) y = -8191;
    else if(y > 8191) y = 8191;
    if(vx < -8191) vx = -8191;
    else if(vx > 8191) vx = 8191;
    if(vy < -8191) vy = -8191;
    else if(vy > 8191) vy = 8191;
    
    x = CHANGE_SIGN(x);
    y = CHANGE_SIGN(y);
    vx = CHANGE_SIGN(vx);
    vy = CHANGE_SIGN(vy);
    
    DmaUsartTxBuf[0] = 0xff;
	//DmaUsartTxBuf[1] = 0xe1 + SendCorrectFlag;
    DmaUsartTxBuf[1] = 0xe2;
	DmaUsartTxBuf[2] = x & 0x7f;
	DmaUsartTxBuf[3] = (x >> 7) & 0x7f;
	DmaUsartTxBuf[4] = y & 0x7f;
	DmaUsartTxBuf[5] = (y >> 7) & 0x7f;
	DmaUsartTxBuf[6] = vx & 0x7f;
	DmaUsartTxBuf[7] = (vx >> 7) & 0x7f;
	DmaUsartTxBuf[8] = vy & 0x7f;
	DmaUsartTxBuf[9] = (vy >> 7) & 0x7f;
    DmaUsartTxBuf[10] = num & 0x7f;
	DmaUsartTxBuf[11] = (num >> 7) & 0x7f;
    DmaUsartTxBuf[12] = (num >> 14) & 0x7f;
	DmaUsartTxBuf[13] = SumCheck(&DmaUsartTxBuf[2], 11);	//x、y、z、t的发送数据相加
    
    if(DMA1_Stream6->NDTR == 0)
    {
        DMA1_Stream6->NDTR = 14;
        DMA1_Stream6->M0AR = (uint32_t)(DmaUsartTxBuf);
        DMA1_Stream6->CR |= (uint32_t)DMA_SxCR_EN;//DMA_Cmd(DMA2_Stream7, ENABLE);
    }
}

/* 终止当前DMA发送，立即将该数发出 */
void DmaSendWord(uint32_t ComX, uint8_t command, uint32_t data)
{
    const uint32_t num = 0;
    
	DmaUsartTimBuf[0] = 0xff;
	DmaUsartTimBuf[1] = command;
	DmaUsartTimBuf[2] = (data >> 0) & 0x7f;
	DmaUsartTimBuf[3] = (data >> 7) & 0x7f;
	DmaUsartTimBuf[4] = (data >> 14) & 0x7f;
	DmaUsartTimBuf[5] = (data >> 21) & 0x7f;
	DmaUsartTimBuf[6] = (data >> 28) & 0x0f;
	DmaUsartTimBuf[7] = 0;
	DmaUsartTimBuf[8] = 0;
	DmaUsartTimBuf[9] = 0;
	DmaUsartTimBuf[10] = num & 0x7f;
	DmaUsartTimBuf[11] = (num >> 7) & 0x7f;
    DmaUsartTimBuf[12] = (num >> 14) & 0x7f;
	DmaUsartTimBuf[13] = SumCheck(&DmaUsartTimBuf[2], 11);	//x、y、z、t的发送数据相加

    DMA2_Stream7->CR &= ~(uint32_t)DMA_SxCR_EN;
    DMA2_Stream7->NDTR = 14;
    DMA2_Stream7->M0AR = (uint32_t)(DmaUsartTimBuf);
    DMA2_Stream7->CR  |= (uint32_t)DMA_SxCR_EN;//DMA_Cmd(DMA2_Stream7, ENABLE);
}

void DmaSendHitInfo(uint16_t dir, float spd, uint16_t ang, uint16_t tim)
{
    const uint32_t u_spd = (uint32_t)(spd * 10);
    
	DmaUsartTimBuf[0] = 0xff;
	DmaUsartTimBuf[1] = 0xe7;
	DmaUsartTimBuf[2] = (dir) & 0x7f;
	DmaUsartTimBuf[3] = (u_spd >> 0) & 0x7f;
	DmaUsartTimBuf[4] = (u_spd >> 7) & 0x7f;
	DmaUsartTimBuf[5] = (ang >> 0) & 0x7f;
	DmaUsartTimBuf[6] = (ang >> 7) & 0x7f;
	DmaUsartTimBuf[7] = (tim >> 0) & 0x7f;
	DmaUsartTimBuf[8] = (tim >> 7) & 0x7f;
	DmaUsartTimBuf[9] = 0;
	DmaUsartTimBuf[10] = 0;
	DmaUsartTimBuf[11] = 0;
    DmaUsartTimBuf[12] = 0;
	DmaUsartTimBuf[13] = SumCheck(&DmaUsartTimBuf[2], 11);	//x、y、z、t的发送数据相加
    
	if(DMA2_Stream7->NDTR == 0)
    {
        DMA2_Stream7->CR &= ~(uint32_t)DMA_SxCR_EN;
        DMA2_Stream7->NDTR = 14;
        DMA2_Stream7->M0AR = (uint32_t)(DmaUsartTimBuf);
        DMA2_Stream7->CR  |= (uint32_t)DMA_SxCR_EN;//DMA_Cmd(DMA2_Stream7, ENABLE);
    }
}

void DmaSendWordQueue(uint8_t command, uint32_t data, uint32_t ifISR)
{
    static UsartTxMsg txMsg;
    
    txMsg.DLC = 14;
    txMsg.Data[0] = 0xff;
	txMsg.Data[1] = command;
	txMsg.Data[2] = (data >> 0) & 0x7f;
	txMsg.Data[3] = (data >> 7) & 0x7f;
	txMsg.Data[4] = (data >> 14) & 0x7f;
	txMsg.Data[5] = (data >> 21) & 0x7f;
	txMsg.Data[6] = (data >> 28) & 0x0f;
	txMsg.Data[7] = 0;
	txMsg.Data[8] = 0;
	txMsg.Data[9] = 0;
	txMsg.Data[10] = 0;
	txMsg.Data[11] = 0;
    txMsg.Data[12] = 0;
	txMsg.Data[13] = SumCheck(&txMsg.Data[2], 11);

    if(DMA2_Stream7->NDTR == 0)
    {
        DMA2_Stream7->CR &= ~(uint32_t)DMA_SxCR_EN;
        DMA2_Stream7->NDTR = 14;
        DMA2_Stream7->M0AR = (uint32_t)(&txMsg.Data[0]);
        DMA2_Stream7->CR  |= (uint32_t)DMA_SxCR_EN;//DMA_Cmd(DMA2_Stream7, ENABLE);
    }
    else
    {
        if(ifISR)
            xQueueOverwriteFromISR( UsartTxQueue, &txMsg, 0);
        else
            xQueueOverwrite( UsartTxQueue, &txMsg);
    }
}



union int32ToChar
{
	int32_t data;
	char ch[4];
};
union u16ToChar
{
	u16 data;
	char ch[2];
};
#define ADDR 0x3f//默认地址63
void DmaSendSpeed(int32_t speed)
{
    static UsartTxMsg txMsg;
	static UsartTxMsg data_part;
//	int32_t speed_mesured = 0;
    u16 crcdata = 0;
	union int32ToChar speedTrans;
	union u16ToChar crcDataTrans;
	speedTrans.data = speed * 873.813;
    txMsg.DLC = 14;
    txMsg.Data[0] = 0xa5;//起始位
	txMsg.Data[1] = ADDR;//地址
	txMsg.Data[2] = 0x02;//control位，2表示后面的data有数据，0表示没有
	txMsg.Data[3] = 0x45;//index位，大概就是命令的标志吧，parameter structure的位置，参考command dictionary
	txMsg.Data[4] = 0x00;//offset byte,偏移位，大概是一个命令不止一个parameter，这个偏移指定是哪个parameter
	txMsg.Data[5] = 0x02;//length，表示后面的数据长度，word为单位。1 word = 2 bytes
	crcDataTrans.data = crcCal(txMsg.Data,6);
	txMsg.Data[6] = crcDataTrans.ch[1];
	txMsg.Data[7] = crcDataTrans.ch[0];
	txMsg.Data[8] = speedTrans.ch[0];
	txMsg.Data[9] = speedTrans.ch[1];
	txMsg.Data[10] = speedTrans.ch[2];
    txMsg.Data[11] = speedTrans.ch[3];
	data_part.Data[0] = speedTrans.ch[0];
	data_part.Data[1] = speedTrans.ch[1];	
	data_part.Data[2] = speedTrans.ch[2];
	data_part.Data[3] = speedTrans.ch[3];
	data_part.DLC = 4;
	crcdata = crcCal(data_part.Data,4);
	
	txMsg.Data[12] = (u8)(crcdata>>8);
	txMsg.Data[13] = (u8)(crcdata);
	
	//发送模式
	IO_GPIO_SetBits(IO_USART2_485_RE);
 
//	if(DMA2_Stream7->NDTR == 0)
//    {
//        DMA2_Stream7->CR &= ~(uint32_t)DMA_SxCR_EN;
//        DMA2_Stream7->NDTR = 14;
//        DMA2_Stream7->M0AR = (uint32_t)(&txMsg.Data[0]);
//        DMA2_Stream7->CR  |= (uint32_t)DMA_SxCR_EN;//DMA_Cmd(DMA2_Stream7, ENABLE);
//    }
    if(DMA1_Stream6->NDTR == 0)
    {
        DMA1_Stream6->CR &= ~(uint32_t)DMA_SxCR_EN;
        DMA1_Stream6->NDTR = 14;
        DMA1_Stream6->M0AR = (uint32_t)(&txMsg.Data[0]);
        DMA1_Stream6->CR  |= (uint32_t)DMA_SxCR_EN;//DMA_Cmd(DMA2_Stream7, ENABLE);
    }
}

void DMA_SendCommand(u8 addr,u8 control,u8 index,u8 offset,u8 dataWords, int32_t dataField)
{
	static UsartTxMsg txMsg;
	static UsartTxMsg data_part;
//	int32_t speed_mesured = 0;
//    u16 crcdata = 0;
	union int32ToChar FormatTrans;
	union u16ToChar crcDataTrans;
	u8 time_count = 0;
	int j = 0;
	//speedTrans.data = speed * 873.813;
   // txMsg.DLC = 14;
    txMsg.Data[0] = 0xa5;//起始位
	txMsg.Data[1] = addr;//地址
	txMsg.Data[2] = control;//control位，2表示后面的data有数据，0表示没有
	txMsg.Data[3] = index;//index位，大概就是命令的标志吧，parameter structure的位置，参考command dictionary
	txMsg.Data[4] = offset;//offset byte,偏移位，大概是一个命令不止一个parameter，这个偏移指定是哪个parameter
	txMsg.Data[5] = dataWords;//length，表示后面的数据长度，word为单位。1 word = 2 bytes
	crcDataTrans.data = crcCal(txMsg.Data,6);
	txMsg.Data[6] = crcDataTrans.ch[1];
	txMsg.Data[7] = crcDataTrans.ch[0];
	
	switch(dataWords)
	{
		case 1:
			FormatTrans.data = dataField;
			txMsg.Data[8] = FormatTrans.ch[0];
			txMsg.Data[9] = FormatTrans.ch[1];
			
			data_part.Data[0] = FormatTrans.ch[0];
			data_part.Data[1] = FormatTrans.ch[1];	
			data_part.DLC = 2;
			crcDataTrans.data = crcCal(data_part.Data,data_part.DLC);
			txMsg.Data[10] = crcDataTrans.ch[1];
			txMsg.Data[11] = crcDataTrans.ch[0];
			txMsg.Data[12] = 0x00;
			txMsg.Data[13] = 0x00;
			
			txMsg.DLC = 14;
		break;
		case 2:
			FormatTrans.data = dataField;
			txMsg.Data[8] = FormatTrans.ch[0];
			txMsg.Data[9] = FormatTrans.ch[1];
			txMsg.Data[10] = FormatTrans.ch[2];
			txMsg.Data[11] = FormatTrans.ch[3];
	
			data_part.Data[0] = FormatTrans.ch[0];
			data_part.Data[1] = FormatTrans.ch[1];	
			data_part.Data[2] = FormatTrans.ch[2];
			data_part.Data[3] = FormatTrans.ch[3];
			data_part.DLC = 4;
			crcDataTrans.data = crcCal(data_part.Data,data_part.DLC);
			txMsg.Data[12] = crcDataTrans.ch[1];
			txMsg.Data[13] = crcDataTrans.ch[0];
			txMsg.Data[14] = 0x00;
			txMsg.Data[15] = 0x00;
			
			txMsg.DLC = 16;
		break;
		default:break;
	}
	//发送模式
	
//	for(j=0;j<txMsg.DLC;j++)
//      {
//        USART_SendData(USART2,txMsg.Data[j]);                    
//        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);          
//      }  
#if 0
	if(DMA2_Stream7->NDTR == 0)
    {
		RS485_Tx;
		vTaskDelay(2*portTICK_MS);
        DMA2_Stream7->CR &= ~(uint32_t)DMA_SxCR_EN;
        DMA2_Stream7->NDTR = txMsg.DLC;
        DMA2_Stream7->M0AR = (uint32_t)(&txMsg.Data[0]);
        DMA2_Stream7->CR  |= (uint32_t)DMA_SxCR_EN;//DMA_Cmd(DMA2_Stream7, ENABLE);
    }
#else
    if(DMA1_Stream6->NDTR == 0)//USART2
    {
		RS485_Tx;
		amc_reply_ok = 0;
		vTaskDelay(2*portTICK_MS);
        DMA1_Stream6->CR &= ~(uint32_t)DMA_SxCR_EN;
        DMA1_Stream6->NDTR = txMsg.DLC;
        DMA1_Stream6->M0AR = (uint32_t)(&txMsg.Data[0]);
        DMA1_Stream6->CR  |= (uint32_t)DMA_SxCR_EN;//DMA_Cmd(DMA2_Stream7, ENABLE);
    }
#endif
	while(amc_reply_ok == 0)
	{
		vTaskDelay(1*portTICK_MS);
		time_count ++;
		if(time_count>20)
		{
			timeout_err ++;
			RS485_Tx;
			break;
		}
	}
}


int32_t DMA_GetPos(u8 addr)
{
	static UsartTxMsg txMsg;
	static UsartTxMsg data_part;

	union int32ToChar FormatTrans;
	union u16ToChar crcDataTrans;
	
    txMsg.Data[0] = 0xa5;//起始位
	txMsg.Data[1] = addr;//地址
	txMsg.Data[2] = 0x00+ ((addr-1)<<2) + (1<<4);//control位，2表示后面的data有数据，0表示没有+辨识位,我们辨识位用addr右移过去+速度位置标志 右移过去
	txMsg.Data[3] = 0x12;//index位，大概就是命令的标志吧，parameter structure的位置，参考command dictionary
	txMsg.Data[4] = 0x00;//offset byte,偏移位，大概是一个命令不止一个parameter，这个偏移指定是哪个parameter
	txMsg.Data[5] = 0x02;//length，表示后面的数据长度，word为单位。1 word = 2 bytes,若为接收，则表示为即将接收到的数据长度
	crcDataTrans.data = crcCal(txMsg.Data,6);
	txMsg.Data[6] = crcDataTrans.ch[1];
	txMsg.Data[7] = crcDataTrans.ch[0];
	
	//发送模式
	RS485_Tx;
	//read_amc_over_flag = 0;
	txMsg.DLC = 8;

    if(DMA1_Stream6->NDTR == 0)//USART2
    {
        DMA1_Stream6->CR &= ~(uint32_t)DMA_SxCR_EN;
        DMA1_Stream6->NDTR = txMsg.DLC+2;
        DMA1_Stream6->M0AR = (uint32_t)(&txMsg.Data[0]);
        DMA1_Stream6->CR  |= (uint32_t)DMA_SxCR_EN;//DMA_Cmd(DMA2_Stream7, ENABLE);
    }

//	while(read_amc_over_flag == 0)
//	{
//	};
	return motor_pos[addr];
}

















/*********************************END OF FILE**********************************/
