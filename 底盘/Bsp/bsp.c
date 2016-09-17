/**
  ******************************************************************************
  * @file    BSP.c
  * @author  
  * @version V1.0
  * @date    22-July-2014
  * @brief   
  ******************************************************************************
  * @note
  *  FreeRTOS中调用操作系统函数的中断优先级不得超过 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5
  ******************************************************************************
  */
#include "bsp.h"
#include "app.h"
#include "Com2Server.h"
#include "Base_Fun.h"
#include "Rsing_Can.h"
#include "Rsing_12864.h"
#include "serial.h"
#include "amc.h"
#define Delay_ms(tims_ms)       vTaskDelay( ( TickType_t )tims_ms * portTICK_MS)

extern void Encoder_Init(void);
static void IO_Configuration(void);

void TIM2_init(void);
void ATConfigInit(eCOMPort Port);

/*
 * 函数名：BSP_Init
 * 描述  ：时钟初始化、硬件初始化
 * 输入  ：无
 * 输出  ：无
 */
void BSP_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
    //蜂鸣器、传感器、电磁阀
    //IO_Configuration();
    
    //滑轨编码器
    //Encoder_Init();
  
#if 0
    ATConfigInit(COM1);
    BEEP_ON;
    vTaskDelay(500*portTICK_MS);
    BEEP_OFF;
    while(1);
#endif
    //时间同步用
    //TIM2_init();
    
    //串口      接收中断打开，DMA发送中断打开       Enable:DEF_RecievePosOn()
    //串口接收中断优先级9,0                         //DMA发送中断优先级8，0
    USART2_Configuration();

//	RS485_Tx;
	vTaskDelay(5*portTICK_MS);
//	USART_SendData(USART2,0x37);
//	vTaskDelay(5*portTICK_MS);
//  RS485_Rx;
	
	
    //定时器DMA发送位置到电脑,DISABLE   Enable:DEF_SendPosOn()
    //TIM7_Configuration(9, 0, 30);      //中断优先级9，0
    
    //请求时间同步
    //DmaSendWordQueue(0xeb, 0, 0);
    
    //控制击球时间
    //TIM5_init(6, 0);        //中断优先级 6,0
    
    //Elmo
    //Elmo_Init(CAN1,0,0);    //CAN1采取中断发送，    发送中断优先级0,0   接收中断优先级 6,0
    
    //CAN2
    //CAN_Config(CAN2);       //CAN2直接发送，        接收中断优先级8,0
    
    //12864
   // LCD_Configuration();
}


void IO_Configuration(void)
{
    // 蜂鸣器
    MyGPIOInit(IO_GPIO_PORT(IO_BEEP), IO_GPIO_PIN(IO_BEEP), GPIO_Mode_OUT, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    
    // 挥拍传感器
    MyGPIOInit(IO_GPIO_PORT(IO_RACKET_RIGHT_RESET), IO_GPIO_PIN(IO_RACKET_RIGHT_RESET), GPIO_Mode_IN, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);

    // 滑轨复位传感器
    MyGPIOInit(IO_GPIO_PORT(IO_SLIDEWAY_X_RESET), IO_GPIO_PIN(IO_SLIDEWAY_X_RESET), GPIO_Mode_IN, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    MyGPIOInit(IO_GPIO_PORT(IO_SLIDEWAY_Y_RESET), IO_GPIO_PIN(IO_SLIDEWAY_Y_RESET), GPIO_Mode_IN, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    
    // 发球&放球按键
    MyGPIOInit(IO_GPIO_PORT(IO_SERVE_MODE), IO_GPIO_PIN(IO_SERVE_MODE), GPIO_Mode_IN, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_UP);
	MyGPIOInit(IO_GPIO_PORT(IO_KEY_TO_SERVE), IO_GPIO_PIN(IO_KEY_TO_SERVE), GPIO_Mode_IN, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_UP);
	MyGPIOInit(IO_GPIO_PORT(IO_SHELF_UP), IO_GPIO_PIN(IO_SHELF_UP), GPIO_Mode_IN, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_UP);
    MyGPIOInit(IO_GPIO_PORT(IO_SHELF_DOWN), IO_GPIO_PIN(IO_SHELF_DOWN), GPIO_Mode_IN, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_UP);
    MyGPIOInit(IO_GPIO_PORT(IO_HIT), IO_GPIO_PIN(IO_HIT), GPIO_Mode_IN, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_UP);
	
    //自检按键
    MyGPIOInit(IO_GPIO_PORT(IO_SELFTEST_KEY), IO_GPIO_PIN(IO_SELFTEST_KEY), GPIO_Mode_IN, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_UP);
    MyGPIOInit(IO_GPIO_PORT(IO_IMERGERCY_MODE), IO_GPIO_PIN(IO_IMERGERCY_MODE), GPIO_Mode_IN, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_UP);
	MyGPIOInit(IO_GPIO_PORT(IO_SELFTEST_SERVE), IO_GPIO_PIN(IO_SELFTEST_SERVE), GPIO_Mode_IN, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_UP);
    
    MyGPIOInit(IO_GPIO_PORT(IO_SKIP_LOOP), IO_GPIO_PIN(IO_SKIP_LOOP), GPIO_Mode_IN, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_UP);
    // 发球电磁阀
    MyGPIOInit(IO_GPIO_PORT(IO_SERVE_SHELF), IO_GPIO_PIN(IO_SERVE_SHELF), GPIO_Mode_OUT, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
	MyGPIOInit(IO_GPIO_PORT(IO_FANGQIU_RELAY), IO_GPIO_PIN(IO_FANGQIU_RELAY), GPIO_Mode_OUT, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    
    Serve_SHELF_DOWN();
    Serve_FANGQIU_OPEN();
}

void TIM2_init(void)            //Debug Time
{
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;

	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);

	/* Time base configuration (TIM2 clocked at 84 MHz)*/
	TIM_TimeBaseStructure.TIM_Period = 0xffffffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;           

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 

	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}

void ATConfigInit(eCOMPort Port)
{
//    simpleSerialPortInit(Port, 115200);
//	Delay_ms(2000);
//    simpleSerialPortPrintf(Port, "AT+DEFAULT\r\n");
//    Delay_ms(2000);
//    
//    simpleSerialPortPrintf(Port, "AT+RESET\r\n");
//    Delay_ms(2000);
//    while(1);
    simpleSerialPortInit(Port, 9600);
	Delay_ms(200);
    simpleSerialPortPrintf(Port, "AT+BAUD8\r\n");
    Delay_ms(200);
    simpleSerialPortClose(Port);
	Delay_ms(200);
    simpleSerialPortInit(Port, 115200);
	Delay_ms(200);
    simpleSerialPortPrintf(Port, "AT+NAMETRobot222\r\n");
    Delay_ms(200);
    simpleSerialPortPrintf(Port, "AT+PIN1111\r\n");
    Delay_ms(200);
    simpleSerialPortPrintf(Port, "AT+RESET\r\n");
    Delay_ms(2000);
    simpleSerialPortClose(Port);
}
/********************************* END OF FILE ********************************/
