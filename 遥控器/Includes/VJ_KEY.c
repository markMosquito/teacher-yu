/*********************************************************/
/**FILENAME :    VJ_KEY.c                               **/
/**PROGRAMMER:   VJ                                     **/
/**DATE:         2014.03.20                             **/
/*********************************************************/

#include "VJ_KEY.h"


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
//
//*NAME:        KEY_Configuration(void)
//*DESCRIBE :   KEY_Configuration(void)
//*INPTU :      NONE
//*OUTPUT:      NONE
//*CALL:        EXTERNAL CALL
//

//void KEY_Configuration(void)
//{
//	GPIO_InitTypeDef  GPIO_InitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_3;                                   //选择要控制的GPIO引脚
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                               //设置引脚为普通输出模式
//	GPIO_Init (GPIOE,&GPIO_InitStructure);                                      //调用库函数初始化GPIO
//}

//
//*NAME:        Key_Scan(GPIO_TypeDef* GPIOx,u16 GPIO_Pin)
//*DESCRIBE :   KEY_Scan
//*INPTU :      GPIOx,GPIO_Pin_x
//*OUTPUT:      KEY_ON,KEY_OFF
//*CALL:        EXTERNAL CALL
//

//u8 Key_Scan(GPIO_TypeDef* GPIOx,u16 GPIO_Pin)
//{
//	if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == KEY_ON )               //判断有无按键按下
//	{
//		vTaskDelay(10);                                                  //延时消抖
//		if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == KEY_ON )             //判断有无按键按下
//		{
//			while(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == KEY_ON )        //当按键松开时
//				;
//			return KEY_ON;                                                 //返回KEY_ON                                           
//		}	
//		else 
//			return KEY_OFF;                                                //返回KEY_OFF 
//	}
//	else 
//		return KEY_OFF;                                                  //返回KEY_OFF 
//}



/*
OUT:Y_KEY1: PA7,    Y_KEY2: PB0,   Y_KEY3: PB1,   Y_KEY4: PB10, default:高电平
IN: X_KEY1: PA6,    X_KEY2: PB5,   X_KEY3: PA4,   X_KEY4: PA1,
*/
void matrixKeyConfig(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
    	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	
    //Y：
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_1 | GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init (GPIOB,&GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init (GPIOA,&GPIO_InitStructure);
    
    //X：
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_4 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init (GPIOA,&GPIO_InitStructure);
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init (GPIOA,&GPIO_InitStructure);
}

#define readY(port, x)   (uint16_t)( (port) & (0x01<<(x)))

static void scanDelay(void)
{
    uint8_t i=0;
    while(i != 255)
    {
        i++;
    }
}

/*
1   2   3   A
4   5   6   B
7   8   9   C
0   .   S   D
*/
uint8_t ScanKey(char *key)
{
    char tmp=0;
    uint8_t y = 0;

    //Y_KEY4---------------------------------------
    GPIOB->BRR |= 0x400;
    scanDelay();
    
    if(readY(GPIOA->IDR, 6) == 0)
    {
        while(readY(GPIOA->IDR, 6) == 0)
        vTaskDelay(2 * portTICK_MS );
        tmp = 'A';
    }
    else if(readY(GPIOA->IDR, 5) == 0)    {while(readY(GPIOA->IDR, 5) == 0)vTaskDelay(2 * portTICK_MS ); tmp = 'B';}
    else if(readY(GPIOA->IDR, 4) == 0)    {while(readY(GPIOA->IDR, 4) == 0)vTaskDelay(2 * portTICK_MS ); tmp = 'C';}
    else if(readY(GPIOA->IDR, 1) == 0)    {while(readY(GPIOA->IDR, 1) == 0)vTaskDelay(2 * portTICK_MS ); tmp = 'D';}
    else
    {
        GPIOB->BSRR  |= 0x400;
        //Y_KEY3----------------------------------------
        GPIOB->BRR |= 0x02;
        scanDelay();
        
        if(readY(GPIOA->IDR, 6) == 0)         {while(readY(GPIOA->IDR, 6) == 0)vTaskDelay(2 * portTICK_MS ); tmp = '3';}
        else if(readY(GPIOA->IDR, 5) == 0)    {while(readY(GPIOA->IDR, 5) == 0)vTaskDelay(2 * portTICK_MS ); tmp = '6';}
        else if(readY(GPIOA->IDR, 4) == 0)    {while(readY(GPIOA->IDR, 4) == 0)vTaskDelay(2 * portTICK_MS ); tmp = '9';}
        else if(readY(GPIOA->IDR, 1) == 0)    {while(readY(GPIOA->IDR, 1) == 0)vTaskDelay(2 * portTICK_MS ); tmp = '#';}
        else
        {
            GPIOB->BSRR |= 0x02;
            //Y_KEY2-----------------------------------------
            GPIOB->BRR |= 0x01;
            scanDelay();
            
            if(readY(GPIOA->IDR, 6) == 0)         {while(readY(GPIOA->IDR, 6) == 0)vTaskDelay(2 * portTICK_MS ); tmp = '2';}
            else if(readY(GPIOA->IDR, 5) == 0)    {while(readY(GPIOA->IDR, 5) == 0)vTaskDelay(2 * portTICK_MS ); tmp = '5';}
            else if(readY(GPIOA->IDR, 4) == 0)    {while(readY(GPIOA->IDR, 4) == 0)vTaskDelay(2 * portTICK_MS ); tmp = '8';}
            else if(readY(GPIOA->IDR, 1) == 0)    {while(readY(GPIOA->IDR, 1) == 0)vTaskDelay(2 * portTICK_MS ); tmp = '0';}
            else
            {
                GPIOB->BSRR  |= 0x01; 
                //Y_KEY1------------------------------------------
                GPIOA->BRR |= 0x80;
                scanDelay();
                
                if(readY(GPIOA->IDR, 6) == 0)         {while(readY(GPIOA->IDR, 6) == 0)vTaskDelay(2 * portTICK_MS ); tmp = '1';}
                else if(readY(GPIOA->IDR, 5) == 0)    {while(readY(GPIOA->IDR, 5) == 0)vTaskDelay(2 * portTICK_MS ); tmp = '4';}
                else if(readY(GPIOA->IDR, 4) == 0)    {while(readY(GPIOA->IDR, 4) == 0)vTaskDelay(2 * portTICK_MS ); tmp = '7';}
                else if(readY(GPIOA->IDR, 1) == 0)    {while(readY(GPIOA->IDR, 1) == 0)vTaskDelay(2 * portTICK_MS ); tmp = '*';}
                
                GPIOA->BSRR  |= 0x80;
            }
        }
    }
    
    if(tmp != 0)
    {
        *key = tmp;
        return 1;
    }
    else
        return 0;
}
