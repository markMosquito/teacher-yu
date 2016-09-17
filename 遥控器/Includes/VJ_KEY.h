/*********************************************************/
/**FILENAME :    VJ_KEY.h                               **/
/**PROGRAMMER:   VJ                                     **/
/**DATE:         2014.03.20                             **/
/*********************************************************/

#ifndef  __VJ_KEY_H
#define  __VJ_KEY_H
#include "stm32f10x.h"

#define KEY_ON 0
#define KEY_OFF 1



u8 Key_Scan(GPIO_TypeDef* GPIOx,u16 GPIO_Pin);                  //函数声明
//void Delay_100us (__IO u32 nTime);                            //函数声明
//void TimeDelay_DEC(void);                                     //函数声明
void KEY_Configuration(void);

void matrixKeyConfig(void);
uint8_t ScanKey(char *key);
#endif
