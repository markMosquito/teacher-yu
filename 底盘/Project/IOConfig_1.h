/**
  ******************************************************************************
  * @file    IOConfig_1.h
  * @author  
  * @version V1.0
  * @date    7-May-2015
  * @brief   Header file for IOConfig.c module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IOCONFIG_SELECT_H
#define __IOCONFIG_SELECT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "Redef_GPIO.h"

/* Pin Define ----------------------------------------------------------------*/
// 蜂鸣器
#define IO_BEEP                 PE(12)

// 12864
#define IO_RST                  PA(4)
#define IO_RS                   PA(6)
#define IO_RW                   PA(7)
#define IO_EN                   PA(5)

// 挥拍传感器
#define IO_RACKET_RIGHT_RESET   PD(2)

// 滑轨复位传感器
#define IO_SLIDEWAY_X_RESET     PD(4)
#define IO_SLIDEWAY_Y_RESET     PD(7)

// 发球
#define IO_SERVE_MODE           PC(3)   //是否发球&放球
#define IO_KEY_TO_SERVE         PD(9)   //放球按键
#define IO_SERVE_SHELF          PE(2)   //发球架电磁阀
#define IO_FANGQIU_RELAY        PE(3)   //放球电磁阀
#define IO_SHELF_UP   			PD(0)   //发球架起来传感器
#define IO_SHELF_DOWN           PD(1)   //发球架落下传感器
#define IO_HIT                  PC(1)   //发球前最后一次按键，控制球拍击球

#define IO_SKIP_LOOP            PD(9)

//自检按键
#define IO_SELFTEST_KEY         PC(1)   //绿色按键
#define IO_SELFTEST_SERVE       PC(2)
//紧急情况
#define IO_IMERGERCY_MODE       PC(2)

// 滑轨编码器方向
#define TIM3_DIR  (1)      //X方向
#define TIM4_DIR  (1)      //Y方向

// 读传感器电平 0:有信号
#define READ_RACKET_R_RESET()       IO_GPIO_ReadInputDataBit(IO_RACKET_RIGHT_RESET)
#define READ_SLIDEWAY_X_RESET()     IO_GPIO_ReadInputDataBit(IO_SLIDEWAY_X_RESET)
#define READ_SLIDEWAY_Y_RESET()     IO_GPIO_ReadInputDataBit(IO_SLIDEWAY_Y_RESET)
#define READ_IF_SEVER()             IO_GPIO_ReadInputDataBit(IO_SERVE_MODE)
#define READ_KEY_TO_SERVE()         IO_GPIO_ReadInputDataBit(IO_KEY_TO_SERVE)
#define READ_SHELF_UP()		        IO_GPIO_ReadInputDataBit(IO_SHELF_UP)
#define READ_SHELF_DOWN()		    IO_GPIO_ReadInputDataBit(IO_SHELF_DOWN)
#define READ_HIT()                  IO_GPIO_ReadInputDataBit(IO_HIT)
#define READ_SELFTEST_KEY()         IO_GPIO_ReadInputDataBit(IO_SELFTEST_KEY)
#define READ_IMERGENCY_MODE()       IO_GPIO_ReadInputDataBit(IO_IMERGERCY_MODE)
#define READ_SEIFTEST_SERVE()       IO_GPIO_ReadInputDataBit(IO_SELFTEST_SERVE)
#define READ_SKIP_LOOP()            IO_GPIO_ReadInputDataBit(IO_SKIP_LOOP)


// 发球电磁阀操作
#define Serve_SHELF_UP()        GPIO_SetBits(IO_GPIO_PORT(IO_SERVE_SHELF), IO_GPIO_PIN(IO_SERVE_SHELF))
#define Serve_SHELF_DOWN()      GPIO_ResetBits(IO_GPIO_PORT(IO_SERVE_SHELF), IO_GPIO_PIN(IO_SERVE_SHELF))
#define Serve_FANGQIU_OPEN()    GPIO_ResetBits(IO_GPIO_PORT(IO_FANGQIU_RELAY), IO_GPIO_PIN(IO_FANGQIU_RELAY))
#define Serve_FANGQIU_CLOSE()   GPIO_SetBits(IO_GPIO_PORT(IO_FANGQIU_RELAY), IO_GPIO_PIN(IO_FANGQIU_RELAY))
#else
#error "__IOCONFIG_SELECT_H has defined!"
#endif //__IOCONFIG_H
