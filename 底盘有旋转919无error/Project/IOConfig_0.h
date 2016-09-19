/**
  ******************************************************************************
  * @file    IOConfig_0.h
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
#define IO_RS                   PA(5)
#define IO_RW                   PA(7)
#define IO_EN                   PA(6)

// 挥拍传感器
#define IO_RACKET_MAIN_RESET    PD(5)
#define IO_RACKET_MAIN_STOP     PD(6)
#define IO_RACKET_LEFT_RESET    PB(8)
#define IO_RACKET_LEFT_STOP     PD(3)
#define IO_RACKET_RIGHT_RESET   PD(2)
#define IO_RACKET_RIGHT_STOP    PD(3)

// 滑轨复位传感器
#define IO_SLIDEWAY_X_RESET     PD(7)
#define IO_SLIDEWAY_Y_RESET     PD(4)

// 发球
#define IO_SERVE_MODE           PD(8)   //是否发球&放球
#define IO_KEY_TO_SERVE         PD(9)
#define IO_SERVE_SHELF          PE(4)   //发球架电磁阀
#define IO_SERVE_SECOND         PE(2)
#define IO_FANGQIU_RELAY        PE(0)   //放球电磁阀
#define IO_SHELF_UP   			PB(5)
#define IO_SHELF_DOWN           PD(1)

// 滑轨编码器方向
#define TIM3_DIR  (-1)      //X方向
#define TIM4_DIR  (-1)      //Y方向

// 读传感器电平 0:有信号
#define READ_RACKET_M_RESET()       IO_GPIO_ReadInputDataBit(IO_RACKET_MAIN_RESET)
#define READ_RACKET_M_STOP()        IO_GPIO_ReadInputDataBit(IO_RACKET_MAIN_STOP)
#define READ_RACKET_L_RESET()       IO_GPIO_ReadInputDataBit(IO_RACKET_LEFT_RESET)
#define READ_RACKET_L_STOP()        IO_GPIO_ReadInputDataBit(IO_RACKET_LEFT_STOP)
#define READ_RACKET_R_RESET()       IO_GPIO_ReadInputDataBit(IO_RACKET_RIGHT_RESET)
#define READ_RACKET_R_STOP()        IO_GPIO_ReadInputDataBit(IO_RACKET_RIGHT_STOP)
#define READ_SLIDEWAY_X_RESET()     IO_GPIO_ReadInputDataBit(IO_SLIDEWAY_X_RESET)
#define READ_SLIDEWAY_Y_RESET()     IO_GPIO_ReadInputDataBit(IO_SLIDEWAY_Y_RESET)
#define READ_IF_SEVER()             IO_GPIO_ReadInputDataBit(IO_SERVE_MODE)
#define READ_KEY_TO_SERVE()         IO_GPIO_ReadInputDataBit(IO_KEY_TO_SERVE)
#define READ_SHELF_UP()		        IO_GPIO_ReadInputDataBit(IO_SHELF_UP)
#define READ_SHELF_DOWN()		    IO_GPIO_ReadInputDataBit(IO_SHELF_DOWN)

// 发球电磁阀操作
#define Serve_SHELF_UP()        GPIO_SetBits(IO_GPIO_PORT(IO_SERVE_SHELF), IO_GPIO_PIN(IO_SERVE_SHELF))
#define Serve_SHELF_DOWN()      GPIO_ResetBits(IO_GPIO_PORT(IO_SERVE_SHELF), IO_GPIO_PIN(IO_SERVE_SHELF))
#define Serve_SECOND_OPEN()     GPIO_SetBits(IO_GPIO_PORT(IO_SERVE_SECOND), IO_GPIO_PIN(IO_SERVE_SECOND))
#define Serve_SECOND_CLOSE()    GPIO_ResetBits(IO_GPIO_PORT(IO_SERVE_SECOND), IO_GPIO_PIN(IO_SERVE_SECOND))
#define Serve_FANGQIU_OPEN()    GPIO_SetBits(IO_GPIO_PORT(IO_FANGQIU_RELAY), IO_GPIO_PIN(IO_FANGQIU_RELAY))
#define Serve_FANGQIU_CLOSE()   GPIO_ResetBits(IO_GPIO_PORT(IO_FANGQIU_RELAY), IO_GPIO_PIN(IO_FANGQIU_RELAY))
#else
#error "__IOCONFIG_SELECT_H has defined!"
#endif //__IOCONFIG_H
