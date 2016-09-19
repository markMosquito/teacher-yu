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
// ������
#define IO_BEEP                 PE(12)

// 12864
#define IO_RST                  PA(4)
#define IO_RS                   PA(5)
#define IO_RW                   PA(7)
#define IO_EN                   PA(6)

// ���Ĵ�����
#define IO_RACKET_MAIN_RESET    PD(5)
#define IO_RACKET_MAIN_STOP     PD(6)
#define IO_RACKET_LEFT_RESET    PB(8)
#define IO_RACKET_LEFT_STOP     PD(3)
#define IO_RACKET_RIGHT_RESET   PD(2)
#define IO_RACKET_RIGHT_STOP    PD(3)

// ���츴λ������
#define IO_SLIDEWAY_X_RESET     PD(7)
#define IO_SLIDEWAY_Y_RESET     PD(4)

// ����
#define IO_SERVE_MODE           PD(8)   //�Ƿ���&����
#define IO_KEY_TO_SERVE         PD(9)
#define IO_SERVE_SHELF          PE(4)   //����ܵ�ŷ�
#define IO_SERVE_SECOND         PE(2)
#define IO_FANGQIU_RELAY        PE(0)   //�����ŷ�
#define IO_SHELF_UP   			PB(5)
#define IO_SHELF_DOWN           PD(1)

// �������������
#define TIM3_DIR  (-1)      //X����
#define TIM4_DIR  (-1)      //Y����

// ����������ƽ 0:���ź�
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

// �����ŷ�����
#define Serve_SHELF_UP()        GPIO_SetBits(IO_GPIO_PORT(IO_SERVE_SHELF), IO_GPIO_PIN(IO_SERVE_SHELF))
#define Serve_SHELF_DOWN()      GPIO_ResetBits(IO_GPIO_PORT(IO_SERVE_SHELF), IO_GPIO_PIN(IO_SERVE_SHELF))
#define Serve_SECOND_OPEN()     GPIO_SetBits(IO_GPIO_PORT(IO_SERVE_SECOND), IO_GPIO_PIN(IO_SERVE_SECOND))
#define Serve_SECOND_CLOSE()    GPIO_ResetBits(IO_GPIO_PORT(IO_SERVE_SECOND), IO_GPIO_PIN(IO_SERVE_SECOND))
#define Serve_FANGQIU_OPEN()    GPIO_SetBits(IO_GPIO_PORT(IO_FANGQIU_RELAY), IO_GPIO_PIN(IO_FANGQIU_RELAY))
#define Serve_FANGQIU_CLOSE()   GPIO_ResetBits(IO_GPIO_PORT(IO_FANGQIU_RELAY), IO_GPIO_PIN(IO_FANGQIU_RELAY))
#else
#error "__IOCONFIG_SELECT_H has defined!"
#endif //__IOCONFIG_H
