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
// ������
#define IO_BEEP                 PE(12)

// 12864
#define IO_RST                  PA(4)
#define IO_RS                   PA(6)
#define IO_RW                   PA(7)
#define IO_EN                   PA(5)

// ���Ĵ�����
#define IO_RACKET_RIGHT_RESET   PD(2)

// ���츴λ������
#define IO_SLIDEWAY_X_RESET     PD(4)
#define IO_SLIDEWAY_Y_RESET     PD(7)

// ����
#define IO_SERVE_MODE           PC(3)   //�Ƿ���&����
#define IO_KEY_TO_SERVE         PD(9)   //���򰴼�
#define IO_SERVE_SHELF          PE(2)   //����ܵ�ŷ�
#define IO_FANGQIU_RELAY        PE(3)   //�����ŷ�
#define IO_SHELF_UP   			PD(0)   //���������������
#define IO_SHELF_DOWN           PD(1)   //��������´�����
#define IO_HIT                  PC(1)   //����ǰ���һ�ΰ������������Ļ���

#define IO_SKIP_LOOP            PD(9)

//�Լ찴��
#define IO_SELFTEST_KEY         PC(1)   //��ɫ����
#define IO_SELFTEST_SERVE       PC(2)
//�������
#define IO_IMERGERCY_MODE       PC(2)

// �������������
#define TIM3_DIR  (1)      //X����
#define TIM4_DIR  (1)      //Y����

// ����������ƽ 0:���ź�
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


// �����ŷ�����
#define Serve_SHELF_UP()        GPIO_SetBits(IO_GPIO_PORT(IO_SERVE_SHELF), IO_GPIO_PIN(IO_SERVE_SHELF))
#define Serve_SHELF_DOWN()      GPIO_ResetBits(IO_GPIO_PORT(IO_SERVE_SHELF), IO_GPIO_PIN(IO_SERVE_SHELF))
#define Serve_FANGQIU_OPEN()    GPIO_ResetBits(IO_GPIO_PORT(IO_FANGQIU_RELAY), IO_GPIO_PIN(IO_FANGQIU_RELAY))
#define Serve_FANGQIU_CLOSE()   GPIO_SetBits(IO_GPIO_PORT(IO_FANGQIU_RELAY), IO_GPIO_PIN(IO_FANGQIU_RELAY))
#else
#error "__IOCONFIG_SELECT_H has defined!"
#endif //__IOCONFIG_H
