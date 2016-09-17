/**
  ******************************************************************************
  * @file    Rsing_12864.h
  * @author  Rsing
  * @version V1.0
  * @date    17-July-2014
  * @brief   Header file for Rsing_12864.c module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RSING_12864_H
#define __RSING_12864_H

/* Includes ------------------------------------------------------------------*/
#if defined (STM32F10X_HD) || defined (STM32F10X_MD)
 #include "stm32f10x.h"
#elif defined (STM32F40_41xxx)
 #include "stm32f4xx.h"
#endif

/* Defines -------------------------------------------------------------------*/
#define PSB_SELECT    0                     //1�����ڣ�0������
#define USE_PSB       0
/********************************* ���˿ڶ��� *********************************/
#if defined (STM32F10X_HD) || defined (STM32F10X_MD)
#if USE_PSB == 1
//    #define GPIO_PSB          GPIOA                 //1�����ڿ��ƣ�0�����ڿ���
//    #define GPIO_PSB_PIN      GPIO_Pin_5
//    #define RCC_GPIO_PSB      RCC_APB2Periph_GPIOA 
#endif
    #define GPIO_RST          GPIOB                 //0����λ
    #define GPIO_RST_PIN      GPIO_Pin_15
    #define RCC_GPIO_RST      RCC_APB2Periph_GPIOB

    #define GPIO_RS       GPIOB                 //1�����ݣ�0������  ;CS������Ƭѡ
    #define GPIO_RS_PIN   GPIO_Pin_12
    #define RCC_GPIO_RS   RCC_APB2Periph_GPIOB

    #define GPIO_RW       GPIOB                 //0��д  ��1����    ;SID��������������
    #define GPIO_RW_PIN   GPIO_Pin_13
    #define RCC_GPIO_RW   RCC_APB2Periph_GPIOB

    #define GPIO_EN       GPIOB                 //ʹ��              ;SCLK������ʱ��
    #define GPIO_EN_PIN   GPIO_Pin_14
    #define RCC_GPIO_EN   RCC_APB2Periph_GPIOB

    #define PSB_H  GPIO_PSB->BSRR = GPIO_PSB_PIN  //PSB ʹ��  ���1
    #define PSB_L  GPIO_PSB->BRR  = GPIO_PSB_PIN  //         ���0
    #define RST_H  GPIO_RST->BSRR = GPIO_RST_PIN  //RST ʹ��  ���1
    #define RST_L  GPIO_RST->BRR  = GPIO_RST_PIN  //         ���0
    #define RS_H   GPIO_RS->BSRR = GPIO_RS_PIN    //RS ָ��  ���1
    #define RS_L   GPIO_RS->BRR  = GPIO_RS_PIN    //         ���0 
    #define RW_H   GPIO_RW->BSRR = GPIO_RW_PIN    //RW ��д  ���1
    #define RW_L   GPIO_RW->BRR  = GPIO_RW_PIN    //         ���0
    #define EN_H   GPIO_EN->BSRR = GPIO_EN_PIN    //EN ʹ��  ���1
    #define EN_L   GPIO_EN->BRR  = GPIO_EN_PIN    //         ���0
#elif defined (STM32F40_41xxx)
    #if USE_PSB == 1
    #define GPIO_PSB          GPIOA                 //1�����ڿ��ƣ�0�����ڿ���
    #define GPIO_PSB_PIN      GPIO_Pin_5
    #define RCC_GPIO_PSB      RCC_AHB1Periph_GPIOA 
    #endif
    
    #define GPIO_RST          GPIOC                 //0����λ
    #define GPIO_RST_PIN      GPIO_Pin_4
    #define RCC_GPIO_RST      RCC_AHB1Periph_GPIOC

    #define GPIO_RS       GPIOB                 //1�����ݣ�0������  ;CS������Ƭѡ
    #define GPIO_RS_PIN   GPIO_Pin_1
    #define RCC_GPIO_RS   RCC_AHB1Periph_GPIOB

    #define GPIO_RW       GPIOB                 //0��д  ��1����    ;SID��������������
    #define GPIO_RW_PIN   GPIO_Pin_0
    #define RCC_GPIO_RW   RCC_AHB1Periph_GPIOB

    #define GPIO_EN       GPIOC                 //ʹ��              ;SCLK������ʱ��
    #define GPIO_EN_PIN   GPIO_Pin_5
    #define RCC_GPIO_EN   RCC_AHB1Periph_GPIOC

    #if USE_PSB == 1
    #define PSB_H  GPIO_PSB->BSRRL = GPIO_PSB_PIN  //PSB ʹ��  ���1
    #define PSB_L  GPIO_PSB->BSRRH  = GPIO_PSB_PIN  //         ���0
    #endif 
    
    #define RST_H  GPIO_RST->BSRRL = GPIO_RST_PIN  //RST ʹ��  ���1
    #define RST_L  GPIO_RST->BSRRH  = GPIO_RST_PIN  //         ���0
    #define RS_H   GPIO_RS->BSRRL = GPIO_RS_PIN    //RS ָ��  ���1
    #define RS_L   GPIO_RS->BSRRH  = GPIO_RS_PIN    //         ���0 
    #define RW_H   GPIO_RW->BSRRL = GPIO_RW_PIN    //RW ��д  ���1
    #define RW_L   GPIO_RW->BSRRH  = GPIO_RW_PIN    //         ���0
    #define EN_H   GPIO_EN->BSRRL = GPIO_EN_PIN    //EN ʹ��  ���1
    #define EN_L   GPIO_EN->BSRRH  = GPIO_EN_PIN    //         ���0
#endif
#if PSB_SELECT == 1
// ���� ------------------------------------------------------------------------
#define GPIO_DATA_0       GPIOB
#define GPIO_DATA_0_PIN   GPIO_Pin_8
#define GPIO_DATA_1       GPIOB
#define GPIO_DATA_1_PIN   GPIO_Pin_9
#define GPIO_DATA_2       GPIOB
#define GPIO_DATA_2_PIN   GPIO_Pin_10
#define GPIO_DATA_3       GPIOB
#define GPIO_DATA_3_PIN   GPIO_Pin_11
#define GPIO_DATA_4       GPIOB
#define GPIO_DATA_4_PIN   GPIO_Pin_12
#define GPIO_DATA_5       GPIOB
#define GPIO_DATA_5_PIN   GPIO_Pin_13
#define GPIO_DATA_6       GPIOB
#define GPIO_DATA_6_PIN   GPIO_Pin_14
#define GPIO_DATA_7       GPIOB
#define GPIO_DATA_7_PIN   GPIO_Pin_15
#define RCC_GPIO_DATA     RCC_APB2Periph_GPIOB  //������ʱ����

// ���� ------------------------------------------------------------------------
#endif

#if defined (STM32F40_41xxx)
#endif
/* Private Functions ---------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void LCD_Config(void);             //12864��ʼ��
void LCD_SetPosition(uint8_t x, uint8_t y);  //��������
void LCD_WriteString(char * str);    //��ʾ�ַ��������ģ�
void LCD_Show(uint8_t x, uint8_t y,char * str);//��ָ��λ�ú���ʾ�ַ�
void LCD_Clear(void);              //����
void LCD_WRGDRAM(uint8_t x, uint8_t clong, uint8_t hight, char *TAB);//��ʾͼƬ
void LCD_Printf(uint16_t Xpos, uint16_t Ypos,const char *fmt,...);
#endif //Rsing_12864.h
