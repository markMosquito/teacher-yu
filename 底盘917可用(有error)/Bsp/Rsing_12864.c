/**
  ******************************************************************************
  * @file    Rsing_12864.c
  * @author  Rsing
  * @version V1.0
  * @date    17-July-2014
  * @brief   Initializate the 12864.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Rsing_12864.h"
#include <stdio.h>
#include <stdarg.h>
/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void HJ12864_GPIO_init(void);
static void LCD_WriteData(u8 Data);
static void LCD_delay(void);
static void LCD_delay_72us(void);
static void SendByteLCD(u8 WLCDData);
static void LCD_WriteCmd(u8 CMD);
static void LCD_WriteData(u8 Data);

/**
  * @brief  HJ12864ZW GPIO Configuration
  * @param  None					
  * @retval None
  */
static void HJ12864_GPIO_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
#if defined (STM32F10X_HD) || defined (STM32F10X_MD)
	//---------------ָ��/���ݶ˿�����
	RCC_APB2PeriphClockCmd(RCC_GPIO_RS, ENABLE);      //�򿪶˿�ʱ�� 
	GPIO_InitStructure.GPIO_Pin =  GPIO_RS_PIN;       // ʹ�ܶ˿�
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //���츴�����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50Mʱ���ٶ�
	GPIO_Init(GPIO_RS, &GPIO_InitStructure);
	
	//---------------��/д�˿�����
	RCC_APB2PeriphClockCmd(RCC_GPIO_RW, ENABLE);      //�򿪶˿�ʱ�� 
	GPIO_InitStructure.GPIO_Pin =  GPIO_RW_PIN;       // ʹ�ܶ˿�
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //��׼���ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50Mʱ���ٶ�
	GPIO_Init(GPIO_RW, &GPIO_InitStructure);
	
	//---------------ʹ�ܶ˿�����
	RCC_APB2PeriphClockCmd(RCC_GPIO_EN, ENABLE);      //�򿪶˿�ʱ�� 
	GPIO_InitStructure.GPIO_Pin =  GPIO_EN_PIN;       // ʹ�ܶ˿�
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50Mʱ���ٶ�
	GPIO_Init(GPIO_EN, &GPIO_InitStructure);
	
	//---------------����/���� PSB
    #if USE_PSB == 1
	RCC_APB2PeriphClockCmd(RCC_GPIO_PSB, ENABLE);      //�򿪶˿�ʱ�� 
	GPIO_InitStructure.GPIO_Pin =  GPIO_PSB_PIN;       // ʹ�ܶ˿�
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50Mʱ���ٶ�
	GPIO_Init(GPIO_PSB, &GPIO_InitStructure);
    #endif
	
	//---------------RST
	RCC_APB2PeriphClockCmd(RCC_GPIO_RST, ENABLE);      //�򿪶˿�ʱ�� 
	GPIO_InitStructure.GPIO_Pin =  GPIO_RST_PIN;       // ʹ�ܶ˿�
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50Mʱ���ٶ�
	GPIO_Init(GPIO_RST, &GPIO_InitStructure);
	
	#if PSB_SELECT == 1
	//---------------���ݶ˿�����
	RCC_APB2PeriphClockCmd(RCC_GPIO_DATA, ENABLE);    //�����ݶ˿�ʱ�� 
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_DATA_0_PIN;   //DATA_0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50Mʱ���ٶ�
	GPIO_Init(GPIO_DATA_0, &GPIO_InitStructure);      //��ʼ���˿�
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_DATA_1_PIN;   //DATA_1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50Mʱ���ٶ�
	GPIO_Init(GPIO_DATA_1, &GPIO_InitStructure);      //��ʼ���˿�
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_DATA_2_PIN;   //DATA_2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50Mʱ���ٶ�
	GPIO_Init(GPIO_DATA_2, &GPIO_InitStructure);      //��ʼ���˿�
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_DATA_3_PIN;   //DATA_3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50Mʱ���ٶ�
	GPIO_Init(GPIO_DATA_3, &GPIO_InitStructure);      //��ʼ���˿�
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_DATA_4_PIN;   //DATA_4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50Mʱ���ٶ�
	GPIO_Init(GPIO_DATA_4, &GPIO_InitStructure);      //��ʼ���˿�
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_DATA_5_PIN;   //DATA_5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50Mʱ���ٶ�
	GPIO_Init(GPIO_DATA_5, &GPIO_InitStructure);      //��ʼ���˿�
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_DATA_6_PIN;   //DATA_6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50Mʱ���ٶ�
	GPIO_Init(GPIO_DATA_6, &GPIO_InitStructure);      //��ʼ���˿�
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_DATA_7_PIN;   //DATA_7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50Mʱ���ٶ�
	GPIO_Init(GPIO_DATA_7, &GPIO_InitStructure);      //��ʼ���˿�
	#endif
    
#elif defined (STM32F40_41xxx) || defined (STM32F40XX)
	//---------------ָ��/���ݶ˿�����
	RCC_AHB1PeriphClockCmd(RCC_GPIO_RS, ENABLE);      //�򿪶˿�ʱ�� 
	GPIO_InitStructure.GPIO_Pin =  GPIO_RS_PIN;       // ʹ�ܶ˿�
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(GPIO_RS, &GPIO_InitStructure);
	
	//---------------��/д�˿�����
	RCC_AHB1PeriphClockCmd(RCC_GPIO_RW, ENABLE);      //�򿪶˿�ʱ�� 
	GPIO_InitStructure.GPIO_Pin =  GPIO_RW_PIN;       // ʹ�ܶ˿�
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(GPIO_RW, &GPIO_InitStructure);
	
	//---------------ʹ�ܶ˿�����
	RCC_AHB1PeriphClockCmd(RCC_GPIO_EN, ENABLE);      //�򿪶˿�ʱ�� 
	GPIO_InitStructure.GPIO_Pin =  GPIO_EN_PIN;       // ʹ�ܶ˿�
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(GPIO_EN, &GPIO_InitStructure);
	
	//---------------����/���� PSB
    #if USE_PSB == 1
	RCC_AHB1PeriphClockCmd(RCC_GPIO_PSB, ENABLE);      //�򿪶˿�ʱ�� 
	GPIO_InitStructure.GPIO_Pin =  GPIO_PSB_PIN;       // ʹ�ܶ˿�
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(GPIO_PSB, &GPIO_InitStructure);
	#endif
    
	//---------------RST
	RCC_AHB1PeriphClockCmd(RCC_GPIO_RST, ENABLE);      //�򿪶˿�ʱ�� 
	GPIO_InitStructure.GPIO_Pin =  GPIO_RST_PIN;       // ʹ�ܶ˿�
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(GPIO_RST, &GPIO_InitStructure);
#endif
}

#if PSB_SELECT == 1
/**
  * @brief  ���8λ���ݵ�GPIO
  * @param  Ҫ�����8λ����
  * @retval None
  */
static void Data_out(unsigned char x)                     //DATA���
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_DATA_7_PIN;   //DATA7
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIO_DATA_7, &GPIO_InitStructure);      //��æ���˿�
	
	GPIO_ResetBits(GPIO_DATA_0, GPIO_DATA_0_PIN);  //DATA0
	GPIO_ResetBits(GPIO_DATA_0, GPIO_DATA_1_PIN);  //DATA1
	GPIO_ResetBits(GPIO_DATA_0, GPIO_DATA_2_PIN);  //DATA2
	GPIO_ResetBits(GPIO_DATA_0, GPIO_DATA_3_PIN);  //DATA3
	GPIO_ResetBits(GPIO_DATA_0, GPIO_DATA_4_PIN);  //DATA4
	GPIO_ResetBits(GPIO_DATA_0, GPIO_DATA_5_PIN);  //DATA5
	GPIO_ResetBits(GPIO_DATA_0, GPIO_DATA_6_PIN);  //DATA6
	GPIO_ResetBits(GPIO_DATA_0, GPIO_DATA_7_PIN);  //DATA7
	if(x&0X01)GPIO_SetBits(GPIO_DATA_0, GPIO_DATA_0_PIN);//DATA0
	if(x&0X02)GPIO_SetBits(GPIO_DATA_0, GPIO_DATA_1_PIN);//DATA1
	if(x&0X04)GPIO_SetBits(GPIO_DATA_0, GPIO_DATA_2_PIN);//DATA2
	if(x&0X08)GPIO_SetBits(GPIO_DATA_0, GPIO_DATA_3_PIN);//DATA3
	if(x&0X10)GPIO_SetBits(GPIO_DATA_0, GPIO_DATA_4_PIN);//DATA4
	if(x&0X20)GPIO_SetBits(GPIO_DATA_0, GPIO_DATA_5_PIN);//DATA5
	if(x&0X40)GPIO_SetBits(GPIO_DATA_0, GPIO_DATA_6_PIN);//DATA6
	if(x&0X80)GPIO_SetBits(GPIO_DATA_0, GPIO_DATA_7_PIN);//DATA7
}

/**
  * @brief  ���LCD�Ƿ���æ״̬
  * @param  None					
  * @retval æ����1, ���з���0
  */
static u8 LCD_CheckBusy()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	u8 busy;
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_DATA_7_PIN;   //DATA7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //����ģʽ = ��������
	GPIO_Init(GPIO_DATA_7, &GPIO_InitStructure);      //��æ���˿�

    RS_L;
    RW_H;
    EN_H;
    busy = GPIO_ReadInputDataBit(GPIO_DATA_7, GPIO_DATA_7_PIN);
    EN_L;

    return busy;
}


/**
  * @brief  д���12864
  * @param  8bit����					
  * @retval None
  */
static void LCD_WriteCmd(u8 cmd)
{
    while( LCD_CheckBusy() );
    RS_L;
    RW_L;
    EN_L;
    Data_out(cmd);
    EN_H;
    EN_L;
}

/**
  * @brief  д���ݵ�12864
  * @param  8bit����					
  * @retval None
  */
static void LCD_WriteData(u8 data)
{
    while( LCD_CheckBusy() );
    RS_H;
    LCD_delay();
    RW_L;
    LCD_delay();
    EN_L;
    Data_out(data);
    EN_H;
    LCD_delay();
    EN_L;
    LCD_delay();
}
#else
static void LCD_delay(void)
{	
   u16 i = 0; 
   while(i) 
   { 
     i--; 
   } 
}
static void LCD_delay_72us(void)
{	
   u16 i = 5000; 
   while(i) 
   { 
     i--; 
   } 
}

static void SendByteLCD(u8 WLCDData)
{
	u8 i;
	for(i=0;i<8;i++)
	{
		EN_L;
        LCD_delay();
		if((WLCDData<<i)&0x80) RW_H;
		else RW_L;
        LCD_delay();
		EN_H;
        LCD_delay();
	}
}

static void SPIWR(u8 Wdata,u8 WRS)
{
	SendByteLCD(0xf8+(WRS<<1));
	SendByteLCD(Wdata&0xf0);
	SendByteLCD((Wdata<<4)&0xf0);
    EN_L;
    LCD_delay();
}

static void LCD_WriteCmd(u8 CMD)
{
	RS_L;
    LCD_delay();
	RS_H;
	SPIWR(CMD,0);
    RS_L;
	LCD_delay_72us();
}
static void LCD_WriteData(u8 Data)
{
	RS_L;
    LCD_delay();
	RS_H;
	SPIWR(Data,1);
    RS_L;
	LCD_delay_72us();
}
#endif
/* Exported functios --------------------------------------------------------*/
/**
  * @brief  ��ʼ��LCD-8λ�ӿ�
  * @param  None					
  * @retval None
  */
void LCD_Configuration(void)
{
	HJ12864_GPIO_init();
    #if USE_PSB == 1
        #if PSB_SELECT == 1
        PSB_H;//����
        #else
        PSB_L;//����
        #endif
    #endif
	RST_H;
	EN_L;
	RST_L;
    LCD_delay_72us();
	RST_H;
    LCD_delay_72us();
	LCD_WriteCmd(0x30); //����ָ�,8λ����
    LCD_delay_72us();
	LCD_WriteCmd(0x06); //��ʼ���趨���������
    LCD_delay_72us();
	LCD_WriteCmd(0x01); //�����ʾDDRAM
    LCD_delay_72us();
	LCD_WriteCmd(0x0C); //��ʾ״̬���أ�������ʾ���������ʾ�أ������ʾ���׹�
    LCD_delay_72us();
	LCD_WriteCmd(0x02); //��ַ����
    LCD_delay_72us();
}

/**
  * @brief  ��������
  * @param  x, y					
  * @retval None
  */
void LCD_SetPosition(u8 x, u8 y)
{
	u8 p;
    switch(x%4)
    {
        case 0: p = 0x80; break; //��һ�п�ʼ��ַ
        case 1: p = 0x90; break; //�ڶ���
        case 2: p = 0x88; break; //������
        case 3: p = 0x98; break; //������
    }
    p += y;
    LCD_WriteCmd(p);
}

/**
  * @brief  ��ʾ�ַ�
  * @param  str
  * @retval None
  */
void LCD_WriteString(char * str)
{
	u8 i = 0;
    while(str[i] != '\0')
    {
        LCD_WriteData(str[i++]);
    }
}

/**
  * @brief  ��ָ��λ����ʾ�ַ�
  * @param  x, y, str
  * @retval None
  */
void LCD_Show(u8 x, u8 y, char * str)
{
	u8 p,i =0;
    switch(x%4)
    {
        case 0: p = 0x80; break; //��һ�п�ʼ��ַ
        case 1: p = 0x90; break; //�ڶ���
        case 2: p = 0x88; break; //������
        case 3: p = 0x98; break; //������
    }
    p += y;
    LCD_WriteCmd(p);
    while(str[i] != '\0')
    {
        LCD_WriteData(str[i++]);
    }
}

/**
  * @brief  ��ָ��λ����ʾָ������ַ�
  * @param  x, y, str
  * @retval None
  */
void LCD_ShowWidth(u8 x, u8 y, u8 width, char * str)
{
	u8 p,i =0;
    switch(x%4)
    {
        case 0: p = 0x80; break; //��һ�п�ʼ��ַ
        case 1: p = 0x90; break; //�ڶ���
        case 2: p = 0x88; break; //������
        case 3: p = 0x98; break; //������
    }
    p += y;
    LCD_WriteCmd(p);
    while(str[i] != '\0' && i < width)
    {
        LCD_WriteData(str[i++]);
    }
    while(i < width)
    {
        LCD_WriteData(' ');
        i++;
    }
}

/**
  * @brief  ����
  * @param  None
  * @retval None
  */
void LCD_Clear(void) 
{ 
	LCD_WriteCmd(0x01); 
	LCD_WriteCmd(0x34); 
	LCD_WriteCmd(0x30); 
}

/**
  * @brief  д��GDRAM ��ͼ
  * @param  2���ֽ�һ��,CLONG��ͼ�γ���,���ֽ�Ϊ��λ;HIGHT��ͼ�θ߶�,TAB��ͼ�����ݱ�.
  * @retval None
  */
//void LCD_WRGDRAM(u8 x, u8 clong, u8 hight, char *TAB)
//{
//	u16 k;
//	u8 i,j;
//	LCD_WriteCmd(0x34);
//	LCD_WriteCmd(0x36);
//	for(j = 0; j < hight; j++)//32
//	{
//		//���ϰ���
//		LCD_WriteCmd(0x80 + j);   //Y������,���ڼ���
//		LCD_WriteCmd(0x80 + x);   //X���꣬�������ڼ����ֽڿ�ʼд��
//		for(i = 0; i < clong; i++)
//		{
//			LCD_WriteData(TAB[clong*j+i]);
//		}
//		//���°���
//		for(k = 0; k < clong; k++)
//		{
//			LCD_WriteData(TAB[clong*(j+hight)+k]);
//		}
//	}
//}

//x:���� y:����
void LCD_Printf(uint16_t Xpos, uint16_t Ypos, const char *fmt,...)
{
	static char LCD_BUF[128]; 
    uint8_t p =0;
    
    va_list ap;
    va_start(ap,fmt);
	vsprintf((char *)LCD_BUF,fmt,ap);
	va_end(ap);
	
    switch(Xpos%4)
    {
        case 0: p = 0x80; break; //��һ�п�ʼ��ַ
        case 1: p = 0x90; break; //�ڶ���
        case 2: p = 0x88; break; //������
        case 3: p = 0x98; break; //������
    }
    p += Ypos;
    LCD_WriteCmd(p);
    LCD_WriteString((char*)LCD_BUF);
}

/*********************************END OF FILE**********************************/
