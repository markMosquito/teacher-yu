/**
  ******************************************************************************
  * @file    DisplayTask.c
  * @author  
  * @version V1.0
  * @date    8-May-2015
  * @brief   DisplayTask
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "DisplayTask.h"
#include "Rsing_12864.h"
#include "app.h"
#include "serial.h"
#include <stdio.h>
#include <stdarg.h>

typedef struct
{
    uint32_t mode;
    uint16_t x;
    uint16_t y;
    uint8_t width;
    char*  pchar;
}Display_Typedef;

/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
QueueHandle_t DisplayQueue;      //显示用
extern point_t ChassisResetPoint;
extern FreeRun_TypeDef ChassisRunPoint;
static uint8_t DisplayIndex;
static char Displaybuf[DISPLAY_QUEUE_NUM][DISPLAY_CHAR_BUF];
static uint32_t DisErrAbsEndT = 0;
extern uint32_t comErrCount;
/* Private function ----------------------------------------------------------*/
void LCD_PrintfDirect(uint16_t Xpos, uint16_t Ypos, uint8_t width, const char *fmt,...);
/**
  * @brief  第0，1行用于显示位置，第2，3行显示调试信息
  * @param  None
  * @retval None
  */

void vDisplayTask( void *p_arg )
{
    TickType_t xLastTESTTime;
    Display_Typedef disTmp;
    memset(&disTmp, 0, sizeof(disTmp));
    
    (void)p_arg;
    /*创建队列用于显示*/
    DisplayQueue = xQueueCreate(DISPLAY_QUEUE_NUM, ( unsigned portBASE_TYPE ) sizeof( Display_Typedef ) );
    
    xLastTESTTime = xTaskGetTickCount();
    while(1)
    {        
        if(xQueueReceive( DisplayQueue, &disTmp, 80*portTICK_MS ) == pdTRUE)
        {
            uint32_t ticks_now = xTaskGetTickCount();
            
            if(disTmp.mode == 0 && ticks_now > DisErrAbsEndT)//正常模式
            {
                LCD_ShowWidth(disTmp.x, disTmp.y, disTmp.width, disTmp.pchar);
            }
            else
            {
                DisErrAbsEndT = ticks_now + 3000*portTICK_MS;
                switch(disTmp.mode)
                {
                    case DISPLAY_MODE_ERRSHOW:
                        LCD_ShowWidth(disTmp.x, disTmp.y, disTmp.width, disTmp.pchar);
                        break;
                    case DISPLAY_MODE_RACKET_ERR:
                        LCD_PrintfDirect(3, 0, 16, "拍号错误");
                        break;
                    case DISPLAY_MODE_CORRECT_ERR:
                        LCD_PrintfDirect(3, 0, 16, "矫正错误");
                        break;
                    case DISPLAY_MODE_CAR_MOVE:
                        LCD_PrintfDirect(3, 0, 16, "车子移动");
                        break;
					case DISPLAY_MODE_CORRECT_OK:
						LCD_PrintfDirect(2, 0, 16, "x:%5d  y:%5d", (int32_t)G_Param.pos_cam_off.x, (int32_t)G_Param.pos_cam_off.y);
						LCD_PrintfDirect(3, 0, 16, "矫正 a:%f", G_Param.pos_cam_off.ang);
						break;
					case DISPLAY_MODE_RESET:
						LCD_PrintfDirect(2, 0, 16, "x:%5d  y:%5d", ChassisResetPoint.x, ChassisResetPoint.y);
                        LCD_PrintfDirect(3, 0, 16, "归位");
						break;
					case DISPLAY_MODE_AVOID:
						LCD_PrintfDirect(2, 0, 16, "x:%5d  y:%5d", ChassisRunPoint.x, ChassisRunPoint.y);
                        LCD_PrintfDirect(3, 0, 16, "避让");
						break;
                    case DISPLAY_MODE_TIM250:
                        LCD_PrintfDirect(3, 0, 16, "TIM<250");
                        break;
                    case DISPLAY_MODE_TIMMAX:
                        LCD_PrintfDirect(3, 0, 16, "TIM>2500");
                        break;
//                    case DISPLAY_PRINTF:
//                        RsPrintf("x:%f, y:%f, ang:%f\n", G_Param.speed.x,G_Param.speed.y,G_Param.speed.ang);
//                        break;
                    default:
                        break;
                }
            }
        }
        LCD_Printf(0,0,"X:%5d  Y:%5d", (int32_t)G_Param.cur_pos.x, (int32_t)G_Param.cur_pos.y);
        LCD_Printf(1,0,"Ang:%f-%d",(float)G_Param.cur_pos.ang, ChassisStatus);
        //LCD_Printf(1,0,"SumErr:%d",comErrCount);
        vTaskDelayUntil(&xLastTESTTime, 80 * portTICK_MS);
    }
}

//x:行数 y:列数
void LCD_QueuePrintf(uint16_t Xpos, uint16_t Ypos, uint8_t width, const char *fmt,...)
{
    char* pChar;
    Display_Typedef dis;
    va_list ap;
    
    pChar = Displaybuf[(DisplayIndex++)%DISPLAY_QUEUE_NUM];
    va_start(ap,fmt);
	vsprintf(pChar,fmt,ap);
	va_end(ap);
    
    dis.mode = 0;
    dis.x = Xpos;
    dis.y = Ypos;
    dis.pchar = pChar;
    dis.width = width;
    
    xQueueSend(DisplayQueue, &dis, 0);
}

void LCD_QueuePrintfErr(uint16_t Xpos, uint16_t Ypos, uint8_t width, const char *fmt,...)
{
    char* pChar;
    Display_Typedef dis;
    va_list ap;
    
    pChar = Displaybuf[(DisplayIndex++)%DISPLAY_QUEUE_NUM];
    va_start(ap,fmt);
	vsprintf(pChar,fmt,ap);
	va_end(ap);
    
    dis.mode = DISPLAY_MODE_ERRSHOW;
    dis.x = Xpos;
    dis.y = Ypos;
    dis.pchar = pChar;
    dis.width = width;
    
    xQueueSend(DisplayQueue, &dis, 0);
}

void LCD_PrintfDirect(uint16_t Xpos, uint16_t Ypos, uint8_t width, const char *fmt,...)
{
    char Disbuf[128];
    va_list ap;
    
    va_start(ap,fmt);
	vsprintf(Disbuf,fmt,ap);
	va_end(ap);

    LCD_ShowWidth(Xpos, Ypos, width, Disbuf);
}

void LCDShowErrFromISR(uint32_t ErrType)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    Display_Typedef dis;
    dis.mode = ErrType;
    xQueueSendFromISR(DisplayQueue, &dis, &xHigherPriorityTaskWoken);
}
/******************************** END OF FILE *********************************/
