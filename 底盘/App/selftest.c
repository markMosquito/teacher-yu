/**
  ******************************************************************************
  * @file    selftest.c
  * @author  
  * @version V1.0
  * @date    30-May-2015
  * @brief   вт╪Л
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "selftest.h"
#include "DisplayTask.h"


//--------------------------------------------------------------
void ElmoShowErr(uint32_t ElmoFlag)
{
    char tmp[9];
    u8 i =0;
    for(i = 0; i < 8; i++)
    {
        if(ElmoFlag&(1<<(7-i)))
            tmp[i] = '1';
        else
            tmp[i] = '0';
    }
    tmp[8] = '\0';
    LCD_QueuePrintfErr(2, 0, 16, "ElmoErr:%s",tmp);
}

