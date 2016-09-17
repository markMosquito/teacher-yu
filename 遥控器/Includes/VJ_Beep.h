#ifndef __VJ_BEEP_H
#define __VJ_BEEP_H

#define BEEP_ON  GPIO_WriteBit(GPIOE,GPIO_Pin_10,Bit_SET)
#define BEEP_OFF GPIO_WriteBit(GPIOE,GPIO_Pin_10,Bit_RESET)


#include "stm32f10x.h"

#include <stdbool.h>  
#include <stdarg.h>    
#include <string.h>   
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

void Beep_GPIOInit(void);
void BEEP1(u32 BTIME);

#endif
