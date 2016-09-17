/**
  ******************************************************************************
  * @file    IOConfig.h
  * @author  
  * @version V1.0
  * @date    7-May-2015
  * @brief   Header file for IOConfig.c module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IOCONFIG_H
#define __IOCONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "CarSelectConfig.h"

#if CAR_SELECT == 0
    #include "IOConfig_0.h"
#elif CAR_SELECT == 1
    #include "IOConfig_1.h"
#elif CAR_SELECT == 2
    #include "IOConfig_2.h"
#endif

#endif //__IOCONFIG_H
