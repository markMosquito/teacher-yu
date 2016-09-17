/**
  ******************************************************************************
  * @file    MotorConfig.h
  * @author  
  * @version V1.0
  * @date    7-May-2015
  * @brief   
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTORCONFIG_H
#define __MOTORCONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "CarSelectConfig.h"

#define LIMIT_VMAX(v, vmax)     ( (v)>(vmax) ? (vmax) : ((v)<(-(vmax)) ? (-(vmax)):(v)) )

#if CAR_SELECT == 0
    #include "MotorConfig_0.h"
#elif CAR_SELECT == 1
    #include "MotorConfig_1.h"
#elif CAR_SELECT == 2
    #include "MotorConfig_2.h"
#endif

#endif //__MOTORCONFIG_H
