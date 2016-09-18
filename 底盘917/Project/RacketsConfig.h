/**
  ******************************************************************************
  * @file    RacketsConfig.h
  * @author  
  * @version V1.0
  * @date    25-May-2015
  * @brief   
  * @note    此文件仅可以包含一次，且仅在HitTask.c中包含
  ******************************************************************************
  * 0~19    Main
  * 20~39   Left
  * 40~59   Right
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RACKETSCONFIG_H
#define __RACKETSCONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "CarSelectConfig.h"

#if CAR_SELECT == 0
    #include "RacketsConfig_0.c"
#elif CAR_SELECT == 1
    #include "RacketsConfig_1.c"
#elif CAR_SELECT == 2
    #include "RacketsConfig_2.c"
#endif
/******************************** END OF FILE *********************************/
#endif //__RACKETSCONFIG_H
