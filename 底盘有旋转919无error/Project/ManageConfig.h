/**
  ******************************************************************************
  * @file    ManageConfig.h
  * @author  
  * @version V1.0
  * @date    7-May-2015
  * @brief   
  ******************************************************************************
  * 
  * 角度单位：mm
  * 位置单位：mm
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MANAGECONFIG_H
#define __MANAGECONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "CarSelectConfig.h"

/* 调试 */
#define COM_RELEASE             (0)
#define COM_DBG_CHASSIS_X       (1)
#define COM_DBG_CHASSIS_Y       (2)
#define COM_DBG_SLIDEWAY_X		(3)
#define COM_DBG_SLIDEWAY_Y		(4)
#define COM_DBG_SLIDEWAY_T		(5)
#define COM_DBG_ALL				(6)
#define COM_DEBUG               (7)

#define COM_DBG                 (COM_RELEASE)	//仅可选择COM_RELEASE/COM_DEBUG
#define COM3_DBG                (COM_RELEASE)

#if (COM_DBG != COM_RELEASE && COM_DBG != COM_DEBUG)
#error "COM_DBG only can be COM_RELEASE or COM_DEBUG!"
#endif

#if CAR_SELECT == 0
    #include "ManageConfig_0.h"
#elif CAR_SELECT == 1
    #include "ManageConfig_1.h"
#elif CAR_SELECT == 2
    #include "ManageConfig_2.h"
#endif

#endif //__MANAGECONFIG_H
