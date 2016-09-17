/**
  ******************************************************************************
  * @file    ManageConfig_2.h
  * @author  
  * @version V1.0
  * @date    7-May-2015
  * @brief   
  ******************************************************************************
  * 角度单位：mm
  * 位置单位：mm
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MANAGECONFIG_SELECT_H
#define __MANAGECONFIG_SELECT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Private functions ---------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/

/* 初始位置选择 */


/* 底盘位置(均采用世界坐标系绝对位置) ----------------------------------------*/
    /* 底盘上电位置（码盘中心相对于世界坐标系的偏移）-------------------------*/
    #if (COM_DBG == COM_RELEASE)
    #define DEF_DP_StartPosX            (3050)//(3030 + 390)//3025 - 349
    #define DEF_DP_StartPosY            (3400)//(4680 - 435)//4690 - 418
    #else
    #define DEF_DP_StartPosX            (3000)  //勿改动，或改动完归回3000
    #define DEF_DP_StartPosY            (3000)
    #endif
    
    /* 发球位置 */     //发球用的
    #define DEF_DP_ServePosX            (4265)//(5638)
    #define DEF_DP_ServePosY            (2470)//(3121)
	#define DEF_DP_ServePosAng          (-24)
    #define DEF_HIT_ServeSpeed          (8)
	#define DEF_HIT_ServeDelay          (85)
    
    /* 接发球位置（上电后第一次接球前位置） ----------------------------------*/
    #define DEF_DP_CatchPosX            DEF_DP_StartPosX//(3000)
    #define DEF_DP_CatchPosY            DEF_DP_StartPosY//(2800)
    
    /* 此后复位位置 ----------------------------------------------------------*/
    #define DEF_DP_ResetPosX            DEF_DP_StartPosX//(3000)
    #define DEF_DP_ResetPosY            DEF_DP_StartPosY//(2800)
    
    /* 边界抱死 --------------------------------------------------------------*/
    //限制边界
    #define DEF_DP_BoundaryAngMax       (35)
    #define DEF_DP_BoundaryAngMin       (-35)
    #define DEF_DP_BoundaryXMax         (6100 -500)
    #define DEF_DP_BoundaryXMin         (0  + 500)
    #define DEF_DP_BoundaryYMax         (6700 - 700-500)
    #define DEF_DP_BoundaryYMin         (0    + 300+500)
    
    //抱死
    #define DEF_DP_LockAngMax           (DEF_DP_BoundaryAngMax + 0)
    #define DEF_DP_LockAngMin           (DEF_DP_BoundaryAngMin - 0)
    #define DEF_DP_LockXMax             (DEF_DP_BoundaryXMax + 300)
    #define DEF_DP_LockXMin             (DEF_DP_BoundaryXMin - 300)
    #define DEF_DP_LockYMax             (DEF_DP_BoundaryYMax + 300)
    #define DEF_DP_LockYMin             (DEF_DP_BoundaryYMin - 300)

/* 滑轨位置(以码盘中心为原点建立坐标系) --------------------------------------*/
    /* 滑轨上滑块归位时位置相对于码盘中心位置---------------------------------*/
    #define DEF_HG_InitPosX             (410.8f)//(417.5)
    #define DEF_HG_InitPosY             (-200.8f)//(-204)
    
    /* 滑轨上滑块发球时位置相对于码盘中心位置---------------------------------*/
    #define DEF_HG_ServePosX            (264)
    #define DEF_HG_ServePosY            (340)
    /* 滑轨边界（用于限幅，不抱死）-------------------------------------------*/
    #define DEF_HG_BoundaryXMax         (537 - 20)//(605 - 20)
    #define DEF_HG_BoundaryXMin         (-436 + 20)//(-471 + 20)
    
    #define DEF_HG_BoundaryYMax         (328 - 70)
    #define DEF_HG_BoundaryYMin         (-303 + 70)
    
/* 拍子-----------------------------------------------------------------------*/
	#define RACKET_SERVE_SENSOR_ANG 	(-3.3f)//(-8.18526363f)//(-8.65894794f)//(11.0526323f)     //
    #define RACKET_MAIN_SENSOR_ANG      (-6.2f)//(-5.29263153f)//(-5.09052658f)      //初始位置
    #define RACKET_MAIN_RESET_ANG       (10.0f)
    #define RACKET_SERVE_RESET_ANG      (-15.0f)           //顺时针为正
	
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#else
#error "__MANAGECONFIG_SELECT_H has defined!"
#endif //__MANAGECONFIG_SELECT_H
