/**
  ******************************************************************************
  * @file    ManageConfig_1.h
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
#define CAR_NUM  (0)

/* Private functions ---------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/

/* 初始位置选择 */


/* 底盘位置(均采用世界坐标系绝对位置) ----------------------------------------*/
    /* 底盘上电位置（码盘中心相对于世界坐标系的偏移）-------------------------*/
    #if (COM_DBG == COM_RELEASE)
    #define DEF_DP_StartPosX            (3050)//(460 + 390)//3025 - 349
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
    #define DEF_DP_BoundaryAngMax       (DEF_DP_LockAngMax)
    #define DEF_DP_BoundaryAngMin       (DEF_DP_LockAngMin)
    #define DEF_DP_BoundaryXMax         (DEF_DP_LockXMax - 300)
    #define DEF_DP_BoundaryXMin         (DEF_DP_LockXMin + 300)
    #define DEF_DP_BoundaryYMax         (DEF_DP_LockYMax - 300)
    #define DEF_DP_BoundaryYMin         (DEF_DP_LockYMin + 300)
    
    //抱死
    #define DEF_DP_LockAngMax           ( 35)
    #define DEF_DP_LockAngMin           (-35)
    #define DEF_DP_LockXMax             (6100 - 800)
    #define DEF_DP_LockXMin             (0    + 800)
    #define DEF_DP_LockYMax             (6700 - 700)
    #define DEF_DP_LockYMin             (0    + 900)

/* 滑轨位置(以码盘中心为原点建立坐标系) --------------------------------------*/
    /* 滑轨上滑块归位时位置相对于码盘中心位置---------------------------------*/
    #define DEF_HG_InitPosX             ((184.7+783.2)/2)//((414.4+546.5)/2)
    #define DEF_HG_InitPosY             ((-26-215.7)/2)//(-37)
    
    /* 滑轨上滑块发球时位置相对于码盘中心位置---------------------------------*/
    #define DEF_HG_ServePosX            (264)
    #define DEF_HG_ServePosY            (340)
    /* 滑轨边界（用于限幅，不抱死）-------------------------------------------*/
    #define DEF_HG_BoundaryXMax         (500.0f - 20)//(503 - 20)//(607 - 20)
    #define DEF_HG_BoundaryXMin         (-443.3f + 20)//(-478 + 20)//(-460 + 20)
    
    #define DEF_HG_BoundaryYMax         (294.0f - 20)
    #define DEF_HG_BoundaryYMin         (-210.0f + 20)
    
/* 拍子-----------------------------------------------------------------------*/
#if CAR_NUM == 3
	#define RACKET_SERVE_SENSOR_ANG 	((float)(-(((160.459579f+107.302177f)/2)-180.0f)))//((float)(32.94847965f))//(-1.8326f)//(-9.3f)//(-11.0526323f)//(38.7064285f)
#elif CAR_NUM == 4
    #define RACKET_SERVE_SENSOR_ANG 	((float)(-(((157.896530f+104.473045f)/2)-180.0f+1.5f)))
#elif CAR_NUM == 2
    #define RACKET_SERVE_SENSOR_ANG     ((float)(-((-149.54f + 58.67f)/2))+2.0f)
#else
	#define RACKET_SERVE_SENSOR_ANG 	((float)(((-46.2939148f+129.975662f)/2)))//((float)(32.94847965f))//(-1.8326f)//(-9.3f)//(-11.0526323f)//(38.7064285f)
#endif
    
#if CAR_NUM == 2
    #define RACKET_MAIN_SENSOR_ANG      (-5.32304382f-1.5f)//初始位置
#elif CAR_NUM == 4
    #define RACKET_MAIN_SENSOR_ANG      (-5.32304382f)//初始位置
#else
    
    #define RACKET_MAIN_SENSOR_ANG      (-5.32304382f) //初始位置
#endif
    
    #define RACKET_MAIN_RESET_ANG       (10.0f)
    #define RACKET_SERVE_RESET_ANG      (15.0f)           //顺时针为正
	
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#else
#error "__MANAGECONFIG_SELECT_H has defined!"
#endif //__MANAGECONFIG_SELECT_H
