/**
  ******************************************************************************
  * @file    ManageConfig_0.h
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
    #define DEF_DP_StartPosX            (5280)//3025 - 349,5291,  5307
    #define DEF_DP_StartPosY            (4250)//4690 - 418,4254,  4260
    #else
    #define DEF_DP_StartPosX            (3000)
    #define DEF_DP_StartPosY            (3000)
    #endif
    
    /* 发球位置 */
    #define DEF_DP_ServePosX            (3667)
    #define DEF_DP_ServePosY            (4220)
	#define DEF_DP_ServePosAng          (-50)
	#define DEF_HIT_ServeDelay          (110)
    
    /* 接发球位置（上电后第一次接球前位置） ----------------------------------*/
    #define DEF_DP_CatchPosX            (5291- 2100)
    #define DEF_DP_CatchPosY            (4254 - 1000)
    
    /* 此后复位位置 ----------------------------------------------------------*/
    #define DEF_DP_ResetPosX            (5291 - 2100)
    #define DEF_DP_ResetPosY            (4254 - 1000)
    
    /* 边界抱死 --------------------------------------------------------------*/
    //限制边界
    #define DEF_DP_BoundaryAngMax       (20)
    #define DEF_DP_BoundaryAngMin       (-20)
    #define DEF_DP_BoundaryXMax         (6100 - 100)
    #define DEF_DP_BoundaryXMin         (0    + 100)
    #define DEF_DP_BoundaryYMax         (6700 - 100)
    #define DEF_DP_BoundaryYMin         (0    + 100)
    
    //抱死
    #define DEF_DP_LockAngMax           (DEF_DP_BoundaryAngMax + 15)
    #define DEF_DP_LockAngMin           (DEF_DP_BoundaryAngMin - 15)
    #define DEF_DP_LockXMax             (DEF_DP_BoundaryXMax + 300)
    #define DEF_DP_LockXMin             (DEF_DP_BoundaryXMin - 300)
    #define DEF_DP_LockYMax             (DEF_DP_BoundaryYMax + 300)
    #define DEF_DP_LockYMin             (DEF_DP_BoundaryYMin - 300)
    
/* 拍子的Z（接收点时用于选拍）*/
    
/* 滑轨位置(以码盘中心为原点建立坐标系) --------------------------------------*/
    /* 滑轨上滑块归位时位置相对于码盘中心位置---------------------------------*/
    #define DEF_HG_InitPosX             (458)
    #define DEF_HG_InitPosY             (-261)
    
    #define DEF_HG_ServePosX            (85)
    #define DEF_HG_ServePosY            (132)
    /* 滑轨边界（用于限幅，不抱死）-------------------------------------------*/
    #define DEF_HG_BoundaryXMax         (DEF_HG_InitPosX - 25)
    #define DEF_HG_BoundaryXMin         (-622 + 25)
    
    #define DEF_HG_BoundaryYMax         (281 - 25)
    #define DEF_HG_BoundaryYMin         (DEF_HG_InitPosY + 25)

/* 拍子偏移 x,y:mm(相对于码盘中心)  t:ms(数越大挥拍越早) ---------------------*/
    #define RACKET_MAIN_FRONT_Z          (0 + CAR_SELECT*100)
	#define RACKET_MAIN_MIDDLE_Z         (1 + CAR_SELECT*100)
	#define RACKET_MAIN_BEHIND_Z         (2 + CAR_SELECT*100)
    
	#define RACKET_LEFT_FRONT_Z          (4 + CAR_SELECT*100)
	#define RACKET_LEFT_MIDDLE_Z         (5 + CAR_SELECT*100)
	#define RACKET_LEFT_BEHIND_Z         (6 + CAR_SELECT*100)
    #define RACKET_LEFT_LEFT_Z           (7 + CAR_SELECT*100)
    
	#define RACKET_RIGHT_FRONT_Z         (8 + CAR_SELECT*100)
	#define RACKET_RIGHT_MIDDLE_Z        (9 + CAR_SELECT*100)
	#define RACKET_RIGHT_BEHIND_Z        (10 + CAR_SELECT*100)
    #define RACKET_RIGHT_RIGHT_Z         (11 + CAR_SELECT*100)
    
    #define RACKET_MAIN_YELLOW_Z         (12 + CAR_SELECT*100)
    #define RACKET_LEFT_YELLOW_Z         (13 + CAR_SELECT*100)
    #define RACKET_RIGHT_YELLOW_Z        (14 + CAR_SELECT*100)
    
    #define COM_DBG_Z                   (15)
/* 拍子偏移 x,y:mm(相对于码盘中心)  t:ms(数越大挥拍越早) ---------------------*/
    #define RACKET_NO_OFF_X             (0)
    #define RACKET_NO_OFF_Y             (0)
    #define RACKET_NO_OFF_T             (0)

   //主拍0 - 3
	#define RACKET_MAIN_FRONT_OFF_X           (-75)
    #define RACKET_MAIN_FRONT_OFF_Y           (-480)
    #define RACKET_MAIN_FRONT_OFF_T           (52 + 24 + 8)
    #define RACKET_MAIN_FRONT_SPEED           (5)
    
    #define RACKET_MAIN_MIDDLE_OFF_X           (-75)
    #define RACKET_MAIN_MIDDLE_OFF_Y           (-480)
    #define RACKET_MAIN_MIDDLE_OFF_T           (52 + 24 + 8)
    #define RACKET_MAIN_MIDDLE_SPEED           (5)
    
    #define RACKET_MAIN_BEHIND_OFF_X           (-75)
    #define RACKET_MAIN_BEHIND_OFF_Y           (-480)
    #define RACKET_MAIN_BEHIND_OFF_T           (52 + 24 + 8)
    #define RACKET_MAIN_BEHIND_SPEED           (5)

    //左拍4 - 7
    #define RACKET_LEFT_FRONT_OFF_X           (-75)
    #define RACKET_LEFT_FRONT_OFF_Y           (-480)
    #define RACKET_LEFT_FRONT_OFF_T           (52 + 24 + 8)
    #define RACKET_LEFT_FRONT_SPEED           (5)
    
    #define RACKET_LEFT_MIDDLE_OFF_X           (-75)
    #define RACKET_LEFT_MIDDLE_OFF_Y           (-480)
    #define RACKET_LEFT_MIDDLE_OFF_T           (52 + 24 + 8)
    #define RACKET_LEFT_MIDDLE_SPEED           (5)
    
    #define RACKET_LEFT_BEHIND_OFF_X           (-75)
    #define RACKET_LEFT_BEHIND_OFF_Y           (-480)
    #define RACKET_LEFT_BEHIND_OFF_T           (52 + 24 + 8)
    #define RACKET_LEFT_BEHIND_SPEED           (5)
    
    #define RACKET_LEFT_LEFT_OFF_X           (-75)
    #define RACKET_LEFT_LEFT_OFF_Y           (-480)
    #define RACKET_LEFT_LEFT_OFF_T           (52 + 24 + 8)
    #define RACKET_LEFT_LEFT_SPEED           (5)
	
    //右拍8 - 11
	#define RACKET_RIGHT_FRONT_OFF_X           (-75)
    #define RACKET_RIGHT_FRONT_OFF_Y           (-480)
    #define RACKET_RIGHT_FRONT_OFF_T           (52 + 24 + 8)
    #define RACKET_RIGHT_FRONT_SPEED           (5)
    
    #define RACKET_RIGHT_MIDDLE_OFF_X           (-75)
    #define RACKET_RIGHT_MIDDLE_OFF_Y           (-480)
    #define RACKET_RIGHT_MIDDLE_OFF_T           (52 + 24 + 8)
    #define RACKET_RIGHT_MIDDLE_SPEED           (5)
    
    #define RACKET_RIGHT_BEHIND_OFF_X           (-75)
    #define RACKET_RIGHT_BEHIND_OFF_Y           (-480)
    #define RACKET_RIGHT_BEHIND_OFF_T           (52 + 24 + 8)
    #define RACKET_RIGHT_BEHIND_SPEED           (5)
    
    #define RACKET_RIGHT_RIGHT_OFF_X           (-75)
    #define RACKET_RIGHT_RIGHT_OFF_Y           (-480)
    #define RACKET_RIGHT_RIGHT_OFF_T           (52 + 24 + 8)
    #define RACKET_RIGHT_RIGHT_SPEED           (5)
	
    //黄区12 - 14
    #define RACKET_MAIN_YELLOW_OFF_X            (-60)
    #define RACKET_MAIN_YELLOW_OFF_Y            (-160)
    #define RACKET_MAIN_YELLOW_OFF_T             (91 + 24+ 8)
    #define RACKET_MAIN_YELLOW_SPEED              (3)
    
    #define RACKET_LEFT_YELLOW_OFF_X            (-60)
    #define RACKET_LEFT_YELLOW_OFF_Y            (-160)
    #define RACKET_LEFT_YELLOW_OFF_T             (91 + 24+ 8)
    #define RACKET_LEFT_YELLOW_SPEED              (3)
    
    #define RACKET_RIGHT_YELLOW_OFF_X            (-60)
    #define RACKET_RIGHT_YELLOW_OFF_Y            (-160)
    #define RACKET_RIGHT_YELLOW_OFF_T             (91 + 24+ 8)
    #define RACKET_RIGHT_YELLOW_SPEED              (3)

	#define RACKET_SERVE_SENSOR_ANG 	(6.4f)
    #define RACKET_MAIN_RESET_ANG       (10.0f)
    #define RACKET_SERVE_RESET_ANG      (15.0f)
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#else
#error "__MANAGECONFIG_SELECT_H has defined!"
#endif //__MANAGECONFIG_SELECT_H
