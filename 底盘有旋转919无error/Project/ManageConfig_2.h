/**
  ******************************************************************************
  * @file    ManageConfig_2.h
  * @author  
  * @version V1.0
  * @date    7-May-2015
  * @brief   
  ******************************************************************************
  * �Ƕȵ�λ��mm
  * λ�õ�λ��mm
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MANAGECONFIG_SELECT_H
#define __MANAGECONFIG_SELECT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Private functions ---------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/

/* ��ʼλ��ѡ�� */


/* ����λ��(��������������ϵ����λ��) ----------------------------------------*/
    /* �����ϵ�λ�ã����������������������ϵ��ƫ�ƣ�-------------------------*/
    #if (COM_DBG == COM_RELEASE)
    #define DEF_DP_StartPosX            (3050)//(3030 + 390)//3025 - 349
    #define DEF_DP_StartPosY            (3400)//(4680 - 435)//4690 - 418
    #else
    #define DEF_DP_StartPosX            (3000)  //��Ķ�����Ķ�����3000
    #define DEF_DP_StartPosY            (3000)
    #endif
    
    /* ����λ�� */     //�����õ�
    #define DEF_DP_ServePosX            (4265)//(5638)
    #define DEF_DP_ServePosY            (2470)//(3121)
	#define DEF_DP_ServePosAng          (-24)
    #define DEF_HIT_ServeSpeed          (8)
	#define DEF_HIT_ServeDelay          (85)
    
    /* �ӷ���λ�ã��ϵ���һ�ν���ǰλ�ã� ----------------------------------*/
    #define DEF_DP_CatchPosX            DEF_DP_StartPosX//(3000)
    #define DEF_DP_CatchPosY            DEF_DP_StartPosY//(2800)
    
    /* �˺�λλ�� ----------------------------------------------------------*/
    #define DEF_DP_ResetPosX            DEF_DP_StartPosX//(3000)
    #define DEF_DP_ResetPosY            DEF_DP_StartPosY//(2800)
    
    /* �߽籧�� --------------------------------------------------------------*/
    //���Ʊ߽�
    #define DEF_DP_BoundaryAngMax       (35)
    #define DEF_DP_BoundaryAngMin       (-35)
    #define DEF_DP_BoundaryXMax         (6100 -500)
    #define DEF_DP_BoundaryXMin         (0  + 500)
    #define DEF_DP_BoundaryYMax         (6700 - 700-500)
    #define DEF_DP_BoundaryYMin         (0    + 300+500)
    
    //����
    #define DEF_DP_LockAngMax           (DEF_DP_BoundaryAngMax + 0)
    #define DEF_DP_LockAngMin           (DEF_DP_BoundaryAngMin - 0)
    #define DEF_DP_LockXMax             (DEF_DP_BoundaryXMax + 300)
    #define DEF_DP_LockXMin             (DEF_DP_BoundaryXMin - 300)
    #define DEF_DP_LockYMax             (DEF_DP_BoundaryYMax + 300)
    #define DEF_DP_LockYMin             (DEF_DP_BoundaryYMin - 300)

/* ����λ��(����������Ϊԭ�㽨������ϵ) --------------------------------------*/
    /* �����ϻ����λʱλ���������������λ��---------------------------------*/
    #define DEF_HG_InitPosX             (410.8f)//(417.5)
    #define DEF_HG_InitPosY             (-200.8f)//(-204)
    
    /* �����ϻ��鷢��ʱλ���������������λ��---------------------------------*/
    #define DEF_HG_ServePosX            (264)
    #define DEF_HG_ServePosY            (340)
    /* ����߽磨�����޷�����������-------------------------------------------*/
    #define DEF_HG_BoundaryXMax         (537 - 20)//(605 - 20)
    #define DEF_HG_BoundaryXMin         (-436 + 20)//(-471 + 20)
    
    #define DEF_HG_BoundaryYMax         (328 - 70)
    #define DEF_HG_BoundaryYMin         (-303 + 70)
    
/* ����-----------------------------------------------------------------------*/
	#define RACKET_SERVE_SENSOR_ANG 	(-3.3f)//(-8.18526363f)//(-8.65894794f)//(11.0526323f)     //
    #define RACKET_MAIN_SENSOR_ANG      (-6.2f)//(-5.29263153f)//(-5.09052658f)      //��ʼλ��
    #define RACKET_MAIN_RESET_ANG       (10.0f)
    #define RACKET_SERVE_RESET_ANG      (-15.0f)           //˳ʱ��Ϊ��
	
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#else
#error "__MANAGECONFIG_SELECT_H has defined!"
#endif //__MANAGECONFIG_SELECT_H
