/**
  ******************************************************************************
  * @file    HitTask.h
  * @author  
  * @version V1.0
  * @date    7-May-2015
  * @brief   Header file for HitTask.c module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HITTASK_H
#define __HITTASK_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "007_elmo.h"
#include "manage.h"

/* Defines -------------------------------------------------------------------*/
typedef enum
{
    RT_MAIN,
    RT_LEFT,
    RT_RIGHT,
    RT_NO
}RacketDir_Typedef;

typedef struct
{
    //uint32_t id;        /* ���ӵı�� */
		uint8_t flag_time_to_hit;
    float   ang;
    float   spd;
    //int32_t XOff;       /* Xƫ�� */
    //int32_t YOff;       /* Yƫ�� */
    //int32_t TOff;
    RacketDir_Typedef dir;  /* ���ӷ����������� */
}Racket_TypeDef;



/* ���������*/
#define RACKET_MAX_NUM      (99)    /* �������ĵ�id��Χ:0<=id<=RACKET_MAX_NUM*/

#define RACKET_ID_NORACKET  (RACKET_MAX_NUM+1)    /* �����ģ����ܻ��죬ֻ�ܵ���*/
#define RACKET_ID_HIT_TIM   (RACKET_MAX_NUM+2)    /* ����ʱ�䵽 */

/*----------------------------------------------------------------------------*/
#define HIT_MAIN_MAX_SPEED          ((float)(MOTOR_MAXSPEED_HIT_MAIN/MOTOR_R2CNT_HIT_MAIN))//ת/��
#define HIT_SERVE_MAX_SPEED         ((float)(MOTOR_MAXSPEED_HIT_SERVE/MOTOR_R2CNT_HIT_SERVE))//ת/��
/* �����ٶ� ��λ��ת/s */
#define HIT_MAIN_PPM(v, pos)        {\
                                    int32_t vcnt;\
                                    vcnt = MOTOR_DIR_HIT_MAIN*MOTOR_R2CNT_HIT_MAIN*(v);\
                                    Elmo_PPM(MOTOR_ID_HIT_MAIN,LIMIT_VMAX(vcnt, MOTOR_MAXSPEED_HIT_MAIN), (pos), POS_ABS);}

#define HIT_MAIN_PVM(v)             {\
                                    int32_t vcnt;\
                                    vcnt = MOTOR_DIR_HIT_MAIN*MOTOR_R2CNT_HIT_MAIN*(v);\
                                    Elmo_PVM(MOTOR_ID_HIT_MAIN, LIMIT_VMAX(vcnt, MOTOR_MAXSPEED_HIT_MAIN));}

#define HIT_MAIN_STOP()             Elmo_Stop(MOTOR_ID_HIT_MAIN)
#define HIT_MAIN_CLOSE()            Elmo_Close(MOTOR_ID_HIT_MAIN)

#define HIT_SEREVE_PPM(v, pos)      {\
                                    int32_t vcnt;\
                                    vcnt = MOTOR_DIR_HIT_SERVE*MOTOR_R2CNT_HIT_SERVE*(v);\
                                    Elmo_PPM(MOTOR_ID_HIT_SERVE,LIMIT_VMAX(vcnt, MOTOR_MAXSPEED_HIT_SERVE), (pos), POS_ABS);}

#define HIT_SERVE_PVM(v)            {\
                                    int32_t vcnt;\
                                    vcnt = MOTOR_DIR_HIT_SERVE*MOTOR_R2CNT_HIT_SERVE*(v);\
                                    Elmo_PVM(MOTOR_ID_HIT_SERVE, LIMIT_VMAX(vcnt, MOTOR_MAXSPEED_HIT_SERVE));}

#define HIT_SEREVE_STOP()           Elmo_Stop(MOTOR_ID_HIT_SERVE)
#define HIT_SEREVE_CLOSE()          Elmo_Close(MOTOR_ID_HIT_SERVE)

/* Exported variables---------------------------------------------------------*/
extern Racket_TypeDef RacketsMap[RACKET_MAX_NUM+3];
extern Racket_TypeDef RacketReceive;       //���յ�������ѡ��
//extern Racket_TypeDef* pCurRacket;    //��ǰ��ѡ����

/* Exported constants --------------------------------------------------------*/


/* Exported Function----------------------------------------------------------*/
void RacketsMapInit(void);
void TIM5_init(uint8_t PPr, uint8_t SPr);
void AllRacketsHoming(void);
uint32_t ReadRacketAngWithOff(uint32_t id, float *pAng);
void RacketRun2Ang(uint32_t id, float spd, float Ang, uint32_t timeOut);
void hittim_test(uint32_t dir);

void hit_test_main(void);
void hit_test_left(void);
void hit_test_right(void);

#endif //HitTask.h
