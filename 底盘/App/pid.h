#ifndef __PID_H
#define __PID_H
#include "stm32f4xx.h"

enum _now_last
{
    NOW = 1,
    LAST = 0,
};


/*λ��ʽPID*/
typedef struct
{
    /* Pid ���� */
    float Kp;              /*KP*/
    float Ki;              /*Ki*/
    float Kd;              /*Kd*/
    float out_max;         /*PID�����������ֵ*/
    float out_min;         /*PID����������Сֵ*/
    float iteg_max;        /*�����޷�*/
    float dead_zone;       /*����*/
    
    float set_value;       /*����ֵ*/
    float real_value[2];   /*���ֵ(ʵ��ֵ)*/
    float err[2];          /*ƫ��*/
    float pid_out;         /*PID������*/
    float err_iteg;        /*������*/

    float up;
    float ui;
    float ud;
}
pid_t;

void update_pid_param(pid_t * pid,float setvalue ,float * param);
float pid_calc(pid_t * pid, float real_value);
#endif
