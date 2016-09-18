#ifndef __PID_H
#define __PID_H
#include "stm32f4xx.h"

enum _now_last
{
    NOW = 1,
    LAST = 0,
};


/*位置式PID*/
typedef struct
{
    /* Pid 参数 */
    float Kp;              /*KP*/
    float Ki;              /*Ki*/
    float Kd;              /*Kd*/
    float out_max;         /*PID运算结果的最大值*/
    float out_min;         /*PID运算结果的最小值*/
    float iteg_max;        /*积分限幅*/
    float dead_zone;       /*死区*/
    
    float set_value;       /*给定值*/
    float real_value[2];   /*输出值(实际值)*/
    float err[2];          /*偏差*/
    float pid_out;         /*PID运算结果*/
    float err_iteg;        /*误差积分*/

    float up;
    float ui;
    float ud;
}
pid_t;

void update_pid_param(pid_t * pid,float setvalue ,float * param);
float pid_calc(pid_t * pid, float real_value);
#endif
