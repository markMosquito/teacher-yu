/*只有PD调节*/
#include "pid.h"
#include "string.h"
#include "MyMath.h"

/*更新PID参数*/
void update_pid_param(pid_t * pid,float setvalue ,float * param)
{
    memset(pid, 0, sizeof(pid_t));
    
    pid->set_value = setvalue; /*更新设定值*/
    pid->Kp        = param[0];
    pid->Ki        = param[1];
    pid->Kd        = param[2];
    pid->out_max   = param[3];
    pid->out_min   = param[4];
    pid->iteg_max  = param[5];
    pid->dead_zone = param[6];
};



/*PID计算*/
float pid_calc(pid_t * pid, float real_value)
{
    pid->real_value[NOW] = real_value;                       /*系统当前的实际值*/
    pid->err[NOW] = pid->set_value - pid->real_value[NOW];   /*获取误差*/
    
    /*死区控制*/
    if (my_abs(pid->err[NOW]) < pid->dead_zone) 
        pid->err[NOW] = 0; 
    pid->err_iteg += pid->err[NOW];                          /*误差积分*/
    
    /*积分限幅*/
    if (pid->err_iteg > pid->iteg_max)       pid->err_iteg =  pid->iteg_max;
    else if (pid->err_iteg < -pid->iteg_max) pid->err_iteg = -pid->iteg_max;
    
    /*PID运算*/
    pid->up = pid->err[NOW] * pid->Kp;
    pid->ui = pid->err_iteg * pid->Ki;
    pid->ud = (pid->real_value[LAST] - pid->real_value[NOW]) * pid->Kd; /*微分先行PID*/
    
    pid->pid_out = pid->up + pid->ui + pid->ud;

    /*对PID输出进行限幅*/
    if (pid->pid_out > pid->out_max)      pid->pid_out = pid->out_max;
    else if (pid->pid_out < pid->out_min) pid->pid_out = pid->out_min;
    
    pid->real_value[LAST] = pid->real_value[NOW];            /*系统实际迭代*/
    pid->err[LAST] = pid->err[NOW];                          /*系统偏差迭代*/
    
    return pid->pid_out;                                     
}
