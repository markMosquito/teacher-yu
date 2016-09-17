/*ֻ��PD����*/
#include "pid.h"
#include "string.h"
#include "MyMath.h"

/*����PID����*/
void update_pid_param(pid_t * pid,float setvalue ,float * param)
{
    memset(pid, 0, sizeof(pid_t));
    
    pid->set_value = setvalue; /*�����趨ֵ*/
    pid->Kp        = param[0];
    pid->Ki        = param[1];
    pid->Kd        = param[2];
    pid->out_max   = param[3];
    pid->out_min   = param[4];
    pid->iteg_max  = param[5];
    pid->dead_zone = param[6];
};



/*PID����*/
float pid_calc(pid_t * pid, float real_value)
{
    pid->real_value[NOW] = real_value;                       /*ϵͳ��ǰ��ʵ��ֵ*/
    pid->err[NOW] = pid->set_value - pid->real_value[NOW];   /*��ȡ���*/
    
    /*��������*/
    if (my_abs(pid->err[NOW]) < pid->dead_zone) 
        pid->err[NOW] = 0; 
    pid->err_iteg += pid->err[NOW];                          /*������*/
    
    /*�����޷�*/
    if (pid->err_iteg > pid->iteg_max)       pid->err_iteg =  pid->iteg_max;
    else if (pid->err_iteg < -pid->iteg_max) pid->err_iteg = -pid->iteg_max;
    
    /*PID����*/
    pid->up = pid->err[NOW] * pid->Kp;
    pid->ui = pid->err_iteg * pid->Ki;
    pid->ud = (pid->real_value[LAST] - pid->real_value[NOW]) * pid->Kd; /*΢������PID*/
    
    pid->pid_out = pid->up + pid->ui + pid->ud;

    /*��PID��������޷�*/
    if (pid->pid_out > pid->out_max)      pid->pid_out = pid->out_max;
    else if (pid->pid_out < pid->out_min) pid->pid_out = pid->out_min;
    
    pid->real_value[LAST] = pid->real_value[NOW];            /*ϵͳʵ�ʵ���*/
    pid->err[LAST] = pid->err[NOW];                          /*ϵͳƫ�����*/
    
    return pid->pid_out;                                     
}
