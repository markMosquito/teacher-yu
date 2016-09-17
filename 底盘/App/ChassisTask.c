/**
  ******************************************************************************
  * @file    ChassisTask.c
  * @author  
  * @version V1.0
  * @date    7-May-2015
  * @brief   ChassisTask
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ChassisTask.h"
#include "app.h"
#include "Global.h"
#include "MyMath.h"
#include "Com2Server.h"
#include "MotorConfig.h"
/* Private constants ---------------------------------------------------------*/
const pid_t PidXNormal;
const pid_t PidXReset;
const pid_t PidYNormal;
const pid_t PidYReset;
const pid_t PidAngNormal;
const pid_t PidAngReset;

const pid_t PidXNew[7]=
{
/* Kp,      Ki,     Kd,         out_max,    out_min,    iteg_max,   dead_zone */
{   5,      0,      15.0,   	8000,       -8000,      0,          25},
{   5,      0,      73.0,   	8000,       -8000,      0,          25},
{   4.8,    0,      175.0,   	8000,       -8000,      0,          25},
{   4.4,    0,      175.0,   	8000,       -8000,      0,          25},
{   4.2,    0,      185.0,   	8000,       -8000,      0,          25},
{   4,      0,      185.0,   	8000,       -8000,      0,          25},
{   4,      0,      200.0,   	8000,       -8000,      0,          25},
};

const pid_t PidYNew[7]=
{
/* Kp,      Ki,     Kd,         out_max,    out_min,    iteg_max,   dead_zone */
{   5,      0,      15.0,   	8000,       -8000,      0,          25},
{   5,      0,      73.0,   	8000,       -8000,      0,          25},
{   4.8,    0,      175.0,   	8000,       -8000,      0,          25},
{   4.4,    0,      175.0,   	8000,       -8000,      0,          25},
{   4.2,    0,      185.0,   	8000,       -8000,      0,          25},
{   4,      0,      185.0,   	8000,       -8000,      0,          25},
{   4,      0,      200.0,   	8000,       -8000,      0,          25},
};

uint32_t SelectPid(int32_t x0, int32_t x1)
{
    uint32_t pidi;
    x1 = my_abs(x1-x0);
    
    if(x1 < 500)          pidi=0;
    else if(x1 < 1000)    pidi=1;
    else if(x1 < 1500)    pidi=2;
    else if(x1 < 2000)    pidi=3;
    else if(x1 < 2500)    pidi=4;
    else if(x1 < 3000)    pidi=5;
    else                  pidi=6;
    
    pidi += 1;
    if(pidi > 6)          pidi=6;
    return pidi;
}

/* Private variables ---------------------------------------------------------*/
uint32_t ChassisStatus = RobotReady;
uint32_t ChassisStop = 0;
point_t ChassisResetPoint = {DEF_DP_ResetPosX, DEF_DP_ResetPosY, 0.0f};
uint32_t YellowFlag = 0;
point_t ChassisTeleV = {0,0};
uint32_t PidXSelected = 6, PidYSelected = 6;
point_t StartPoint, RunPoint;
u32 lowPowerFlag = 0;

/* Private function ----------------------------------------------------------*/
static void PidRunToPoint(int32_t x, int32_t y, float ang, uint32_t ifRun);
void LineRun(int32_t x, int32_t y, int32_t speed);
/**
  * @brief  vChassisTask
  * @param  None
  * @retval None
  */
void vChassisTask( void *p_arg )
{
    uint32_t ticks_now;
    uint32_t TimeOut;
    uint32_t PidSelecti;
    point_t pidRun;
    (void)p_arg;
    
    /* PID初始化 */
    memset(&PidX, 0,sizeof(PidX));
    memset(&PidY, 0,sizeof(PidY));
    memset(&PidAng, 0,sizeof(PidAng));
    
    memcpy(&PidX,   &PidXNormal,    7*sizeof(float));  //仅复制参数（共7个）
    memcpy(&PidY,   &PidYNormal,    7*sizeof(float)); 
    memcpy(&PidAng, &PidAngNormal,  7*sizeof(float)); 

	PidX.real_value[LAST] = ( G_Param.cur_pos.x + G_Param.cur_pos.y)*DEF_sqrt2_div2;
    PidY.real_value[LAST] = (-G_Param.cur_pos.x + G_Param.cur_pos.y)*DEF_sqrt2_div2;
    PidAng.real_value[LAST] = G_Param.cur_pos.ang;
    while(1)
    {
        if(xSemaphoreTake(PosChassisSem, 60) == pdTRUE)       //码盘数据5ms一次，这里最多等6ms
        {
            ticks_now = xTaskGetTickCount();

            /* 状态切换 ------------------------------------------------------*/
            if(ChassisStatusChange != RobotReady)
            {
                if(ChassisStatusChange == RobotRun)                     //若切换至击球状态
                {
                    /* 计算目标点 --------------------------------------------*/
                    pidRun.x = xyzt_structure.x ;
                    pidRun.y = xyzt_structure.y ;
                    pidRun.ang = 0;

                    /* 改变PID参数 --------------------------------------------*/
                #if (COM_DBG == COM_RELEASE)    //非调试状态
                    #if (CAR_SELECT == 1 || 1)
                    
                    /*减少底盘跑动范围*/
                    {
                        float dx,dy;
                        dx = pidRun.x - G_Param.cur_pos.x;
                        dy = pidRun.y - G_Param.cur_pos.y;
                        if(dx > 0)
                        {
                            dx = dx - (DEF_HG_BoundaryXMax - 300);
                            if(dx < 0) dx = 0;
                        }
                        else
                        {
                            dx = dx - (DEF_HG_BoundaryXMin + 300);
                            if(dx > 0) dx = 0;
                        }
                        
                        if(dy > 0)
                        {
                            dy = dy - (DEF_HG_BoundaryYMax - 100);
                            if(dy < 0) dy = 0;
                        }
                        else
                        {
                            dy = dy - (DEF_HG_BoundaryYMin + 100);
                            if(dy > 0) dy = 0;
                        }
                        
                        pidRun.x = G_Param.cur_pos.x + dx;
                        pidRun.y = G_Param.cur_pos.y + dy;
                    }
                    
                    
                    if(ChassisStatus != RobotRun)
                    {
                        StartPoint.x = ( G_Param.cur_pos.x + G_Param.cur_pos.y)/2;
                        StartPoint.y = (-G_Param.cur_pos.x + G_Param.cur_pos.y)/2; 
                        PidXSelected = 0;
                        PidYSelected = 0;
                    }
                    RunPoint.x = ( pidRun.x + pidRun.y)/2;
                    RunPoint.y = (-pidRun.x + pidRun.y)/2;
                    
                    PidSelecti = SelectPid(StartPoint.x, RunPoint.x);
                    if(PidSelecti > PidXSelected)
                    PidXSelected = PidSelecti;
                    PidSelecti = SelectPid(StartPoint.y, RunPoint.y);
                    if(PidSelecti > PidYSelected)
                    PidYSelected = PidSelecti;
                        
                    memcpy(&PidX,   &PidXNew[PidXSelected], 7*sizeof(float));   //仅复制参数（共7个）
                    memcpy(&PidY,   &PidYNew[PidYSelected], 7*sizeof(float));
                    memcpy(&PidAng, &PidAngNormal,  7*sizeof(float));
                    TimeOut = xyzt_structure.absEndT;
                    #else
                    memcpy(&PidX,   &PidXNormal,    7*sizeof(float));   //仅复制参数（共7个）
                    memcpy(&PidY,   &PidYNormal,    7*sizeof(float));
                    memcpy(&PidAng, &PidAngNormal,  7*sizeof(float));
                    TimeOut = xyzt_structure.absEndT;
                    #endif
                #else       //若调试底盘PID，不复制参数
                    if(DbgChoose == COM_DBG_CHASSIS_X || DbgChoose == COM_DBG_CHASSIS_Y)
                    {
                        TimeOut = xTaskGetTickCount() + 5000*portTICK_MS;    /**@Note调试5秒超时*/
                    }
                    else
                    {
                        memcpy(&PidX,   &PidXNormal,    7*sizeof(float));   //仅复制参数（共7个）
                        memcpy(&PidY,   &PidYNormal,    7*sizeof(float));
                        memcpy(&PidAng, &PidAngNormal,  7*sizeof(float));
                        TimeOut = xyzt_structure.absEndT;
                    }
                #endif

                    ChassisStop = 0;
                    ChassisStatus = RobotRun;
                }
                else if(ChassisStatusChange == RobotReset)      //若切换至复位状态
                {
                    if(ChassisStatus != RobotReset)
                    {
                        memcpy(&PidX,   &PidXReset,     7*sizeof(float));   //仅复制参数（共7个）
                        memcpy(&PidY,   &PidYReset,     7*sizeof(float));
                        memcpy(&PidAng, &PidAngReset,   7*sizeof(float));
                    }
                    
                    pidRun.x = ChassisResetPoint.x;
                    pidRun.y = ChassisResetPoint.y;
                    pidRun.ang = 0;
                    TimeOut = ticks_now + 5000*portTICK_MS;
                    ChassisStatus = RobotReset;
                }
                else if(ChassisStatusChange == RobotStop)       //底盘抱死
                {
                    ChassisStatus = RobotStop;
                }
                else if(ChassisStatusChange == RobotFreeRun || ChassisStatusChange == RobotAvoid)    //底盘运行ChassisRunPoint
                {
					if(ChassisStatus != ChassisStatusChange)
					{
						memcpy(&PidX,   &PidXNormal,    7*sizeof(float));   //仅复制参数（共7个）
						memcpy(&PidY,   &PidYNormal,    7*sizeof(float));
						memcpy(&PidAng, &PidAngNormal,  7*sizeof(float));
					}

                    memcpy(&pidRun, &ChassisRunPoint, 2*sizeof(int32_t));
					pidRun.ang = 0;
                    TimeOut = ticks_now + ChassisRunPoint.tim*portTICK_MS;
                    
                    ChassisStatus = ChassisStatusChange;
                }
                else
                {
                    ChassisStatus = RobotStop;
                }

                /* 边界限幅 --------------------------------------------------*/
                if(pidRun.x > DEF_DP_BoundaryXMax)
                    pidRun.x = DEF_DP_BoundaryXMax;
                else if(pidRun.x < DEF_DP_BoundaryXMin)
                    pidRun.x = DEF_DP_BoundaryXMin;
                if(pidRun.y > DEF_DP_BoundaryYMax)
                    pidRun.y = DEF_DP_BoundaryYMax;
                else if(pidRun.y < DEF_DP_BoundaryYMin)
                    pidRun.y = DEF_DP_BoundaryYMin;

                ChassisStatusChange = RobotReady;
            }

            /* 根据当前状态做出响应 ------------------------------------------*/
            switch(ChassisStatus)
            {
                case RobotRun:  /* 击球 --------------------------------------*/
                    if(ChassisStop != 0)
                    {
                        Chassis_Elmo_Stop();
                        ChassisStatus = RobotReady;
                        break;
                    }
                    if(ticks_now > TimeOut)/* 超时:ChassisStatus->RoboReady */
                    {
                        Chassis_Elmo_Stop();
                        ChassisStatus = RobotReady;
                        break;
                    }
                    break;

                case RobotReset: /* 复位 -------------------------------------*/
                    if ((my_abs(G_Param.cur_pos.x - pidRun.x) <= 5) &&
                        (my_abs(G_Param.cur_pos.y - pidRun.y) <= 5) &&
                        (my_abs(G_Param.cur_pos.ang - 0) <= 0.25f))
                    {
                        Chassis_Elmo_Stop();
                        ChassisStatus = RobotReady;
                        break;
                    }
                    if(ticks_now > TimeOut)/* 超时:ChassisStatus->RoboReady */
                    {
                        Chassis_Elmo_Stop();
                        ChassisStatus = RobotReady;
                        break;
                    }
                    break;
                    
                case RobotFreeRun:
                    if(ticks_now > TimeOut)/* 超时:ChassisStatus->RoboReady */
                    {
                        Chassis_Elmo_Stop();
                        ChassisStatus = RobotReady;
                        break;
                    }
					if (   (my_abs(G_Param.cur_pos.x - pidRun.x) <= 40)
                        && (my_abs(G_Param.cur_pos.y - pidRun.y) <= 40))
                    {
                        pidRun.ang = ChassisRunPoint.ang;
                        if(my_abs(G_Param.cur_pos.ang - pidRun.ang) <= 0.25f)
                        {
                            Chassis_Elmo_Stop();
                            ChassisStatus = RobotReady;
                            break;
                        }
                    }
                    break;
                case RobotAvoid:
					if(ticks_now > TimeOut)/* 超时:ChassisStatus->RoboReady */
                    {
                        Chassis_Elmo_Stop();
                        ChassisStatus = RobotReady;
						ChassisStatusChange = RobotReset;
                        break;
                    }
                    break;
                case RobotStop:
                    Chassis_Elmo_Stop();
                    ChassisStatus = RobotReady;
                    break;
                default:
                    break;
            }

            /* 边界抱死 ------------------------------------------------------*/
            if(   (G_Param.cur_pos.x > DEF_DP_LockXMax || G_Param.cur_pos.x < DEF_DP_LockXMin)
                ||(G_Param.cur_pos.y > DEF_DP_LockYMax || G_Param.cur_pos.y < DEF_DP_LockYMin))
            {
                DEF_SendPosOff();
                DEF_RecievePosOff();
                DmaSendWordQueue(0xee, 4, 0);
                if(SlidewayTaskHandle != 0 )    vTaskSuspend(SlidewayTaskHandle);
                if(HitTaskHandle != 0 )         vTaskSuspend(HitTaskHandle);
                if(BeepTaskHandle != 0 )        vTaskSuspend(BeepTaskHandle);
                BEEP_ON;
                Elmo_Stop(0);
                LCD_QueuePrintfErr(3,0,16, "DP边界");
                vTaskDelay(500*portTICK_MS);
                if(CheckTaskHandle != 0 )       vTaskSuspend(CheckTaskHandle);
                vTaskSuspend(ChassisTaskHandle);
            }
            else if(G_Param.cur_pos.ang > DEF_DP_LockAngMax || G_Param.cur_pos.ang < DEF_DP_LockAngMin)
            {
                DEF_SendPosOff();
                DEF_RecievePosOff();
                DmaSendWordQueue(0xee, 5, 0);
                if(SlidewayTaskHandle != 0 )    vTaskSuspend(SlidewayTaskHandle);
                if(HitTaskHandle != 0 )         vTaskSuspend(HitTaskHandle);
                if(BeepTaskHandle != 0 )        vTaskSuspend(BeepTaskHandle);
                Elmo_Stop(0);
                LCD_QueuePrintfErr(3,0,16, "DP角度边界");
                vTaskDelay(500*portTICK_MS);
                if(CheckTaskHandle != 0 )       vTaskSuspend(CheckTaskHandle);
                vTaskSuspend(ChassisTaskHandle);
            }
            if(lowPowerFlag)
            {
                DEF_SendPosOff();
                DEF_RecievePosOff();
                if(SlidewayTaskHandle != 0 )    vTaskSuspend(SlidewayTaskHandle);
                if(HitTaskHandle != 0 )         vTaskSuspend(HitTaskHandle);
                if(BeepTaskHandle != 0 )        vTaskSuspend(BeepTaskHandle);
                BEEP_ON;
                Elmo_Stop(0);
                vTaskDelay(500*portTICK_MS);
                if(CheckTaskHandle != 0 )       vTaskSuspend(CheckTaskHandle);
                vTaskSuspend(ChassisTaskHandle);
            }
            //Chassis_Elmo_Close();
            /* 计算PID并输出 -------------------------------------------------*/
            if(ChassisStatus != RobotReady)  /* Only Run/Reset/Avoid/FreeRun */
            {
                PidRunToPoint(pidRun.x, pidRun.y, pidRun.ang, 1); //底盘运行
                //Chassis_Elmo_Close();
            }
            else    /*底盘非运行状态，只运行PID(保存上次状态)，不控制底盘电机*/
            {
                PidRunToPoint(pidRun.x, pidRun.y, 0, 0);  
            }
        }
        else
        {
            DEF_SendPosOff();
            DEF_RecievePosOff();
            DmaSendWordQueue(0xee, 2, 0);
            if(SlidewayTaskHandle != 0 )    vTaskSuspend(SlidewayTaskHandle);
            if(HitTaskHandle != 0 )         vTaskSuspend(HitTaskHandle);
            if(BeepTaskHandle != 0 )        vTaskSuspend(BeepTaskHandle);
            BEEP_ON;
            Elmo_Stop(0);
            LCD_QueuePrintfErr(3,0,16, "DP GYRO ERR");
            vTaskDelay(500*portTICK_MS);
            if(CheckTaskHandle != 0 )       vTaskSuspend(CheckTaskHandle);
            vTaskSuspend(ChassisTaskHandle);
        }
    }
}

/* 底盘PID调节 ---------------------------------------------------------------*/
pid_t PidX, PidY, PidAng;
                        /*  Kp,         Ki,     Kd,         out_max,    out_min,    iteg_max,   dead_zone */
const pid_t PidXNormal = DEF_PID_PidXNormal;
const pid_t PidXReset  = DEF_PID_PidXReset;
const pid_t PidYNormal = DEF_PID_PidYNormal;
const pid_t PidYReset  = DEF_PID_PidYReset;

const pid_t PidAngNormal=DEF_PID_PidAngNormal;
const pid_t PidAngReset= DEF_PID_PidAngReset;
extern uint32_t servingFlag;
static void PidRunToPoint(int32_t x, int32_t y, float ang, uint32_t ifRun)
{
    static float cur_x,cur_y;      //当前位置在对角线坐标系
    static int32_t spd[4];
    static int32_t xOut,yOut,angOut,xOutAng,yOutAng;
    
    static int32_t max_spd;        //最大转速
    static float   percent;	    //缩放比

    PidX.set_value =  (x + y)*DEF_sqrt2_div2;
    PidY.set_value = (-x + y)*DEF_sqrt2_div2;

    cur_x = ( G_Param.cur_pos.x + G_Param.cur_pos.y)*DEF_sqrt2_div2;
    cur_y = (-G_Param.cur_pos.x + G_Param.cur_pos.y)*DEF_sqrt2_div2;
   
    xOut = pid_calc(&PidX, cur_x)* LINE_SPEED_CONVERT_RATIO;
    yOut = pid_calc(&PidY, cur_y)* LINE_SPEED_CONVERT_RATIO;
    //旋转角度解算 --------------------------------------------------------------------------------
    percent = ang2rad(G_Param.cur_pos.ang);
    if(0 && servingFlag)
    {
        xOutAng =  xOut*cosf(percent) + yOut*sinf(percent);
        yOutAng = -xOut*sinf(percent) + yOut*cosf(percent);
    }
    else
    {
        xOutAng = xOut;
        yOutAng = yOut;
    }
    spd[0] = xOutAng;
    spd[2] = spd[0];
    spd[1] = yOutAng;
    spd[3] = spd[1];
    
    //Angle ---------------------------------------------------------------------------------------
	PidAng.set_value = ang;
    angOut = pid_calc(&PidAng, G_Param.cur_pos.ang);
    angOut *= ROTATE_SPEED_CONVERT_RATIO;
    spd[0] -= angOut;
    spd[1] -= angOut;
    spd[2] += angOut;
    spd[3] += angOut;

    //OutPut ---------------------------------------------------------------------------------------
    //求得轮子速度的绝对值的最大值
    max_spd = my_max(my_max(my_max(my_abs(spd[0]), 
                                   my_abs(spd[1])), 
                            my_abs(spd[2])), 
                     my_abs(spd[3]));
    
    //限速
    if (max_spd > CHASSIS_MAX_SPEED)
    {
        //根据比例缩放电机速度
        percent = (float)CHASSIS_MAX_SPEED / (float)max_spd;
        spd[0] *= percent;
        spd[1] *= percent;
        spd[2] *= percent;
        spd[3] *= percent;
    }
    
    if(ifRun)
    {
        Chassis_Elmo_PVM(spd[0], spd[1], spd[2], spd[3]);
    }
}

/******************************** END OF FILE *********************************/
