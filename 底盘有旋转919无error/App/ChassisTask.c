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
    
    /* PID��ʼ�� */
    memset(&PidX, 0,sizeof(PidX));
    memset(&PidY, 0,sizeof(PidY));
    memset(&PidAng, 0,sizeof(PidAng));
    
    memcpy(&PidX,   &PidXNormal,    7*sizeof(float));  //�����Ʋ�������7����
    memcpy(&PidY,   &PidYNormal,    7*sizeof(float)); 
    memcpy(&PidAng, &PidAngNormal,  7*sizeof(float)); 

	PidX.real_value[LAST] = ( G_Param.cur_pos.x + G_Param.cur_pos.y)*DEF_sqrt2_div2;
    PidY.real_value[LAST] = (-G_Param.cur_pos.x + G_Param.cur_pos.y)*DEF_sqrt2_div2;
    PidAng.real_value[LAST] = G_Param.cur_pos.ang;
    while(1)
    {
        if(xSemaphoreTake(PosChassisSem, 60) == pdTRUE)       //��������5msһ�Σ���������6ms
        {
            ticks_now = xTaskGetTickCount();

            /* ״̬�л� ------------------------------------------------------*/
            if(ChassisStatusChange != RobotReady)
            {
                if(ChassisStatusChange == RobotRun)                     //���л�������״̬
                {
                    /* ����Ŀ��� --------------------------------------------*/
                    pidRun.x = xyzt_structure.x ;
                    pidRun.y = xyzt_structure.y ;
                    pidRun.ang = 0;

                    /* �ı�PID���� --------------------------------------------*/
                #if (COM_DBG == COM_RELEASE)    //�ǵ���״̬
                    #if (CAR_SELECT == 1 || 1)
                    
                    /*���ٵ����ܶ���Χ*/
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
                        
                    memcpy(&PidX,   &PidXNew[PidXSelected], 7*sizeof(float));   //�����Ʋ�������7����
                    memcpy(&PidY,   &PidYNew[PidYSelected], 7*sizeof(float));
                    memcpy(&PidAng, &PidAngNormal,  7*sizeof(float));
                    TimeOut = xyzt_structure.absEndT;
                    #else
                    memcpy(&PidX,   &PidXNormal,    7*sizeof(float));   //�����Ʋ�������7����
                    memcpy(&PidY,   &PidYNormal,    7*sizeof(float));
                    memcpy(&PidAng, &PidAngNormal,  7*sizeof(float));
                    TimeOut = xyzt_structure.absEndT;
                    #endif
                #else       //�����Ե���PID�������Ʋ���
                    if(DbgChoose == COM_DBG_CHASSIS_X || DbgChoose == COM_DBG_CHASSIS_Y)
                    {
                        TimeOut = xTaskGetTickCount() + 5000*portTICK_MS;    /**@Note����5�볬ʱ*/
                    }
                    else
                    {
                        memcpy(&PidX,   &PidXNormal,    7*sizeof(float));   //�����Ʋ�������7����
                        memcpy(&PidY,   &PidYNormal,    7*sizeof(float));
                        memcpy(&PidAng, &PidAngNormal,  7*sizeof(float));
                        TimeOut = xyzt_structure.absEndT;
                    }
                #endif

                    ChassisStop = 0;
                    ChassisStatus = RobotRun;
                }
                else if(ChassisStatusChange == RobotReset)      //���л�����λ״̬
                {
                    if(ChassisStatus != RobotReset)
                    {
                        memcpy(&PidX,   &PidXReset,     7*sizeof(float));   //�����Ʋ�������7����
                        memcpy(&PidY,   &PidYReset,     7*sizeof(float));
                        memcpy(&PidAng, &PidAngReset,   7*sizeof(float));
                    }
                    
                    pidRun.x = ChassisResetPoint.x;
                    pidRun.y = ChassisResetPoint.y;
                    pidRun.ang = 0;
                    TimeOut = ticks_now + 5000*portTICK_MS;
                    ChassisStatus = RobotReset;
                }
                else if(ChassisStatusChange == RobotStop)       //���̱���
                {
                    ChassisStatus = RobotStop;
                }
                else if(ChassisStatusChange == RobotFreeRun || ChassisStatusChange == RobotAvoid)    //��������ChassisRunPoint
                {
					if(ChassisStatus != ChassisStatusChange)
					{
						memcpy(&PidX,   &PidXNormal,    7*sizeof(float));   //�����Ʋ�������7����
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

                /* �߽��޷� --------------------------------------------------*/
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

            /* ���ݵ�ǰ״̬������Ӧ ------------------------------------------*/
            switch(ChassisStatus)
            {
                case RobotRun:  /* ���� --------------------------------------*/
                    if(ChassisStop != 0)
                    {
                        Chassis_Elmo_Stop();
                        ChassisStatus = RobotReady;
                        break;
                    }
                    if(ticks_now > TimeOut)/* ��ʱ:ChassisStatus->RoboReady */
                    {
                        Chassis_Elmo_Stop();
                        ChassisStatus = RobotReady;
                        break;
                    }
                    break;

                case RobotReset: /* ��λ -------------------------------------*/
                    if ((my_abs(G_Param.cur_pos.x - pidRun.x) <= 5) &&
                        (my_abs(G_Param.cur_pos.y - pidRun.y) <= 5) &&
                        (my_abs(G_Param.cur_pos.ang - 0) <= 0.25f))
                    {
                        Chassis_Elmo_Stop();
                        ChassisStatus = RobotReady;
                        break;
                    }
                    if(ticks_now > TimeOut)/* ��ʱ:ChassisStatus->RoboReady */
                    {
                        Chassis_Elmo_Stop();
                        ChassisStatus = RobotReady;
                        break;
                    }
                    break;
                    
                case RobotFreeRun:
                    if(ticks_now > TimeOut)/* ��ʱ:ChassisStatus->RoboReady */
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
					if(ticks_now > TimeOut)/* ��ʱ:ChassisStatus->RoboReady */
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

            /* �߽籧�� ------------------------------------------------------*/
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
                LCD_QueuePrintfErr(3,0,16, "DP�߽�");
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
                LCD_QueuePrintfErr(3,0,16, "DP�Ƕȱ߽�");
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
            /* ����PID����� -------------------------------------------------*/
            if(ChassisStatus != RobotReady)  /* Only Run/Reset/Avoid/FreeRun */
            {
                PidRunToPoint(pidRun.x, pidRun.y, pidRun.ang, 1); //��������
                //Chassis_Elmo_Close();
            }
            else    /*���̷�����״̬��ֻ����PID(�����ϴ�״̬)�������Ƶ��̵��*/
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

/* ����PID���� ---------------------------------------------------------------*/
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
    static float cur_x,cur_y;      //��ǰλ���ڶԽ�������ϵ
    static int32_t spd[4];
    static int32_t xOut,yOut,angOut,xOutAng,yOutAng;
    
    static int32_t max_spd;        //���ת��
    static float   percent;	    //���ű�

    PidX.set_value =  (x + y)*DEF_sqrt2_div2;
    PidY.set_value = (-x + y)*DEF_sqrt2_div2;

    cur_x = ( G_Param.cur_pos.x + G_Param.cur_pos.y)*DEF_sqrt2_div2;
    cur_y = (-G_Param.cur_pos.x + G_Param.cur_pos.y)*DEF_sqrt2_div2;
   
    xOut = pid_calc(&PidX, cur_x)* LINE_SPEED_CONVERT_RATIO;
    yOut = pid_calc(&PidY, cur_y)* LINE_SPEED_CONVERT_RATIO;
    //��ת�ǶȽ��� --------------------------------------------------------------------------------
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
    //��������ٶȵľ���ֵ�����ֵ
    max_spd = my_max(my_max(my_max(my_abs(spd[0]), 
                                   my_abs(spd[1])), 
                            my_abs(spd[2])), 
                     my_abs(spd[3]));
    
    //����
    if (max_spd > CHASSIS_MAX_SPEED)
    {
        //���ݱ������ŵ���ٶ�
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
