/**
  ******************************************************************************
  * @file    SlidewayTask.c
  * @author  
  * @version V1.0
  * @date    7-May-2015
  * @brief   SlidewayTask
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "SlidewayTask.h"
#include "app.h"
#include "Global.h"
#include "MyMath.h"
#include "Com2Server.h"
#include "007_elmo.h"

/* Private constants ---------------------------------------------------------*/
                             /* Kp,         Ki,     Kd,         out_max,    out_min,    iteg_max,   dead_zone */
const pid_t PidSlideXNormal = DEF_PID_PidSlideXNormal;
const pid_t PidSlideXReset  = DEF_PID_PidSlideXReset;
const pid_t PidSlideYNormal = DEF_PID_PidSlideYNormal;
const pid_t PidSlideYReset  = DEF_PID_PidSlideYReset;

/* Private variables ---------------------------------------------------------*/
uint32_t SlidewayStatus = RobotReady;

uint32_t SlidewayStop = 0;  //����ʱֹͣ
pid_t PidSlideX, PidSlideY;
const float compensation_t = 0.045;

/**
  * @brief  vChassisTask
  * @param  None
  * @retval None
  */
extern uint8_t serve_reset_ready_flag;
void vSlidewayTask( void *p_arg )
{
    uint32_t ticks_now;
    uint32_t TimeOut;
    point_t pidRun;
    float x0,y0,x1,y1;

    (void)p_arg;
    /* PID��ʼ�� */
    memset(&PidSlideX,  0, sizeof(pid_t));
    memset(&PidSlideY,  0, sizeof(pid_t));
    
    memcpy(&PidSlideX, &PidSlideXNormal, 7*sizeof(float));  //�����Ʋ�������7����
    memcpy(&PidSlideY, &PidSlideYNormal, 7*sizeof(float)); 
	
    PidSlideX.real_value[LAST] = G_Param.slideway_x_mm;
    PidSlideY.real_value[LAST] = G_Param.slideway_y_mm;
    while(1)
    {
        if(xSemaphoreTake(PosSlideSem, 60) == pdTRUE) //��������5msһ�Σ���������6ms�����ϴ����������
        {
            ticks_now = xTaskGetTickCount();

            /* ״̬�л� ------------------------------------------------------*/
            if(SlidewayStatusChange != RobotReady)
            {
                if(SlidewayStatusChange == RobotRun)        //���л�������״̬
                {
                    #if (COM_DBG == COM_RELEASE)    //�ǵ���״̬
                    memcpy(&PidSlideX, &PidSlideXNormal, 7*sizeof(float));  //�����Ʋ�������7����
                    memcpy(&PidSlideY, &PidSlideYNormal, 7*sizeof(float));
                    TimeOut = xyzt_structure.absEndT;
                    SlidewayStop = 0;
                    SlidewayStatus = RobotRun;
                    
                    #else    //����״̬,�ڵ��Ի���PIDʱ�����Ʋ���
                    if(DbgChoose == COM_DBG_SLIDEWAY_X || DbgChoose == COM_DBG_SLIDEWAY_Y)
                    {
                        pidRun.x = xyzt_structure.x - DEF_DP_StartPosX;
                        pidRun.y = xyzt_structure.y - DEF_DP_StartPosY;
                        TimeOut = ticks_now + 5000*portTICK_MS;
                        SlidewayStop = 0;
                        SlidewayStatus = RobotFreeRun;
                    }
                    else
                    {
                        memcpy(&PidSlideX, &PidSlideXNormal, 7*sizeof(float));  //�����Ʋ�������7����
                        memcpy(&PidSlideY, &PidSlideYNormal, 7*sizeof(float));
                        TimeOut = xyzt_structure.absEndT;
                        SlidewayStop = 0;
                        SlidewayStatus = RobotRun;
                    }
                    #endif
                }
                else if(SlidewayStatusChange == RobotReset) //���л�����λ״̬
                {
                    memcpy(&PidSlideX, &PidSlideXReset, 7*sizeof(float));   //�����Ʋ�������7����
                    memcpy(&PidSlideY, &PidSlideYReset, 7*sizeof(float)); 
                    
                    pidRun.x = 0;
                    pidRun.y = 0;
                    TimeOut = ticks_now + 2000 * portTICK_MS;

                    SlidewayStatus = RobotReset;
                }
                else if(SlidewayStatusChange == RobotStop)                  //�յ�ֹͣ�ź�
                {
                    SlidewayStatus = RobotStop;
                }
                else if(SlidewayStatusChange == RobotFreeRun)
                {
                    memcpy(&PidSlideX, &PidSlideXReset, 7*sizeof(float));  //�����Ʋ�������7����
                    memcpy(&PidSlideY, &PidSlideYReset, 7*sizeof(float));

                    memcpy(&pidRun, &SlidewayRunPoint, 2*sizeof(int32_t));
                    TimeOut = ticks_now + SlidewayRunPoint.tim * portTICK_MS;
                    
                    SlidewayStatus = RobotFreeRun;
                }
                else if(SlidewayStatusChange == RobotTest)
                {
                    memcpy(&PidSlideX, &PidSlideXNormal, 7*sizeof(float));  //�����Ʋ�������7����
                    memcpy(&PidSlideY, &PidSlideYNormal, 7*sizeof(float));
                    
                    memcpy(&pidRun, &SlidewayRunPoint, 2*sizeof(int32_t));
                    TimeOut = ticks_now + SlidewayRunPoint.tim * portTICK_MS;
                    
                    SlidewayStatus = RobotTest;
                }
                
                SlidewayStatusChange = RobotReady;
            }

            /* ���ݵ�ǰ״̬������Ӧ-------------------------------------------*/
            switch(SlidewayStatus)
            {
                case RobotRun:  /* ���� --------------------------------------*/
                    if (SlidewayStop != 0 || ticks_now > TimeOut)  //����յ�ֹͣ�źŻ��߳�ʱ
                    {
                        Slideway_Elmo_Stop();
                        SlidewayStatus = RobotReady;
                        break;
                    }
                    if(ticks_now > xyzt_structure.absEndT - (400*portTICK_MS))  //ʱ���ڵ�����
                    {
                        if(RacketReceive.dir == RT_NO)
                        {
                            pidRun.x = 0;
                            pidRun.y = 0;
                        }
                        else
                        {
                            x0 = (xyzt_structure.x - G_Param.cur_pos.x - compensation_t*G_Param.speed.x);
                            y0 = (xyzt_structure.y - G_Param.cur_pos.y - compensation_t*G_Param.speed.y);

                            x1 = (x0 * my_cos(G_Param.cur_pos.ang) + y0 * my_sin(G_Param.cur_pos.ang));
                            y1 = (y0 * my_cos(G_Param.cur_pos.ang) - x0 * my_sin(G_Param.cur_pos.ang));
                            
                            pidRun.x = x1;
                            pidRun.y = y1;
                        }
                    }
                    break;
                    
                case RobotReset: /* ��λ -------------------------------------*/
                    if( my_abs(G_Param.slideway_x_mm) < 3 && my_abs(G_Param.slideway_y_mm) < 5)
                    {
                        Slideway_Elmo_Stop();
                        SlidewayStatus = RobotReady;
                        break;
                    }
                    if (ticks_now > TimeOut)    /* ��ʱ:SlidewayStatus->RoboReady */
                    {
                        Slideway_Elmo_Stop();
                        SlidewayStatus = RobotReady;
                        break;
                    }
                    break;
                    
                case RobotStop:
                    Slideway_Elmo_Stop();
                    SlidewayStatus = RobotReady;
                    break;
                
                case RobotFreeRun:
                    if( my_abs(G_Param.slideway_x_mm - pidRun.x) <= 1 && my_abs(G_Param.slideway_y_mm - pidRun.y) <= 1)
                    {
                        Slideway_Elmo_Stop();
                        SlidewayStatus = RobotReady;
                        break;
                    }
                    if (ticks_now > TimeOut)   /* ��ʱ:SlidewayStatus->RoboReady */
                    {
                        Slideway_Elmo_Stop();
                        SlidewayStatus = RobotReady;
                        break;
                    }
                    break;
                    
                case RobotTest:
                    if( my_abs(G_Param.slideway_x_mm - pidRun.x) < 3 && my_abs(G_Param.slideway_y_mm - pidRun.y) < 3)
                    {
                        Slideway_Elmo_Stop();
                        SlidewayStatus = RobotReady;
                        break;
                    }
                    if (ticks_now > TimeOut)   /* ��ʱ:SlidewayStatus->RoboReady */
                    {
                        Slideway_Elmo_Stop();
                        SlidewayStatus = RobotReady;
                        break;
                    }
                    break;

                default:
                    break;
            }
            
            /* �߽��޷� ------------------------------------------------------*/
            if(SlidewayStatus != RobotFreeRun)
            {
                if(pidRun.x > DEF_HG_BoundaryXMax)
                    pidRun.x = DEF_HG_BoundaryXMax;
                else if(pidRun.x < DEF_HG_BoundaryXMin)
                    pidRun.x = DEF_HG_BoundaryXMin;
                if(pidRun.y > DEF_HG_BoundaryYMax)
                    pidRun.y = DEF_HG_BoundaryYMax;
                else if(pidRun.y < DEF_HG_BoundaryYMin)
                    pidRun.y = DEF_HG_BoundaryYMin;
            }
            else
            {
                if(pidRun.x > DEF_HG_BoundaryXMax+15)
                    pidRun.x = DEF_HG_BoundaryXMax+15;
                else if(pidRun.x < DEF_HG_BoundaryXMin-15)
                    pidRun.x = DEF_HG_BoundaryXMin-15;
                if(pidRun.y > DEF_HG_BoundaryYMax+15)
                    pidRun.y = DEF_HG_BoundaryYMax+15;
                else if(pidRun.y < DEF_HG_BoundaryYMin-15)
                    pidRun.y = DEF_HG_BoundaryYMin-15;
            }
            
            PidSlideX.set_value = pidRun.x;
            PidSlideY.set_value = pidRun.y;
            pid_calc(&PidSlideX, G_Param.slideway_x_mm);
            pid_calc(&PidSlideY, G_Param.slideway_y_mm);
            
            /*�����촦������״̬���߽��޷�&����PID&���Ƶ��-------------------*/
            if(SlidewayStatus != RobotReady)    /* Only Run/Reset/FreeRun */
            {  
                XY_Elmo_PVM(PidSlideX.pid_out, PidSlideY.pid_out);
            }
            /*���������״̬��ֻ����PID(�����ϴ�״̬)�������ƻ�����*/
        }
        else    /* 6msδ���յ������������� -----------------------------------*/
        {
            DEF_SendPosOff();
            DEF_RecievePosOff();
            DmaSendWordQueue(0xee, 3, 0);
            if(ChassisTaskHandle != 0 )     vTaskSuspend(ChassisTaskHandle);
            if(HitTaskHandle != 0 )         vTaskSuspend(HitTaskHandle);
            if(BeepTaskHandle != 0 )        vTaskSuspend(BeepTaskHandle);
            BEEP_ON;
            Elmo_Stop(0);
            LCD_QueuePrintf(3,0,16, "HG GYRO ERR");
            vTaskDelay(500*portTICK_MS);
            if(CheckTaskHandle != 0 )       vTaskSuspend(CheckTaskHandle);
            vTaskSuspend(SlidewayTaskHandle);
        }
    }
}


void SlidewayHoming(void)
{
    uint8_t xFlag = 0, yFlag = 0;
    
    const float test_t = 0.5f;
    float enc_x[2] = {0, 0}, enc_y[2] = {0, 0};
    float enc_dx, enc_dy;
    float acc, acc_t, acc_x, all_x;
    
    //���Ѿ�ɨ��������
    if(READ_SLIDEWAY_X_RESET() == 0)
    {
        X_Elmo_PVM(-300);
        while(READ_SLIDEWAY_X_RESET() == 0)
            vTaskDelay(portTICK_MS);
        vTaskDelay(200*portTICK_MS);
        X_Elmo_Stop();
    }
    if(READ_SLIDEWAY_Y_RESET() == 0)
    {
        Y_Elmo_PVM(300);
        while(READ_SLIDEWAY_Y_RESET() == 0)
            vTaskDelay(portTICK_MS);
        vTaskDelay(200*portTICK_MS);
        Y_Elmo_Stop();
    }
    vTaskDelay(100*portTICK_MS);
    
    //��λ
    xFlag = 0;
    yFlag = 0;
    XY_Elmo_PVM(300, -300);
    //vTaskDelay(100*portTICK_MS);
    while(1)
    {
        if(xFlag != 1 && READ_SLIDEWAY_X_RESET() == 0)
        {
            SetSlidewayX_mm(DEF_HG_InitPosX);
            xFlag = 1;
            X_Elmo_Stop();
        }
        if(yFlag != 1 && READ_SLIDEWAY_Y_RESET() == 0)
        {
            SetSlidewayY_mm(DEF_HG_InitPosY);
            yFlag = 1;
            Y_Elmo_Stop();
        }
        if(xFlag == 1 && yFlag == 1)
        {
            break;
        }
        vTaskDelay(1*portTICK_MS);
    }
    
    /* ���������Ƿ���ȷ */
    if( 0 !=  Elmo_Read_PositionActual(MOTOR_ID_SLIDEWAY_X, NULL))
        while(1);
    if( 0 !=  Elmo_Read_PositionActual(MOTOR_ID_SLIDEWAY_Y, NULL))
        while(1);
    enc_x[0] = ReadSlidewayX_mm();
    enc_y[0] = ReadSlidewayY_mm();
    
    XY_Elmo_PVM(-200, 200);
    vTaskDelay(test_t*1000* portTICK_MS);
    XY_Elmo_PVM(0, 0);

    acc = 200000.0f/MOTOR_MM2CNT_SLIDEWAY_X;
    acc_t = 200.0f / acc;
    if(acc_t < test_t)
    {
        acc_x = (200*200)/(2*acc);
        all_x = acc_x + (test_t - acc_t)*200;
    }
    else
    {
        all_x = 0.5f * acc * test_t*test_t;
    }
    
    if( 0 !=  Elmo_Read_PositionActual(MOTOR_ID_SLIDEWAY_X, NULL))
        while(1);
    if( 0 !=  Elmo_Read_PositionActual(MOTOR_ID_SLIDEWAY_Y, NULL))
        while(1);
    enc_x[1] = ReadSlidewayX_mm();
    enc_y[1] = ReadSlidewayY_mm();
    
    enc_dx = -(enc_x[1] - enc_x[0]);
    enc_dy = enc_y[1] - enc_y[0];
    
    if(!(
           fabsf(enc_dx - all_x) < 10.0f
        && fabsf(enc_dy - all_x) < 10.0f
        && fabsf(enc_dx - enc_dy) < 5.0f))
    {
        Elmo_Close(0);
        Buzzer(3, 50, 50);
        while(1) vTaskDelay(portTICK_MS);
    }
    
}
/**
  * @brief  ���û����ٶ�
  * @param  float speedX (mm/s)
  *         float speedY (mm/s)
  * @retval None
  */
void XY_Elmo_PVM(float speedX, float speedY)
{
    int32_t spd[2];
    spd[0] = speedX*MOTOR_MM2CNT_SLIDEWAY_X;
    spd[1] = speedY*MOTOR_MM2CNT_SLIDEWAY_Y;
    
    if(spd[0] > MOTOR_MAXSPEED_SLIDEWAY_X)
        spd[0] = MOTOR_MAXSPEED_SLIDEWAY_X;
    else if(spd[0] < -MOTOR_MAXSPEED_SLIDEWAY_X)
        spd[0] = -MOTOR_MAXSPEED_SLIDEWAY_X;
    
    if(spd[1] > MOTOR_MAXSPEED_SLIDEWAY_Y)
        spd[1] = MOTOR_MAXSPEED_SLIDEWAY_Y;
    else if(spd[1] < -MOTOR_MAXSPEED_SLIDEWAY_Y)
        spd[1] = -MOTOR_MAXSPEED_SLIDEWAY_Y;

    Slideway_Elmo_PVM(spd[0], spd[1]);
}
/******************************** END OF FILE *********************************/
