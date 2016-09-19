/**
  ******************************************************************************
  * @file    HitTask.c
  * @author  
  * @version V1.0
  * @date    7-May-2015
  * @brief   HitTask
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "HitTask.h"
#include "app.h"
#include "bsp.h"
#include "Com2Server.h"

#include "MyMath.h"
#include "RacketsConfig.h"

Racket_TypeDef RacketReceive;   //接收到的球拍选择
Racket_TypeDef CurRacket;       //当前已选球拍

/* Private constants ---------------------------------------------------------*/
int32_t hitMainEncOff = 0;
int32_t hitServeEncOff = 0;

/* Private types -------------------------------------------------------------*/
enum HitStatusEnum
{
    HitReset = 0,   //复位完成
    HitPrepareReady, //拍子准备完成
    HitErr
};

/* Private variables ---------------------------------------------------------*/
int16_t hitting = -16383;  //用于绘图标出挥拍过程
uint32_t tFirst = 0;
/* Private function prototypes -----------------------------------------------*/
static void main_racket_hit(void);
static void left_racket_hit(void);
static void right_racket_hit(void);
void AllRacketsHoming(void);
void RacketRun2Ang(uint32_t id, float spd, float Ang, uint32_t timeOut);
/* Private functions ---------------------------------------------------------*/
void SendRacketPosErr(void);

/**
  * @brief  vHitTask,挥拍中间过程不可以换拍，右拍若没有到位则需重启
  * @param  None
  * @retval None
  */
void vHitTask( void *p_arg )
{
    Racket_TypeDef HitReceive;
    (void)p_arg;
    
    while(1)
    {
        if(xQueueReceive( HitInQueue, &HitReceive, portMAX_DELAY ) == pdTRUE)
        {   
            if(HitReceive.flag_time_to_hit == 1)    //如果挥拍时间到
            {
                if(CurRacket.dir == RT_MAIN)
                    main_racket_hit();
                else if(CurRacket.dir == RT_LEFT)
                    left_racket_hit();
                else if(CurRacket.dir == RT_RIGHT)
                    right_racket_hit();
                else if(CurRacket.dir == RT_NO)
                    ;

                CurRacket.dir  = RT_NO;
                SlidewayStatusChange = RobotReset;
                ChassisStatusChange = RobotReset;
                xyzt_structure.running_flag = 0;
                tFirst = 0;
            }
            else
            {
                if(HitReceive.dir != CurRacket.dir)   /*如果要换挥拍的方向*/
                {
                    /*如果之前未选拍，则说明主拍未下去*/
                    if(CurRacket.dir == RT_NO)
                    {
                        /*不管选什么拍，主拍都应下去*/
                        RacketRun2Ang(MOTOR_ID_HIT_MAIN, -0.35f, RACKET_MAIN_SENSOR_ANG-0.15f, my_abs(RACKET_MAIN_RESET_ANG*1000/(360.0f*0.35f)) + 50);
                        vTaskDelay(25*portTICK_MS);
                        /*如果选择左右拍，则左右拍挥到0°*/
                        if( HitReceive.dir == RT_LEFT || HitReceive.dir == RT_RIGHT)
                        {
                            RacketRun2Ang(MOTOR_ID_HIT_SERVE, 0.5f, 0.0f, my_abs(RACKET_SERVE_RESET_ANG*1000/(360.0f*0.5f)) + 50);
                        }
                    }
                    
                    /*如果之前选的是侧拍*/
                    else if( CurRacket.dir == RT_LEFT || CurRacket.dir == RT_RIGHT)
                    {
                        if(!(HitReceive.dir == RT_LEFT || HitReceive.dir == RT_RIGHT))
                        {
                            /*侧拍挥到复位状态：RACKET_SERVE_RESET_ANG*/
                            RacketRun2Ang(MOTOR_ID_HIT_SERVE, -0.5f, RACKET_SERVE_RESET_ANG, my_abs(RACKET_SERVE_RESET_ANG*1000/(360.0f*0.5f)) + 50);
                            vTaskDelay(15*portTICK_MS);
                        }
                        if(HitReceive.dir == RT_NO) //如果选择不挥拍，则主拍也挥到复位状态
                        {
                            RacketRun2Ang(MOTOR_ID_HIT_MAIN, 0.5f, RACKET_MAIN_RESET_ANG, my_abs(RACKET_MAIN_RESET_ANG*1000/(360.0f*0.5f)) + 50);
                        }
                    }
                    
                    /* 如果之前选的是主拍 */
                    else if(CurRacket.dir == RT_MAIN)
                    {
                        /*如果选择左右拍，则左右拍挥到0°*/
                        if( HitReceive.dir == RT_LEFT || HitReceive.dir == RT_RIGHT)
                        {
                            RacketRun2Ang(MOTOR_ID_HIT_SERVE, 0.5f, 0.0f, my_abs(RACKET_SERVE_RESET_ANG*1000/(360.0f*0.5f)) + 50);
                        }
                        if(HitReceive.dir == RT_NO) //如果选择不挥拍，则主拍也挥到复位状态
                        {
                            RacketRun2Ang(MOTOR_ID_HIT_MAIN, 0.5f, RACKET_MAIN_RESET_ANG, my_abs(RACKET_MAIN_RESET_ANG*1000/(360.0f*0.5f)) + 50);
                        }
                    }

                    CurRacket = HitReceive;
                }
            }
        }
    }
}

/* Private variables ---------------------------------------------------------*/
/* 所有拍子复位----------------------------------------------------------------*/
extern int16_t currentActual_Main;
void AllRacketsHoming(void)
{
    int32_t enc;
    uint32_t TimeOut;

    int32_t enc_0, enc_1;
    int32_t resetFlag = 0;
    int32_t velocity = 0;
    int32_t stallFlag = 0, stallStartTim = 0;

    /* 主拍找零 ---------------------------------------------------------------*/
    TimeOut = xTaskGetTickCount() + 5000*portTICK_MS;
    HIT_MAIN_PVM(-0.3f);
    //vTaskDelay(50*portTICK_MS);
    while(1)
    {
        Elmo_Read_CurrentActual(MOTOR_ID_HIT_MAIN);
        Elmo_Read_VelocityActual(MOTOR_ID_HIT_MAIN, &velocity);
        if(fabsf(currentActual_Main) > 50 && fabsf(velocity) < 500)  //电流大于持续电流的50%，速度小于500cnt/s
        {
            if(stallFlag)
            {
                if(xTaskGetTickCount() > stallStartTim + 200*portTICK_MS)
                    break;
            }
            else
            {
                stallFlag = 1;
                stallStartTim = xTaskGetTickCount();
            }
        }
        else
            stallFlag = 0;
        
        if(xTaskGetTickCount() > TimeOut)
            break;
        vTaskDelay(portTICK_MS/2);
    }
    enc = xTaskGetTickCount()  + 5000*portTICK_MS - TimeOut;
    HIT_MAIN_STOP();
    vTaskDelay(50*portTICK_MS);
    HIT_MAIN_CLOSE();
    vTaskDelay(1000*portTICK_MS);
    HIT_MAIN_STOP();
    
    if(Elmo_Read_POS(MOTOR_ID_HIT_MAIN, &enc) != 0 )
        while(1);
    else
    {
        hitMainEncOff = -enc + RACKET_MAIN_SENSOR_ANG/(MOTOR_DIR_HIT_MAIN * MOTOR_CNT2ANG_HIT_MAIN);
    }
    
    /* 侧拍找零 ---------------------------------------------------------------*/
    //先判断是否扫到传感器
    if(READ_RACKET_R_RESET() == 0)
    {
        HIT_SERVE_PVM(-0.5f);
        while(READ_RACKET_R_RESET() == 0) vTaskDelay(50*portTICK_MS);;
        vTaskDelay(300*portTICK_MS);
    }
    
    //再归位
    HIT_SERVE_PVM(0.5f);

    while(1)
    {
        if(resetFlag == 0)      //还未检测到传感器.等待检测到传感器
        {
            if(READ_RACKET_R_RESET() == 0) //检测到传感器
            {
                if(Elmo_Read_POS(MOTOR_ID_HIT_SERVE, &enc_0) == 0 )
                {
                    resetFlag = 1;
                }
                else
                {
                    //BEEP_ON;
                    LCD_QueuePrintf(3, 0, 16, "ERR:S_POS");
                }
            }
        }
        else if(resetFlag == 1) //已检测到传感器，等待扫过传感器
        {
            if(READ_RACKET_R_RESET() != 0) //未检测到传感器(已走过传感器)
            {
                if(Elmo_Read_POS(MOTOR_ID_HIT_SERVE, &enc_1) == 0 )
                {
                    hitServeEncOff = -(enc_0 + enc_1)/2 + RACKET_SERVE_SENSOR_ANG/(MOTOR_DIR_HIT_SERVE * MOTOR_CNT2ANG_HIT_SERVE);
                }
                else
                {
                    //BEEP_ON;
                    LCD_QueuePrintf(3, 0, 16, "ERR:S_POS");
                }
                break;
            }
        }
        vTaskDelay(portTICK_MS/3);
    }
    HIT_SEREVE_STOP();
    
    /* 侧拍复位 ---------------------------------------------------------------*/
    RacketRun2Ang(MOTOR_ID_HIT_SERVE, 0.3f, RACKET_SERVE_RESET_ANG, 1000);
    
    /* 主拍复位 ---------------------------------------------------------------*/
    RacketRun2Ang(MOTOR_ID_HIT_MAIN, 0.3f, RACKET_MAIN_RESET_ANG, 2000);
}
double speedArrFromIndexToReal(int index)
{
    switch (index)
    {
    case 0:return 3.0;
    case 1:return 3.5;
    case 2:return 4.0;
    case 3:return 4.5;
    case 4:return 5.0;
    case 5:return 5.5;
    case 6:return 6.0;
    default:return 0;
    }
}
/* 挥拍-----------------------------------------------------------------------*/
static void main_racket_hit(void)
{
    hitting = 16383;
    //挥拍&拍停
    RacketRun2Ang(MOTOR_ID_HIT_MAIN, speedArrFromIndexToReal(CurRacket.spd), CurRacket.ang + 20.0f, 120);

    //停止底盘和滑轨
    ChassisStop = 1;
    SlidewayStop = 1;
    hitting = -16383;
    SendRacketPosErr();
    vTaskDelay(100 * portTICK_MS);
    
    //球拍复位&拍停
    RacketRun2Ang(MOTOR_ID_HIT_MAIN, -0.5f, RACKET_MAIN_RESET_ANG-6.5f, 1000);
}


static void right_racket_hit(void)
{   
    hitting = 16383;
     
    //挥拍
    RacketRun2Ang(MOTOR_ID_HIT_SERVE, speedArrFromIndexToReal(CurRacket.spd), -CurRacket.ang - 20.0f, 120);

    //停止底盘和滑轨
    ChassisStop = 1;
    SlidewayStop = 1;
    hitting = -16383;
    SendRacketPosErr();
    
    //主拍抬起
    RacketRun2Ang(MOTOR_ID_HIT_MAIN, 0.5f, RACKET_MAIN_RESET_ANG, 1000);
    
    //拍停
    vTaskDelay(100 * portTICK_MS);

    //主拍下去
    RacketRun2Ang(MOTOR_ID_HIT_MAIN, 0.3f, RACKET_MAIN_SENSOR_ANG, 1000);
    
    vTaskDelay(100 * portTICK_MS);
    
    //侧拍复位
    RacketRun2Ang(MOTOR_ID_HIT_SERVE, 1.0f, RACKET_SERVE_RESET_ANG, 2000);
    
    //主拍复位
    RacketRun2Ang(MOTOR_ID_HIT_MAIN, 0.5f, RACKET_MAIN_RESET_ANG, 1000); 
}

static void left_racket_hit(void)  /* 左侧挥拍复位和主拍无干涉，可省略步骤*/
{   
    hitting = 16383;
    
    //挥拍
    RacketRun2Ang(MOTOR_ID_HIT_SERVE, speedArrFromIndexToReal(CurRacket.spd), CurRacket.ang + 20.0f, 120);

    //停止底盘和滑轨
    ChassisStop = 1;
    SlidewayStop = 1; 
    hitting = -16383;
    SendRacketPosErr();
    
    //主拍抬起
    RacketRun2Ang(MOTOR_ID_HIT_MAIN, 0.5f, RACKET_MAIN_RESET_ANG, 1000);
    
    //拍停
    vTaskDelay(100 * portTICK_MS);
    
    //球拍复位
    RacketRun2Ang(MOTOR_ID_HIT_SERVE, 1.0f, RACKET_SERVE_RESET_ANG, 2000);
    
    //主拍复位
    //RacketRun2Ang(MOTOR_ID_HIT_MAIN, 0.5f, RACKET_MAIN_RESET_ANG, 1000);
}

/**
  * @brief  TIM5定时器初始化，产生中断触发击球
  * @param  PPr：抢占优先级
  *         SPr：响应优先级
  * @retval None
  */
void TIM5_init(uint8_t PPr, uint8_t SPr)
{
    TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
    NVIC_InitTypeDef          NVIC_InitStructure;

    /* TIM5 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 , ENABLE);

    /* Enable the TIM5 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PPr;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = SPr;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Time base configuration (TIM5 clocked at 84 MHz)*/
    TIM_TimeBaseStructure.TIM_Period = 65535;
    TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;   //100us
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;           

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 

    /* TIM5 IT enable */
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

    /* TIM5 enable counter */
    TIM_Cmd(TIM5, DISABLE);
}

/**
  * @brief  TIM5减计数溢出中断
  * @param  None
  * @retval None
  */
void TIM5_IRQHandler(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    if(TIM_GetITStatus(TIM5,TIM_IT_Update) != RESET)
    {
        RacketReceive.flag_time_to_hit = 1;
        xQueueOverwriteFromISR(HitInQueue, &RacketReceive, &xHigherPriorityTaskWoken);
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
        TIM_Cmd(TIM5, DISABLE);
    }
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

/**
  * @brief  LCD显示挥拍时误差
  * @param  None
  * @retval None
  */
void SendRacketPosErr(void)
{
    int32_t racketx0, rackety0;
    int32_t racketx, rackety;

    racketx0 = G_Param.slideway_x_mm;
    rackety0 = G_Param.slideway_y_mm;
    racketx = racketx0*my_cos(G_Param.cur_pos.ang) - rackety0*my_sin(G_Param.cur_pos.ang) + G_Param.cur_pos.x;
    rackety = racketx0*my_sin(G_Param.cur_pos.ang) + rackety0*my_cos(G_Param.cur_pos.ang) + G_Param.cur_pos.y;
    racketx = xyzt_structure.x - racketx;
    rackety = xyzt_structure.y - rackety;
    LCD_QueuePrintf(2, 0, 16, "x:%5d  y:%5d", racketx, rackety);
    LCD_QueuePrintf(3, 0, 16, "t:%5d  a:%f", tFirst, G_Param.cur_pos.ang);
    //LCD_QueuePrintf(3, 0, 16, "x:%5d  y:%5d", xyzt_structure.x_orl, xyzt_structure.y_orl);
}

/**
  * @brief  读取拍子编码器并加上起始偏移，再转换为角度
  * @param  id仅支持MOTOR_ID_HIT_MAIN MOTOR_ID_HIT_SERVE pAng为浮点角度指针
  * @retval 0：成功
  *         1：失败
  */
uint32_t ReadRacketAngWithOff(uint32_t id, float *pAng)
{
    int32_t enc;
    if(id == MOTOR_ID_HIT_MAIN)
    {
        if(Elmo_Read_POS(MOTOR_ID_HIT_MAIN, &enc) !=0)
            return 1;
        else
        {
            *pAng = (enc + hitMainEncOff)*MOTOR_DIR_HIT_MAIN*MOTOR_CNT2ANG_HIT_MAIN;
            return 0;
        }
    }
    else if(id == MOTOR_ID_HIT_SERVE)
    {
        if(Elmo_Read_POS(MOTOR_ID_HIT_SERVE, &enc) !=0)
            return 1;
        else
        {
            *pAng = (enc + hitServeEncOff)*MOTOR_DIR_HIT_SERVE*MOTOR_CNT2ANG_HIT_SERVE;
            return 0;
        }
    }
    else
        return 1;
}

/**
  * @brief  拍子在超时时间内以指定速度挥到指定角度，容差1°
  * @param  id：    仅支持MOTOR_ID_HIT_MAIN MOTOR_ID_HIT_SERVE
  *         spd：   速度
  *         Ang：   绝对角度，顺时针为正方向
  *         timeOut：相对超时时间
  * @retval 0：成功
  *         1：失败
  */
void RacketRun2Ang(uint32_t id, float spd, float Ang, uint32_t timeOut)
{
    uint32_t TimOut;
    float angTmp;
    TimOut = xTaskGetTickCount() + timeOut*portTICK_MS;
    
    if(id == MOTOR_ID_HIT_MAIN)
    {
        ReadRacketAngWithOff(MOTOR_ID_HIT_MAIN, &angTmp);
        
        if(Ang > 90.0f)         Ang = 90.0f;    //Main挥拍限幅
        
        /* 确定挥拍方向，并判断是否在容差范围内不挥拍 */
        if(Ang - angTmp >= 0)
        {
            spd = my_abs(spd);
            if(angTmp - Ang >  - 0.15f)     return;
        }
        else
        {
            spd = -my_abs(spd);
            if(angTmp - Ang < 0.15f)        return;
        }
        
        /* 开始挥拍 */
        HIT_MAIN_PVM(spd);
        while(1)
        {
            ReadRacketAngWithOff(MOTOR_ID_HIT_MAIN, &angTmp);
            if(spd > 0)
            {
                if(angTmp - Ang >  - 0.15f)     break;
            }
            else
            {
                if(angTmp - Ang < 0.15f)        break;
            }
            if(xTaskGetTickCount() > TimOut)    break;
            vTaskDelay(portTICK_MS);
        }
        HIT_MAIN_STOP();
    }
    else if(id == MOTOR_ID_HIT_SERVE)
    {
        ReadRacketAngWithOff(MOTOR_ID_HIT_SERVE, &angTmp);
        
        /* 确定挥拍方向，并判断是否在容差范围内不挥拍 */
        if(Ang - angTmp >= 0)
        {
            spd = my_abs(spd);
            if(angTmp - Ang >  - 0.15f)     return;
        }
        else
        {
            spd = -my_abs(spd);
            if(angTmp - Ang < 0.15f)        return;
        }
        
        /* 开始挥拍 */
        HIT_SERVE_PVM(spd);
        while(1)
        {
            ReadRacketAngWithOff(MOTOR_ID_HIT_SERVE, &angTmp);
            if(spd > 0)
            {
                if(angTmp - Ang >  - 0.15f)     break;
            }
            else
            {
                if(angTmp - Ang < 0.15f)        break;
            }
            if(xTaskGetTickCount() > TimOut)    break;
            vTaskDelay(portTICK_MS);
        }
        HIT_SEREVE_STOP();
    }
}

/**
  * @brief  拍子映射初始化
  * @param  None
  * @retval None
  */
void RacketsMapInit(void)
{
    memset(&RacketReceive, 0, sizeof(RacketReceive));
    RacketReceive.dir = RT_NO;
    memset(&CurRacket, 0, sizeof(CurRacket));
    CurRacket.dir = RT_NO;
}


/* 挥拍时间测试---------------------------------------------------------------*/
#define RACKET_MAIN     (0x01)
#define RACKET_LEFT     (0x02)
#define RACKET_RIGHT    (0x04)
#define hitAngMin       (0.25f)

//#define RACKET_TEST_SELECT  (RACKET_MAIN|RACKET_LEFT|RACKET_RIGHT)
#define RACKET_TEST_SELECT  (RACKET_MAIN)

#define MAIN_ANG_MAX    (86)
#define LEFT_ANG_MAX    (131)
#define RIGHT_ANG_MAX   (LEFT_ANG_MAX)
float mainSpd[] = {3,3.5,4,4.5,5,5.5,6};
float leftSpd[] = {3,3.5,4,4.5,5,5.5,6};
float rightSpd[] = {3,3.5,4,4.5,5,5.5,6};

#define RACKET_ANG_TEST_TIME    (512)  //ms
#define RACKET_REPEAT_TIMES     (5)

uint32_t    mainTimeBuf [RACKET_REPEAT_TIMES + 1][MAIN_ANG_MAX];    //mainTimeBuf[RACKET_REPEAT_TIMES]为Sum
uint32_t    leftTimeBuf [RACKET_REPEAT_TIMES + 1][LEFT_ANG_MAX];
uint32_t    rightTimeBuf[RACKET_REPEAT_TIMES + 1][RIGHT_ANG_MAX];

/**
  * @brief  将读取到的时间与整数角度对应 okTime[角度]=时间,角度仅支持正数
  * @param  None
  * @retval None
  */
void LookForTime(float *oriAng, uint32_t *oriTime, uint32_t oriNum, uint32_t *okTime, uint32_t okNum)
{
	uint32_t counti = 0, oriCounti = 0;
	float curAng;
	float diff_old, diff_new;

	if (oriNum < 1) return;

	for (counti = 30; counti < okNum; counti++)
	{
		curAng = counti;

		while (oriCounti < oriNum - 1)
		{
			oriCounti++;
			diff_old = curAng - oriAng[oriCounti - 1];
			diff_new = oriAng[oriCounti] - curAng;
			if (diff_old >= 0 && diff_new >= 0)
			{
				if (diff_old - diff_new >= 0) //curAng和之前的差比之后的大，则选择之后的
					okTime[counti] = oriTime[oriCounti];
				else
					okTime[counti] = oriTime[oriCounti - 1];
				break;
			}
		}
		oriCounti--;
	}
}

/**
  * @brief  读取挥拍时间
  * @param  None
  * @retval None
  */
float       ReadAng[RACKET_ANG_TEST_TIME];
uint32_t    ReadTime[RACKET_ANG_TEST_TIME];
void ReadRacketTime(uint32_t type, float spd, int angNum, uint32_t *timBuf)
{
    uint32_t firstTime, TimOut;
    float angTmp=0;
    
    uint32_t index=0,j;
    
    float ang = (float)angNum;
    memset(ReadAng, 0, sizeof(ReadAng));
    memset(ReadTime, 0, sizeof(ReadTime));
    
    firstTime = xTaskGetTickCount();
    TimOut = firstTime + RACKET_ANG_TEST_TIME*portTICK_MS/2;
    
    if(type == RACKET_MAIN)                     //主拍-----------------------------------
    {
        ReadRacketAngWithOff(MOTOR_ID_HIT_MAIN, &angTmp);

        if(ang > 90.0f)         ang = 90.0f;    //Main挥拍限幅
        
        /* 确定挥拍方向，并判断是否在容差范围内不挥拍 */
		if(ang - angTmp >= 0)
        {
            spd = my_abs(spd);
            if(angTmp - ang >  - hitAngMin)     return;
        }
		else
        {
            spd = -my_abs(spd);
            if(angTmp - ang < hitAngMin)        return;
        }
        
        /* 开始挥拍 */
        HIT_MAIN_PVM(spd);
        
        while(index < RACKET_ANG_TEST_TIME)
        {
            ReadRacketAngWithOff(MOTOR_ID_HIT_MAIN, &angTmp);
            ReadAng[index] = angTmp;
            ReadTime[index++] = xTaskGetTickCount() - firstTime;
            
            if(spd > 0)
            {
                if(angTmp - ang >  - hitAngMin)     break;
            }
            else
            {
                if(angTmp - ang < hitAngMin)        break;
            }
            
            if(xTaskGetTickCount() > TimOut)        break;
            
            vTaskDelay(portTICK_MS);
        }
        
        HIT_MAIN_STOP();
        /* 计算整数角度的时间 */
        LookForTime(ReadAng, ReadTime, index, timBuf, angNum);
    }
    else if(type == RACKET_LEFT)                 //左拍-----------------------------------
    {
        ReadRacketAngWithOff(MOTOR_ID_HIT_SERVE, &angTmp);
        
        /* 确定挥拍方向，并判断是否在容差范围内不挥拍 */
		if(ang - angTmp >= 0)
        {
            spd = my_abs(spd);
            if(angTmp - ang >  - hitAngMin)     return;
        }
		else
        {
            spd = -my_abs(spd);
            if(angTmp - ang < hitAngMin)        return;
        }
        
        /* 开始挥拍 */
        HIT_SERVE_PVM(spd);
        
        while(index < RACKET_ANG_TEST_TIME)
        {
            ReadRacketAngWithOff(MOTOR_ID_HIT_SERVE, &angTmp);
            ReadAng[index] = angTmp;
            ReadTime[index++] = xTaskGetTickCount() - firstTime;
            
            if(spd > 0)
            {
                if(angTmp - ang >  - hitAngMin)     break;
            }
            else
            {
                if(angTmp - ang < hitAngMin)        break;
            }
            
            if(xTaskGetTickCount() > TimOut)        break;
            
            vTaskDelay(portTICK_MS);
        }
        HIT_SEREVE_STOP();
        /* 计算整数角度的时间 */
        LookForTime(ReadAng, ReadTime, index, timBuf, angNum);
    }
    else if(type == RACKET_RIGHT)                 //右拍-----------------------------------
    {
        ang = (float)-angNum;
        ReadRacketAngWithOff(MOTOR_ID_HIT_SERVE, &angTmp);
        
        /* 确定挥拍方向，并判断是否在容差范围内不挥拍 */
		if(ang - angTmp >= 0)
        {
            spd = my_abs(spd);
            if(angTmp - ang >  - hitAngMin)     return;
        }
		else
        {
            spd = -my_abs(spd);
            if(angTmp - ang < hitAngMin)        return;
        }
        
        /* 开始挥拍 */
        HIT_SERVE_PVM(spd);
        
        while(index < RACKET_ANG_TEST_TIME)
        {
            ReadRacketAngWithOff(MOTOR_ID_HIT_SERVE, &angTmp);
            ReadAng[index] = angTmp;
            ReadTime[index++] = xTaskGetTickCount() - firstTime;
            
            if(spd > 0)
            {
                if(angTmp - ang >  - hitAngMin)     break;
            }
            else
            {
                if(angTmp - ang < hitAngMin)        break;
            }
            
            if(xTaskGetTickCount() > TimOut)        break;
            
            vTaskDelay(portTICK_MS);
        }
        
        HIT_SEREVE_STOP();
        /*角度转为正数*/
        for(j = 0; j < index; j++)    ReadAng[j] = -ReadAng[j];
        
        /* 计算整数角度的时间 */
        LookForTime(ReadAng, ReadTime, index, timBuf, angNum);
    }
}

#include "serial.h"
/**
  * @brief  挥拍时间测试
  * @param  None
  * @retval None
  */
void mainRacketTimeTest(float speed, uint32_t ang)
{
    uint32_t i = 0, angi = 0;
    
    memset(mainTimeBuf, 0, sizeof(mainTimeBuf));
    
    /* Test ----------------------------------------------------------------- */
    for(i = 0; i < RACKET_REPEAT_TIMES; i++)
    {
        /*正式挥拍--------------------------------------------------*/
        RacketRun2Ang(MOTOR_ID_HIT_MAIN, -0.25f, RACKET_MAIN_SENSOR_ANG, my_abs((RACKET_MAIN_RESET_ANG-RACKET_MAIN_SENSOR_ANG)*1000/(360.0f*0.25f)) + 50);
        vTaskDelay(400*portTICK_MS);
        
        /*挥拍*/
        ReadRacketTime(RACKET_MAIN, speed, ang, &mainTimeBuf[i][0]);
        vTaskDelay(500*portTICK_MS);
        
        /*主拍复位--------------------------------------------------*/
        RacketRun2Ang(MOTOR_ID_HIT_MAIN, -0.5f, RACKET_MAIN_RESET_ANG, 2000);
        vTaskDelay(300*portTICK_MS);
    }
    
    /* Send ----------------------------------------------------------------- */
    for(angi = 30; angi < ang; angi++)
    {
        for(i = 0; i < RACKET_REPEAT_TIMES ;i++)
        {
            mainTimeBuf[RACKET_REPEAT_TIMES][angi] += mainTimeBuf[i][angi];
        }
        
        /* 计算平均值 */
        mainTimeBuf[RACKET_REPEAT_TIMES][angi]
            = mainTimeBuf[RACKET_REPEAT_TIMES][angi]/RACKET_REPEAT_TIMES;
        
        DmaSendHitInfo((uint16_t)RT_MAIN, speed, angi, mainTimeBuf[RACKET_REPEAT_TIMES][angi]);
        vTaskDelay(10*portTICK_MS);
    }
}

void leftRacketTimeTest(float speed, uint32_t ang)
{
    uint32_t i=0, angi=0;
    
    memset(leftTimeBuf, 0, sizeof(leftTimeBuf));
    
    /* Test ----------------------------------------------------------------- */
    for(i = 0; i < RACKET_REPEAT_TIMES; i++)
    {
        RacketRun2Ang(MOTOR_ID_HIT_SERVE, 0.5f, 0.0f, 2000);
        vTaskDelay(400*portTICK_MS);
        /*正式挥拍--------------------------------------------------*/
        ReadRacketTime(RACKET_LEFT, speed, ang, &leftTimeBuf[i][0]);
        vTaskDelay(500*portTICK_MS);
        /*侧拍复位--------------------------------------------------*/
        RacketRun2Ang(MOTOR_ID_HIT_SERVE, 0.5f, RACKET_SERVE_RESET_ANG, 5000);
        vTaskDelay(300*portTICK_MS);
    }
    
    /* Send ----------------------------------------------------------------- */
    for(angi = 30; angi < ang; angi++)
    {
        for(i = 0; i < RACKET_REPEAT_TIMES ;i++)
        {
            leftTimeBuf[RACKET_REPEAT_TIMES][angi] += leftTimeBuf[i][angi];
        }
        
        /* 计算平均值 */
        leftTimeBuf[RACKET_REPEAT_TIMES][angi]
            = leftTimeBuf[RACKET_REPEAT_TIMES][angi]/RACKET_REPEAT_TIMES;
        
        DmaSendHitInfo((uint16_t)RT_LEFT, speed, angi, leftTimeBuf[RACKET_REPEAT_TIMES][angi]);
        vTaskDelay(10*portTICK_MS);
    }
}

void rightRacketTimeTest(float speed, uint32_t ang)
{
    uint32_t i=0, angi=0;
    
    memset(rightTimeBuf, 0, sizeof(rightTimeBuf));
    
    /* Test ----------------------------------------------------------------- */
    for(i = 0; i < RACKET_REPEAT_TIMES; i++)
    {
        RacketRun2Ang(MOTOR_ID_HIT_SERVE, 0.5f, 0.0f, 2000);
        vTaskDelay(400*portTICK_MS);
        /*正式挥拍--------------------------------------------------*/
        ReadRacketTime(RACKET_RIGHT, speed, ang, &rightTimeBuf[i][0]);
        vTaskDelay(500*portTICK_MS);
        /*侧拍复位--------------------------------------------------*/
        RacketRun2Ang(MOTOR_ID_HIT_SERVE, 0.5f, RACKET_SERVE_RESET_ANG, 5000);
        vTaskDelay(300*portTICK_MS);
    }

    /* Send ----------------------------------------------------------------- */
    for(angi = 30; angi < ang; angi++)
    {
        for(i = 0; i < RACKET_REPEAT_TIMES ;i++)
        {
            rightTimeBuf[RACKET_REPEAT_TIMES][angi] += rightTimeBuf[i][angi];
        }
        
        /* 计算平均值 */
        rightTimeBuf[RACKET_REPEAT_TIMES][angi]
            = rightTimeBuf[RACKET_REPEAT_TIMES][angi]/RACKET_REPEAT_TIMES;
        
        DmaSendHitInfo((uint16_t)RT_RIGHT, speed, angi, rightTimeBuf[RACKET_REPEAT_TIMES][angi]);
        vTaskDelay(10*portTICK_MS);
    }
}


extern uint32_t test_state;
void hittim_test(uint32_t dir)
{
    switch(dir)
    {
        case RT_MAIN:
            /* Start ---------------------------------------------------------*/
            DmaSendHitInfo((uint16_t)(RT_MAIN + 0x10), 0, 0, 0);
            vTaskDelay(100*portTICK_MS);
        
            /* 主拍复位 */
            RacketRun2Ang(MOTOR_ID_HIT_MAIN, 0.25f, RACKET_MAIN_RESET_ANG, 5000);
            vTaskDelay(1000*portTICK_MS);

            {
                uint32_t spd_i=0;

                for(spd_i=0; spd_i< sizeof(mainSpd)/sizeof(float); spd_i++)
                {
                    mainRacketTimeTest(mainSpd[spd_i], MAIN_ANG_MAX);
                    vTaskDelay(200*portTICK_MS);
                    if(test_state != TEST_HITTIME)
                    break;
                }
            }
            
            /* End -----------------------------------------------------------*/
            DmaSendHitInfo((uint16_t)(RT_MAIN + 0x20), 0, 0, 0);
            vTaskDelay(100*portTICK_MS);
            break;
        
        case RT_LEFT:
            /* Start ---------------------------------------------------------*/
            DmaSendHitInfo((uint16_t)(RT_LEFT + 0x10), 0, 0, 0);
            vTaskDelay(100*portTICK_MS);
            
            /* 主拍下去 */
            RacketRun2Ang(MOTOR_ID_HIT_MAIN, 0.25f, RACKET_MAIN_SENSOR_ANG-0.15f, 5000);
            vTaskDelay(1000*portTICK_MS);
        
            /* 侧拍复位 */
            RacketRun2Ang(MOTOR_ID_HIT_SERVE, 0.25f, RACKET_SERVE_RESET_ANG, 5000);
            vTaskDelay(300*portTICK_MS);
        
            {
                uint32_t spd_i=0;

                for(spd_i=0; spd_i< sizeof(mainSpd)/sizeof(float); spd_i++)
                {
                    leftRacketTimeTest(leftSpd[spd_i], LEFT_ANG_MAX);
                    vTaskDelay(200*portTICK_MS);
                    if(test_state != TEST_HITTIME)
                    break;
                }
            }
            
            /* 主拍复位 */
            RacketRun2Ang(MOTOR_ID_HIT_MAIN, 0.25f, RACKET_MAIN_RESET_ANG, 5000);
            vTaskDelay(1000*portTICK_MS);
            
            /* End -----------------------------------------------------------*/
            DmaSendHitInfo((uint16_t)(RT_LEFT + 0x20), 0, 0, 0);
            vTaskDelay(100*portTICK_MS);
            break;
        
        case RT_RIGHT:
            /* Start ---------------------------------------------------------*/
            DmaSendHitInfo((uint16_t)(RT_RIGHT + 0x10), 0, 0, 0);
            vTaskDelay(100*portTICK_MS);
            
            /* 主拍下去 */
            RacketRun2Ang(MOTOR_ID_HIT_MAIN, 0.25f, RACKET_MAIN_SENSOR_ANG-0.15f, 5000);
            vTaskDelay(1000*portTICK_MS);
        
            /* 侧拍复位 */
            RacketRun2Ang(MOTOR_ID_HIT_SERVE, 0.25f, RACKET_SERVE_RESET_ANG, 5000);
            vTaskDelay(300*portTICK_MS);
        
            {
                uint32_t spd_i=0;

                for(spd_i=0; spd_i< sizeof(mainSpd)/sizeof(float); spd_i++)
                {
                    rightRacketTimeTest(rightSpd[spd_i], RIGHT_ANG_MAX);
                    vTaskDelay(200*portTICK_MS);
                    if(test_state != TEST_HITTIME)
                    break;
                }
            }
            
            /* 主拍复位 */
            RacketRun2Ang(MOTOR_ID_HIT_MAIN, 0.25f, RACKET_MAIN_RESET_ANG, 5000);
            vTaskDelay(1000*portTICK_MS);
            
            /* End -----------------------------------------------------------*/
            DmaSendHitInfo((uint16_t)(RT_RIGHT + 0x20), 0, 0, 0);
            vTaskDelay(100*portTICK_MS);
            break;
        
        default:
            break;
    }
    vTaskDelay(200*portTICK_MS);
}
/**
  * @brief  挥拍可靠性测试
  * @param  None
  * @retval None
  */
void hit_test_main(void)
{
    int i = 0;
    vTaskDelay(100*portTICK_MS);
    
    for(i = 0; i < 30; i++)
    {
        RacketRun2Ang(MOTOR_ID_HIT_MAIN, -0.35f, RACKET_MAIN_SENSOR_ANG-0.20f, my_abs(RACKET_MAIN_RESET_ANG*1000/(360.0f*0.35f)) + 50);
		vTaskDelay(250*portTICK_MS);
        
        RacketRun2Ang(MOTOR_ID_HIT_MAIN, 6, 55 + 20.0f, 120);

        vTaskDelay(100 * portTICK_MS);
    
        //球拍复位&拍停
        RacketRun2Ang(MOTOR_ID_HIT_MAIN, -0.5f, RACKET_MAIN_RESET_ANG-6.5f, 1000);
        
        vTaskDelay(1000*portTICK_MS);
        
        DmaSendWordQueue(0xe6, TEST_HIT_MAIN, 0);
        
        if(test_state != TEST_HIT_MAIN)
            break;
    }
}

void hit_test_left(void)
{
    int i = 0;
    RacketRun2Ang(MOTOR_ID_HIT_MAIN, -0.25f, RACKET_MAIN_SENSOR_ANG, my_abs((RACKET_MAIN_RESET_ANG-RACKET_MAIN_SENSOR_ANG)*1000/(360.0f*0.25f)) + 50);
    vTaskDelay(400*portTICK_MS);
    for(i = 0; i < 30; i++)
    {
        RacketRun2Ang(MOTOR_ID_HIT_SERVE, 0.5f, 0.0f, 2000);
        vTaskDelay(400*portTICK_MS);
        /*正式挥拍---------------------------------------------------*/
        /*挥拍*/
        RacketRun2Ang(MOTOR_ID_HIT_SERVE, 6, 80, 1000);
        vTaskDelay(300*portTICK_MS);
        /*侧拍复位--------------------------------------------------*/
        RacketRun2Ang(MOTOR_ID_HIT_SERVE, 0.5f, RACKET_SERVE_RESET_ANG, 5000);
        vTaskDelay(500*portTICK_MS);
        
        DmaSendWordQueue(0xe6, TEST_HIT_LEFT, 0);
        
        if(test_state != TEST_HIT_LEFT)
            break;
    }
    RacketRun2Ang(MOTOR_ID_HIT_MAIN, 0.5f, RACKET_MAIN_RESET_ANG, 1000);
    vTaskDelay(400*portTICK_MS);
}

void hit_test_right(void)
{
    int i = 0;
    RacketRun2Ang(MOTOR_ID_HIT_MAIN, -0.25f, RACKET_MAIN_SENSOR_ANG, my_abs((RACKET_MAIN_RESET_ANG-RACKET_MAIN_SENSOR_ANG)*1000/(360.0f*0.25f)) + 50);
    vTaskDelay(400*portTICK_MS);
    for(i = 0; i < 30; i++)
    { 
        RacketRun2Ang(MOTOR_ID_HIT_SERVE, 0.5f, 0.0f, 2000);
        vTaskDelay(400*portTICK_MS);
        
        /*正式挥拍---------------------------------------------------*/
        /*挥拍*/ 
        RacketRun2Ang(MOTOR_ID_HIT_SERVE, 6, -80, 1000);
        vTaskDelay(300*portTICK_MS);
        
        /*侧拍复位--------------------------------------------------*/
        RacketRun2Ang(MOTOR_ID_HIT_SERVE, 0.5f, RACKET_SERVE_RESET_ANG, 5000);
        vTaskDelay(500*portTICK_MS);
        
        DmaSendWordQueue(0xe6, TEST_HIT_RIGHT, 0);
        if(test_state != TEST_HIT_RIGHT)
            break;
    }
    RacketRun2Ang(MOTOR_ID_HIT_MAIN, 0.5f, RACKET_MAIN_RESET_ANG, 1000);
    vTaskDelay(400*portTICK_MS);
}

/******************************** END OF FILE *********************************/
