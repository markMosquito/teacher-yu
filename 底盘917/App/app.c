/**
  ******************************************************************************
  * @file    APP.c
  * @author  
  * @version V1.0
  * @date    8-May-2015
  * @brief   Initializate the APP
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app.h"
#include "bsp.h"
#include "mymath.h"
#include "HitTask.h"
#include "amc.h"

#include "Com2Server.h"
#include "selftest.h"
#if (COM_DBG != COM_RELEASE || 1)
#include "comtest.h"
#endif
#define  DBG_RACKET_ENC     0
#if DBG_RACKET_ENC != 0
float racket_enc_test[2]={0,0};
#endif
/* Private variables ---------------------------------------------------------*/
uint32_t ifServe = 0;    //是否发球
uint32_t ifSelfTest = 0;
uint32_t CarReadyFlag = 0;
uint32_t SelfTestFlag = 0;
extern int32_t hitServeEncOff;
/* TaskHandles ---------------------------------------------------------------*/
TaskHandle_t ChassisTaskHandle;
TaskHandle_t SlidewayTaskHandle;
TaskHandle_t HitTaskHandle;
TaskHandle_t ServeTaskHandle;
TaskHandle_t DisplayTaskHandle;
TaskHandle_t CheckTaskHandle;
TaskHandle_t PosCalTaskHandle;
TaskHandle_t RestartTaskHandle;
TaskHandle_t AmcMotionTaskHandle;
TaskHandle_t AmcPosTaskHandle;
/* Semaphores ----------------------------------------------------------------*/
xSemaphoreHandle ElmoSem = 0;       //Can1收到8个Elmo的心跳之后送出一个信号量
xSemaphoreHandle PosSem = 0;        //收到并计算码盘+陀螺仪数据 (中断处理完送出)
xSemaphoreHandle PosChassisSem = 0; //发送位置信号量给底盘
xSemaphoreHandle PosSlideSem = 0;   //发送位置信号量给滑轨
xSemaphoreHandle PosRacketMainSem = 0;      //收到Elmo编码器的值
xSemaphoreHandle PosRacketServeSem = 0;     //收到Elmo编码器的值
xSemaphoreHandle currentMainSem = 0;     //收到Elmo编码器的值


/* QueueHandles --------------------------------------------------------------*/
QueueHandle_t HitInQueue;       //挥拍任务接收用邮箱
QueueHandle_t UsartTxQueue;     //串口DMA发送队列
extern u32 ElmoHeartbeat;

/* Private function ----------------------------------------------------------*/
uint32_t test_state = TEST_NO;
uint32_t test_value = 0;

u32 tt[4];
void slidewayTest(void);

/**
  * @brief  vStartTask
  * @param  None
  * @retval None
  */
  #include "IO_Sensor.h"
  int i,j;
  int motor1_value_begining = 0;
  int motor2_value_begining = 0;
  extern u8 read_amc_over_flag;
  extern int32_t motor_pos[4];
  u8 init_success = 0;
static void vStartTask( void *p_arg )
{
    (void)p_arg;
    vTaskDelay(500*portTICK_MS);
    /*创建信号量*/
    ElmoSem = xSemaphoreCreateBinary();
    PosSem = xSemaphoreCreateBinary();
    PosChassisSem = xSemaphoreCreateBinary();
    PosSlideSem = xSemaphoreCreateBinary();
    
    PosRacketMainSem = xSemaphoreCreateBinary();
    PosRacketServeSem = xSemaphoreCreateBinary();
    currentMainSem = xSemaphoreCreateBinary();
    
    /*创建邮箱用于挥拍*/
    HitInQueue = xQueueCreate( 1, ( unsigned portBASE_TYPE ) sizeof( Racket_TypeDef ) );
    
    /* 创建串口DMA发送队列 */
    UsartTxQueue = xQueueCreate( 16, ( unsigned portBASE_TYPE ) sizeof( UsartTxMsg ) );

    //参数初始化
    RacketsMapInit();
    GlobalVariableInit();
    
    /*延时等待硬件上电完成*/
    vTaskDelay(800*portTICK_MS);
    
    /*BSP_Init*/
    BSP_Init();
	IO_Sensor_Init();
	#if 1//试验，让id为MOTOR_MARCH_1的转两秒
		DMA_GetPos(MOTOR_DIR_1);
		vTaskDelay(20*portTICK_MS);
		DMA_SendCommand(MOTOR_DIR_1,0x02,0x07,0x00,0x01,0x0f);//get the write access
		vTaskDelay(10*portTICK_MS);
		DMA_SendCommand(MOTOR_DIR_1,0x02,0x01,0x00,0x01,0x00);//enable the bridge
		vTaskDelay(10*portTICK_MS);
		DMA_SendCommand(MOTOR_DIR_1,0x02,0x45,0x00,0x02,-2030);//发送速度	
		while(IO_DIR1_LIMIT_LEVEL == 0);//检测到
		DMA_SendCommand(MOTOR_DIR_1,0x02,0x45,0x00,0x02,0);//停止
		read_amc_over_flag = 0;
		while(read_amc_over_flag == 0)
		{
			DMA_GetPos(MOTOR_DIR_1);
			vTaskDelay(4*portTICK_MS);
		}
		motor1_value_begining = motor_pos[2];
		
		
		
		//方向电机2
		DMA_SendCommand(MOTOR_DIR_2,0x02,0x07,0x00,0x01,0x0f);//get the write access
		vTaskDelay(10*portTICK_MS);
		DMA_SendCommand(MOTOR_DIR_2,0x02,0x01,0x00,0x01,0x00);//enable the bridge
		vTaskDelay(10*portTICK_MS);
		DMA_SendCommand(MOTOR_DIR_2,0x02,0x45,0x00,0x02,-2030);//发送速度
		while(IO_DIR2_LIMIT_LEVEL == 0);
		DMA_SendCommand(MOTOR_DIR_2,0x02,0x45,0x00,0x02,0);//停止
		read_amc_over_flag = 0;
		while(read_amc_over_flag == 0)
		{
			DMA_GetPos(MOTOR_DIR_2);
			vTaskDelay(4*portTICK_MS);
		}
		motor2_value_begining = motor_pos[3];
		init_success = 1;
		
		DMA_SendCommand(MOTOR_DIR_1,0x02,0x01,0x00,0x01,0x01);//disable the bridge
		vTaskDelay(5*portTICK_MS);
		DMA_SendCommand(MOTOR_DIR_2,0x02,0x01,0x00,0x01,0x01);//disable the bridge
		vTaskDelay(4*portTICK_MS);
	#endif

	/*设置四个电机可写，并且enable*/
    DMA_SendCommand(1,0x02,0x07,0x00,0x01,0x0f);//get the write access
	vTaskDelay(10*portTICK_MS);
	DMA_SendCommand(1,0x02,0x01,0x00,0x01,0x00);//enable the bridge
	vTaskDelay(10*portTICK_MS);
		
		
	DMA_SendCommand(MOTOR_MARCH_2,0x02,0x07,0x00,0x01,0x0f);//get the write access
	vTaskDelay(5*portTICK_MS);
	DMA_SendCommand(MOTOR_MARCH_2,0x02,0x01,0x00,0x01,0x00);//enable the bridge
	vTaskDelay(5*portTICK_MS);
		
	//DMA_SendCommand(2,0x02,0x45,0x00,0x02,30000);
	vTaskDelay(10*portTICK_MS);
	DMA_SendCommand(MOTOR_DIR_1,0x02,0x07,0x00,0x01,0x0f);//get the write access
	vTaskDelay(5*portTICK_MS);
	DMA_SendCommand(MOTOR_DIR_1,0x02,0x01,0x00,0x01,0x00);//enable the bridge
	vTaskDelay(5*portTICK_MS);
	DMA_SendCommand(MOTOR_DIR_2,0x02,0x07,0x00,0x01,0x0f);//get the write access
	vTaskDelay(5*portTICK_MS);
	DMA_SendCommand(MOTOR_DIR_2,0x02,0x01,0x00,0x01,0x00);//enable the bridge
	vTaskDelay(5*portTICK_MS);
    
   
   // AllRacketsHoming();
   


    vTaskDelay(100*portTICK_MS);
   
   //while(1);
   // xTaskCreate(vAmcPosTask,    "vAmcPosTask",      AMC_POS_TASK_STK,    NULL, AMC_POS_TASK_PRIO,     &AmcPosTaskHandle);
   //while(1);
    xTaskCreate(vAmcMotionTask,  "vAmcMotionTask",    AMC_MOTION_TASK_STK,  NULL, AMC_MOTION_TASK_PRIO,   &AmcMotionTaskHandle);
    //创建底盘任务
  
    /* 开始与电脑通讯 --------------------------------------------------------*/
    //DEF_RecievePosOn(); //CarReadyFlag
    
	while(1)
	{
		vTaskDelay(100*portTICK_MS);
	}

}

void slidewayTest()
{
    while(SlidewayStatus != RobotReady)
        vTaskDelay(100*portTICK_MS);
    {
        u8 i;
        for(i = 0; i <30;i++)
        {
            /*---------------------------------------*/
            
            SlidewayRunPoint.x = DEF_HG_BoundaryXMin + 50;
            SlidewayRunPoint.y = DEF_HG_BoundaryYMax - 30;
            SlidewayRunPoint.tim = 1000*portTICK_MS;
            SlidewayStatusChange = RobotTest;
            vTaskDelay(100*portTICK_MS);
            while(SlidewayStatus != RobotReady)
                vTaskDelay(5*portTICK_MS);
            vTaskDelay(500*portTICK_MS);
            
            SlidewayStatusChange = RobotReset;
            while(SlidewayStatus != RobotReady)
                vTaskDelay(100*portTICK_MS);
            vTaskDelay(1000*portTICK_MS);
            /*---------------------------------------*/
            
            SlidewayRunPoint.x = DEF_HG_BoundaryXMax - 50;
            SlidewayRunPoint.y = DEF_HG_BoundaryYMax - 30;
            SlidewayRunPoint.tim = 1000*portTICK_MS;
            SlidewayStatusChange = RobotTest;
            vTaskDelay(100*portTICK_MS);
            while(SlidewayStatus != RobotReady)
                vTaskDelay(5*portTICK_MS);
            vTaskDelay(500*portTICK_MS);
            
            SlidewayStatusChange = RobotReset;
            while(SlidewayStatus != RobotReady)
                vTaskDelay(100*portTICK_MS);
            vTaskDelay(1000*portTICK_MS);
            /*---------------------------------------*/
            
            SlidewayRunPoint.x = DEF_HG_BoundaryXMax - 50;
            SlidewayRunPoint.y = DEF_HG_BoundaryYMin + 30;
            SlidewayRunPoint.tim = 1000*portTICK_MS;
            SlidewayStatusChange = RobotTest;
            vTaskDelay(100*portTICK_MS);
            while(SlidewayStatus != RobotReady)
                vTaskDelay(5*portTICK_MS);
            vTaskDelay(500*portTICK_MS);
            
            SlidewayStatusChange = RobotReset;
            while(SlidewayStatus != RobotReady)
                vTaskDelay(100*portTICK_MS);
            vTaskDelay(1000*portTICK_MS);
            /*---------------------------------------*/
            
            SlidewayRunPoint.x = DEF_HG_BoundaryXMin + 50;
            SlidewayRunPoint.y = DEF_HG_BoundaryYMin + 30;
            SlidewayRunPoint.tim = 1000*portTICK_MS;
            SlidewayStatusChange = RobotTest;
            vTaskDelay(100*portTICK_MS);
            while(SlidewayStatus != RobotReady)
                vTaskDelay(5*portTICK_MS);
            vTaskDelay(500*portTICK_MS);
            
            
            SlidewayStatusChange = RobotReset;
            while(SlidewayStatus != RobotReady)
                vTaskDelay(100*portTICK_MS);
            vTaskDelay(1000*portTICK_MS);
            
            DmaSendWordQueue(0xe6, TEST_SLIDEWAY, 0);
            if(test_state != TEST_SLIDEWAY)
                break;
            /*---------------------------------------*/
        }
    }
}
//-------------------------------------------------------------------------------------------------------------
///**
//  * @brief  vMainTask
//  * @param  None
//  * @retval None
//  */
//static void vMainTask( void *p_arg )
//{
//    TickType_t xLastTESTTime;
//    (void)p_arg;

//    xLastTESTTime = xTaskGetTickCount();
//    while (1)
//    {
//        vTaskDelayUntil(&xLastTESTTime, 1000 * portTICK_MS);
//    }
//}
//--------------------------------------------------------------


//--------------------------------------------------------------
void StartTaskCreate(void)
{
    /* Create one task. */
    xTaskCreate(vStartTask,         /* Pointer to the function that implements the task.              */
        "Start",                    /* Text name for the task.  This is to facilitate debugging only. */
        AppTaskStartStk,            /* Stack depth in words.                                          */
        NULL,                       /* We are not using the task parameter.                           */
        APP_CFG_TASK_START_PRIO,    /* This task will run at priority x.                              */
        NULL);                      /* We are not using the task handle.                              */

    /* Create one task. */
    xTaskCreate(vBeepTask,          /* Pointer to the function that implements the task.              */
        "Beep",                     /* Text name for the task.  This is to facilitate debugging only. */
        AppTaskBeepStk,             /* Stack depth in words.                                          */
        NULL,                       /* We are not using the task parameter.                           */
        APP_CFG_TASK_BEEP_PRIO,     /* This task will run at priority x.                              */
        &BeepTaskHandle);
}
//----------------------------------------------------------------
/**
  * @brief  vCheckTask
  * @param  None
  * @retval None
  */
void vCheckTask(void *p_arg)
{
    u32 i=0;
    (void)p_arg;

    SendHeart2Elmo();
    while (1)
    {
        if(xSemaphoreTake(ElmoSem, 150 * portTICK_MS) == pdTRUE) //Elmo设置100ms发送一次心跳,在此设置超时为150ms
        {
            SendHeart2Elmo();
        }
        else
        {
            DEF_SendPosOff();
            DEF_RecievePosOff();
            DmaSendWordQueue(0xee, 6, 0);
            if(ChassisTaskHandle != 0 )     vTaskSuspend(ChassisTaskHandle);
            if(SlidewayTaskHandle != 0 )    vTaskSuspend(SlidewayTaskHandle);
            if(HitTaskHandle != 0 )         vTaskSuspend(HitTaskHandle);
            if(BeepTaskHandle != 0 )        vTaskSuspend(BeepTaskHandle);
            for(i=0;i<5;i++)
            {
                Elmo_Stop(0);
                vTaskDelay(5*portTICK_MS);
                SendHeart2Elmo();
                vTaskDelay(95*portTICK_MS);
            }
            ElmoShowErr(ElmoHeartbeat);
            LCD_QueuePrintfErr(3, 0, 16, "ElmoErr Heart");
            ElmoErrBeep(ElmoHeartbeat);
            vTaskSuspend(CheckTaskHandle);
        }
    }
}

//--------------------------------------------------------------
/*Global.c*/
extern uint32_t motionlessCount, anglessCount;
extern uint32_t SendCorrectFlag, AngCorrectFlag;   /*当前是否发送矫正命令*/
extern uint32_t PosBufCount;
extern point_f PosBuf[20];
extern point_f PosSpeed;

/*ChassisTask.c*/
extern uint32_t ChassisStatus;
extern uint32_t ChassisStop;
extern point_t ChassisResetPoint;
extern uint32_t YellowFlag;
extern point_t ChassisTeleV;
extern uint32_t PidXSelected, PidYSelected;

/*SlidewayTask.c*/
extern uint32_t SlidewayStatus;
extern uint32_t SlidewayStop;

/*HitTask.c*/
extern int32_t hitMainEncOff;
extern int32_t hitServeEncOff;
extern Racket_TypeDef* pCurRacket;
extern int16_t hitting;
extern uint32_t tFirst;

/*Com2Server.c*/
extern UsartRxMsg UsartRx1,UsartRx3;
extern uint32_t SyncFlag;
extern uint32_t SyncTimOut;

/**
  * @brief  GlobalVariableInit,全局变量初始化
  * @param  None
  * @retval None
  */
void GlobalVariableInit(void)
{
    /* app.c */
    ifServe = 0;
    ifSelfTest = 0;
    CarReadyFlag = 0;
    SelfTestFlag = 0;
    
    ChassisTaskHandle = 0;
    SlidewayTaskHandle = 0;
    HitTaskHandle = 0;
    CheckTaskHandle = 0;
    PosCalTaskHandle = 0;
    
    /* Global.c */
    memset(&G_Param, 0, sizeof(G_Param));
    G_Param.pos_off.x = DEF_DP_StartPosX;
    G_Param.pos_off.y = DEF_DP_StartPosY;
    G_Param.pos_cam_off.angcosf = 1.0f;
	G_Param.pos_cam_off.angsinf = 0.0f;
    
    motionlessCount=0;
    anglessCount = 0;
    SendCorrectFlag = 0;
    AngCorrectFlag = 0;   /*当前是否发送矫正命令*/
    PosBufCount = 0;
    memset(&PosBuf[0], 0, sizeof(point_f)*20);
    memset(&PosSpeed, 0, sizeof(point_f));

    /* Manage.c */
    ChassisStatusChange = RobotReady;
    SlidewayStatusChange = RobotReady;
    memset(&xyzt_structure, 0, sizeof(xyzt_structure));
    
    /* ChassisTask.c */
    ChassisStatus = RobotReady;
    ChassisStop = 0;
    ChassisResetPoint.x = DEF_DP_ResetPosX;
    ChassisResetPoint.y = DEF_DP_ResetPosY;
    ChassisResetPoint.ang = 0;
    YellowFlag = 0;
    memset(&ChassisTeleV, 0, sizeof(point_t));
    PidXSelected = 6;
    PidYSelected = 6;
    
    /* SlidewayTask.c */
    SlidewayStatus = RobotReady;
    SlidewayStop = 0;

    /* HitTask.c */
    hitMainEncOff = 0;
    hitServeEncOff = 0;
    hitting = -16383;  //用于绘图标出挥拍过程
    tFirst = 0;
    
    /* Com2Server.c */
    memset(&UsartRx1, 0, sizeof(UsartRx3));
    memset(&UsartRx3, 0, sizeof(UsartRx3));
    SyncFlag = 0;
    SyncTimOut = 0;
}

void ElmoErrBeep(uint32_t ElmoFlag)
{
    while(1)
    {
        u32 i;
        Buzzer(2, 255, 0);
        vTaskDelay(5*portTICK_MS);
        while(eTaskGetState(BeepTaskHandle) != eSuspended)
            vTaskDelay(portTICK_MS);
        vTaskDelay(1000*portTICK_MS);
        for(i = 0; i < 8; i++)
        {
            if(!((ElmoFlag>>i)&0x01))
            {
                Buzzer(i+1, 100, 100);    //有电机未检测到
                vTaskDelay(5*portTICK_MS);
                while(eTaskGetState(BeepTaskHandle) != eSuspended)
                    vTaskDelay(portTICK_MS);
                vTaskDelay(500*portTICK_MS);
            }
        }
        vTaskDelay(1000*portTICK_MS);
    }
}

/******************************** END OF FILE *********************************/
