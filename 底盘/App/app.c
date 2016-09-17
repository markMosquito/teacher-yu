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
uint32_t ifServe = 0;    //�Ƿ���
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
/* Semaphores ----------------------------------------------------------------*/
xSemaphoreHandle ElmoSem = 0;       //Can1�յ�8��Elmo������֮���ͳ�һ���ź���
xSemaphoreHandle PosSem = 0;        //�յ�����������+���������� (�жϴ������ͳ�)
xSemaphoreHandle PosChassisSem = 0; //����λ���ź���������
xSemaphoreHandle PosSlideSem = 0;   //����λ���ź���������
xSemaphoreHandle PosRacketMainSem = 0;      //�յ�Elmo��������ֵ
xSemaphoreHandle PosRacketServeSem = 0;     //�յ�Elmo��������ֵ
xSemaphoreHandle currentMainSem = 0;     //�յ�Elmo��������ֵ


/* QueueHandles --------------------------------------------------------------*/
QueueHandle_t HitInQueue;       //�����������������
QueueHandle_t UsartTxQueue;     //����DMA���Ͷ���
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
  
static void vStartTask( void *p_arg )
{
    (void)p_arg;
    vTaskDelay(500*portTICK_MS);
    /*�����ź���*/
    ElmoSem = xSemaphoreCreateBinary();
    PosSem = xSemaphoreCreateBinary();
    PosChassisSem = xSemaphoreCreateBinary();
    PosSlideSem = xSemaphoreCreateBinary();
    
    PosRacketMainSem = xSemaphoreCreateBinary();
    PosRacketServeSem = xSemaphoreCreateBinary();
    currentMainSem = xSemaphoreCreateBinary();
    
    /*�����������ڻ���*/
    HitInQueue = xQueueCreate( 1, ( unsigned portBASE_TYPE ) sizeof( Racket_TypeDef ) );
    
    /* ��������DMA���Ͷ��� */
    UsartTxQueue = xQueueCreate( 16, ( unsigned portBASE_TYPE ) sizeof( UsartTxMsg ) );

    //������ʼ��
    RacketsMapInit();
    GlobalVariableInit();
    
    /*��ʱ�ȴ�Ӳ���ϵ����*/
    vTaskDelay(800*portTICK_MS);
    
    /*BSP_Init*/
    BSP_Init();
	#if 1//���飬��idΪMOTOR_MARCH_1��ת����
		DMA_GetPos(MOTOR_MARCH_1);
		vTaskDelay(20*portTICK_MS);
		DMA_SendCommand(MOTOR_MARCH_1,0x02,0x07,0x00,0x01,0x0f);//get the write access
		vTaskDelay(10*portTICK_MS);
		DMA_SendCommand(MOTOR_MARCH_1,0x02,0x01,0x00,0x01,0x00);//enable the bridge
		vTaskDelay(10*portTICK_MS);
		DMA_SendCommand(MOTOR_MARCH_1,0x02,0x45,0x00,0x02,873.813*300);//�����ٶ�
		
		vTaskDelay(2000*portTICK_MS);
		DMA_SendCommand(MOTOR_MARCH_1,0x02,0x45,0x00,0x02,0);//�����ٶ�
		vTaskDelay(2000*portTICK_MS);
		DMA_GetPos(MOTOR_MARCH_1);
		while(1)
		{
//			DMA_SendCommand(MOTOR_MARCH_1,0x02,0x45,0x00,0x02,873.813*300);//�����ٶ�
//		
//			vTaskDelay(2000*portTICK_MS);
//			DMA_SendCommand(MOTOR_MARCH_1,0x02,0x45,0x00,0x02,0);//�����ٶ�
//			vTaskDelay(500*portTICK_MS);
//			DMA_GetPos(MOTOR_MARCH_1);
//			vTaskDelay(10*portTICK_MS);
		}
		
	#endif

	/*�����ĸ������д������enable*/
//  DMA_SendCommand(MOTOR_MARCH_1,0x02,0x07,0x00,0x01,0x0f);//get the write access
//	vTaskDelay(5*portTICK_MS);
//	DMA_SendCommand(MOTOR_MARCH_1,0x02,0x01,0x00,0x01,0x00);//enable the bridge
//	vTaskDelay(5*portTICK_MS);
//	DMA_SendCommand(MOTOR_MARCH_2,0x02,0x07,0x00,0x01,0x0f);//get the write access
//	vTaskDelay(5*portTICK_MS);
//	DMA_SendCommand(MOTOR_MARCH_2,0x02,0x01,0x00,0x01,0x00);//enable the bridge
//	vTaskDelay(5*portTICK_MS);
//	DMA_SendCommand(MOTOR_DIR_1,0x02,0x07,0x00,0x01,0x0f);//get the write access
//	vTaskDelay(5*portTICK_MS);
//	DMA_SendCommand(MOTOR_DIR_1,0x02,0x01,0x00,0x01,0x00);//enable the bridge
//	vTaskDelay(5*portTICK_MS);
//	DMA_SendCommand(MOTOR_DIR_2,0x02,0x07,0x00,0x01,0x0f);//get the write access
//	vTaskDelay(5*portTICK_MS);
//	DMA_SendCommand(MOTOR_DIR_2,0x02,0x01,0x00,0x01,0x00);//enable the bridge
//	vTaskDelay(5*portTICK_MS);
    
    


    
   
   
   // AllRacketsHoming();
   


    vTaskDelay(100*portTICK_MS);
   
   
    //xTaskCreate(vPosCalTask,    "vPosCalTask",      POSCAL_TASK_STK,    NULL, POSCAL_TASK_PRIO,     &PosCalTaskHandle);
   
    xTaskCreate(vAmcMotionTask,  "vAmcMotionTask",    AMC_MOTION_TASK_STK,  NULL, AMC_MOTION_TASK_PRIO,   &AmcMotionTaskHandle);
    //������������
  
    /* ��ʼ�����ͨѶ --------------------------------------------------------*/
    //DEF_RecievePosOn(); //CarReadyFlag
    



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
        if(xSemaphoreTake(ElmoSem, 150 * portTICK_MS) == pdTRUE) //Elmo����100ms����һ������,�ڴ����ó�ʱΪ150ms
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
extern uint32_t SendCorrectFlag, AngCorrectFlag;   /*��ǰ�Ƿ��ͽ�������*/
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
  * @brief  GlobalVariableInit,ȫ�ֱ�����ʼ��
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
    AngCorrectFlag = 0;   /*��ǰ�Ƿ��ͽ�������*/
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
    hitting = -16383;  //���ڻ�ͼ������Ĺ���
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
                Buzzer(i+1, 100, 100);    //�е��δ��⵽
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
