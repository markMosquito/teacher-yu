/* Includes ------------------------------------------------------------------*/
#include "Global.h"
#include <string.h>

#include "bsp.h"
#include "vect.h"
#include "app.h"
#include "MyMath.h"
#include "Com2Server.h"
#include "007_elmo.h"

/* Exported variables --------------------------------------------------------*/
GlobalParamTypeDef G_Param; /*全局位置结构体*/
uint32_t motionlessCount = 0, anglessCount = 0;
uint32_t SendCorrectFlag = 0, AngCorrectFlag = 0;   /*当前是否发送矫正命令*/
point_f PosOriwithOutAng;
uint32_t PosBufCount=0;
point_f PosBuf[20];
point_f PosSpeed;
extern Elmo slidewayGroup;
/**
  * @brief  vPosCalTask
  * @param  None
  * @retval None
  */
void vPosCalTask( void *p_arg )
{
    (void)p_arg;
    while(1)
    {
        if(xSemaphoreTake(PosSem, 60) == pdTRUE)       //码盘数据5ms一次，这里最多等6ms
        {
            Elmo_Read_PositionActualRequest(slidewayGroup.NodeID);
            /*计算在陀螺仪坐标系的xy*/
            PosOriwithOutAng.x   = (float)(G_Param.pos_ori.x - G_Param.correctPos.x) * GYRO_CNT2M_RATIO;
            PosOriwithOutAng.y   = (float)(G_Param.pos_ori.y - G_Param.correctPos.y) * GYRO_CNT2M_RATIO;
            
            /*计算角度矫正后的实际坐标 mm */
            G_Param.cur_pos.x   =   PosOriwithOutAng.x*G_Param.pos_cam_off.angcosf
                                  - PosOriwithOutAng.y*G_Param.pos_cam_off.angsinf
                                  + G_Param.pos_off.x       //起始点偏移量
                                  + G_Param.pos_cam_off.x;  //摄像头偏移量

            G_Param.cur_pos.y   =   PosOriwithOutAng.x*G_Param.pos_cam_off.angsinf
                                  + PosOriwithOutAng.y*G_Param.pos_cam_off.angcosf
                                  + G_Param.pos_off.y       //起始点偏移量
                                  + G_Param.pos_cam_off.y;  //摄像头偏移量

            G_Param.cur_pos.ang =   G_Param.pos_ori.ang
                                  + G_Param.pos_cam_off.ang;//摄像头偏移量

            /*计算当前速度*/
            G_Param.speed.x   = (G_Param.cur_pos.x   - G_Param.pos_old.x) * 200;
            G_Param.speed.y   = (G_Param.cur_pos.y   - G_Param.pos_old.y) * 200;
            G_Param.speed.ang = (G_Param.cur_pos.ang - G_Param.pos_old.ang) * 200;

            memcpy(&G_Param.pos_old, &G_Param.cur_pos, sizeof(point_f));

            /* 判断车是否静止 */
            #if 0
            if( my_abs(G_Param.speed.x) < 50 && my_abs(G_Param.speed.y) < 50 && my_abs(G_Param.speed.ang) < 5
                && ChassisStatus == RobotReady && SlidewayStatus == RobotReady)
            {
				motionlessCount++;
                if(motionlessCount > 20)//如果100ms内车没有动，发送矫正命令
                {
                    SendCorrectFlag = 1;
                }
                if(G_Param.speed.x == 0 && G_Param.speed.y == 0 && my_abs(G_Param.speed.ang) < 0.5f)
                {
                    anglessCount++;
                    if(anglessCount > 20)
                    {
                        AngCorrectFlag = 1;
                    }
                }
                else
                {
                    anglessCount = 0;
                    AngCorrectFlag = 0;
                }
            }
            else
            {
                motionlessCount = 0;
                SendCorrectFlag = 0;
                anglessCount = 0;
                AngCorrectFlag = 0;
            }
            #endif
            /*判断是否静止*/
            memcpy(&PosBuf[PosBufCount%20], &G_Param.cur_pos, sizeof(point_f));
            PosSpeed.x = PosBuf[PosBufCount%20].x - PosBuf[(PosBufCount-20)%20].x;
            PosSpeed.y = PosBuf[PosBufCount%20].y - PosBuf[(PosBufCount-20)%20].y;
            PosBufCount++;
            if(my_abs(PosSpeed.x) < 20 && my_abs(PosSpeed.y) < 20 && my_abs(G_Param.speed.ang) < 10
                && ChassisStatus == RobotReady && SlidewayStatus == RobotReady)
            {
                SendCorrectFlag = 1;
                if(G_Param.speed.x == 0 && G_Param.speed.y == 0 && my_abs(G_Param.speed.ang) < 0.5f)
                {
                    anglessCount++;
                    if(anglessCount > 10)
                    {
                        AngCorrectFlag = 1;
                    }
                }
                else
                {
                    anglessCount = 0;
                    AngCorrectFlag = 0;
                }
            }
            else
            {
                motionlessCount = 0;
                SendCorrectFlag = 0;
                anglessCount = 0;
                AngCorrectFlag = 0;
            }
            
            /*滑轨*/
            G_Param.slideway_x_mm = ReadSlidewayX_mm();
            G_Param.slideway_y_mm = ReadSlidewayY_mm();
            
            /**/
            xSemaphoreGive( PosChassisSem );
            xSemaphoreGive( PosSlideSem );
        }
        else
        {
            DEF_SendPosOff();
            DEF_RecievePosOff();
            DmaSendWordQueue(0xee, 1, 0);
            if(ChassisTaskHandle != 0 )     vTaskSuspend(ChassisTaskHandle);
            if(SlidewayTaskHandle != 0 )    vTaskSuspend(SlidewayTaskHandle);
            if(HitTaskHandle != 0 )         vTaskSuspend(HitTaskHandle);
            //if(BeepTaskHandle != 0 )        vTaskSuspend(BeepTaskHandle);
            BEEP_ON;
            Elmo_Stop(0);
			LCD_QueuePrintfErr(3,0,16, "GYRO ERR");
            vTaskDelay(3000*portTICK_MS);
            if(CheckTaskHandle != 0 )       vTaskSuspend(CheckTaskHandle);
            vTaskSuspend(PosCalTaskHandle);
        }
    }
}

extern u32 lowPowerFlag;
void CAN2_RX0_IRQHandler(void)
{
	static CanRxMsg RxMessage;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    
    if((CAN2->RF0R & CAN_RF0R_FMP0) != RESET) //if (CAN_GetITStatus(CAN2,CAN_IT_FMP0) != RESET) 
    {
        //CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);
        RxMessage.StdId = (uint32_t)0x000007FF & (CAN2->sFIFOMailBox[CAN_FIFO0].RIR >> 21);
        RxMessage.DLC = (uint8_t)0x0F & CAN2->sFIFOMailBox[CAN_FIFO0].RDTR;
        /* Get the data field */
        memcpy(&RxMessage.Data[0], (const void *)&CAN2->sFIFOMailBox[CAN_FIFO0].RDLR, 8);
        /* Release FIFO0 */
        CAN2->RF0R |= CAN_RF0R_RFOM0;
        
		if(RxMessage.StdId == CAN_ID_GYRO)	/*原始陀螺仪模块信息*/
		{
			/*原始坐标信息*/
			if(RxMessage.DLC == 8)
			{
				memcpy(&G_Param.pos_ori.x, &RxMessage.Data[0], 8);
			}

			/*原始角度信息*/
			else if(RxMessage.DLC == 4)
			{ 
				memcpy(&G_Param.pos_ori.ang, &RxMessage.Data[0], 4);
				xSemaphoreGiveFromISR( PosSem, &xHigherPriorityTaskWoken );
			}
        }
        else if(RxMessage.StdId == CAN_ID_SWITCH)
        {
            if(RxMessage.DLC == 1)
			{
				switch(RxMessage.Data[0])
                {
                    case 0: //电量低
                        DmaSendWordQueue(0xee, 7, 1);
                        break;
                    case 1: //断电
                        lowPowerFlag = 1;
                        DmaSendWordQueue(0xee, 8, 1);
                        break;
                    default:
                        break;
                }
			}
        }
    }
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

/**
  * @brief  软件复位陀螺仪
  * @param  None
  * @retval None
  */
void GyroReset(void)
{
    CanTxMsg TxMessage;

    TxMessage.StdId = CAN_ID_GYRO_SET;//如果控制器ID改变，修改这里。
    TxMessage.DLC = 2;//必须为2	   
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.Data[0] = 0x55;
    TxMessage.Data[1] = 0xff;
    CAN_Transmit(CAN2, &TxMessage);
}

#ifndef TIM3_DIR
#error "TIM3_DIR is not define!"
#endif

#ifndef TIM4_DIR
#error "TIM4_DIR is not define!"
#endif
/**
  * @brief  滑轨编码器初始化 TIM3, TIM4均为16位定时器(在此滑轨长度上不会溢出)
  * @param  None
  * @retval None
  */
void Encoder_Init(void)
{
    GPIO_InitTypeDef   GPIO;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//pa6    pa7
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//pd12   pd13

    GPIO.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO.GPIO_Mode = GPIO_Mode_AF;
    GPIO.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &GPIO);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);

    GPIO.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO.GPIO_Mode = GPIO_Mode_AF;
    GPIO.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOD, &GPIO);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);

#if (TIM3_DIR == 1)
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
#elif (TIM3_DIR == -1)
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
#else
#error "TIM3_DIR define err!"
#endif

#if (TIM4_DIR == 1)
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
#elif (TIM4_DIR == -1)
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
#else
#error "TIM4_DIR define err!"
#endif

    TIM3->CNT = 0;
    TIM4->CNT = 0;

    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}


//此处单位为mm
int32_t slidewayXoff = 0, slidewayYoff = 0;

void SetSlidewayX_mm(float mm)
{
    int32_t pos=0;
    Elmo_Read_PositionActual(MOTOR_ID_SLIDEWAY_X, &pos);
    slidewayXoff = -pos + mm*MOTOR_MM2CNT_SLIDEWAY_X/MOTOR_DIR_SLIDEWAY_X;
}

void SetSlidewayY_mm(float mm)
{
    int32_t pos=0;
    Elmo_Read_PositionActual(MOTOR_ID_SLIDEWAY_Y, &pos);
    slidewayYoff = -pos + mm*MOTOR_MM2CNT_SLIDEWAY_Y/MOTOR_DIR_SLIDEWAY_Y;
}

//此处单位为mm
float ReadSlidewayX_mm(void)
{
    return (float)(elmo[MOTOR_ID_SLIDEWAY_X].positionActual + slidewayXoff) * MOTOR_DIR_SLIDEWAY_X/MOTOR_MM2CNT_SLIDEWAY_X;
}

float ReadSlidewayY_mm(void)
{
    return (float)(elmo[MOTOR_ID_SLIDEWAY_Y].positionActual + slidewayYoff) * MOTOR_DIR_SLIDEWAY_Y/MOTOR_MM2CNT_SLIDEWAY_Y;
}
/*********************************END OF FILE**********************************/
