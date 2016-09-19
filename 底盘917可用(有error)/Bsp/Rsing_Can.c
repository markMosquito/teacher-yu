/**
  ******************************************************************************
  * @file    Rsing_Can.c
  * @author  Rsing
  * @version V1.0
  * @date    14-November-2014
  * @brief   Initializate the Can.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Rsing_Can.h"
#include <string.h>
#include "Redef_GPIO.h"

/* Private variables ---------------------------------------------------------*/
/* Define --------------------------------------------------------------------*/
#define IO_CAN1_TX     PA(12)
#define IO_CAN1_RX     PA(11)
#define IO_CAN2_TX     PB(13)
#define IO_CAN2_RX     PB(12)

//CAN1 ---------------------------------------------------------
#define GPIO_CAN1_TX            IO_GPIO_PORT(IO_CAN1_TX)
#define GPIO_CAN1_TX_PIN        IO_GPIO_PIN(IO_CAN1_TX)
#define RCC_GPIO_CAN1_TX        IO_RCC_AHB1_PERIPH(IO_CAN1_TX)
#define GPIO_PINSOURCE_CAN1_TX  IO_GPIO_PINSOURCE(IO_CAN1_TX)

#define GPIO_CAN1_RX            IO_GPIO_PORT(IO_CAN1_RX)
#define GPIO_CAN1_RX_PIN        IO_GPIO_PIN(IO_CAN1_RX)
#define RCC_GPIO_CAN1_RX        IO_RCC_AHB1_PERIPH(IO_CAN1_RX)
#define GPIO_PINSOURCE_CAN1_RX  IO_GPIO_PINSOURCE(IO_CAN1_RX)
//CAN2 ---------------------------------------------------------
#define GPIO_CAN2_TX            IO_GPIO_PORT(IO_CAN2_TX)
#define GPIO_CAN2_TX_PIN        IO_GPIO_PIN(IO_CAN2_TX)
#define RCC_GPIO_CAN2_TX        IO_RCC_AHB1_PERIPH(IO_CAN2_TX)
#define GPIO_PINSOURCE_CAN2_TX  IO_GPIO_PINSOURCE(IO_CAN2_TX)

#define GPIO_CAN2_RX            IO_GPIO_PORT(IO_CAN2_RX)
#define GPIO_CAN2_RX_PIN        IO_GPIO_PIN(IO_CAN2_RX)
#define RCC_GPIO_CAN2_RX        IO_RCC_AHB1_PERIPH(IO_CAN2_RX)
#define GPIO_PINSOURCE_CAN2_RX  IO_GPIO_PINSOURCE(IO_CAN2_RX)


T_SysErrors g_SysErrors = {0,};

/* Private variables ---------------------------------------------------------*/
// ��������
const uint8_t TxMailBox_IdleNO_Tbl[] = {
				0xFF/*0x0:Busy*/,	0/*0x1:0*/, 	1/*010b:1*/, 
				0/*011b:0,1*/,		2/*100b:2*/,	0/*101b:0,2*/, 
				1/*110b:1,2*/,		0/*111b:0,1,2*/};

// ���в���
/* �����жϣ�Ӧ�ò㷢�ͺ���ֱ�ӽ���������Ϊ�����͵�CAN���������ʽ
 * T_CanFrame/CAN_TypeDef���
 * ����ʱ��������п���ֱ�ӽ�����ѹ�����䣬��������ѹ�����
 */
T_CanQueue CanTxQueue;
// CAN���ͻ�����
static T_CanFrame CanTxBuf[CAN_QUE_TX_Nframe] = {0,};


/*********************************************************************************************************
** ��������: CAN_QueueCreate
** ��������: ��ʼ�����ݶ���
** �䡡��: Buf     �� Ϊ���з���Ĵ洢�ռ��ַ
**         pDataBuf�� ���ݴ�Ż�����
**		   nData:	  ����Ԫ�ظ���,��*pDataBufΪ��Ԫ�ĸ���
**         ReadEmpty��Ϊ���ж���ʱ�������
**         WriteFull��Ϊ����д��ʱ�������
**         WriteFullParam: ����д����������ʱ�Ĳ���
** �䡡��: NOT_OK:��������
**         QUEUE_OK:�ɹ�
** ȫ�ֱ���: ��
** ����ģ��: OS_ENTER_CRITICAL,OS_EXIT_CRITICAL
********************************************************************************************************/
static t_err CAN_QueueCreate(register T_CanQueue *Queue, T_CanFrame *pDataBuf,
                          	t_size_opt nData    /*, t_size_opt nSizeOfData*/
							#if (CAN_QUEUE_READ_EMPTY_EN > 0)
                          	,t_err (* ReadEmpty)()
							#endif
							#if (CAN_QUEUE_WRITE_FULL_EN > 0)
                          	,t_err (* WriteFull)(uint32_t)
							,uint32_t   WriteFullParam
							#endif
                          	)
{    	
	if (Queue != NULL && pDataBuf != NULL)                  /* �жϲ����Ƿ���Ч */
	{
        CAN_QUE_OS_CPU_SR_ALLOC();
		CAN_QUE_OS_ENTER_CRITICAL();
		                                                	/* ��ʼ���ṹ������ */
		Queue->Buf = pDataBuf;
		Queue->MaxData = nData;				               	/* ������п��Դ洢��������Ŀ */
		Queue->End = &Queue->Buf[Queue->MaxData];           /* �������ݻ���Ľ�����ַ,�ѳ��������� */
		Queue->Out = Queue->Buf;
		Queue->In = Queue->Buf;
		Queue->NData = 0;
		
		#if (CAN_QUEUE_READ_EMPTY_EN > 0)
		Queue->ReadEmpty = ReadEmpty;
		#endif
        
		#if (CAN_QUEUE_WRITE_FULL_EN > 0)
		Queue->WriteFull = WriteFull;
		Queue->WriteFullParam = WriteFullParam;
		#endif
        
		CAN_QUE_OS_EXIT_CRITICAL();

		return QUEUE_OK;
	}
 	else
	{
		return NOT_OK;
	}
}

/* Write ---------------------------------------------------------------------*/
/******************************************************************************
** ��������: CAN_QueueWrite
** ��������: дָ����������������
** �䡡��: Queue:ָ����е�ָ��
**         pData:��Ϣ����
** ������: NOT_OK     ����������
**         QUEUE_OK   ���յ���Ϣ
**         QUEUE_EMPTY������Ϣ
** ȫ�ֱ���: ��
** ����ģ��: OS_ENTER_CRITICAL,OS_EXIT_CRITICAL
*******************************************************************************/
#if defined ( __ICCARM__ )
t_err CAN_QueueWriteQuick(register T_CanQueue *Queue, T_CanFrame *pData)
{
	register t_err err;
	register T_CanFrame *pIn;
	
    err = NOT_OK;
    if (Queue != NULL)                             /* �����Ƿ���Ч */
    {
        if (Queue->NData < Queue->MaxData)         	/* �����Ƿ���  */
        {                                  			/* ����        */
        	pIn = Queue->In;
            *pIn = *pData;                 			/* �������    */
            ++pIn;
            if (pIn >= Queue->End)
            {
                pIn = Queue->Buf;
            }
            Queue->In = pIn;                    	/* �������ָ��*/
            Queue->NData++;                         /* ��������    */
            err = QUEUE_OK;
        }
        else
        {                                           /* ��           */
            err = QUEUE_FULL;
			#if (CAN_QUEUE_WRITE_FULL_EN > 0)
            if (Queue->WriteFull != NULL)        	/* �����û������� */
            {
                err = Queue->WriteFull(Queue->WriteFullParam);
            }
			#endif
		}
    }
    return err;
}
#endif

/* Read ----------------------------------------------------------------------*/
#if CAN_QUEUE_READ_EN > 0
/******************************************************************************
** ��������: CAN_QueueRead
** ��������: ��ȡ�����е�һ�����ݵ�Ԫ
** �䡡��: Queue:ָ����е�ָ��
**         pData:���ݴ�ŵ�
** ������: NOT_OK     ����������
**         QUEUE_OK   ���յ���Ϣ
**         QUEUE_EMPTY������Ϣ
** ȫ�ֱ���: ��
** ����ģ��: OS_ENTER_CRITICAL,OS_EXIT_CRITICAL
*******************************************************************************/
__inline t_err CAN_QueueRead(register T_CanQueue *Queue, T_CanFrame *pData)
{
	register t_err err;
	register uint16 uData;
	register T_CanFrame *pOut, *pOutOld;
    
    CAN_QUE_OS_CPU_SR_ALLOC();
    
    CAN_QUE_OS_ENTER_CRITICAL();
	if (Queue != NULL)                              /* �����Ƿ���Ч */
	{                                               /* ��Ч */
		if (Queue->NData > 0)                       /* �����Ƿ�Ϊ�� */
		{                                    		/* ����      */
			pOut = Queue->Out;                      /* ���ݳ���  */
			pOutOld = pOut;
			++pOut;
			if (pOut >= Queue->End)
			{
				pOut = Queue->Buf;
			}
			Queue->Out = pOut;                  	/* ��������ָ�� */
			Queue->NData--;                         /* ���ݼ���      */
			
			CAN_INT_RX_ENABLE();					/* �������н����ж� */
			*pData = *pOutOld;						/* �����ж��ٸ��������Լ����ж�ʱ�� */
			err = QUEUE_OK;
		}
		else
		{                                           /* ��              */
			err = QUEUE_EMPTY;
			#if (CAN_QUEUE_READ_EMPTY_EN > 0)
			if (Queue->ReadEmpty != NULL)           /* �����û������� */
			{
				err = Queue->ReadEmpty(Queue);
			}
			#endif
		}

	}
    else
    {
        err = NOT_OK;
    }
    CAN_QUE_OS_EXIT_CRITICAL();
	return err;
}
#endif	// end: #if CAN_QUEUE_READ_EN > 0

// ���ж�
__inline static t_err CAN_QueueReadQuick(T_CanFrame *pData, register T_CanQueue *Queue)
{
	register t_err err;
	register T_CanFrame *pOut;
	
	err = NOT_OK;
	
	if (Queue != NULL)                           	/* �����Ƿ���Ч */
	{                                               /* ��Ч */
		//CAN_QUE_OS_ENTER_CRITICAL();
		if (Queue->NData > 0)                       /* �����Ƿ�Ϊ�� */
		{                                           /* ����         */
			pOut = Queue->Out;                      /* ���ݳ���     */
			*pData = *pOut;
			++pOut;
			if (pOut >= Queue->End)
			{
				pOut = Queue->Buf;
			}
			Queue->Out = pOut;                  	/* ��������ָ�� */
			Queue->NData--;                         /* ���ݼ���      */
			err = QUEUE_OK;
		}
		else
		{                                           /* ��              */
			err = QUEUE_EMPTY;
			#if (CAN_QUEUE_READ_EMPTY_EN > 0)
			if (Queue->ReadEmpty != NULL)           /* �����û������� */
			{
				err = Queue->ReadEmpty(Queue);
			}
			#endif
		}
		//CAN_QUE_OS_EXIT_CRITICAL();
	}
	return err;
}

#if CAN1_SCE_IRQ_Handler_EXT_EN == 0
/* CAN1���Ƽ�״̬�仯�ж� */
void CAN1_SCE_IRQHandler (void)
{
    if(CAN_GetITStatus(CAN1, CAN_IT_LEC))
    {
        g_SysErrors.CAN_LEC[(CAN1->ESR & CAN_ESR_LEC) >> 4]++;// CAN���ִ���״̬����:λ���,��ʽ,ȷ��,����,����,CRC
        CAN_ClearITPendingBit(CAN1,CAN_IT_LEC);
    }
    if(CAN_GetITStatus(CAN1, CAN_IT_BOF))       // CAN���ߴ������
    {
        g_SysErrors.CAN_BOF++;
        CAN_ClearITPendingBit(CAN1,CAN_IT_BOF);
    }
    if(CAN_GetITStatus(CAN1, CAN_IT_EPV))       // CAN���󱻶�����
    {
        g_SysErrors.CAN_EPV++;
        CAN_ClearITPendingBit(CAN1,CAN_IT_EPV);
    }
    if(CAN_GetITStatus(CAN1, CAN_IT_EWG))       // CAN���󾯸��־����
    {
        g_SysErrors.CAN_EWG++;
        CAN_ClearITPendingBit(CAN1,CAN_IT_EWG);
    }
}
#endif		// end: #if CAN1_SCE_IRQ_Handler_EXT_EN == 0


/*----------------------------------------------------------------------------
  CAN transmit interrupt handler
 *----------------------------------------------------------------------------*/
void CAN1_TX_IRQHandler(void)	// CAN1_TX_IRQHandler_Name
{
	register t_ureg dw_r;
	register CAN_TxMailBox_TypeDef *pTxMailBox;

    /* ����FIFO���ж� */
    if(CanTxQueue.NData > 0)
    {
    	// ������Ͷ��в�Ϊ��
		dw_r = (CAN1->TSR >> CAN_TSR_TME_BITnL) & 0x07;	// ���ҿյķ�������
        dw_r = TxMailBox_IdleNO_Tbl[dw_r];			// ��ȡһ���������
        pTxMailBox = &CAN1->sTxMailBox[dw_r];
        CAN_QueueReadQuick((T_CanFrame *)pTxMailBox, &CanTxQueue);
        SETBITs(pTxMailBox->TIR, CAN_TIxR_TXRQ);    // ������
    }
    else
    {
		/* Clear CAN_TSR_RQCPx (rc_w1) */
		CAN1->TSR = CAN_TSR_RQCP0|CAN_TSR_RQCP1|CAN_TSR_RQCP2;	// �����������,�Է�ֹ�ٴν����ж�, Add by Xsky 2011-07-30 17:51
        CAN_INT_TX_DISABLE();
	}
}

/**@note �����жϱ��� δ�����񱣻�*/
t_err CAN_SendFrame(T_CanFrame *pCanFrame)
{
	register CAN_TxMailBox_TypeDef *pTxMailBox;
	register t_ureg dw_r;
	t_err err = ERR_NONE;
	/* ���ͱ��ĵ�����Ϊ��Ӧ�ó���ѡ��1�����õķ������䣻���ñ�ʶ����
	   ���ݳ��Ⱥʹ��������ݣ�Ȼ���CAN_TIxR�Ĵ�����TXRQλ�á�1�����������͡� 
	   ����״̬CAN_TSR,Datasheet.CN:p441 */
	taskENTER_CRITICAL();
    CAN_INT_TX_DISABLE();           //�رշ����ж�
    if(CanTxQueue.NData > 0)        // ���в�Ϊ����ֱ��д�뷢�Ͷ���
    {
        #ifdef DBG_CAN_QUE_MAX      // ����,��¼���Ͷ������洢����            
        if(CanTxQueue.NData < g_History.SysErrors.CAN_SendQueMaxN)
        {
            g_History.SysErrors.CAN_SendQueMaxN = CanTxQueue.NData;
        }
        #endif
        err = CAN_QueueWriteQuick(&CanTxQueue, pCanFrame);
        CAN_INT_TX_ENABLE();        // ���������ж�
    }
    else    // �������Ϊ��
    {
        dw_r = (CAN1->TSR >> CAN_TSR_TME_BITnL) & 0x07;
        if(dw_r)	// �п��еķ�������
        {
            pTxMailBox = &CAN1->sTxMailBox[TxMailBox_IdleNO_Tbl[dw_r]];
			pTxMailBox->TIR = pCanFrame->IxR;
            pTxMailBox->TDTR = pCanFrame->DTxR;
            pTxMailBox->TDLR = pCanFrame->Data.u32[0];
            pTxMailBox->TDHR = pCanFrame->Data.u32[1];
            pTxMailBox->TIR |= CAN_TIxR_TXRQ;
            CAN_INT_TX_ENABLE();	// ���������ж�
        }
        else		// �޿�������
        {
            err = CAN_QueueWriteQuick(&CanTxQueue, pCanFrame);
            CAN_INT_TX_ENABLE();    // ���������ж�
        }
    }
	taskEXIT_CRITICAL();
	return err;
}

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initialization for CAN
  * @param  None
  * @retval None
  */
void CAN_Config(CAN_TypeDef* CANx)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    CAN_InitTypeDef          CAN_InitStructure;
    CAN_FilterInitTypeDef    CAN_FilterInitStructure;      
    NVIC_InitTypeDef         NVIC_InitStructure;

    if( CANx == CAN1 )
    {
        /* Enable GPIO clock */
        RCC_AHB1PeriphClockCmd(RCC_GPIO_CAN1_TX|RCC_GPIO_CAN1_RX, ENABLE);

        /* Enable CAN clock */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

        /* Configure CAN RX and TX pins */
        GPIO_InitStructure.GPIO_Pin = GPIO_CAN1_TX_PIN;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_Init(GPIO_CAN1_TX, &GPIO_InitStructure);
        
        GPIO_InitStructure.GPIO_Pin = GPIO_CAN1_RX_PIN;
        GPIO_Init(GPIO_CAN1_RX, &GPIO_InitStructure);

        /* Connect CAN pins to AF9 */
        GPIO_PinAFConfig(GPIO_CAN1_TX, GPIO_PINSOURCE_CAN1_TX, GPIO_AF_CAN1);
        GPIO_PinAFConfig(GPIO_CAN1_RX, GPIO_PINSOURCE_CAN1_RX, GPIO_AF_CAN1);		
    }
    else if( CANx == CAN2 )
    {  
        /* Enable GPIO clock */
        RCC_AHB1PeriphClockCmd(RCC_GPIO_CAN2_TX|RCC_GPIO_CAN2_RX, ENABLE);

        /* Enable CAN clock */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);

        /* Configure CAN RX and TX pins */
        GPIO_InitStructure.GPIO_Pin = GPIO_CAN2_TX_PIN;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_Init(GPIO_CAN2_TX, &GPIO_InitStructure);
        
        GPIO_InitStructure.GPIO_Pin = GPIO_CAN2_RX_PIN;
        GPIO_Init(GPIO_CAN2_RX, &GPIO_InitStructure);

        /* Connect CAN pins to AF9 */
        GPIO_PinAFConfig(GPIO_CAN2_TX, GPIO_PINSOURCE_CAN2_TX, GPIO_AF_CAN2);
        GPIO_PinAFConfig(GPIO_CAN2_RX, GPIO_PINSOURCE_CAN2_RX, GPIO_AF_CAN2);
    }

    /* CAN register init */
    CAN_DeInit(CANx);
    CAN_StructInit(&CAN_InitStructure);	 

    /* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;   // Enable or disable the time triggered communication mode.   
	CAN_InitStructure.CAN_ABOM = DISABLE;   // Enable or disable the automatic bus-off management.
	CAN_InitStructure.CAN_AWUM = DISABLE;   // Enable or disable the automatic wake-up mode.
	CAN_InitStructure.CAN_NART = DISABLE;   // Enable or disable the non-automatic retransmission mode.
	CAN_InitStructure.CAN_RFLM = DISABLE;   // Enable or disable the Receive FIFO Locked mode.
	CAN_InitStructure.CAN_TXFP = ENABLE;    // Enable or disable the transmit FIFO priority. 
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;

    /* Baudrate = 1Mbps (CAN clocked at 42 MHz) */
    CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
    CAN_InitStructure.CAN_Prescaler = 3;     //CAN������42/(1+9+4)/3=1Mbps
    CAN_Init(CANx, &CAN_InitStructure);

    if( CANx == CAN1 )
    {
        /* CAN filter init */
        CAN_FilterInitStructure.CAN_FilterNumber = 0;                        
        CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;      
        CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;     
        CAN_FilterInitStructure.CAN_FilterIdHigh = (((u32)0x0012<<21)&0xFFFF0000)>>16;               
        CAN_FilterInitStructure.CAN_FilterIdLow = (((u32)0x0012<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF;                     
        CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;           
        CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;               
        CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;               
        CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
        CAN_FilterInit(&CAN_FilterInitStructure);

        /* CAN FIFO0 message pending interrupt enable */ 
        CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);

        /* Enable the CAN1 global Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        
		CAN_ITConfig(CAN1,CAN_IT_TME, ENABLE);

		/* Enable the CAN1 global Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
        
        /* Enable the CAN1 global Interrupt */
        #ifdef DBG_CAN
        CAN1->ESR |= CAN_ESR_LEC_USER;				// ���ϴδ��������Ϊ�û�״̬(���򲻻����ø�״̬)�������µ�״̬
        CAN_ITConfig(CAN1, CAN_IT_ERR|CAN_IT_LEC|CAN_IT_BOF|CAN_IT_EPV|CAN_IT_EWG, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_SCE_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
        #endif
        /* Ӧ�ó�����г�ʼ�� */
        // TX, ���Ͷ���
        CAN_QueueCreate(&CanTxQueue, CanTxBuf, lenof(CanTxBuf));			// ���Ͷ���
    }
    else if( CANx == CAN2 )
    {
        /* CAN filter init */
        CAN_FilterInitStructure.CAN_FilterNumber = 14;                        
        CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;      
        CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;     
        CAN_FilterInitStructure.CAN_FilterIdHigh = (((u32)0x0012<<21)&0xFFFF0000)>>16;               
        CAN_FilterInitStructure.CAN_FilterIdLow = (((u32)0x0012<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF;                     
        CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;           
        CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;               
        CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;               
        CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
        CAN_FilterInit(&CAN_FilterInitStructure);

        /* CAN FIFO0 message pending interrupt enable */ 
        CAN_ITConfig(CAN2,CAN_IT_FMP0, ENABLE);

        /* Enable the CAN2 global Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);	
    }
}

/**
  * @brief  can����һ������
  * @param  CAN_Num:CAN�˿�
            len:���ݳ���,���Ϊ8	
            msg:����ָ��,���Ϊ8���ֽ�.
            id: ���ͱ�ʶ��
  * @retval 0,�ɹ�;
            1,ʧ��
  */
uint8_t Can_Send_Msg(CAN_TypeDef* CAN_Num, uint8_t* msg, uint8_t len, uint8_t id)
{	 
    u8 mbox;
    u16 i=0;
    CanTxMsg TxMessage;
    
    TxMessage.StdId=id;					 	//��׼��ʶ����11λ
    TxMessage.IDE=CAN_Id_Standard;	        //ʹ�ñ�׼��ʶ��
    TxMessage.RTR=CAN_RTR_Data;			    //��������֡
    TxMessage.DLC=len;						//����len�ֽ�
    
    for(i=0;i<len;i++)
        TxMessage.Data[i]=msg[i];
    
    mbox = CAN_Transmit(CAN_Num, &TxMessage);     //��ȡ�����

    i = 0;
    while((CAN_TransmitStatus(CAN_Num, mbox)!=CAN_TxStatus_Ok)&&(i<0xfff))i++;	//�ȴ����ͽ���  
    if(i>=0xfff)return 1;
    
    return 0;		
}


/*********************************END OF FILE**********************************/
