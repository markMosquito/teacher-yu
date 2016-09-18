/**
  ******************************************************************************
  * @file    Rsing_Can.h
  * @author  Rsing
  * @version V1.0
  * @date    14-November-2014
  * @brief   Header file for Rsing_Can.c module. 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RSING_CAN_H
#define __RSING_CAN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "007_elmo.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"


/****************************************************************************/
// ���뿪��
#define RELEASE		1		// 1: ����, 0:�Ƿ���
#define IS_RELEASE()		(RELEASE == 1)
#define IS_N_RELEASE()		(RELEASE != 1)

#if IS_N_RELEASE()
    #define  DBG_CAN				// ����CAN
    #define	 DBG_CAN_QUE_MAX		// CAN���ü�¼�������ֵ
#endif



/* -------------------------------------------------------------------------- */
typedef uint32_t 			t_ureg;				/* �޷��żĴ������� */
typedef int32_t				t_sreg;				/* �з��żĴ������� */
typedef uint16_t			t_ureg_opt;			/* �޷����Ż��Ĵ������� */
typedef int16_t				t_sreg_opt;			/* �з����Ż��Ĵ������� */

typedef t_ureg				t_bool;				/* boolean���� */
typedef t_ureg				t_err;				/* ���������� */
typedef uint32_t			t_size;				/* ���ݳ������� */
typedef uint16_t			t_size_opt;			/* �Ż����ݳ��� */

#ifndef BIT
    #define BIT(n)			(1<<(n))
#endif
#define LSHFT(x,n)		((x) << (n))
#define RSHFT(x,n)		((x) >> (n))
#define SETBIT(x,n) 	x |= (1<<(n))
#define CLRBIT(x,n) 	x &= (~(1<<(n)))
#define SETBITs(x,msk)	x|=(msk)
#define CLRBITs(x,msk)	x&=(~(msk))
#define lenof(x) 	( sizeof((x)) / sizeof(*(x)) )	// Add By Xsky 2008-9-03 15:56
    
#ifndef NOT_OK
    #define NOT_OK              0xff                        /* ��������                                     */
#endif
#define  ERR_NONE					0x0000
#define QUEUE_FULL          8                           /* ������                                       */
#define QUEUE_EMPTY         4                           /* ������                                       */
#define QUEUE_OK            0                           /* �����ɹ�                                     */


typedef struct T_SysErrors_{
	// CAN_����
	uint32_t	CAN_LEC[6];					// CAN���ִ���״̬����:λ���,��ʽ,ȷ��,����,����,CRC
	uint32_t	CAN_BOF;					// CAN���ߴ������
	uint32_t	CAN_EPV;					// CAN���󱻶�����
	uint32_t	CAN_EWG;					// CAN���󾯸��־����
	//uint32_t	CAN_TX;					// CAN���ʹ���
	//uint32_t	CAN_ALST;				// CAN�����ٲ�����Ĵ���
	//uint32_t	CAN_FOVR;				// CAN�������
	uint32_t	CAN_SendQueOVF;				// CAN���Ͷ������
	uint32_t	CAN_RcvQueOVF;				// CAN���ն������
	#if IS_N_RELEASE()
	uint8_t	CAN_SendQueMaxN;			// CAN���Ͷ������ֵ
	uint8_t	CAN_RcvQueMaxN;				// CAN���ն������Ԫ�ظ���
	#endif
}T_SysErrors;


/* Defines -------------------------------------------------------------------*/
#define  CAN_QUE_RX_Nframe	128		/* CAN���ն���֡��Ŀ,�Խṹ��T_CanFrameΪ��Ԫ������ */
#define  CAN_QUE_TX_Nframe	256		/* CAN���Ͷ���֡��Ŀ,�Խṹ��T_CanFrameΪ��Ԫ������ */


// STM32 CAN�Ĵ�����ض���
#define CAN_TSR_TME_BITnL			26						/* CAN���� */
#define CAN_TIxR_IDE				((uint32_t)BIT(2))		/* 0:��׼��ʶ��, 1:��չ��ʶ�� */
#define CAN_TIxR_RTR				((uint32_t)BIT(1))		/* 0:����֡, 1:Զ��֡ */
#define CAN_TIxR_TXRQ         		((uint32_t)0x00000001)	/* register CAN_TIxR */
#define CAN_TDTxR_DLC         		((uint32_t)0x0000000F)	/* register CAN_TDTxR  */
#define CAN_ESR_LEC_BITnL			4						/* CAN_ESC_LEC���λ�� */
#define CAN_ESR_LEC_USER			CAN_ESR_LEC				/* CAN_ESC_LECλ���� */
#define CAN_ESR_TEC_BITnL			16						/* CAN_ESR_TEC���λ�� */

#define CAN_INT_TX_DISABLE()		CLRBITs(CAN1->IER, CAN_IT_TME)	/* �ر�CAN�����ж� */
#define CAN_INT_TX_ENABLE()			SETBITs(CAN1->IER, CAN_IT_TME)	/* ����CAN�����ж� */
#define CAN_INT_RX0_DISABLE()		CLRBITs(CAN1->IER, CAN_IT_FMP0) /* �ر�CAN���ж� */
#define CAN_INT_RX0_ENABLE()		SETBITs(CAN1->IER, CAN_IT_FMP0) /* ����CAN���ж� */
#define CAN_INT_RX1_DISABLE()		CLRBITs(CAN1->IER, CAN_IT_FMP1) /* �ر�CAN���ж� */
#define CAN_INT_RX1_ENABLE()		SETBITs(CAN1->IER, CAN_IT_FMP1) /* ����CAN���ж� */
#define CAN_INT_RX_DISABLE()		CLRBITs(CAN1->IER, CAN_IT_FMP0 | CAN_IT_FMP1) /* �ر�CAN���ж� */
#define CAN_INT_RX_ENABLE()			SETBITs(CAN1->IER, CAN_IT_FMP0 | CAN_IT_FMP1) /* ����CAN���ж� */


#ifndef CAN1_SCE_IRQ_Handler_EXT_EN
    #define CAN1_SCE_IRQ_Handler_EXT_EN	0
#endif

#ifndef CAN_QUEUE_CRITICAL_EN
    #define CAN_QUEUE_CRITICAL_EN		1
#endif


#if (CAN_QUEUE_CRITICAL_EN > 0)
    #ifdef INC_FREERTOS_H
        #define OS_CPU_SR_ALLOC()           UBaseType_t uxSavedInterruptStatus
        #define OS_ENTER_CRITICAL()		    uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR()
        #define OS_EXIT_CRITICAL()		    portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptStatus )
    #else
        #define OS_CPU_SR_ALLOC()           OS_CPU_SR  cpu_sr = (OS_CPU_SR)0
    #endif
    #define CAN_QUE_OS_CPU_SR_ALLOC()       OS_CPU_SR_ALLOC()
    #define CAN_QUE_OS_ENTER_CRITICAL()		OS_ENTER_CRITICAL()
    #define CAN_QUE_OS_EXIT_CRITICAL()		OS_EXIT_CRITICAL()
#else
    #define CAN_QUE_OS_ENTER_CRITICAL()		
    #define CAN_QUE_OS_EXIT_CRITICAL()		
#endif

#ifndef CAN_QUEUE_WRITE_FULL_EN
    #define CAN_QUEUE_WRITE_FULL_EN		0
#endif

#ifndef CAN_QUEUE_READ_EMPTY_EN
    #define CAN_QUEUE_READ_EMPTY_EN		0
#endif

#ifndef CAN_QUEUE_READ_EN
    #define  CAN_QUEUE_READ_EN          0
#endif

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

typedef union
{
    uint8_t 	u08[8];		/* �����ֽ������������� */
    uint16_t	u16[4];		
    uint32_t	u32[2];		/* ����Ĵ�����������߶�ȡЧ�� */
}T_CanData;

// ��Ϊ��������������������ݽṹ������ͬ�����Զ������ݿ�ֱ��ʹ��
typedef struct
{
    uint32_t	IxR;	/* ��TIxR/RIxR��Ӧ */                      //TIR
    uint32_t 	DTxR;	/* ��TDTxR/RDTxR��Ӧ */                    //TDTR
	T_CanData 	Data; 	/* TLxR:THxR/RLxR:RHxR, ���ֽ����� */      //TDLR,TDTR
}T_CanFrame;

typedef struct
{
    // �����Խ�In, Out, ����Ϊ������ʽ�ȶ���Ϊָ��ִ��Ч�ʵ�
    T_CanFrame		*Out;					/* ָ���������λ�� 		*/
    T_CanFrame 		*In;					/* ָ����������λ�� 		*/
    T_CanFrame 		*End;					/* ָ��Buf�Ľ���λ��		*/
    uint16_t			NData;					/* ���������ݸ���			*/
    uint16_t			MaxData;				/* ����������洢�����ݸ��� */
    #if (CAN_QUEUE_READ_EMPTY_EN > 0)
    t_err			(* ReadEmpty)(void);	/* ���մ����� 			*/
    #endif
    #if (CAN_QUEUE_WRITE_FULL_EN > 0)
    t_err			(* WriteFull)(uint32);	/* д�������� 			*/
    uint32			WriteFullParam;
    #endif
    T_CanFrame 		*Buf; 					/* �洢���ݵĿռ�			*/
}T_CanQueue;

/* Private functions ---------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
extern T_CanQueue CanTxQueue;
extern const uint8_t TxMailBox_IdleNO_Tbl[];
/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
#if defined ( __CC_ARM )
// �����ж�
__inline t_err CAN_QueueWriteQuick(register T_CanQueue *Queue, T_CanFrame *pData)
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

#if defined ( __ICCARM__ )
#define __inline
t_err CAN_QueueWriteQuick(register T_CanQueue *Queue, T_CanFrame *pData);
#endif

t_err CAN_SendFrame(T_CanFrame *pCanFrame);
void CAN_Config(CAN_TypeDef* CANx);
uint8_t Can_Send_Msg(CAN_TypeDef* CAN_Num, uint8_t* msg, uint8_t len, uint8_t id);
#endif //Rsing_Can.h
