/*
********************************************************************************
	*@  file: 007_elmo.h
	*@  author: 007
	*@  data: 3/20/2015
	*@  version: v2.0
	*@  brief: �����޸ģ���Ҫ�Ҷ�������
	*...............................................................................
	*@
	*@ Notes: (1) �޸�ElMO_NUM��ֵȷ��Elmo����,����Ϊ4��Elmo.
	*@            ��������Elmo,�ڼӺź������.
	*@        (2) �޸ĵ������,�����,��ֵ����,������ת��.
	*@            ��Щ�ǵ��̵ĵ���Ĳ���.����ʹ������Elmo,��ֱ�ӵ��ú���.
	*@            ��������������������Ϊ����5��ID��.
	*@        (3) ***********************************************************
	*@            *  MAXON����ͺ�       �ת��(RPM)  �����(A) ����(W) *
	*@            *  EC-4pole-30-305013  16200          9.21        200     *
	*@            *  RE-40-148867        6930           5.77        150     *
	*@            *  RE-30-310007        8050           3.44        60      *
	*@            *  RE-25-339152        9620           1.42        20      *
	*@            ***********************************************************
	*@
	               
********************************************************************************
*/

#ifndef __007_ELMO_H
#define __007_ELMO_H
#include "stm32f4xx.h"
#include <math.h>
#include <string.h>
#include "MotorConfig.h"

/*******************************************************************************
* �궨�� 
*******************************************************************************/
/* 0:���ٶȿ���ģʽ�������ٶ�ģʽ,��ʱ���ٶ�ʧЧ */ 
/* 1:��λ�ÿ���ģʽ�������ٶ�ģʽ,��ʱ���ٶ���Ч */ 
#define  JV_IN_PCM          0


/* ѭ�����в��� */
#define ELMO_NUM            (8)                             // Elmo����,�����ϸ��չ��ظ�����ELMO���ã����ö����ã�����
#define CAN_BUF_NUM         1000                            //����ָ������
#define CAN_ID_DELAY        0x128                           //��ʱָ��

/* ELMO��ز��� */
#define RATE_CURRENT        9.210                           // �����(A)
#define PEAK_CURRENT        (9.210*2)                       // ��ֵ����(A)
#define MAX_VOLOCITY        16200                           // ������ת��(rpm)

/* ����������ز��� */
#define RATE_CUR            RATE_CURRENT                    // �����(A)          CL[1]
#define MAX_CURRENT         PEAK_CURRENT                    // ��ֵ����(A)          PL[1]
#define MAX_PM_SPEED        (MAX_VOLOCITY*2000/60)          // ���ƽ���ٶ�(cnt/s)  VH[2]
#define MIN_PM_SPEED        (u32)(-MAX_PM_SPEED)            // ��Сƽ���ٶ�(cnt/s)  VL[2]
#define MAX_FB_SPEED        ((MAX_VOLOCITY+1000)*2000/60)   // ������ٶ�(cnt/s)  HL[2]
#define MIN_FB_SPEED        (u32)(-MAX_FB_SPEED)            // ��С�����ٶ�(cnt/s)  LL[2]

/* ƽ���˶���ز��� */ 
#define PM_ACC              3500000                         // ƽ�����ٶ�(cnt/s^2)  AC
#define PM_DEC              3500000                         // ƽ�����ٶ�(cnt/s^2)  DC
#define QUICKSTOP_DEC       1000000000                      // ��ͣ���ٶ�(cnt/s^2)  SD
#define POSITION_LIMIT_MAX  1000000000                      // ���λ�ü���         VH[3] HL[3]
#define POSITION_LIMIT_MIN  (u32)-1000000000                // ��Сλ�ü���         VL[3] LL[3]

/* CANopen COB identifiers */
#define COBID_NMT_SERVICE   0x000
#define COBID_SYNC          0x080
#define COBID_EMERGENCY     0x080
#define COBID_TIME_STAMP    0x100
#define COBID_TPDO1         0x180
#define COBID_RPDO1         0x200
#define COBID_TPDO2         0x280
#define COBID_RPDO2         0x300
#define COBID_TPDO3         0x380
#define COBID_RPDO3         0x400
#define COBID_TPDO4         0x480
#define COBID_RPDO4         0x500
#define COBID_TSDO          0x580
#define COBID_RSDO          0x600
#define COBID_HEARTBEAT     0x700

/* NMT Command Specifier */
#define NMT_ENTER_OPERATIONAL      0x01
#define NMT_ENTER_STOPPED          0x02
#define NMT_ENTER_PRE_OPERATIONAL  0x80
#define NMT_RESET_NODE             0x81
#define NMT_RESET_COMMUNICATION    0x82

/* Binary Interpreter Command */
#define UM_IDLE             0x00
#define UM_TCM              0x01                      // Torque control mode,���ؿ���ģʽ
#define UM_PCM              0x05                      // Position control mode��λ�ÿ���ģʽ
#define UM_UNC              0x06                      // ����ȷ��ģʽ��������ȫ���������

#if JV_IN_PCM
   #define UM_SCM           0x05                      // Position control mode��
#else
   #define UM_SCM           0x02                      // Speed control mode
#endif

#define TYPE_INTEGER      0
#define TYPE_FLOAT        1

#define MO_OFF            0
#define MO_ON             1

#define POS_REL           0
#define POS_ABS           1



/*******************************************************************************
* �ṹ��
*******************************************************************************/
/* CANѭ������Ԫ�� */ 
typedef struct __CANDATA 
{
   u16 COBID;               // CANopen COB identifier + NodeID
   u8  DLC;                 // Data Length Code
   u8  DATA[8];             // Data
} CANDATA;

/* CANѭ�����нṹ�� */
typedef struct __CANQUEUE
{
   u16 Front;        
   u16 Rear;
   CANDATA CANBUF[CAN_BUF_NUM];
} CANQUEUE;

/* Elmo�ṹ��,��¼�ڵ�ID,״̬�Ϳ��Ʋ��� */
typedef struct __Elmo
{
    u8 NodeID;         // elmo����
    u8 CurOPMode;      // ��ǰ����ģʽ
    int16_t currentActual;
    int32_t positionActual;
    int32_t velocityActual;
}Elmo;

typedef struct 
{
    Elmo ElmoBase;
    u8 Start;
    u8 End;
}ElmoGroupTypedef;

/*����ת��������*/
typedef union 
{
    uint32_t u32_form;
    int32_t  s32_form;
    uint8_t  u8_form[4];
    int8_t   s8_form[4];
    float    float_form;
}DataConvert;



/*******************************************************************************
* ��������
*******************************************************************************/
/* ELMO��ʼ������,������� */
extern u32 Elmo_Init( CAN_TypeDef* CANx, uint8_t PPr, uint8_t SPr);
extern void Elmo_Reinit(void);

/* ELMO���ƺ������������ */
extern u8 Elmo_PTM(u8 elmoID, float torque);
extern u8 Elmo_PVM(u8 elmoID, s32 speed);  
extern u8 Elmo_PPM(u8 elmoID, u32 speed, s32 position, u8 PPMmode);		//		POS_ABS  POS_REL 
extern u8 Elmo_Close(u8 elmoID);
extern u8 Elmo_Stop(u8 elmoID);
extern u8 Elmo_SetAcc(u8 elmoID, u32 acc, u32 dec);
extern uint32_t Elmo_Read_POS(u8 elmoID, int32_t *pEncData);
extern void Elmo_Set_POS(u8 elmoID,s32 POS);
extern uint32_t Elmo_Read_CurrentActual(u8 elmoID);

/* CANOpen��ʵ�ֺ���,��������� */
static void NMTCmd(u8 NodeID, u8 MNTCmd);
static void RPDO2_Cmd_data(u8 NodeID, u8 *Cmd, u8 Index, u8 Type, u32 Data);
static void RPDO2_Cmd_string(u8 NodeID, u8 *Cmd);

/* Ӳ����ʼ������,��������� */
static void CAN_init(CAN_TypeDef* CANx);
static int Self_test(void);
static void Variate_init(void);

/* ���ݷ��͡�ת������ʱ����,��������� */
static void Elmo_SendCmd(void);
//static void Elmo_software_delay_ms(unsigned int t);
static u32 f2h(float x);

/* SDO */
void RSDO(u8 NodeID, u16 Index, u8 SubIndex, u32 Data);
void RSDORead(u8 NodeID, u16 Index, u8 SubIndex, u32 Data);

/* ����������elmo */
void SendHeart2Elmo(void);

// GroupControl
void Chassis_Elmo_PVM(s32 speed1, s32 speed2, s32 speed3, s32 speed4);
void Slideway_Elmo_PVM(s32 speed1, s32 speed2);
void Chassis_Elmo_Stop(void);
void Slideway_Elmo_Stop(void);
void Chassis_Elmo_Close(void);
void Slideway_Elmo_Close(void);

u8 Elmo_SetCurrent(u8 elmoID, float rateCurrent, float maxCurrent);
void Elmo_Read_PositionActualRequest(u8 elmoID);
uint32_t Elmo_Read_PositionActual(u8 elmoID, int32_t *pos);
uint32_t Elmo_Read_VelocityActual(u8 elmoID, int32_t *val);

extern Elmo elmo[ELMO_NUM + 1];
#endif


