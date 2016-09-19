/*
    FreeRTOS V8.0.1 - Copyright (C) 2014 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that has become a de facto standard.             *
     *                                                                       *
     *    Help yourself get started quickly and support the FreeRTOS         *
     *    project by purchasing a FreeRTOS tutorial book, reference          *
     *    manual, or both from: http://www.FreeRTOS.org/Documentation        *
     *                                                                       *
     *    Thank you!                                                         *
     *                                                                       *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available from the following
    link: http://www.freertos.org/a00114.html

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
    Integrity Systems to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/


/*
 * This version of comtest. c is for use on systems that have limited stack
 * space and no display facilities.  The complete version can be found in
 * the Demo/Common/Full directory.
 *
 * Creates two tasks that operate on an interrupt driven serial port.  A
 * loopback connector should be used so that everything that is transmitted is
 * also received.  The serial port does not use any flow control.  On a
 * standard 9way 'D' connector pins two and three should be connected together.
 *
 * The first task posts a sequence of characters to the Tx queue, toggling an
 * LED on each successful post.  At the end of the sequence it sleeps for a
 * pseudo-random period before resending the same sequence.
 *
 * The UART Tx end interrupt is enabled whenever data is available in the Tx
 * queue.  The Tx end ISR removes a single character from the Tx queue and
 * passes it to the UART for transmission.
 *
 * The second task blocks on the Rx queue waiting for a character to become
 * available.  When the UART Rx end interrupt receives a character it places
 * it in the Rx queue, waking the second task.  The second task checks that the
 * characters removed from the Rx queue form the same sequence as those posted
 * to the Tx queue, and toggles an LED for each correct character.
 *
 * The receiving task is spawned with a higher priority than the transmitting
 * task.  The receiver will therefore wake every time a character is
 * transmitted so neither the Tx or Rx queue should ever hold more than a few
 * characters.
 *
 */

/* Scheduler include files. */
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"

/* Demo program include files. */
#include "serial.h"
#include "comtest.h"
#include "app.h"
#include "Global.h"
#include <stdarg.h>

#define comSTACK_SIZE				        512
/* The Tx task will transmit the sequence of characters at a pseudo random
interval.  This is the maximum and minimum block time between sends. */
//#define comTX_BLOCK_TIME		( ( TickType_t ) 0x32 )

/* We should find that each character can be queued for Tx immediately and we
don't have to block to send. */
#define comNO_BLOCK					( ( TickType_t ) 0 )

/* The Rx task will block on the Rx queue for a long period. */
#define comRX_BLOCK_TIME			( ( TickType_t ) portMAX_DELAY )

#define comBUFFER_LEN				( ( unsigned portBASE_TYPE ) 2048 )//���ڷ��͡����ն��г���

/* Handle to the com port used by both tasks. */
//static xComPortHandle xPort = NULL;

/* The transmit task as described at the top of the file. */
static portTASK_FUNCTION_PROTO( vComTxTask, pvParameters );

/* The receive task as described at the top of the file. */
//static portTASK_FUNCTION_PROTO( vComRxTask, pvParameters );

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
void vAltStartComTestTasks( eCOMPort Port, unsigned portBASE_TYPE uxPriority_TX, unsigned portBASE_TYPE uxPriority_RX, unsigned long ulBaudRate)
{
	/* Initialise the com port then spawn the Rx and Tx tasks. */

	xSerialPortInit(Port, ulBaudRate, comBUFFER_LEN );

	/* The Tx task is spawned with a lower priority than the Rx task. */
	xTaskCreate( vComTxTask, "COMTx", comSTACK_SIZE, NULL, uxPriority_TX, ( TaskHandle_t * ) NULL );
	//xTaskCreate( vComRxTask, "COMRx", comSTACK_SIZE, NULL, uxPriority_RX, ( TaskHandle_t * ) NULL );
}
/*-----------------------------------------------------------*/

//-----------------------------------------------------------------
#include "Com2Server.h"
#include "MyMath.h"
#include "ChassisTask.h"
#include "SlidewayTask.h"
#include "HitTask.h"

extern pid_t PidSlideX, PidSlideY;
extern int16_t hitting;
int16_t correctDis = 0;
extern int16_t currentActual_Main;
extern Elmo elmo[ELMO_NUM + 1];
/**
  * @brief  ���ڣ���ʱ������
  * @param  None					
  * @retval None
  */
static portTASK_FUNCTION( vComTxTask, pvParameters )
{
    TickType_t xLastTESTTime;

    /* Just to stop compiler warnings. */
    ( void ) pvParameters;
    xLastTESTTime = xTaskGetTickCount();
    for( ;; )
    {
        //Elmo_Read_CurrentActual(MOTOR_ID_HIT_MAIN);
        upload( elmo[MOTOR_ID_SLIDEWAY_X].currentActual, elmo[MOTOR_ID_SLIDEWAY_Y].currentActual, elmo[MOTOR_ID_HIT_MAIN].currentActual, elmo[MOTOR_ID_HIT_SERVE].currentActual, 0);
//        #if COM3_DBG == COM_DBG_HITTIM
//        #elif (COM_DBG != COM_RELEASE)
//        if (DbgChoose == COM_DBG_CHASSIS_X)     //����PID-X
//            upload( PidX.set_value, PidX.real_value[NOW], PidX.pid_out, PidX.up, PidX.ud);

//        else if (DbgChoose == COM_DBG_CHASSIS_Y)     //����PID-Y
//            upload( PidY.set_value, PidY.real_value[NOW], PidY.pid_out, PidY.up, PidY.ud);

//        else if (DbgChoose == COM_DBG_SLIDEWAY_X)    //����PID-X
//            upload( PidSlideX.set_value, PidSlideX.real_value[NOW], PidSlideX.pid_out, PidSlideX.ud, hitting);

//        else if (DbgChoose == COM_DBG_SLIDEWAY_Y)    //����PID-Y
//            upload( PidSlideY.set_value, PidSlideY.real_value[NOW], PidSlideY.pid_out, PidSlideX.ud, hitting);

//        else if (DbgChoose == COM_DBG_SLIDEWAY_T)    //����ʱ��
//        {
//            float x0,y0,x1,y1;
//            x0 = (xyzt_structure.x_orl - G_Param.cur_pos.x);
//            y0 = (xyzt_structure.y_orl - G_Param.cur_pos.y);
//            
//            x1 = (x0 * my_cos(G_Param.cur_pos.ang) + y0 * my_sin(G_Param.cur_pos.ang)) - pRacketReceive->XOff;
//            y1 = (y0 * my_cos(G_Param.cur_pos.ang) - x0 * my_sin(G_Param.cur_pos.ang)) - pRacketReceive->YOff;
//            
//            /* Ӧ����X�� ʵ����X�� Ӧ����Y�� ʵ����Y�� ����״̬*/
//            upload(x1, G_Param.slideway_x_mm, y1, G_Param.slideway_y_mm, hitting);
//        }
//        
//        else if (DbgChoose == COM_DBG_ALL)
//        {
//            int32_t racketx0, rackety0;
//            int32_t racketx, rackety;
//            
//            racketx0 = G_Param.slideway_x_mm + pRacketReceive->XOff;
//            rackety0 = G_Param.slideway_y_mm + pRacketReceive->YOff;
//            racketx = racketx0*my_cos(G_Param.cur_pos.ang) - rackety0*my_sin(G_Param.cur_pos.ang) + G_Param.cur_pos.x;
//            rackety = racketx0*my_sin(G_Param.cur_pos.ang) + rackety0*my_cos(G_Param.cur_pos.ang) + G_Param.cur_pos.y;
//            /* ���X�� ����X�� ���Y�� ����Y�� ����״̬*/
//            upload(xyzt_structure.x_orl, racketx, xyzt_structure.y_orl, rackety, hitting);
//        }
//        #else
////            int32_t racketx0, rackety0;
////            int32_t racketx, rackety;
////            
////            racketx0 = G_Param.slideway_x_mm + RacketOffX[racketSelect];
////            rackety0 = G_Param.slideway_y_mm + RacketOffY[racketSelect];
////            racketx = racketx0*my_cos(G_Param.cur_pos.ang) - rackety0*my_sin(G_Param.cur_pos.ang) + G_Param.cur_pos.x;
////            rackety = racketx0*my_sin(G_Param.cur_pos.ang) + rackety0*my_cos(G_Param.cur_pos.ang) + G_Param.cur_pos.y;
////            racketx = xyzt_structure.x_orl - racketx;
////            rackety = xyzt_structure.y_orl - rackety;
////            if(xyzt_structure.running_flag == 1)
////            /* ���X�� ���X�� ���Y�� ���Y�� ����״̬*/
////            upload(xyzt_structure.x_orl, racketx, xyzt_structure.y_orl, rackety, hitting);

////        float x0,y0,x1,y1;
////            x0 = (xyzt_structure.x_orl - G_Param.cur_pos.x);
////            y0 = (xyzt_structure.y_orl - G_Param.cur_pos.y);
////            
////            x1 = (x0 * my_cos(G_Param.cur_pos.ang) + y0 * my_sin(G_Param.cur_pos.ang)) - pRacketReceive->XOff;
////            y1 = (y0 * my_cos(G_Param.cur_pos.ang) - x0 * my_sin(G_Param.cur_pos.ang)) - pRacketReceive->YOff;
////            if(xyzt_structure.running_flag == 1)
////            /* Ӧ����X�� ʵ����X�� Ӧ����Y�� ʵ����Y�� ����״̬*/
////            upload(x1, G_Param.slideway_x_mm, y1, G_Param.slideway_y_mm, hitting);
//        
//        upload(G_Param.speed.x, G_Param.speed.y, G_Param.speed.ang*100, correctDis, hitting);
//        if(correctDis > 0)
//            correctDis = -16383;
//        #endif
        vTaskDelayUntil(&xLastTESTTime, 5 * portTICK_MS);
    }
}

/*-----------------------------------------------------------*/

/**
  * @brief  ���ڽ��մ���
  * @param  None					
  * @retval None
  */
//UsartRxMsg UsartRx3;
//static portTASK_FUNCTION( vComRxTask, pvParameters )
//{
//    uint8_t come_data;
//    /* Just to stop compiler warnings. */
//	( void ) pvParameters;
//    
//	for( ;; )
//	{
//        xSerialGetChar( 0, (signed char *)&come_data, comRX_BLOCK_TIME );
//        if (come_data == 0xff)  /* ���������ʼ�ֽ� */
//        {
//            UsartRx3.DLC = 0;
//        }
//        else
//        {
//            UsartRx3.Data[UsartRx3.DLC++] = come_data;
//        }
//	}
//} 
/*-----------------------------------------------------------*/

//��λ������-----------------------------------------------------------------
/*****************************************************************************************
* ��������Sum_Check
* ����  ��8λУ���������
* ����  :--unsigned char *Buf: �ݴ�Ҫ��������ݵ�������׵�ַ
* 		 unsigned char CRC_CNT ���ݴ�Ҫ��������ݵ�����Ĵ�С
* ���  ��У����
* ע��  ����
* ���ߣ�Zeus
*****************************************************************************************/
static unsigned short Sum_Check(unsigned char *Buf, unsigned char Sum_Cnt)
{
    uint16_t Sum_Temp=0;	
    unsigned char i;

    for (i = 0;i < Sum_Cnt; i ++)
    {      
    	Sum_Temp += (uint8_t)*Buf;
    	Buf ++;
    }
    return((uint8_t)(Sum_Temp%256));
}
/*****************************************************************************************
* ��������upload
* ����  �����������������ʾ����Visual_Scope��
* ����  :--int16_t Channel_1_Data : Ҫ�����ͨ��һ������
* 		int16_t Channel_2_Data ��Ҫ�����ͨ����������
* 		int16_t Channel_3_Data ��Ҫ�����ͨ����������
* 		int16_t Channel_4_Data: Ҫ�����ͨ���ĵ�����
		int16_t Channel_5_Data: Ҫ�����ͨ���ĵ�����
* ���  ����
* ע��  ����
* ���ߣ�Zeus
*****************************************************************************************/
void upload(int16_t Channel_1_Data ,int16_t Channel_2_Data,
		int16_t Channel_3_Data,int16_t Channel_4_Data,int16_t Channel_5_Data)
{
    unsigned int temp[5] = {0};
    unsigned char databuf[11] = {0};
    unsigned char startFlag[2] = {0};
    unsigned char i;

    if(Channel_1_Data == 0x7fff)
    Channel_1_Data = 0x7ffe;
    if(Channel_2_Data == 0x7fff)
    Channel_2_Data = 0x7ffe;
    if(Channel_3_Data == 0x7fff)
    Channel_3_Data = 0x7ffe;
    if(Channel_4_Data == 0x7fff)
    Channel_4_Data = 0x7ffe;
    if(Channel_5_Data == 0x7fff)
    Channel_5_Data = 0x7ffe;


    temp[0] = (unsigned int)Channel_1_Data;
    temp[1] = (unsigned int)Channel_2_Data;
    temp[2] = (unsigned int)Channel_3_Data;
    temp[3] = (unsigned int)Channel_4_Data;
    temp[4] = (unsigned int)Channel_5_Data;

    for(i=0;i<5;i++) 
    {
        databuf[i*2]   = (int8_t)((temp[i]&0xFF00) >> 8);
        databuf[i*2+1] = (int8_t)(temp[i]&0xFF);
    }

    databuf[10] = (uint8_t)Sum_Check(databuf,10);

    startFlag[0] = 0x7f;
    startFlag[1] = 0xff;
    
    for(i=0;i<2;i++)
    {
		xSerialPutChar( 0, startFlag[i], 0 );
    }

    for(i=0;i<11;i++)
    {
        xSerialPutChar( 0, databuf[i], 0 );
    }
}

