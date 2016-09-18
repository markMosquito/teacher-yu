#ifndef __AMC_H__
#define __AMC_H__

#include "stm32f4xx.h"


#define MOTOR_MARCH_1                 (1)
#define MOTOR_MARCH_2                 (2)

#define MOTOR_DIR_1                 (3)
#define MOTOR_DIR_2                 (4)

typedef struct 
{
	int32_t speed;
    float angle;
	float wheel_angle;
    
}
motion_target;


u16 crcCal(u8 *data,u16 length);
#endif
