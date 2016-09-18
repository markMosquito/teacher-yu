#ifndef __IO_SENSOR_H__
#define __IO_SENSOR_H__

#include "Redef_GPIO.h"


#define IO_DIR1_LIMIT PD(2)
#define IO_DIR2_LIMIT PD(3)

#define IO_DIR1_LIMIT_LEVEL IO_GPIO_ReadInputDataBit(IO_DIR1_LIMIT)
#define IO_DIR2_LIMIT_LEVEL IO_GPIO_ReadInputDataBit(IO_DIR2_LIMIT)

void IO_Sensor_Init(void);
#endif

