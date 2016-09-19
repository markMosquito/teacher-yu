#include "IO_Sensor.h"

void IO_Sensor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(IO_RCC_AHB1_PERIPH(IO_DIR1_LIMIT), ENABLE);
	RCC_AHB1PeriphClockCmd(IO_RCC_AHB1_PERIPH(IO_DIR2_LIMIT), ENABLE);

	/* Configure IO_DIR1_LIMIT */
	GPIO_InitStructure.GPIO_Pin = IO_GPIO_PIN(IO_DIR1_LIMIT);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init( IO_GPIO_PORT(IO_DIR1_LIMIT), &GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = IO_GPIO_PIN(IO_DIR2_LIMIT);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init( IO_GPIO_PORT(IO_DIR2_LIMIT), &GPIO_InitStructure );

}
