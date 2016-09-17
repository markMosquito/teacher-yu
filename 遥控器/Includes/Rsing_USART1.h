#ifndef __RSING_USART1_H
#define	__RSING_USART1_H

#include "stm32f10x.h"
#include <stdio.h>

extern void USART1_Config(void);
extern int fputc(int ch, FILE *f);
extern void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...);

#endif /* __RSING_USART1_H */
