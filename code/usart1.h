#ifndef __USART1_H
#define	__USART1_H

#include "stm32f10x.h"
#include <stdio.h>

void USART1_Config(void);
void USART2_Config(void);//´®¿Ú1
void usart3_config(void);
int fputc(int ch, FILE *f);
void USART3_sendData(uint16_t Data);
void print_float(float dat);
void USART_sendData(USART_TypeDef* USARTx, uint16_t Data);
void wave(float dat1,float dat2,float dat3/*,float dat4,float dat5,float dat6,float dat7,float dat8.....*/);
void  wave8(float dat1,float dat2,float dat3,float dat4,float dat5,float dat6,float dat7,float dat8/*.....*/);
#endif /* __USART1_H */
