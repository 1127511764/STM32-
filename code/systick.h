#ifndef _SYSTICK_H_
#define _SYSTICK_H_

#include<stm32f10x.h>

void    systick_delay(uint32_t time);
void    delay_ms(uint32_t ms);
void    delay_us(uint32_t us);

#endif

