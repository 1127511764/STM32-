#ifndef _MOTO_H_
#define _MOTO_H_
#include "stm32f10x.h"

#define Moto_PwmMax 2000

void Moto_Pwm(uint16_t MOTO1_PWM,uint16_t MOTO2_PWM,uint16_t MOTO3_PWM,uint16_t MOTO4_PWM);
void Moto_Init(void);

#endif
