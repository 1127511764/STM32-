#include "moto.h"
#include "systick.h"
//无需用户调用
void Tim2_init(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  			TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/**********************************************************
	72 000 000/72=1M
	1000 000/2500=400Hz
	所以产生的PWM为400Hz
	周期为2.5ms，对应2500的计算值，1ms~2ms对应的计算值为1000~2000；
	**********************************************************/
	TIM_TimeBaseStructure.TIM_Period = 2499;		//计数上线	
	TIM_TimeBaseStructure.TIM_Prescaler = 71;	//pwm时钟分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 2000;       //初始占空比为0
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);	
}

void Moto_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  /* GPIOA and GPIOC clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	Tim2_init();
  delay_ms (1500);
	Moto_Pwm(1000,1000,1000,1000);
	delay_ms (1500);
}

void Moto_Pwm(uint16_t MOTO1_PWM,uint16_t MOTO2_PWM,uint16_t MOTO3_PWM,uint16_t MOTO4_PWM)
{		
	if(MOTO1_PWM>2000)	MOTO1_PWM = 2000;
	if(MOTO2_PWM>2000)	MOTO2_PWM = 2000;
	if(MOTO3_PWM>2000)	MOTO3_PWM = 2000;
	if(MOTO4_PWM>2000)	MOTO4_PWM = 2000;
	
	if(MOTO1_PWM<1000)	MOTO1_PWM =1000 ;
	if(MOTO2_PWM<1000)	MOTO2_PWM =1000 ;
	if(MOTO3_PWM<1000)	MOTO3_PWM =1000 ;
	if(MOTO4_PWM<1000)	MOTO4_PWM =1000 ;
	
	TIM2->CCR1 = MOTO1_PWM;
	TIM2->CCR2 = MOTO2_PWM;
	TIM2->CCR3 = MOTO3_PWM;
	TIM2->CCR4 = MOTO4_PWM;
}
