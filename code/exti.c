#include <stm32f10x.h>
/*
 * 函数名：EXTI_Config
 * 描述  ：配置PA0,PA13,PA15为线中断口，并设置中断优先级
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void EXTI_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
  	EXTI_InitTypeDef EXTI_InitStructure;
  
    /*开启外设时钟*/
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);

    /* 初始化 GPIOA.0	  设置为下拉输入*/
  	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*GPIOA.0 中断线配置*/
 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource4);
    
	/*GPIOA.0 中断初始化配置*/
 	EXTI_InitStructure.EXTI_Line=EXTI_Line4;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	/*根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器*/
  	EXTI_Init(&EXTI_InitStructure);		  	
}


