#include <stm32f10x.h>
/*
 * ��������EXTI_Config
 * ����  ������PA0,PA13,PA15Ϊ���жϿڣ��������ж����ȼ�
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void EXTI_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
  	EXTI_InitTypeDef EXTI_InitStructure;
  
    /*��������ʱ��*/
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);

    /* ��ʼ�� GPIOA.0	  ����Ϊ��������*/
  	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*GPIOA.0 �ж�������*/
 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource4);
    
	/*GPIOA.0 �жϳ�ʼ������*/
 	EXTI_InitStructure.EXTI_Line=EXTI_Line4;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	/*����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���*/
  	EXTI_Init(&EXTI_InitStructure);		  	
}


