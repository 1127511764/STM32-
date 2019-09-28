#include<stm32f10x.h>
#include "stm32f10x_conf.h"
//-------------------------------------------------------------------------------------------------------------------
//  @brief      systick��ʱ����
//  @param      time            ��Ҫ��ʱ��ʱ��
//  @return     void
//  @since      v2.0
//  Sample usage:               systick_delay(1000);   //��ʱ1000���ں�ʱ������
//-------------------------------------------------------------------------------------------------------------------
void systick_delay(uint32_t time)
{
    if(time == 0)   return;

    SysTick->CTRL = 0x00;//�ȹ��� systick ,���־λ
    SysTick->LOAD = time;//������ʱʱ��
    SysTick->VAL = 0x00;//��ռ�����
    SysTick->CTRL = ( 0 | SysTick_CTRL_ENABLE_Msk     //ʹ�� systick
                  //| SysTick_CTRL_TICKINT_Msk        //ʹ���ж� (ע���˱�ʾ�ر��ж�)
                    | SysTick_CTRL_CLKSOURCE_Msk      //ʱ��Դѡ�� (core clk)
                );
    while( !(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));//�ȴ�ʱ�䵽
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���뼶systick��ʱ����
//  @param      ms              ��ʱ���ٺ���
//  @return     void
//  @since      v2.0
//  Sample usage:               systick_delay_ms(1000);   //��ʱ1000����
//-------------------------------------------------------------------------------------------------------------------
void delay_ms(uint32_t ms)
{
	while(ms--) systick_delay(72000);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���뼶systick��ʱ����
//  @param      ms              ��ʱ���ٺ���
//  @return     void
//  @since      v2.0
//  Sample usage:               systick_delay_ms(1000);   //��ʱ1000����
//-------------------------------------------------------------------------------------------------------------------
void delay_us(uint32_t us)
{
	while(us--) systick_delay(72);
}



