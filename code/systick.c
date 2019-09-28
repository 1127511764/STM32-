#include<stm32f10x.h>
#include "stm32f10x_conf.h"
//-------------------------------------------------------------------------------------------------------------------
//  @brief      systick延时函数
//  @param      time            需要延时的时间
//  @return     void
//  @since      v2.0
//  Sample usage:               systick_delay(1000);   //延时1000个内核时钟周期
//-------------------------------------------------------------------------------------------------------------------
void systick_delay(uint32_t time)
{
    if(time == 0)   return;

    SysTick->CTRL = 0x00;//先关了 systick ,清标志位
    SysTick->LOAD = time;//设置延时时间
    SysTick->VAL = 0x00;//清空计数器
    SysTick->CTRL = ( 0 | SysTick_CTRL_ENABLE_Msk     //使能 systick
                  //| SysTick_CTRL_TICKINT_Msk        //使能中断 (注释了表示关闭中断)
                    | SysTick_CTRL_CLKSOURCE_Msk      //时钟源选择 (core clk)
                );
    while( !(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));//等待时间到
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      毫秒级systick延时函数
//  @param      ms              延时多少毫秒
//  @return     void
//  @since      v2.0
//  Sample usage:               systick_delay_ms(1000);   //延时1000毫秒
//-------------------------------------------------------------------------------------------------------------------
void delay_ms(uint32_t ms)
{
	while(ms--) systick_delay(72000);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      毫秒级systick延时函数
//  @param      ms              延时多少毫秒
//  @return     void
//  @since      v2.0
//  Sample usage:               systick_delay_ms(1000);   //延时1000毫秒
//-------------------------------------------------------------------------------------------------------------------
void delay_us(uint32_t us)
{
	while(us--) systick_delay(72);
}



