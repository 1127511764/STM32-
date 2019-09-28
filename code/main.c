/*
飞控的系统架构和智能车相比较为复杂
中断较多
控制中断余隙，主函数里使用了分时调度的思想
这可为学习操作系统打下基础，
如果想深度学习系统和写自己的操作系统
本系统的思想可以作为一种参考，既不失实时性
又不失用户信息传送的功能此系统架构可适用于控制和交互都存在的系统中希望能给学弟们一些提示吧
个人单片机学习所得。。。。。。。。。。。。。。。。。。
*/
 
#include <stm32f10x.h>
#include "math.h"
#include "usart1.h"
#include "time.h"
#include "I2C.h"
#include "systick.h"
#include "MPU9250.h"
#include "moto.h"
#include "control.h"
#include "CXW_Attitude_Algorithm.h"
#include "tft.h"
#include "stdio.h"
#include "exti.h"
u8 ctrl=0,instruction_flag=0;
float H=0,H_old,V=0,pitch_0=0,roll_0=0,yaw_0=0;

void key_down_up(void)
{
	while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1){;}
	delay_ms (100);
  while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==0){;}
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);	
	delay_ms(1000);
	GPIO_SetBits(GPIOC,GPIO_Pin_13);	
	LCD_Clear(WHITE);
}

char  key_down(void)
{
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==0)	
	{
		LCD_Clear(WHITE);
		GPIO_ResetBits(GPIOC,GPIO_Pin_13);	
		delay_ms(1000);
		GPIO_SetBits(GPIOC,GPIO_Pin_13);
		return 1;	
	}
  else
	{
		return 0;
	}
}

void get_hight(void)
{
	static uint8_t f=0;
	f++;
	if(f==5)
	{
		f=0;
	  GPIO_SetBits (GPIOA ,GPIO_Pin_5);	
	  delay_us(10);
	  GPIO_ResetBits (GPIOA ,GPIO_Pin_5);
	}
}

int main(void)//主函数
{
LCD_Init();

{
GPIO_InitTypeDef  GPIO_InitStructure; 
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	
	
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//测试引脚
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
GPIO_Init(GPIOC, &GPIO_InitStructure);
GPIO_SetBits(GPIOC,GPIO_Pin_13);	
	
GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;//按键引脚
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
GPIO_Init(GPIOB, &GPIO_InitStructure);

{//超声波出发引脚初始化
  GPIO_InitTypeDef  GPIO_InitStructure; 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits (GPIOA ,GPIO_Pin_5);	
}
}

	
//中断向量
{
	NVIC_InitTypeDef NVIC_InitStructure;
	//外部中断4   优先级最高
  	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;	
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
  	NVIC_Init(&NVIC_InitStructure); 
	
	/*TIM4系统时间中断 提供系统时间，用于测距的精确时间计算  */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    //串口3预留图像信息通信接口
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* TIM3 控制中断 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* USART1中断 */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
USART1_Config();//控制串口初始化
LCD_ShowString(0,0, "prees key for...");
LCD_ShowString(0,16,"1: Moto_Init ");
LCD_ShowString(0,32,"2: calculate_mpu");
key_down_up();
LCD_ShowString(0,0,"Motor initing...");
Moto_Init();//电机初始化
Init_MPU9250();//初始化MPU9250
Init_HMC5883L();//磁力计初始化	
LCD_ShowString(0,0,"calculating...G");
calculate_mpu_G();
LCD_ShowString(0,0,"calculating...M");
LCD_ShowString(0,16, "prees key finish");
while(key_down()==0)
{
	calculate_mpu_M();//陀螺仪,磁力计校准
	LCD_show_float(0,32 ,mx_max);
	LCD_show_float(0,48 ,mx_min);
	LCD_show_float(0,64 ,my_max);
	LCD_show_float(0,80 ,my_min);
	LCD_show_float(0,96 ,mz_max);
	LCD_show_float(0,112,mz_min);
}
LCD_ShowString(0,0,"ensure attitude");
LCD_ShowString(0,16, "prees key for...");
LCD_ShowString(0,32,"alto contrul");
LCD_ShowString(0,50, "P");
LCD_ShowString(0,65, "R");
LCD_ShowString(0,80, "Y");
LCD_ShowString(0,96,"H");

TIM3_Init(2500);//控制定时器初始化
TIM4_Init(1);
EXTI_Config(); 
while(key_down()==0)
{
 
  {//分时显示
			static char main_T=0;
			main_T++;
		  if(main_T==4)
			{
				main_T=0;
				LCD_show_float(15,96,H);
			}
			if(main_T==3)
			{
				LCD_show_float(15,80,angle.yaw );
			}
			if(main_T==2)
			{
				LCD_show_float(15,65,angle.roll );
			}                     
			if(main_T==1)         
			{                     
				LCD_show_float(15,50,angle.pitch );
			}
			
	 }
}

LCD_ShowString(0,32,"alto ctrling...");
LCD_ShowString(0,50, "P");
LCD_ShowString(0,65, "R");
LCD_ShowString(0,80, "Y");
LCD_ShowString(0,96,"H");
pitch_0=angle.pitch ;
roll_0=angle .roll ;
yaw_0=angle.yaw;
ctrl=1;
	while(1)
	{
		{//分时显示
			static char main_T=0;
			main_T++;
			if(main_T==4)
			{
				main_T=0;
				LCD_show_float(15,96,H);
			}
			if(main_T==3)
			{
				LCD_show_float(15,80,angle.yaw );
			}
			if(main_T==2)
			{
				LCD_show_float(15,65,angle.roll );
			}                     
			if(main_T==1)         
			{                     
				LCD_show_float(15,50,angle.pitch );
			}
	  }		
	}
}

void TIM3_IRQHandler()//控制中断
{
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	READ_MPU9250_ACCEL_GYRO();
  Multiple_Read_HMC5883L();	
	Get_Angle();
	get_hight();

//	wave(gz,gx,gy);
	if(ctrl==1)
	{
		control();
	}
}

int sys_t=0;

void TIM4_IRQHandler()//系统时间更新
{
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	sys_t++;
}

void USART1_IRQHandler(void)//控制串口中断
{
	uint16_t Res;
	Res = USART1->DR;
	if(Res==0xff)
	{
		instruction_flag=1;
	}
  if(Res==0xfe)
	{
		instruction_flag=2;
	}
	USART_ClearFlag(USART1,USART_FLAG_RXNE);
}

float H_Kalman(float X_P,float X_V)//卡尔曼融合
{
	static float  P0_0=0,Kg=0.0,P1_0=0.00001;
	float X;
	P1_0=P0_0+0.8;//系统协方差
	Kg=P1_0/(P1_0+20);//观测协方差
	X=X_P+(X_V-X_P)*Kg;
	P0_0=(1-Kg)*P1_0;
	return X;
}

void EXTI4_IRQHandler(void)//测高中断
{
if(GPIO_ReadInputDataBit (GPIOA,GPIO_Pin_4))//上升沿时间记录
	{
		sys_t=0;
	}
if(!GPIO_ReadInputDataBit (GPIOA,GPIO_Pin_4))//下降沿时间记录高度跟新
	{
		char static f_f=0;
		if(f_f==0)
		{
			H_old=0.340*sys_t;
			f_f=1;
		}
		H=H_Kalman(H_old,0.340*sys_t);
		if(fabs(H-H_old)>50)//限幅滤波
       H=H_old;
		V=H-H_old;
		H_old=H;				
	}
	EXTI_ClearITPendingBit(EXTI_Line4);  
}










