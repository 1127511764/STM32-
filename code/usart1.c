/******************** CXW **************************
 USARTP配置
**********************************************************************************/

#include "usart1.h"
#include "stm32f10x.h"
#include<stm32f10x_conf.h>
void USART1_Config(void)//串口1
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* 配置串口1 (USART1) 时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	/*串口GPIO端口配置*/
	/* 配置串口1 (USART1 Tx (PA.09))*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
	/* 配置串口1 USART1 Rx (PA.10)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 串口1工作模式(USART1 mode)配置 */
	USART_InitStructure.USART_BaudRate =256000 ;//9600;////波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure); 
	USART_Cmd(USART1, ENABLE);//使能串口
	NVIC_EnableIRQ(USART1_IRQn);	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);	
	}	

void USART2_Config(void)//串口2
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	USART_InitStructure.USART_BaudRate = 115200;//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure); 
	USART_Cmd(USART2, ENABLE);//使能串口
	NVIC_EnableIRQ(USART2_IRQn);
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
}
void usart3_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 ,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  ,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 256000;                 /*设置波特率为115200*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  /*设置数据位为8位*/
    USART_InitStructure.USART_StopBits = USART_StopBits_1;       /*设置停止位为1位*/
    USART_InitStructure.USART_Parity = USART_Parity_No;          /*无奇偶校验*/    
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*没有硬件流控*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;      /*发送与接收*/
    /*完成串口COM1的时钟配置、GPIO配置，根据上述参数初始化并使能*/
	
	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE);
	NVIC_EnableIRQ(USART3_IRQn);	
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);	
}
void USART3_sendData(uint16_t Data)
{
  USART3->DR = (Data & (uint16_t)0x01FF);
  while( USART_GetFlagStatus(USART3,USART_FLAG_TC)!= SET);
}
void USART_sendData(USART_TypeDef* USARTx, uint16_t Data)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_DATA(Data)); 
    
  /* Transmit Data */
  USARTx->DR = (Data & (uint16_t)0x01FF);
	while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);
}


/*
 * 函数名：fputc
 * 描述  ：重定向c库函数printf到USART1
 * 输入  ：无
 * 输出  ：无
 * 调用  ：由printf调用
 */
int fputc(int ch, FILE *f)
{
	/* 将Printf内容发往串口 */
	USART_sendData(USART1, (unsigned char) ch);	
	return (ch);
}

/**
  * @brief  山外调试上位机 虚拟示波器专用用函数
  * @param    
  * @retval 
 **/
void print_float(float dat)
{
	
	union 
	{
		char c;
		float f;
	}fl;
	char *ch=&fl.c;
	int i;
	fl.f =dat;
	for(i=0;i<sizeof(float);i++)
	USART_sendData(USART1,*(ch+i));
}
//************************************************************************************  示波器//耗时2.3ms
void  wave(float dat1,float dat2,float dat3/*,float dat4,float dat5,float dat6,float dat7,float dat8.....*/)
{
	USART_sendData(USART1,0x03);
	USART_sendData(USART1,0xFC);
	print_float(dat1);
	print_float(dat2);	
	print_float(dat3);
	
/*  print_float(dat4);
print_float(dat5);
	print_float(dat6);
	print_float(dat7);......
	*/
	USART_sendData(USART1,0xFC);
	USART_sendData(USART1,0x03);	
}
//************************************************************************************  示波器//耗时2.3ms
void  wave8(float dat1,float dat2,float dat3,float dat4,float dat5,float dat6,float dat7,float dat8/*.....*/)
{
	USART_sendData(USART1,0x03);
	USART_sendData(USART1,0xFC);
	print_float(dat1);
	print_float(dat2);	
	print_float(dat3);	
  print_float(dat4);
  print_float(dat5);
	print_float(dat6);
	print_float(dat7);
	print_float(dat8);
	/**/
	USART_sendData(USART1,0xFC);
	USART_sendData(USART1,0x03);	
}
