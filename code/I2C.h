#ifndef _I2C_H_
#define _I2C_H_


#include<stm32f10x.h>
typedef enum {FALSE = 0, TRUE = !FALSE} bool;
//************************************
/*模拟IIC端口输出输入定义*/
#define SCL_H         GPIOB->BSRR = GPIO_Pin_7
#define SCL_L         GPIOB->BRR  = GPIO_Pin_7 
   
#define SDA_H         GPIOB->BSRR = GPIO_Pin_6
#define SDA_L         GPIOB->BRR  = GPIO_Pin_6

#define SCL_read      GPIOB->IDR  & GPIO_Pin_7
#define SDA_read      GPIOB->IDR  & GPIO_Pin_6

void I2C_GPIO_Config(void);
bool Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);	
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address);
bool I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
bool I2C_WaitAck(void);
void I2C_SendByte(u8 SendByte) ;
unsigned char I2C_RadeByte(void) ;





#endif




