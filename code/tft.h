#ifndef _TFT_H_
#define _TFT_H_

#include "systick.h"	 
#include "stdlib.h"
#include<stm32f10x.h>

/****************************************************************************************************
//=======================================Һ���������߽���==========================================//
//CLK	      ��PB13	//SPIʱ���ź�
//SDI(DIN)	��PB15	//SPI���������ź�
//RS(D/C)   ��PB10	//�Ĵ���/����ѡ���ź�(RS=0�������߷��͵���ָ�RS=1�������߷��͵�����������)

//==================================����л���������ʾ=======================================//
//��lcd.hͷ�ļ����޸ĺ�#define USE_HORIZONTAL ֵΪ0ʹ������ģʽ.1,ʹ�ú���ģʽ
**************************************************************************************************/	

//LCD��Ҫ������
typedef struct  
{										    
	u16 width;			//LCD ���
	u16 height;			//LCD �߶�
	u16 id;				//LCD ID
	u8  dir;			//���������������ƣ�0��������1��������	
	u16	 wramcmd;		//��ʼдgramָ��
	u16  setxcmd;		//����x����ָ��
	u16  setycmd;		//����y����ָ��	 
}_lcd_dev; 	

//LCD����
extern _lcd_dev lcddev;	//����LCD��Ҫ����
/////////////////////////////////////�û�������///////////////////////////////////	 
//֧�ֺ��������ٶ����л���֧��8/16λģʽ�л�
#define USE_HORIZONTAL  	1	//�����Ƿ�ʹ�ú��� 		0,��ʹ��.1,ʹ��.
//////////////////////////////////////////////////////////////////////////////////	  
//����LCD�ĳߴ�
#if USE_HORIZONTAL==1	//ʹ�ú���
#define LCD_W 128
#define LCD_H 128
#else
#define LCD_W 128
#define LCD_H 128
#endif

//TFTLCD������Ҫ���õĺ���		   
extern u16  POINT_COLOR;//Ĭ�Ϻ�ɫ    
extern u16  BACK_COLOR; //������ɫ.Ĭ��Ϊ��ɫ

////////////////////////////////////////////////////////////////////
//-----------------LCD�˿ڶ���---------------- 
//QDtechȫϵ��ģ������������ܿ��Ʊ��������û�Ҳ���Խ�PWM���ڱ�������
//�ӿڶ�����Lcd_Driver.h�ڶ��壬����ݽ����޸Ĳ��޸���ӦIO��ʼ��LCD_GPIO_Init()
#define LCD_CTRL   	  	GPIOB		//����TFT���ݶ˿�
#define RCC_LCD_CTRL    RCC_APB2Periph_GPIOB
//#define LCD_RS         	GPIO_Pin_14//PB10������TFT --RS
//#define LCD_SCL        	GPIO_Pin_15//PB13������TFT -- CLK
//#define LCD_SDA        	GPIO_Pin_13//PB15������TFT - SDI
#define LCD_RS         	GPIO_Pin_15//GPIO_Pin_14//PB10������TFT --RS
#define LCD_SCL        	GPIO_Pin_14//GPIO_Pin_15//PB13������TFT -- CLK
#define LCD_SDA        	GPIO_Pin_13//GPIO_Pin_13//PB15������TFT - SDI
							    
//////////////////////////////////////////////////////////////////////
//Һ�����ƿ���1�������궨��  
#define	LCD_RS_SET  	LCD_CTRL->BSRR=LCD_RS    
#define	LCD_SDA_SET  	LCD_CTRL->BSRR=LCD_SDA    
#define	LCD_SCL_SET  	LCD_CTRL->BSRR=LCD_SCL        


//Һ�����ƿ���0�������궨��  
#define	LCD_RS_CLR  	LCD_CTRL->BRR=LCD_RS    
#define	LCD_SDA_CLR  	LCD_CTRL->BRR=LCD_SDA    
#define	LCD_SCL_CLR  	LCD_CTRL->BRR=LCD_SCL      


//������ɫ
#define WHITE       0xFFFF
#define BLACK      	0x0000	  
#define BLUE       	0x001F  
#define BRED        0XF81F
#define GRED 			 	0XFFE0
#define GBLUE			 	0X07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define BROWN 			0XBC40 //��ɫ
#define BRRED 			0XFC07 //�غ�ɫ
#define GRAY  			0X8430 //��ɫ
//GUI��ɫ

#define DARKBLUE      	 0X01CF	//����ɫ
#define LIGHTBLUE      	 0X7D7C	//ǳ��ɫ  
#define GRAYBLUE       	 0X5458 //����ɫ
//������ɫΪPANEL����ɫ 
 
#define LIGHTGREEN     	0X841F //ǳ��ɫ
//#define LIGHTGRAY     0XEF5B //ǳ��ɫ(PANNEL)
#define LGRAY 			 		0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ

#define LGRAYBLUE      	0XA651 //ǳ����ɫ(�м����ɫ)
#define LBBLUE          0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)
	    															  
extern u16 BACK_COLOR, POINT_COLOR ;  

void LCD_Init(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);
void LCD_Clear(u16 Color);	 
void LCD_SetCursor(u16 Xpos, u16 Ypos);  
void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd);								    
void LCD_WriteReg(u16 LCD_Reg, u16 LCD_RegValue);
void LCD_WR_DATA(u8 data);
void LCD_WR_DATA_16Bit(u16 data);
void LCD_WriteRAM_Prepare(void);
void LCD_SetParam(void);
void Gui_Drawbmp16(u16 x,u16 y,u16 w,u16 h,const unsigned char *p) ;
void LCD_ShowChar(u8 x,u8 y,u8 chr);
void LCD_ShowString(u8 x,u8 y,char *chr);
void LCD_show_float(u16 x,u16 y,double dat);

extern const unsigned char gImage_aa[35000];
//	Gui_Drawbmp16(35,20,75,100,gImage_aa);
extern const unsigned char F8X16[];

#endif

