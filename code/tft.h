#ifndef _TFT_H_
#define _TFT_H_

#include "systick.h"	 
#include "stdlib.h"
#include<stm32f10x.h>

/****************************************************************************************************
//=======================================液晶屏数据线接线==========================================//
//CLK	      接PB13	//SPI时钟信号
//SDI(DIN)	接PB15	//SPI总线数据信号
//RS(D/C)   接PB10	//寄存器/数据选择信号(RS=0数据总线发送的是指令；RS=1数据总线发送的是像素数据)

//==================================如何切换横竖屏显示=======================================//
//打开lcd.h头文件，修改宏#define USE_HORIZONTAL 值为0使用竖屏模式.1,使用横屏模式
**************************************************************************************************/	

//LCD重要参数集
typedef struct  
{										    
	u16 width;			//LCD 宽度
	u16 height;			//LCD 高度
	u16 id;				//LCD ID
	u8  dir;			//横屏还是竖屏控制：0，竖屏；1，横屏。	
	u16	 wramcmd;		//开始写gram指令
	u16  setxcmd;		//设置x坐标指令
	u16  setycmd;		//设置y坐标指令	 
}_lcd_dev; 	

//LCD参数
extern _lcd_dev lcddev;	//管理LCD重要参数
/////////////////////////////////////用户配置区///////////////////////////////////	 
//支持横竖屏快速定义切换，支持8/16位模式切换
#define USE_HORIZONTAL  	1	//定义是否使用横屏 		0,不使用.1,使用.
//////////////////////////////////////////////////////////////////////////////////	  
//定义LCD的尺寸
#if USE_HORIZONTAL==1	//使用横屏
#define LCD_W 128
#define LCD_H 128
#else
#define LCD_W 128
#define LCD_H 128
#endif

//TFTLCD部分外要调用的函数		   
extern u16  POINT_COLOR;//默认红色    
extern u16  BACK_COLOR; //背景颜色.默认为白色

////////////////////////////////////////////////////////////////////
//-----------------LCD端口定义---------------- 
//QDtech全系列模块采用了三极管控制背光亮灭，用户也可以接PWM调节背光亮度
//接口定义在Lcd_Driver.h内定义，请根据接线修改并修改相应IO初始化LCD_GPIO_Init()
#define LCD_CTRL   	  	GPIOB		//定义TFT数据端口
#define RCC_LCD_CTRL    RCC_APB2Periph_GPIOB
//#define LCD_RS         	GPIO_Pin_14//PB10连接至TFT --RS
//#define LCD_SCL        	GPIO_Pin_15//PB13连接至TFT -- CLK
//#define LCD_SDA        	GPIO_Pin_13//PB15连接至TFT - SDI
#define LCD_RS         	GPIO_Pin_15//GPIO_Pin_14//PB10连接至TFT --RS
#define LCD_SCL        	GPIO_Pin_14//GPIO_Pin_15//PB13连接至TFT -- CLK
#define LCD_SDA        	GPIO_Pin_13//GPIO_Pin_13//PB15连接至TFT - SDI
							    
//////////////////////////////////////////////////////////////////////
//液晶控制口置1操作语句宏定义  
#define	LCD_RS_SET  	LCD_CTRL->BSRR=LCD_RS    
#define	LCD_SDA_SET  	LCD_CTRL->BSRR=LCD_SDA    
#define	LCD_SCL_SET  	LCD_CTRL->BSRR=LCD_SCL        


//液晶控制口置0操作语句宏定义  
#define	LCD_RS_CLR  	LCD_CTRL->BRR=LCD_RS    
#define	LCD_SDA_CLR  	LCD_CTRL->BRR=LCD_SDA    
#define	LCD_SCL_CLR  	LCD_CTRL->BRR=LCD_SCL      


//画笔颜色
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
#define BROWN 			0XBC40 //棕色
#define BRRED 			0XFC07 //棕红色
#define GRAY  			0X8430 //灰色
//GUI颜色

#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
//以上三色为PANEL的颜色 
 
#define LIGHTGREEN     	0X841F //浅绿色
//#define LIGHTGRAY     0XEF5B //浅灰色(PANNEL)
#define LGRAY 			 		0XC618 //浅灰色(PANNEL),窗体背景色

#define LGRAYBLUE      	0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE          0X2B12 //浅棕蓝色(选择条目的反色)
	    															  
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

