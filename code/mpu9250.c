#include "I2C.h"
#include "systick.h"
#include "IMU.h"
#include "math.h"
#include "usart1.h"
#include "MPU9250.h"
// ����MPU9250�ڲ���ַ
//****************************************
#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG			0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
 
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08

#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I		  0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)


//****************************

#define	GYRO_ADDRESS   0xD0	  //���ݵ�ַ
#define MAG_ADDRESS    0x18   //�ų���ַ
#define ACCEL_ADDRESS  0xD0 
#define	HMC5883L_ADDRESS   0x1A	  //����������IIC�����еĴӵ�ַ

unsigned char BUF[14];       //�������ݻ�����
float gx,  gy,  gz,  ax,  ay,  az,  mx,  my,  mz, MOR,gx0=0,  gy0=0,  gz0=0;
float mx_max=0,mx_min=0,my_max=0,my_min=0,mz_max=0,mz_min=0;
void Init_MPU9250(void)
{
I2C_GPIO_Config();		 //����IICʹ�ö˿�
Single_Write(ACCEL_ADDRESS, PWR_MGMT_1, 0x00);  	//�������״̬
Single_Write(ACCEL_ADDRESS, SMPLRT_DIV, 0x07);     	
Single_Write(ACCEL_ADDRESS, CONFIG, 0x03);         //��ͨ�˲�	
Single_Write(ACCEL_ADDRESS, GYRO_CONFIG, 0x10);    //���������� +-1000	
Single_Write(ACCEL_ADDRESS, ACCEL_CONFIG, 0x09);   //���ٶ����� +-4G	
}
//******��ȡMPU9250����****************************************
void READ_MPU9250_ACCEL_GYRO(void)
{  
	  short T_X,T_Y,T_Z;		 //X,Y,Z��
	  I2C_Start();                          //��ʼ�ź�
    I2C_SendByte(ACCEL_ADDRESS);                   //�����豸��ַ+д�ź�
	  I2C_WaitAck();
    I2C_SendByte(0x3B);                   //���ʹ洢��Ԫ��ַ����0x3��ʼ	
	  I2C_WaitAck();
    I2C_Start();                          //��ʼ�ź�
    I2C_SendByte(ACCEL_ADDRESS+1);     //�����豸��ַ+���ź�
	  I2C_WaitAck();
	  for (int i=0; i<14; i++)                   //������ȡ6����ַ���ݣ��洢��BUF
    {
        BUF[i] = I2C_RadeByte();          //BUF[0]�洢����
        if (i == 13)
           I2C_NoAck();                   //���һ��������Ҫ��NOACK
        else
           I2C_Ack();                     //��ӦACK
    }
    I2C_Stop();                           //ֹͣ�ź�
		
   T_X=	(BUF[0]<<8)|BUF[1];
   T_Y=	(BUF[2]<<8)|BUF[3];	
   T_Z=	(BUF[4]<<8)|BUF[5];
	
		if(T_X>8192)
		{
			 T_X=8192;
		}
		if(T_X<-8192)
		{
			 T_X=-8192;
		}
		if(T_Y>8192)
		{
			 T_Y=8192;
		}
		if(T_Y<-8192)
		{
			 T_Y=-8192;
		}
		if(T_Z>8192)
		{
			 T_Z=8192;
		}
		if(T_Z<-8192)
		{
			 T_Z=-8192;
		}
		
	
	 ax=T_X/8192.0;
	 ay=T_Y/8192.0;
	 az=T_Z/8192.0;

	 T_X=	(BUF[8 ]<<8)|BUF[9];
   T_Y=	(BUF[10]<<8)|BUF[11];	
   T_Z=	(BUF[12]<<8)|BUF[13];
	 gx=T_X-gx0;
	 gy=T_Y-gy0;
	 gz=T_Z-gz0;
}


u8 Init_HMC5883L(void)
{
   u8 date;
   date = Single_Write(HMC5883L_ADDRESS,0x0B,0x01);
   date = Single_Write(HMC5883L_ADDRESS,0x09,0x1D);
   return date; 		 
}


void Multiple_Read_HMC5883L(void)
{    	
	short T_X,T_Y,T_Z;		 //X,Y,Z��  	
	I2C_Start();                          //��ʼ�ź�
	I2C_SendByte(HMC5883L_ADDRESS);                   //�����豸��ַ+д�ź�
	I2C_WaitAck();
	I2C_SendByte(0x00);                   //���ʹ洢��Ԫ��ַ����0x3��ʼ	
	I2C_WaitAck();
	I2C_Start();                          //��ʼ�ź�
	I2C_SendByte(HMC5883L_ADDRESS+1);     //�����豸��ַ+���ź�
	I2C_WaitAck();
	for (int i=0; i<6; i++)                   //������ȡ6����ַ���ݣ��洢��BUF
	{
		BUF[i] = I2C_RadeByte();          //BUF[0]�洢����
		if (i == 5)
			I2C_NoAck();                   //���һ��������Ҫ��NOACK
		else
			I2C_Ack();                     //��ӦACK
	}
	I2C_Stop();                           //ֹͣ�ź�
	T_X=((u16)BUF[1]<<8)|BUF[0];
	T_Y=((u16)BUF[3]<<8)|BUF[2];//��ȡ����Y������
	T_Z=((u16)BUF[5]<<8)|BUF[4];//��ȡ����Z������
	mx=T_X;
	my=T_Y;
	mz=T_Z;
	mx=(mx-(mx_max+mx_min)/2.0)/(mx_max-mx_min);
	my=(my-(my_max+my_min)/2.0)/(my_max-my_min);
	mz=(mz-(mz_max+mz_min)/2.0)/(mz_max-mz_min);
}	

void calculate_mpu_G(void)
{
	short T_X,T_Y,T_Z;		 //X,Y,Z��
		for(int i=0;i<8000;i++)
		{
			I2C_Start();                          //��ʼ�ź�
			I2C_SendByte(ACCEL_ADDRESS);                   //�����豸��ַ+д�ź�
			I2C_WaitAck();
			I2C_SendByte(0x3B);                   //���ʹ洢��Ԫ��ַ����0x3��ʼ	
			I2C_WaitAck();
			I2C_Start();                          //��ʼ�ź�
			I2C_SendByte(ACCEL_ADDRESS+1);     //�����豸��ַ+���ź�
			I2C_WaitAck();
			for (int i=0; i<14; i++)                   //������ȡ6����ַ���ݣ��洢��BUF
			{
					BUF[i] = I2C_RadeByte();          //BUF[0]�洢����
					if (i == 13)
						 I2C_NoAck();                   //���һ��������Ҫ��NOACK
					else
						 I2C_Ack();                     //��ӦACK
			}
			I2C_Stop();                           //ֹͣ�ź�
			
		 T_X=	(BUF[8 ]<<8)|BUF[9];
		 T_Y=	(BUF[10]<<8)|BUF[11];	
		 T_Z=	(BUF[12]<<8)|BUF[13];
		 gx0+=T_X;
		 gy0+=T_Y;
		 gz0+=T_Z;
		}
		gx0/=8000.0;
		gy0/=8000.0;  
		gz0/=8000.0;
	
}

void calculate_mpu_M(void)
{
	short T_X,T_Y,T_Z;		 //X,Y,Z��
	static char cf=0;
	
	
	I2C_Start();                          //��ʼ�ź�
	I2C_SendByte(HMC5883L_ADDRESS);                   //�����豸��ַ+д�ź�
	I2C_WaitAck();
	I2C_SendByte(0x00);                   //���ʹ洢��Ԫ��ַ����0x3��ʼ	
	I2C_WaitAck();
	I2C_Start();                          //��ʼ�ź�
	I2C_SendByte(HMC5883L_ADDRESS+1);     //�����豸��ַ+���ź�
	I2C_WaitAck();
	for (int i=0; i<6; i++)                   //������ȡ6����ַ���ݣ��洢��BUF
	{
		BUF[i] = I2C_RadeByte();          //BUF[0]�洢����
		if (i == 5)
			I2C_NoAck();                   //���һ��������Ҫ��NOACK
		else
			I2C_Ack();                     //��ӦACK
	}
	I2C_Stop();                           //ֹͣ�ź�
	T_X=((u16)BUF[1]<<8)|BUF[0];
	T_Y=((u16)BUF[3]<<8)|BUF[2];//��ȡ����Y������
	T_Z=((u16)BUF[5]<<8)|BUF[4];//��ȡ����Z������
	
	if(cf==0)
		{
			mx_max=mx_min=T_X;
			my_max=my_min=T_Y;
			mz_max=mz_min=T_Z;
			cf=1;
		}

		if(mx_max<T_X)
			mx_max=T_X;
		if(mx_min>T_X)
			mx_min=T_X;
		
		if(my_max<T_Y)
			my_max=T_Y;
		if(my_min>T_Y)
			my_min=T_Y;
		
		if(mz_max<T_Z)
			mz_max=T_Z;
		if(mz_min>T_Z)
			mz_min=T_Z;
}











