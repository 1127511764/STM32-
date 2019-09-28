#include "MPU9250.h"
#include "math.h"
#include "usart1.h"
#include "CXW_Attitude_Algorithm.h"
struct _angle angle;

float AQ=0.0005,AR=50,
	    Acc_Q=1,Acc_R=50;

float AngleX_Kalman(float X_P,float X_V)//�������ں�
{
	static float  P0_0=0,Kg=0.0,P1_0=0.0001;
	float X;
	P1_0=P0_0+AQ;//ϵͳЭ����
	Kg=P1_0/(P1_0+AR);//�۲�Э����
	X=X_P+(X_V-X_P)*Kg;
	P0_0=(1-Kg)*P1_0;
	return X;
}

float AccX_Kalman(float X_P,float X_V)//�������ں�
{
	static float  P0_0=0,Kg=0.0,P1_0=0;
	float X;
	P1_0=P0_0+Acc_Q;//ϵͳЭ����
	Kg=P1_0/(P1_0+Acc_R);//�۲�Э����
	X=X_P+(X_V-X_P)*Kg;
	P0_0=(1-Kg)*P1_0;
	return X;
}


float AngleY_Kalman(float X_P,float X_V)//�������ں�
{
	static float  P0_0=0,Kg=0.0,P1_0=0;
	float X;
	P1_0=P0_0+AQ;//ϵͳЭ����
	Kg=P1_0/(P1_0+AR);//�۲�Э����
	X=X_P+(X_V-X_P)*Kg;
	P0_0=(1-Kg)*P1_0;
	return X;
}
float AccY_Kalman(float X_P,float X_V)//�������ں�
{
	static float  P0_0=0,Kg=0.0,P1_0=0;
	float X;
	P1_0=P0_0+Acc_Q;//ϵͳЭ����
	Kg=P1_0/(P1_0+Acc_R);//�۲�Э����
	X=X_P+(X_V-X_P)*Kg;
	P0_0=(1-Kg)*P1_0;
	return X;
}

float AngleZ_Kalman(float X_P,float X_V)//�������ں�
{
	static float  P0_0=0,Kg=0.0,P1_0=0;
	float X;
	P1_0=P0_0+0.006;//ϵͳЭ����
	Kg=P1_0/(P1_0+100);//�۲�Э����
	X=X_P+(X_V-X_P)*Kg;
	P0_0=(1-Kg)*P1_0;
	return X;
}

float AX=0,AY=0,GX=0,GY=0,GZ=0,Accp_X=0,Accp_Y=0,Acc_X,Acc_Y;
void Get_Angle(void)
{
	static char Get_Angle_f=0;//�״�ִ�б�־λ
	static float dt=0.00250000;
	if(Get_Angle_f==0)
	{
		angle.pitch=-asinf(ax)*57.324841;
		angle.roll =-asinf(ay)*57.324841;
		if(my>=0&&mx>=0)
		{
			angle.yaw  =atanf(my/mx)*57.324841;
		}
		if(my>=0&&mx<0)
		{
			angle.yaw  =180-atanf(my/-mx)*57.324841;
		}
		if(my<0&&mx<0)
		{
			angle.yaw  =180+atanf(my/mx)*57.324841;
		}
		if(my<0&&mx>=0)
		{
			angle.yaw  =360-atanf(-my/mx)*57.324841;
		}
		Get_Angle_f=1;
	}	
	
	GX=angle.pitch+(gy)*0.030517*dt;
	GY=angle.roll -(gx)*0.030517*dt;
	GZ=angle.yaw  -(gz)*0.030517*dt;
	
	Accp_X=-sinf(GX/57.324841);
	Accp_Y=-sinf(GY/57.324841);
	Acc_X=AccX_Kalman(Accp_X,ax);
	Acc_Y=AccY_Kalman(Accp_Y,ay);
	
	AX=-asinf(Acc_X)*57.324841;
	AY=-asinf(Acc_Y)*57.324841;

	angle.pitch=AngleX_Kalman(GX,AX);//���ŽǶ��ں�
	angle.roll =AngleY_Kalman(GY,AY);//���ŽǶ��ں�
	
	if(my>=0&&mx>=0)
	{
		angle.yaw  =atanf(my/mx)*57.324841;
	}
	if(my>=0&&mx<0)
	{
		angle.yaw  =180-atanf(my/-mx)*57.324841;
	}
	if(my<0&&mx<0)
	{
		angle.yaw  =180+atanf(my/mx)*57.324841;
	}
	if(my<0&&mx>=0)
	{
		angle.yaw  =360-atanf(-my/mx)*57.324841;
	}
}
