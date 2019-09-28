/*
����pid���ҿ��Ƶ��������о����ȣ���õ��ķ�����
����ϵͳ�����ƽ���˼������ʵ�Ǽ��˶���΢�����Ҳ�������ڵ�pid����Ҫԭ��֮һ
ϣ���Ժ������Ƶ�ѧ�ܿ��Զ�ೢ��
*/
#include "math.h"
#include "MPU9250.h"
#include "moto.h"
#include "usart1.h"
#include "control.h"
#include "CXW_Attitude_Algorithm.h"

PID pid_pitch_in,pid_pitch_out,pid_roll_in,pid_roll_out,pid_yaw_in,pid_yaw_out,pid_h;
PID_par pid_balance_par,pid_h_par,pid_roll_par;
uint16_t PWM_1,PWM_2,PWM_3,PWM_4,PWM_H=0;
float PWM_B=1000;																							//ԭʼֵ1000
extern float H,V,H_old,pitch_0,roll_0, yaw_0;
char control_time=0,H_ctrl_time=0;


/*
����1��
	pid_balance_par.O_P=440.0;
	pid_balance_par.O_I=1.9;
	pid_balance_par.O_D=9000.0;
	
	pid_balance_par.I_P=0.025;
	pid_balance_par.I_I=0.000;
	pid_balance_par.I_D=0.15;
	
	pid_roll_par.I_P=0.085;
	pid_roll_par.I_I=0;
	pid_roll_par.I_D=0;

	pid_h_par.O_P =0.50;
	pid_h_par.O_D =15.5;

*/

/*
����PID���Ծ��飺

(1): �������˻��Լ��ָܻ�ƽ�⣬��Ҫ�ڻ�P���⻷P����Ϊ0

(2): �⻷P�����ӻ�ʹ�����˻����Զ��ָ�ƽ���Խ��ͣ����ڻ�P�����ӻ�ʹ�����˻����Զ��ָ�ƽ��������;

(3): �ڻ���P �� �⻷��P �����������˻�ϵͳ����Ӧ�ٶȣ����ڻ����⻷��������ͬ���ڻ�����0.01���൱���⻷����100;

(4): ���⻷��P���ڻ���P������λ������֪��ϵͳ��PֵӦ��Ϊ�⻷P+�ڻ�P*100��

(5): ��ϵͳPֵΪ800ʱ���ڻ�P*100 - �⻷P = 400 ʱ���ﵽ�ɿ���ƽ���ٽ�ֵ�������ֵԽ��ϵͳ����ԽС���������޷�������
�������ԣ��ڲ�ֵΪ600ʱ���ɻ��ȶ�����ã������������޷�������
		 ��ϵͳPֵΪ1200ʱ���ڻ�P*100 - �⻷P = 800 ʱ�����ﵽ�ɿ���ƽ���ٽ�ֵ��ͬ���ֵԽ��ϵͳ����ԽС���������޷�������
�������ԣ��ڲ�ֵΪ1000ʱ���ɻ��ȶ������
		 ���Եó����ۣ�(ǰ�᣺ϵͳPֵ����800��ϵͳPֵС��800ʱ���ظ���������������)
												ϵͳPֵ - 400 = �ڻ�P*100 - �⻷P  ʱ�� �ɻ��ﵽ��ƽ���ٽ�״̬
												ϵͳPֵ - 200 = �ڻ�P*100 - �⻷P  ʱ�� �ɻ��ﵽ�����ƽ��Pֵ
		 ��ϵͳPֵԽ����Ӧ�ٶ�Խ�ߡ�
		 
(6): �ڻ�Dֵ�������ɻ����������ƽ�����ܣ�����ֵһ��Ϊ�ڻ�Pֵ��5��
		 �⻷Dֵ�����ӷɻ�������DֵԽ�󣬶���Խ�󣬵��ɻ������������������൱�������⻷��Pֵ


���ۣ�
		����PID�������̣�		

		(1) ��ֵ�⻷Pֵ���ڻ�Pֵ����ʹ���ֵΪ�� ϵͳPֵ - 200 = �ڻ�Pֵ*100 - �⻷Pֵ;  ϵͳPֵ = �ڻ�Pֵ*100 + �⻷Pֵ;(���飺ϵͳPֵ>800)

		(2) ��ֵ�ڻ�Dֵ������ֵΪ�� �ڻ�Dֵ = �ڻ�Pֵ * ����;(���飺����ȡֵ��ΧӦ��5~8֮��)
		
*/

void control(void)
{
	pid_balance_par.O_P=100;
	pid_balance_par.O_I=0;
	pid_balance_par.O_D=0;

	pid_balance_par.I_P=0.17;
	pid_balance_par.I_I=0;
	pid_balance_par.I_D=1.36;



	pid_roll_par.O_P=400;
	pid_roll_par.O_I=0;
	pid_roll_par.O_D=0;

	pid_roll_par.I_P=0.090;
	pid_roll_par.I_I=0;
	pid_roll_par.I_D=0;



	pid_h_par.O_P =0.50;
	pid_h_par.O_D =15.5;

control_time++;
/*
pid_pitch_out.e     = ��ʼֵ - ��ǰ�Ƕ�ֵ;		���
pid_pitch_out.es    += ���;                   �ۼ�����
pid_pitch_out.ec    = ��ǰ��� - �ϴ����;
pid_pitch_out.eold  = �ϴ����

*/

if(control_time==2)
{
	control_time=0;
	pid_pitch_out.e   =0-angle.pitch;													
	pid_pitch_out.es  =pid_pitch_out.es+pid_pitch_out.e;
	pid_pitch_out.ec  =pid_pitch_out.e-pid_pitch_out.eold;
	pid_pitch_out.eold=pid_pitch_out.e; 	
	pid_pitch_out.out =pid_balance_par.O_P*pid_pitch_out.e
										+pid_balance_par.O_I*pid_pitch_out.es 
										+pid_balance_par.O_D*pid_pitch_out.ec;
	
	pid_roll_out.e   =angle.roll-roll_0;
	pid_roll_out.es  =pid_roll_out.es+pid_roll_out.e;
	pid_roll_out.ec  =pid_roll_out.e-pid_roll_out.eold;
	pid_roll_out.eold=pid_roll_out.e; 	
	pid_roll_out.out =pid_balance_par.O_P*pid_roll_out.e
										+pid_balance_par.O_I*pid_roll_out.es 
										+pid_balance_par.O_D*pid_roll_out.ec;
}

pid_pitch_in.e   =pid_pitch_out.out-gy;//
pid_pitch_in.es  =pid_pitch_in.es+pid_pitch_in.e;
pid_pitch_in.ec  =pid_pitch_in.e -pid_pitch_in.eold;
pid_pitch_in.eold=pid_pitch_in.e; 	
pid_pitch_in.out =pid_balance_par.I_P*pid_pitch_in.e
								 +pid_balance_par.I_I*pid_pitch_in.es 
								 +pid_balance_par.I_D*pid_pitch_in.ec;

//if (pid_pitch_in.out > 50) pid_pitch_in.out = 50;


pid_roll_in.e   =pid_roll_out.out-gx;//pid_roll_out.out
pid_roll_in.es  =pid_roll_in.es+pid_roll_in.e;
pid_roll_in.ec  =pid_roll_in.e -pid_roll_in.eold;
pid_roll_in.eold=pid_roll_in.e; 	
pid_roll_in.out =pid_balance_par.I_P*pid_roll_in.e
								+pid_balance_par.I_I*pid_roll_in.es 
							  +pid_balance_par.I_D*pid_roll_in.ec;

//if (pid_roll_in.out > 50) pid_roll_in.out = 50;


pid_yaw_out.e   =yaw_0-angle.yaw;
pid_yaw_out.es  =pid_yaw_out.es+pid_yaw_out.e;
pid_yaw_out.ec  =pid_yaw_out.e-pid_yaw_out.eold;
pid_yaw_out.eold=pid_yaw_out.e; 	
pid_yaw_out.out =pid_roll_par.O_P*pid_pitch_out.e
								 +pid_roll_par.O_I*pid_pitch_out.es 
								 +pid_roll_par.O_D*pid_pitch_out.ec;


pid_yaw_in.e   =pid_yaw_out.out+gz;
pid_yaw_in.es  =pid_yaw_in.es+pid_yaw_in.e;
pid_yaw_in.ec  =pid_yaw_in.e -pid_yaw_in.eold;
pid_yaw_in.eold=pid_yaw_in.e; 	
pid_yaw_in.out =pid_roll_par.I_P*pid_yaw_in.e
							 +pid_roll_par.I_I*pid_yaw_in.es 
							 +pid_roll_par.I_D*pid_yaw_in.ec;

H_ctrl_time++;
if(H_ctrl_time==15)
{
	H_ctrl_time=0;


	static char up_f=0;
	if(up_f==0)
	{
		if(H<100)
		{
			PWM_B+=4.0;
		}
		else 
		{
			up_f=1;
		}
		pid_h.out=0;
	}
	
	if(up_f==1)
	{  
		if(H<200)
		{
			PWM_B+=1.5;
		}
		else 
		{
			up_f=2;
		}
		pid_h.out=0;
	}
	if(up_f==2)
	{
		if(H<400)
		{
			PWM_B+=0.1;
		}
		else 
		{
			up_f=3;
		}
		pid_h.out=0;
	}
	if(up_f==3)
	{
		pid_h.e =(400-H);
		pid_h.ec=-V;
		pid_h.out =pid_h_par.O_P *pid_h.e+pid_h_par.O_D*pid_h.ec;
		if(pid_h.out>80)
		{
			pid_h.out=80;
		}
		if(pid_h.out<-80)
		{
			pid_h.out=-80;
		}
	}
}


PWM_1=PWM_B-pid_roll_in.out+pid_pitch_in.out-pid_yaw_in.out+pid_h.out+pid_pitch_in.out;	//a0                              
PWM_2=PWM_B+pid_roll_in.out+pid_pitch_in.out+pid_yaw_in.out+pid_h.out+pid_pitch_in.out;	//a1                              
PWM_3=PWM_B-pid_roll_in.out-pid_pitch_in.out+pid_yaw_in.out+pid_h.out-pid_pitch_in.out;	//a2                              
PWM_4=PWM_B+pid_roll_in.out-pid_pitch_in.out-pid_yaw_in.out+pid_h.out-pid_pitch_in.out;	//a3                              


Moto_Pwm(PWM_1,PWM_2,PWM_3,PWM_4);
}
