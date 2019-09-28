/*
串级pid是我控制调试以来感觉最稳，最好调的方法，
它在系统函数逼近的思想里其实是加了二阶微分项，这也是它由于单pid的重要原因之一
希望以后做控制的学弟可以多多尝试
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
float PWM_B=1000;																							//原始值1000
extern float H,V,H_old,pitch_0,roll_0, yaw_0;
char control_time=0,H_ctrl_time=0;


/*
参数1：
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
串级PID调试经验：

(1): 若想无人机自己能恢复平衡，需要内环P和外环P都不为0

(2): 外环P的增加会使得无人机的自动恢复平衡性降低，而内环P的增加会使得无人机的自动恢复平衡性上升;

(3): 内环的P 和 外环的P 都会增加无人机系统的响应速度，但内环和外环的量级不同，内环增加0.01，相当于外环增加100;

(4): 在外环的P和内环的P经过单位换算后可知，系统的P值应当为外环P+内环P*100。

(5): 在系统P值为800时，内环P*100 - 外环P = 400 时，达到飞控自平衡临界值，这个差值越大，系统抖动越小，但抖动无法消除，
经过测试，在差值为600时，飞机稳定性最好，但抖动依旧无法消除。
		 在系统P值为1200时，内环P*100 - 外环P = 800 时，，达到飞控自平衡临界值，同理差值越大，系统抖动越小，但抖动无法消除，
经过测试，在差值为1000时，飞机稳定性最好
		 所以得出结论：(前提：系统P值大于800，系统P值小于800时，回复力过弱，不建议)
												系统P值 - 400 = 内环P*100 - 外环P  时， 飞机达到自平衡临界状态
												系统P值 - 200 = 内环P*100 - 外环P  时， 飞机达到最佳自平衡P值
		 而系统P值越大，响应速度越高。
		 
(6): 内环D值：消除飞机抖动，提高平衡性能，其数值一般为内环P值的5倍
		 外环D值：增加飞机抖动，D值越大，抖动越大，但飞机灵敏度明显上升，相当于增加外环的P值


结论：
		串级PID调试流程：		

		(1) 赋值外环P值和内环P值，并使其差值为： 系统P值 - 200 = 内环P值*100 - 外环P值;  系统P值 = 内环P值*100 + 外环P值;(建议：系统P值>800)

		(2) 赋值内环D值，其数值为： 内环D值 = 内环P值 * 倍数;(建议：倍数取值范围应在5~8之间)
		
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
pid_pitch_out.e     = 初始值 - 当前角度值;		误差
pid_pitch_out.es    += 误差;                   累计误差和
pid_pitch_out.ec    = 当前误差 - 上次误差;
pid_pitch_out.eold  = 上次误差

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
