/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��IMU.c
 * ����    ����̬����         
 * ʵ��ƽ̨��HT�ɿ�
 * ��汾  ��ST3.5.0
 * ����    ��Air Nano Team 
**********************************************************************************/
#include "IMU.h"
#include "math.h"
#include "MPU9250.h"

struct _angle angle;

/*	
	Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
	R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��	
*/
#define KALMAN_Q        0.02
#define KALMAN_R        6.0000
/*           ����������������ٶȽ����˲�����           */
static double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
   kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
   p_last = p_now; //����covarianceֵ
   x_last = x_now; //����ϵͳ״ֵ̬
   return x_now;                
 }
static double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
   kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
   p_last = p_now; //����covarianceֵ
   x_last = x_now; //����ϵͳ״ֵ̬
   return x_now;                
 }
static double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
   kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
   p_last = p_now; //����covarianceֵ
   x_last = x_now; //����ϵͳ״ֵ̬
   return x_now;                
 }

 
 static double KalmanFilter_A(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
   kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
   p_last = p_now; //����covarianceֵ
   x_last = x_now; //����ϵͳ״ֵ̬
   return x_now;                
 }
float AQ=0.0001,AR=10;
float AngleX_Kalman(float X_P,float X_V)//�������ں�
{
	static float  P0_0=0.0,Kg=0.00,P1_0=0.0;
	float X;
	P1_0=P0_0+AQ;//ϵͳЭ����
	Kg=P1_0/(P1_0+AR);//�۲�Э����
	X=X_P+(X_V-X_P)*Kg;
	P0_0=(1-Kg)*P1_0;
	return X;
}
 
//   ������ƽ��������
float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration ����һ��ţ�ٵ�����
	return y;
} 
//  ��float�����ݾ���ֵ
float FL_ABS(float x)
{
   if(x < 0)  return -x;
	 else return x; 
}
/*   �������Ǻ�����̩��չ��ʽ �����ֵ*/
float COS(float x)
{
	float result;
  result = 1 - x * x/2;
	return result; 
}

float SIN(float y)
{
	float result;
  result = y - y * y * y /6;
	return result; 
}
/***********************************************
  * @brief  �ɱ���������Ӧ����
  * @param  None
  * @retval None
************************************************/
float VariableParameter(float error)
{
	float  result = 0;
	
	if(error < 0)
	{
	   error = -error;
	}
  if(error >0.8f)
	{
	   error = 0.8f;
	}
	result = 1 - 1.28 * error;
	if(result < 0)
	{
	   result = 0;
	}
	return result;
}
/*************************************/

void Prepare_Data(void)
{

	READ_MPU9250_MAG();
	READ_MPU9250_ACCEL_GYRO();
	ax = KalmanFilter_x(ax,KALMAN_Q,KALMAN_R);  // ACC X�Ῠ�����˲�
	ay = KalmanFilter_y(ay,KALMAN_Q,KALMAN_R);  // ACC Y�Ῠ�����˲�
	az = KalmanFilter_z(az,KALMAN_Q,KALMAN_R);  // ACC Z�Ῠ�����˲�
	gx = gx*Gyro_Gr;
  gy = gy*Gyro_Gr;
  gz = gz*Gyro_Gr;
}



float qa0, qa1, qa2, qa3;
#define Kp 0.8f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0015f                     // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.00125f                 // �������ڵ�һ��  ������ 2.5MS �ɼ�һ��  ���� halfT��1.25MS

/**************************************
 * ��������Get_Attitude
 * ����  ���õ���ǰ��̬
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 *************************************/
void Get_Attitude(void)
{
	IMUupdate(gx,  gy,  gz,  ax,  ay,  az);	
}

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  float norm;
  float vx, vy, vz;// wx, wy, wz;
  float ex, ey, ez;
  // �Ȱ���Щ�õõ���ֵ���
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  //  float q0q3 = q0*q3;//
  float q1q1 = q1*q1;
  //  float q1q2 = q1*q2;//
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
	
	if(ax*ay*az==0)
 		return;	
	
  norm = Q_rsqrt(ax*ax + ay*ay + az*az);       //acc���ݹ�һ��
  ax = ax *norm;
  ay = ay * norm;
  az = az * norm;

  // estimated direction of gravity and flux (v and w)              �����������������/��Ǩ
  vx = 2*(q1q3 - q0q2);												//��Ԫ����xyz�ı�ʾ
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                           					 //�������������õ���־������
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + VariableParameter(ex) * ex * Ki;								  //�������л���
  eyInt = eyInt + VariableParameter(ey) * ey * Ki;
  ezInt = ezInt + VariableParameter(ez) * ez * Ki;
// adjusted gyroscope measurements

  gx = gx + Kp *  VariableParameter(ex) * ex + exInt;	
	gy = gy + Kp *  VariableParameter(ey) * ey + eyInt;	
	gz = gz + Kp *  VariableParameter(ez) * ez + ezInt;	
  								
  // integrate quaternion rate and normalise						   //��Ԫ�ص�΢�ַ���
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // normalise quaternion
  norm = Q_rsqrt(q0q0 + q1q1 + q2q2 + q3q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;

  qa0 = q0;
  qa1 = q1;
  qa2 = q2;
  qa3 = q3;
	
  angle.roll = atan2(2*q2q3 + 2*q0q1, -2*q1q1 - 2*q2q2 + 1); // roll
	angle.pitch = asin(-2*q1q3 + 2*q0q2); // pitch
	 static char t=0;
	 
	 if(t<8)
	 {
		 t++;
	 }
	 if(t>6)
	 {
		 angle.yaw   =KalmanFilter_A(atanf(mx/my)*RtA,KALMAN_Q,KALMAN_R);
	 }
	 
	angle.roll *= RtA;
	angle.pitch *= RtA;
}













