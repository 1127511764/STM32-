#ifndef _CXW_ATTITUDE_ALGORITHM_H_
#define _CXW_ATTITUDE_ALGORITHM_H_


struct _angle{
                float pitch;
				float roll;
                float yaw;
             };			 
void Get_Angle(void);
extern struct _angle angle;
extern float AX,AY,GX,GY,Accp_X,Accp_Y,Acc_X,Acc_Y,mor;
						 
#endif
						 
