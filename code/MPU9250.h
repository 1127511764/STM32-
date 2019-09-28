#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <stm32f10x.h>
void Init_MPU9250(void);
void READ_MPU9250_ACCEL_GYRO(void);
void READ_MPU9250_MAG(void);
extern float gx,  gy,  gz,  ax,  ay,  az,  mx,  my,  mz, MOR,gx0,  gy0,  gz0;
extern float mx_max,mx_min,my_max,my_min,mz_max,mz_min;

u8 Init_HMC5883L(void);
void Multiple_Read_HMC5883L(void);
void calculate_mpu_G(void);
void calculate_mpu_M(void);

#endif

