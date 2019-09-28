#ifndef _control_H_
#define	_control_H_
#include "stm32f10x.h"

typedef  struct 	PID_par
	   {
			float O_P;
			float O_I;
			float O_D; 
      float I_P;
			float I_I;
			float I_D;			 
		 }PID_par;

typedef  struct 	PID
	   {
			float out;	
			float eold;
			float e;
			float ec;
			float es;  
		 }PID;

void control(void);

#endif

