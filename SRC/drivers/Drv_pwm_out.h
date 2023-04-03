#ifndef _PWM_OUT_H_
#define _PWM_OUT_H_

#include "stm32f4xx.h"

u8 PWM_Out_Init(void);
void SetPwm(double pwm[]);

int SetAngle(double angle);
void Set_Channel1_Angle(double angle);
void Set_Channel2_Angle(double angle);
void Set_Channel3_Angle(double angle);
void Set_Channel4_Angle(double angle);

int SetThrottle(double Throttle);
void Set_Channel5_Throttle(double Throttle);
void Set_Channel6_Throttle(double Throttle);
#endif

