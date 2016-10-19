#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
#define PI 3.14159265
extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
int TIM1_UP_IRQHandler(void);  
int balance(float angle,float gyro);
int velocity(int encoder_left,int encoder_right);
int turn(int encoder_left,int encoder_right,float gyro);
void Set_Pwm(int moto1,int moto2);
void Key(void);
void Xianfu_Pwm(void);
u8 Turn_Off(float angle, int voltage);
void Get_Angle(u8 way);
int myabs(int a);

// -YJ- 2016.10.01
int att_pid(float pitch_cmd, float pitch, float w);
float lowpass_fiter(float y_pre, float y_new, float dt, float filt_hz);
int velocity_pid(float vel_cmd, int encoder_left, int encoder_right);
int turn_pid(int encoder_left, int encoder_right, float gyro_z);



    

#endif
