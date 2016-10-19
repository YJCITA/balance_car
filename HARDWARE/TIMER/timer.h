#ifndef __TIMER_H
#define __TIMER_H
#include <sys.h>	 

void Timer1_Init(u16 arr,u16 psc);  
void TIM2_IRQHandler(void);

void TIM4_Cap_Init(u16 arr,u16 psc); // ³¬Éù²¨²¶×½
void TIM4_IRQHandler(void);
void Read_Distane(void);

#endif
