#ifndef __TIME_CNT_h__
#define __TIME_CNT_h__


extern void TIMER0A_Handler(void);
extern void TIMER1A_Handler(void);
extern void TIMER2A_Handler(void);
extern void TIMER3A_Handler(void);


void Time_init(void);


extern Testime Time0_Delta;
#endif









