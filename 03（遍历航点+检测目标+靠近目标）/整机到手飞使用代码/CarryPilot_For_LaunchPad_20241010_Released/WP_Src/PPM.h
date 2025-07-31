#ifndef __ppm_h__
#define __ppm_h__

void PPM_Init(void);
extern uint16 PPM_Databuf[10],Receiver_PPM_Databuf[10];
extern uint8_t ppm_update_flag;
#endif
