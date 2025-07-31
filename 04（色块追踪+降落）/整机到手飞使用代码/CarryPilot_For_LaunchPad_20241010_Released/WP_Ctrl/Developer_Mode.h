#ifndef _DEVELOPER_MODE_
#define _DEVELOPER_MODE_


#define SDK_MODE_VALID_MAX  32


extern uint8_t SDK1_Mode_Setup,Forward_Vision_Mode_Setup;
extern int16_t task_select_cnt;
	
void Auto_Flight_Ctrl(uint8_t *mode);
#endif



