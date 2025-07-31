#ifndef _RESERVED_SERIALPORT_H_
#define _RESERVED_SERIALPORT_H_

typedef enum 
{
	FRONT_RANGE_FINDER=0,
	GPS_M8N,
	THIRD_PARTY_STATE,
}RESERVED_UART_MAP;


#define RESERVED_UART_DEFAULT THIRD_PARTY_STATE;//THIRD_PARTY_STATE;
															//FRONT_RANGE_FINDER
															//GPS_M8N
															//THIRD_PARTY_STATE


#define UART2_DEFAULT 0x00
#define AVOID_OBSTACLE_DEFAULT 0x00

void Reserved_Serialport_Init(void);
void us100_front_statemachine(void);
void Uart2_Serialport_Init(void);


void NCLink_Data_Prase_Prepare_Lite(uint8_t data);
void NCLink_Send_IMU_Feedback_PC(void);


void quad_getangle(float *q,float* rpy);


extern uint8_t send_check_back;


#endif

