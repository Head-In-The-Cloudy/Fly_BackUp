#ifndef __USART_H__
#define __USART_H__
#include "Ringbuf.h"

void ConfigureUART0(unsigned long bound);
void ConfigureUART1(unsigned long bound);
void ConfigureUART2(unsigned long bound);
void ConfigureUART3(void);
void ConfigureUART6(unsigned long bound,uint16_t len);
void ConfigureUART7(unsigned long baud);
void USART1_Send(uint8_t *pui8Buffer, uint32_t ui32Count);
void USART3_Send(uint8_t *pui8Buffer, uint32_t ui32Count);
void USART2_Send(uint8_t *pui8Buffer, uint32_t ui32Count);
void USART6_Send(uint8_t *pui8Buffer, uint32_t ui32Count);
void wust_sendware(unsigned char *wareaddr, int16_t waresize);
void USART0_Send(uint8_t *pui8Buffer, uint32_t ui32Count);
void USART7_Send(uint8_t *pui8Buffer, uint32_t ui32Count);


void Vcan_Send(void);
extern RingBuff_t COM0_Rx_Buf,COM1_Rx_Buf,COM2_Rx_Buf,COM3_Rx_Buf,COM4_Rx_Buf,COM5_Rx_Buf,COM6_Rx_Buf,COM7_Rx_Buf;
#endif


