#ifndef __SPEAKER_H__
#define __SPEAKER_H__


void ConfigureUART_Speaker(unsigned long bound);
void Speaker_Send(uint8_t *pui8Buffer, uint32_t ui32Count);
void speaker_notify_run(uint16_t *mode);

extern uint16_t speaker_mode;

#endif


