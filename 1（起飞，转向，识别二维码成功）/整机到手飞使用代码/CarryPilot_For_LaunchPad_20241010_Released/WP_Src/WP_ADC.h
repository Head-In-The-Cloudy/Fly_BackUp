#ifndef __WP_ADC_H__
#define __WP_ADC_H__


void ADC_Init(void);   
void Get_Battery_Voltage(void);  
void Battery_Voltage_Detection(void);  

void ADC_Sample_Trigger(void);


extern float Battery_Voltage;
extern uint32_t adc_value[8];
#endif
