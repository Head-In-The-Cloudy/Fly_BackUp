#ifndef _ADC_BUTTON_
#define _ADC_BUTTON_


	
void ADC_Button_Init(void);
void ADC_Button_Read(void);
void ADC_Button_Scan(void);
extern uint32_t wireless_adc_value[2];
extern float Button_ADCResult[2];
extern uint8_t Key_Value[2];
extern uint16_t ADC_PPM_Databuf[10];
extern uint8_t adc_ppm_update_flag,adc_key1_ctrl_enable,adc_key2_ctrl_enable,wireless_adc_key_ctrl_enable;
extern button_state   ADC_Button1,ADC_Button2,ADC_Button3,ADC_Button4;

#endif


