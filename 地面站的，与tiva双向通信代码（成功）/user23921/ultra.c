#include "ultra.h"
#include "delay.h"

#include "nano.h"
#include "stdio.h"

//������ ����״̬
//[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
//[6]:0,��û���񵽵͵�ƽ;1,�Ѿ����񵽵͵�ƽ��.
//[5:0]:����͵�ƽ������Ĵ���
extern uint8_t  TIM4CH1_CAPTURE_STA;							//���벶��״̬		    				
extern uint16_t TIM4CH1_CAPTURE_VAL;							//���벶��ֵ(TIM4��16λ)
extern uint8_t  TIM10CH1_CAPTURE_STA;							//���벶��״̬		    				
extern uint16_t TIM10CH1_CAPTURE_VAL;							//���벶��ֵ(TIM4��16λ)

float len_1 = 0;
float len_2 = 0;
uint32_t time_1= 0;
uint32_t time_2= 0;
uint8_t count = 0;
 



void trig(void)
{

	
	HAL_GPIO_WritePin(TRIG_1_GPIO_Port, TRIG_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TRIG_2_GPIO_Port, TRIG_2_Pin, GPIO_PIN_SET);
	delay_us(70);

	HAL_GPIO_WritePin(TRIG_2_GPIO_Port, TRIG_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TRIG_1_GPIO_Port, TRIG_1_Pin, GPIO_PIN_RESET);
	

		
}

void get_len(void)
{
	
	HAL_Delay(100);
	
	
	trig();                          //Trig���� ����10us�� ����        
	if(TIM4CH1_CAPTURE_STA & 0X80)           //�ɹ�����
	{
		
		time_1 =(TIM4CH1_CAPTURE_STA & 0X3F);
		time_1 *=65535;                          //ARRΪ65536 ������� ���� �����ֵ
		time_1 +=TIM4CH1_CAPTURE_VAL;            //���� ��ʱ ��������ֵ ��ȡ �ź�������ʱ��

    	len_1 = time_1 * 342.62*100/2000000;


		TIM4CH1_CAPTURE_STA=0;                 //������ ����
		TIM4CH1_CAPTURE_VAL=0;			

	}  

	if(TIM10CH1_CAPTURE_STA & 0X80)           //�ɹ�����
	{
		time_2 =(TIM10CH1_CAPTURE_STA & 0X3F);
		time_2 *=65535;                          //ARRΪ65536 ������� ���� �����ֵ
		time_2 +=TIM10CH1_CAPTURE_VAL;            //���� ��ʱ ��������ֵ ��ȡ �ź�������ʱ��

    	len_2 = time_2 * 342.62*100/2000000;


		TIM10CH1_CAPTURE_STA=0;                 //������ ����
		TIM10CH1_CAPTURE_VAL=0;			

	}  
	
}

