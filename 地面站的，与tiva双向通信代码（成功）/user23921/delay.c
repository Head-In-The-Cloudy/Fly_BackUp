#include "delay.h"

static uint32_t fac_us=0;							//us��ʱ������

//��ʼ���ӳٺ���
//SYSTICK��ʱ�ӹ̶�ΪAHBʱ��!!!
//SYSCLK:ϵͳʱ��Ƶ��
void delay_init(uint8_t SYSCLK)
{
	fac_us=SYSCLK;		
}								    



//ע����Ϊhal��ĵ�Ч�� �ڷ�ת���ŵ�ƽʱ��Ż���1us���ӳ� ��ʹ��iic��ͨ��ʱҪע����һ��
//��ʱnus
//nusΪҪ��ʱ��us��.	
//nus:0~190887435(���ֵ��2^32/fac_us@fac_us=22.5)	 
void delay_us(uint32_t nus)
{		
	uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload=SysTick->LOAD;				//LOAD��ֵ	    	 
	ticks=nus*fac_us; 						//��Ҫ�Ľ����� 
	told=SysTick->VAL;        				//�ս���ʱ�ļ�����ֵ
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//ʱ�� ����/���� Ҫ�ӳٵ�ʱ��,���˳�.
		}  
	};
}










































