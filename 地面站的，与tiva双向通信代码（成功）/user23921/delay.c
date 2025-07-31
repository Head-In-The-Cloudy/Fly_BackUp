#include "delay.h"

static uint32_t fac_us=0;							//us延时倍乘数

//初始化延迟函数
//SYSTICK的时钟固定为AHB时钟!!!
//SYSCLK:系统时钟频率
void delay_init(uint8_t SYSCLK)
{
	fac_us=SYSCLK;		
}								    



//注意因为hal库的低效性 在翻转引脚电平时大概会有1us的延迟 在使用iic等通信时要注意这一点
//延时nus
//nus为要延时的us数.	
//nus:0~190887435(最大值即2^32/fac_us@fac_us=22.5)	 
void delay_us(uint32_t nus)
{		
	uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload=SysTick->LOAD;				//LOAD的值	    	 
	ticks=nus*fac_us; 						//需要的节拍数 
	told=SysTick->VAL;        				//刚进入时的计数器值
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//时间 超过/等于 要延迟的时间,则退出.
		}  
	};
}










































