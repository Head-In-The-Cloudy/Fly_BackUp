#include "ultra.h"
#include "delay.h"

#include "nano.h"
#include "stdio.h"

//超声波 捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
//[5:0]:捕获低电平后溢出的次数
extern uint8_t  TIM4CH1_CAPTURE_STA;							//输入捕获状态		    				
extern uint16_t TIM4CH1_CAPTURE_VAL;							//输入捕获值(TIM4是16位)
extern uint8_t  TIM10CH1_CAPTURE_STA;							//输入捕获状态		    				
extern uint16_t TIM10CH1_CAPTURE_VAL;							//输入捕获值(TIM4是16位)

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
	
	
	trig();                          //Trig捕获 大于10us的 脉冲        
	if(TIM4CH1_CAPTURE_STA & 0X80)           //成功捕获
	{
		
		time_1 =(TIM4CH1_CAPTURE_STA & 0X3F);
		time_1 *=65535;                          //ARR为65536 溢出次数 乘以 溢出阈值
		time_1 +=TIM4CH1_CAPTURE_VAL;            //加上 此时 计数器的值 获取 信号往返的时间

    	len_1 = time_1 * 342.62*100/2000000;


		TIM4CH1_CAPTURE_STA=0;                 //计数器 置零
		TIM4CH1_CAPTURE_VAL=0;			

	}  

	if(TIM10CH1_CAPTURE_STA & 0X80)           //成功捕获
	{
		time_2 =(TIM10CH1_CAPTURE_STA & 0X3F);
		time_2 *=65535;                          //ARR为65536 溢出次数 乘以 溢出阈值
		time_2 +=TIM10CH1_CAPTURE_VAL;            //加上 此时 计数器的值 获取 信号往返的时间

    	len_2 = time_2 * 342.62*100/2000000;


		TIM10CH1_CAPTURE_STA=0;                 //计数器 置零
		TIM10CH1_CAPTURE_VAL=0;			

	}  
	
}

