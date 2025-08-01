#include "Headfile.h"
#include "speaker.h"


extern void NCLink_GS_Prase_Data(uint8_t data);

#if FLIGHT_ESC_PWM==0
void UART4_IRQHandler(void)//UART1中断函数
{				
  uint32_t flag = UARTIntStatus(UART4_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志		
  UARTIntClear(UART4_BASE,flag);//清除中断标志	
  while(UARTCharsAvail(UART4_BASE))//判断FIFO是否还有数据		
  {
		uint8_t ch=UARTCharGet(UART4_BASE);
		NCLink_GS_Prase_Data(ch);
  }
}


void ConfigureUART_Speaker(unsigned long bound)//串口4初始化
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//使能GPIO外设		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);//使能UART外设
  GPIOPinConfigure(GPIO_PC4_U4RX);//GPIO模式配置 PC4--RX PC5--TX 
  GPIOPinConfigure(GPIO_PC5_U4TX);
  GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);//GPIO的UART模式配置
	UARTClockSourceSet(UART4_BASE, UART_CLOCK_PIOSC);
	UARTStdioConfig(0, bound, 16000000);
  UARTFIFODisable(UART4_BASE);//使能UART4中断	
  UARTIntEnable(UART4_BASE,UART_INT_RX);//使能UART1接收中断		
  UARTIntRegister(UART4_BASE,UART4_IRQHandler);//UART1中断地址注册	
  IntPrioritySet(INT_UART4, USER_INT2);
} 



void Speaker_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//发送N个字节长度的数据
{
  while(ui32Count--)
  {
    UARTCharPut(UART4_BASE, *pui8Buffer++);
  }
}
#else
void UART5_IRQHandler(void)//UART1中断函数
{				
  uint32_t flag = UARTIntStatus(UART5_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志		
  UARTIntClear(UART5_BASE,flag);//清除中断标志	
  while(UARTCharsAvail(UART5_BASE))//判断FIFO是否还有数据		
  {
		uint8_t ch=UARTCharGet(UART5_BASE);
		NCLink_GS_Prase_Data(ch);
  }
}


void ConfigureUART_Speaker(unsigned long bound)//串口4初始化
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//使能GPIO外设		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);//使能UART外设
  GPIOPinConfigure(GPIO_PE4_U5RX);//GPIO模式配置 PC4--RX PC5--TX 
  GPIOPinConfigure(GPIO_PE5_U5TX);
  GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);//GPIO的UART模式配置
	UARTClockSourceSet(UART5_BASE, UART_CLOCK_PIOSC);
	UARTStdioConfig(1, bound, 16000000);
  UARTFIFODisable(UART5_BASE);//使能UART5中断	
  UARTIntEnable(UART5_BASE,UART_INT_RX);//使能UART1接收中断		
  UARTIntRegister(UART5_BASE,UART5_IRQHandler);//UART1中断地址注册	
  IntPrioritySet(INT_UART5, USER_INT2);
} 


void Speaker_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//发送N个字节长度的数据
{
  while(ui32Count--)
  {
    UARTCharPut(UART5_BASE, *pui8Buffer++);
  }
}
#endif





/**************芯片设置命令*********************/
uint8_t SYN_StopCom[]={0xFD,0X00,0X01,0X02};//停止合成
uint8_t SYN_SuspendCom[]={0XFD,0X00,0X01,0X03};//暂停合成
uint8_t SYN_RecoverCom[]={0XFD,0X00,0X01,0X04};//恢复合成
uint8_t SYN_ChackCom[]={0XFD,0X00,0X01,0X21};//状态查询
uint8_t SYN_PowerDownCom[]={0XFD,0X00,0X01,0X88};//进入POWER DOWN 状态命令



void syn6658_frame_package(uint8_t *data)
{
	/****************需要发送的文本**********************************/ 
	 unsigned char frame_info[50]; //定义的文本长度
	 unsigned int len;  
	 len=strlen((const char *)data); 			//需要发送文本的长度
	/*****************帧固定配置信息**************************************/           
	 frame_info[0]=0xFD ; 		//构造帧头FD
	 frame_info[1]=0x00 ; 		//构造数据区长度的高字节
	 frame_info[2]=len+2; 		//构造数据区长度的低字节
	 frame_info[3]=0x01 ; 		//构造命令字：合成播放命令		 		 
	 frame_info[4]=0x01;      //文本编码格式：GBK 
	/*******************发送帧信息***************************************/		  
	 memcpy(&frame_info[5], data, len);
	 Speaker_Send(frame_info,5+len); //发送帧配置
}


uint16_t speaker_mode=0;
void speaker_notify_run(uint16_t *mode)
{
	switch(*mode)
	{
		case 0:
		{
			//空闲
		}
		break;
		case 1:
		{
			*mode=0;
			syn6658_frame_package((uint8_t *)"sound124 货物已送达请取货");
		}
		break;
		default:
		{
			//空闲
		}
	}
	
}










