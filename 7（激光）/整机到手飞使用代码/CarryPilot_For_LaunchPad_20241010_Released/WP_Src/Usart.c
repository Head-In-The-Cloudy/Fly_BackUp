/* Copyright (c)  2019-2030 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
/*----------------------------------------------------------------------------------------------------------------------/
																									开源并不等于免费
																									开源并不等于免费
																									开源并不等于免费
																									重要的事情说三遍
								先驱者的历史已经证明，在当前国内略浮躁+躺平+内卷的大环境下，对于毫无收益的开源项目，单靠坊间飞控爱好者、
								个人情怀式、自发地主动输出去参与开源项目的方式行不通，好的开源项目需要请专职人员做好售后技术服务、配套
								手册和视频教程要覆盖新手入门到进阶阶段，使用过程中对用户反馈问题和需求进行统计、在实践中完成对产品的一
								次次完善与迭代升级。
-----------------------------------------------------------------------------------------------------------------------
*                                                 为什么选择无名创新？
*                                         感动人心价格厚道，最靠谱的开源飞控；
*                                         国内业界良心之作，最精致的售后服务；
*                                         追求极致用户体验，高效进阶学习之路；
*                                         萌新不再孤单求索，合理把握开源尺度；
*                                         响应国家扶贫号召，促进教育体制公平；
*                                         新时代奋斗最出彩，建人类命运共同体。 
-----------------------------------------------------------------------------------------------------------------------
*               生命不息、奋斗不止；前人栽树，后人乘凉！！！
*               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
*               学习优秀者，简历可推荐到DJI、ZEROTECH、XAG、AEE、GDU、AUTEL、EWATT、HIGH GREAT等公司就业
*               求职简历请发送：15671678205@163.com，需备注求职意向单位、岗位、待遇等
*               无名创新开源飞控QQ群：2号群465082224、1号群540707961、TI MSPM0学习中心828746221
*               CSDN博客：http://blog.csdn.net/u011992534
*               B站教学视频：https://space.bilibili.com/67803559/#/video				优酷ID：NamelessCotrun无名小哥
*               无名创新国内首款TI开源飞控设计初衷、知乎专栏:https://zhuanlan.zhihu.com/p/54471146
*               TI教育无人机品质供应商，开源-教学-培养-竞赛,盘古 TI MCU系统板 NController多功能控制器https://item.taobao.com/item.htm?abbucket=19&id=697442280363 
*               淘宝店铺：https://namelesstech.taobao.com/
*               公司官网:www.nameless.tech
*               修改日期:2024/10/01                  
*               版本：躺赢者PRO_V3——CarryPilot_V7.0.1
*               版权所有，盗版必究。
*               Copyright(C) 2019-2030 武汉无名创新科技有限公司 
*               All rights reserved
-----------------------------------------------------------------------------------------------------------------------
*               重要提示：
*               正常淘宝咸鱼转手的飞控、赠送朋友、传给学弟的都可以进售后群学习交流，
*               不得在网上销售无名创新资料，公司开放代码有软件著作权保护版权，他人不得将
*               资料代码传网上供他人下载，不得以谋利为目去销售资料代码，发现有此类操作者，
*               公司会提前告知，请1天内及时处理，否则你的侵权违规行为会被贴出在抖音、
*               今日头条、百家号、公司官网、微信公众平台、技术博客、知乎等平台予以公示曝光
*               此种侵权所为会成为个人终身污点，影响升学、找工作、社会声誉、很快就很在无人机界出名，后果很严重。
*               因此行为给公司造成重大损失者，会以法律途径解决，感谢您的合作，谢谢！！！
----------------------------------------------------------------------------------------------------------------------*/
#include "Headfile.h"
#include "uart.h"
#include "Usart.h"
#include "Ringbuf.h"


void UART6_IRQHandler(void);


//串口循环队列缓冲数据定义
RingBuff_t COM0_Rx_Buf,COM1_Rx_Buf,COM2_Rx_Buf,COM3_Rx_Buf,COM4_Rx_Buf,COM5_Rx_Buf,COM6_Rx_Buf,COM7_Rx_Buf;
/***********************************************************
@函数名：UART0_IRQHandler
@入口参数：无
@出口参数：无
功能描述：串口0数据接收
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void UART0_IRQHandler(void)//UART0中断函数
{	
  //获取中断标志 原始中断状态 不屏蔽中断标志		
  uint32_t flag = UARTIntStatus(UART0_BASE,1);
  //清除中断标志	
  UARTIntClear(UART0_BASE,flag);		
  //判断FIFO是否还有数据		
  while(UARTCharsAvail(UART0_BASE))		
  {
		uint8_t ch=UARTCharGet(UART0_BASE);
		SDK_Data_Receive_Prepare_2(ch);		
//		RingBuf_Write(ch,&COM0_Rx_Buf,SDK_Target_Length*2);//往环形队列里面写数据
//		if(COM0_Rx_Buf.Ring_Buff[0]!=0xFF)
//		{
//			COM0_Rx_Buf.Head=1;
//			COM0_Rx_Buf.Tail=0; 
//		}		
  }
}

/***********************************************************
@函数名：ConfigureUART0
@入口参数：无
@出口参数：无
功能描述：串口0配置
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void ConfigureUART0(unsigned long bound)//串口0初始化
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//使能GPIO外设		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);//使能UART外设
  GPIOPinConfigure(GPIO_PA0_U0RX);//GPIO模式配置 PA0--RX PA1--TX 
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO的UART模式配置
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), bound,
										(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
										 UART_CONFIG_PAR_NONE));
	
  //UART协议配置 波特率115200 8位 1停止位  无校验位	
  //UART禁用FIFO 默认FIFO Level为4/8 寄存器满8字节后产生中断	//禁用后接收1位就产生中断	
  UARTFIFODisable(UART0_BASE);//使能UART0中断	IntEnable(INT_UART0);	
  UARTIntEnable(UART0_BASE,UART_INT_RX);//使能UART0接收中断		
  UARTIntRegister(UART0_BASE,UART0_IRQHandler);//UART中断地址注册	
  IntPrioritySet(INT_UART0, USER_INT2);
}

/***********************************************************
@函数名：USART0_Send
@入口参数：uint8_t *pui8Buffer, uint32_t ui32Count
@出口参数：无
功能描述：串口0发送N个字节数据
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void USART0_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//发送N个字节长度的数据
{
  while(ui32Count--)
  {
    UARTCharPut(UART0_BASE, *pui8Buffer++);
  }
}
/***********************************************************
@函数名：wust_sendware
@入口参数：unsigned char *wareaddr, int16_t waresize
@出口参数：无
功能描述：山外虚拟示波器发送
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void wust_sendware(unsigned char *wareaddr, int16_t waresize)//山外发送波形
{
#define CMD_WARE     3
  uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};//帧头
  uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};//帧尾
  USART1_Send(cmdf, sizeof(cmdf));
  USART1_Send(wareaddr, waresize);
  USART1_Send(cmdr, sizeof(cmdr));
}

/***********************************************************
@函数名：UART1_IRQHandler
@入口参数：无
@出口参数：无
功能描述：串口1数据接收
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void UART1_IRQHandler(void)//UART1中断函数
{				
  uint32_t flag = UARTIntStatus(UART1_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志	
  UARTIntClear(UART1_BASE,flag);//清除中断标志	
  while(UARTCharsAvail(UART1_BASE))//判断FIFO是否还有数据		
  {			
    //RingBuf_Write(UARTCharGet(UART1_BASE),&COM1_Rx_Buf,100);//往环形队列里面写数据	
    NCLink_Data_Prase_Prepare(UARTCharGet(UART1_BASE)); 		
  }
}


/***********************************************************
@函数名：USART1_Send
@入口参数：uint8_t *pui8Buffer, uint32_t ui32Count
@出口参数：无
功能描述：串口1发送N个字节数据
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void USART1_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//发送N个字节长度的数据
{
  while(ui32Count--)
  {
    UARTCharPut(UART1_BASE, *pui8Buffer++);
  }
}
/***********************************************************
@函数名：ConfigureUART1
@入口参数：无
@出口参数：无
功能描述：串口1配置
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void ConfigureUART1(unsigned long bound)//串口1初始化
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//使能GPIO外设		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);//使能UART外设
  GPIOPinConfigure(GPIO_PB0_U1RX);//GPIO模式配置 PB0--RX PB1--TX 
  GPIOPinConfigure(GPIO_PB1_U1TX);
  GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO的UART模式配置
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), bound,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART1_BASE);//使能UART1中断	
  UARTIntEnable(UART1_BASE,UART_INT_RX);//使能UART1接收中断		
  UARTIntRegister(UART1_BASE,UART1_IRQHandler);//UART1中断地址注册	
  IntPrioritySet(INT_UART1, USER_INT3);
}


/***********************************************************
@函数名：UART2_IRQHandler
@入口参数：无
@出口参数：无
功能描述：串口2数据接收
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void UART2_IRQHandler(void)
{
  uint32_t flag = UARTIntStatus(UART2_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志		
  UARTIntClear(UART2_BASE,flag);//清除中断标志	
  while(UARTCharsAvail(UART2_BASE))//判断FIFO是否还有数据				
  {		
		uint8_t ch=UARTCharGet(UART2_BASE);
		switch(Uart2_Mode)
		{
			case 0x00:
				LC30X_OpticalFlow_Sense_Prase(ch,lc30x_buf);
			break;
			case 0x01:
				RingBuf_Write(ch,&COM2_Rx_Buf,58*2);//往环形队列里面写数据	
				if(COM2_Rx_Buf.Ring_Buff[0]!=0xA5)
				{
					COM2_Rx_Buf.Head=1;
					COM2_Rx_Buf.Tail=0; 
				}	
			break;
			default:LC30X_OpticalFlow_Sense_Prase(ch,lc30x_buf);
		}		
  }
}

/***********************************************************
@函数名：USART2_Send
@入口参数：uint8_t *pui8Buffer, uint32_t ui32Count
@出口参数：无
功能描述：串口2发送N个字节数据
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void USART2_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//发送N个字节长度的数据
{
  while(ui32Count--)
  {
    UARTCharPut(UART2_BASE, *pui8Buffer++);
  }
}
/***********************************************************
@函数名：ConfigureUART2
@入口参数：无
@出口参数：无
功能描述：串口2配置
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void ConfigureUART2(unsigned long bound)//串口6初始化
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//使能GPIO外设		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);//使能UART外设
  
  HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;//解锁PD6
  HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;//确认
  HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;//重新锁定
  
  GPIOPinConfigure(GPIO_PD6_U2RX);//GPIO模式配置 PD6--RX PD7--TX 
  GPIOPinConfigure(GPIO_PD7_U2TX);
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);//GPIO的UART模式配置
  UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), bound,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART2_BASE);//使能UART2中断	
  UARTIntEnable(UART2_BASE,UART_INT_RX);//使能UART6接收中断		
	
  OpticalFlow_Init();//光流滤波参数初始化
	
  UARTIntRegister(UART2_BASE,UART2_IRQHandler);//UART中断地址注册	
  IntPrioritySet(INT_UART2, USER_INT4);
}

/***********************************************************
@函数名：UART3_IRQHandler
@入口参数：无
@出口参数：无
功能描述：串口3数据接收
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void UART3_IRQHandler(void)
{		
  uint32_t flag = UARTIntStatus(UART3_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志		
  UARTIntClear(UART3_BASE,flag);//清除中断标志			
  while(UARTCharsAvail(UART3_BASE))//判断FIFO是否还有数据		
  {	
		uint8_t ch=UARTCharGet(UART3_BASE);
		SDK_Data_Receive_Prepare_1(ch);
//		RingBuf_Write(ch,&COM3_Rx_Buf,SDK_Target_Length*2);//往环形队列里面写数据
//		if(COM3_Rx_Buf.Ring_Buff[0]!=0xFF)
//		{
//			COM3_Rx_Buf.Head=1;
//			COM3_Rx_Buf.Tail=0; 
//		}	
	}	
}

/***********************************************************
@函数名：USART3_Send
@入口参数：uint8_t *pui8Buffer, uint32_t ui32Count
@出口参数：无
功能描述：串口3发送N个字节数据
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void USART3_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//发送N个字节长度的数据
{
  while(ui32Count--)
  {
    UARTCharPut(UART3_BASE, *pui8Buffer++);
  }
}

/***********************************************************
@函数名：ConfigureUART3
@入口参数：无
@出口参数：无
功能描述：串口3配置
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void ConfigureUART3(void)//串口3初始化
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//使能GPIO外设		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);//使能UART外设
  GPIOPinConfigure(GPIO_PC6_U3RX);//GPIO模式配置 PC6--RX PC7--TX 
  GPIOPinConfigure(GPIO_PC7_U3TX);
  GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);//GPIO的UART模式配置
  UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), 256000,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART3_BASE);//使能UART0中断	
  UARTIntEnable(UART3_BASE,UART_INT_RX);//使能UART3接收中断		
  UARTIntRegister(UART3_BASE,UART3_IRQHandler);//UART3中断地址注册	
  IntPrioritySet(INT_UART3, USER_INT2);
}

/***********************************************************
@函数名：UART6_IRQHandler
@入口参数：无
@出口参数：无
功能描述：串口6数据接收
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
uint16_t UART6_LEN=0;
void UART6_IRQHandler(void)
{		
  uint32_t flag = UARTIntStatus(UART6_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志		
  UARTIntClear(UART6_BASE,flag);//清除中断标志	
  while(UARTCharsAvail(UART6_BASE))//判断FIFO是否还有数据		
  {
		switch(Reserved_Uart)
		{
			case FRONT_RANGE_FINDER:
			{
				RingBuf_Write(UARTCharGet(UART6_BASE),&COM6_Rx_Buf,UART6_LEN);//往环形队列里面写数据
			}
			break;
			case GPS_M8N:
			{
				RingBuf_Write(UARTCharGet(UART6_BASE),&COM6_Rx_Buf,UART6_LEN);//往环形队列里面写数据
				if(COM6_Rx_Buf.Ring_Buff[0]!=0XB5)
				{
					COM6_Rx_Buf.Head=1;
					COM6_Rx_Buf.Tail=0; 
				}
			}			
			break;
			case THIRD_PARTY_STATE:
			{
				//uint8_t data=UARTCharGet(UART6_BASE);
				//RingBuf_Write(data,&COM6_Rx_Buf,UART6_LEN);//往环形队列里面写数据
				//if(COM6_Rx_Buf.Ring_Buff[0]!=0xFC)	{COM6_Rx_Buf.Head=1;COM6_Rx_Buf.Tail=0;}
				NCLink_Data_Prase_Prepare_Lite(UARTCharGet(UART6_BASE)); 
			}
			break;
			default:
			{
				RingBuf_Write(UARTCharGet(UART6_BASE),&COM6_Rx_Buf,UART6_LEN);//往环形队列里面写数据
				if(COM6_Rx_Buf.Ring_Buff[0]!=0XB5)
				{
					COM6_Rx_Buf.Head=1;
					COM6_Rx_Buf.Tail=0; 
				}
			}	
		}		
  }
}

/***********************************************************
@函数名：USART6_Send
@入口参数：uint8_t *pui8Buffer, uint32_t ui32Count
@出口参数：无
功能描述：串口6发送N个字节数据
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void USART6_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//发送N个字节长度的数据
{
  while(ui32Count--)
  {
    UARTCharPut(UART6_BASE, *pui8Buffer++);
  }
}


/***********************************************************
@函数名：ConfigureUART6
@入口参数：无
@出口参数：无
功能描述：串口6配置
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void ConfigureUART6(unsigned long bound,uint16_t len)
{
	UART6_LEN=len;
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//使能GPIO外设		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);//使能UART外设
  
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;//解锁PD6
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;//确认
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;//重新锁定
	
	GPIOPinConfigure(GPIO_PD4_U6RX);//GPIO模式配置 PD4--RX PD5--TX 
  GPIOPinConfigure(GPIO_PD5_U6TX);
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);//GPIO的UART模式配置
  UARTConfigSetExpClk(UART6_BASE, SysCtlClockGet(), bound,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART6_BASE);//使能UART0中断	
  UARTIntEnable(UART6_BASE,UART_INT_RX);//使能UART6接收中断		
	
  UARTIntRegister(UART6_BASE,UART6_IRQHandler);//UART6中断地址注册	
  IntPrioritySet(INT_UART6, USER_INT1);
}





/***********************************************************
@函数名：UART7_IRQHandler
@入口参数：无
@出口参数：无
功能描述：串口7数据接收
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void UART7_IRQHandler(void)//UART2中断函数
{		
  uint32_t flag = UARTIntStatus(UART7_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志	
  UARTIntClear(UART7_BASE,flag);//清除中断标志		
  while(UARTCharsAvail(UART7_BASE))//判断FIFO是否还有数据			
  {			
		switch(rangefinder_current)
		{
			case US100:
			{
				RingBuf_Write(UARTCharGet(UART7_BASE),&COM7_Rx_Buf,4);//往环形队列里面写数据
			}			
			break;
			case TFMINI:
			{
				RingBuf_Write(UARTCharGet(UART7_BASE),&COM7_Rx_Buf,18);//往环形队列里面写数据
				if(COM7_Rx_Buf.Ring_Buff[0]!=0x59)
				{
					COM7_Rx_Buf.Head=1;
					COM7_Rx_Buf.Tail=0; 
				}	
			}
			break;	
			case SMD15:
			{
				sdm15_prase(UARTCharGet(UART7_BASE));
			}
			break;
			case TOFSENSE_M:
			{
				TOFSense_M_Prase(UARTCharGet(UART7_BASE));
			}
			break;
			case TOFSENSE:
			case LIDAR3D:	
			default:
			{
				uint8_t data=UARTCharGet(UART7_BASE);
				RingBuf_Write(data,&COM7_Rx_Buf,16*TOFSENSE_CURRENT_WORK);//往环形队列里面写数据  32
				TOFSense_Prase(data);
				if(COM7_Rx_Buf.Ring_Buff[0]!=0x57)
				{
					COM7_Rx_Buf.Head=1;
					COM7_Rx_Buf.Tail=0; 
				}	
			}
			break;	
		}
  }
}

/***********************************************************
@函数名：USART7_Send
@入口参数：uint8_t *pui8Buffer, uint32_t ui32Count
@出口参数：无
功能描述：串口7发送N个字节数据
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void USART7_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//发送N个字节长度的数据
{
  while(ui32Count--)
  {
    UARTCharPut(UART7_BASE, *pui8Buffer++);
  }
}

void ConfigureUART7(unsigned long baud)//串口7初始化
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//使能GPIO外设		
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);//使能UART外设
	GPIOPinConfigure(GPIO_PE0_U7RX);//GPIO模式配置 PE0--RX PE1--TX 
	GPIOPinConfigure(GPIO_PE1_U7TX);
	GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO的UART模式配置
	UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet(), baud,
										(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
										 UART_CONFIG_PAR_NONE));
	RingBuff_Init(&COM7_Rx_Buf); 	
	UARTFIFODisable(UART7_BASE);//使能UART0中断	
	UARTIntEnable(UART7_BASE,UART_INT_RX);//使能UART0接收中断		
	UARTIntRegister(UART7_BASE,UART7_IRQHandler);//UART中断地址注册	
	IntPrioritySet(INT_UART7, USER_INT3);
}




/***********************************************************
@函数名：Vcan_Send
@入口参数：无
@出口参数：无
功能描述：山外数据发送函数，默认发送8个通道，数据类型为float
，每个通道数据可以自己定义
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void Vcan_Send(void)//山外地面站发送
{
  static float DataBuf[8];	
	DataBuf[0]=Receiver_PPM_Databuf[0];
  DataBuf[1]=Receiver_PPM_Databuf[1];
  DataBuf[2]=Receiver_PPM_Databuf[2];
  DataBuf[3]=Receiver_PPM_Databuf[3];
  DataBuf[4]=Receiver_PPM_Databuf[4];
  DataBuf[5]=Receiver_PPM_Databuf[5];
  DataBuf[6]=Receiver_PPM_Databuf[6];
  DataBuf[7]=Receiver_PPM_Databuf[7];
  wust_sendware((unsigned char *)DataBuf,sizeof(DataBuf));
}




