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
#include "WP_PWM.h"

static uint16_t period;

//#define PWM_PERIOD_MAX  3125//2.5ms————400hz
#define PWM_PERIOD_MAX    6250//5ms————200hz
#define PWM_PERIOD_MAX_20MS  (3125*8) 


/***********************************************************
@函数名：PWM_Init
@入口参数：无
@出口参数：无
@功能描述：PWM初始化
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
#if FLIGHT_ESC_PWM==0
void PWM_Init(void)
{
  SysCtlPWMClockSet(SYSCTL_PWMDIV_64); // Set divider to 80M/8=10M=0.1us
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // Enable PWM peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Enable GPIOB peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // Enable GPIOC peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Enable GPIOE peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  // Use alternate function
  GPIOPinConfigure(GPIO_PB6_M0PWM0);
  GPIOPinConfigure(GPIO_PB7_M0PWM1);
	
  GPIOPinConfigure(GPIO_PB4_M0PWM2);
  GPIOPinConfigure(GPIO_PB5_M0PWM3);
  GPIOPinConfigure(GPIO_PE4_M0PWM4);
  GPIOPinConfigure(GPIO_PE5_M0PWM5);
	
  // Use pin with PWM peripheral
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);//M0PWM0
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);//M0PWM1
	
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);//M0PWM2
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);//M0PWM3
  GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);//M0PWM4
  GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);//M0PWM5
	
  // Configure the PWM generator for count down mode with immediate updates to the parameters
  PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  //PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	
  // The period is set to 2.5ms (400 Hz)
  period = PWM_PERIOD_MAX; 
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, period); // Set the period
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, period);
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, period); // Set the period
  //PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, period);
  // Start the timers in generator 0 and 1
  PWMGenEnable(PWM0_BASE, PWM_GEN_0);
  PWMGenEnable(PWM0_BASE, PWM_GEN_1);
  PWMGenEnable(PWM0_BASE, PWM_GEN_2);
  // Enable the outputs
  PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT 
													| PWM_OUT_2_BIT | PWM_OUT_3_BIT
													| PWM_OUT_4_BIT | PWM_OUT_5_BIT
													,  true);
  PWM_Output(1000,1000,1000,1000);
	//预留PWM
	Reserved_PWM1_Output(1500);//PB7——M0PWM1——对应扩展板排针PWM端口的标号P3
	Reserved_PWM2_Output(pinch_pwm_us);//PB6——M0PWM0——对应扩展板排针PWM端口的标号P4
}

/***********************************************************
@函数名：PWM_Output
@入口参数：uint16_t width1,uint16_t width2,uint16_t width3,
uint16_t width4,uint16_t width5,uint16_t width6,
uint16_t width7,uint16_t width8
@出口参数：无
@功能描述：PWM输出
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void PWM_Output(uint16_t width1,uint16_t width2,uint16_t width3,uint16_t width4)
{
	uint16_t pwm[4]={0};
	pwm[0]=1.25*width1;
	pwm[1]=1.25*width2;
	pwm[2]=1.25*width3;
	pwm[3]=1.25*width4;
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_5,pwm[0]);//PE5——M0PWM5 
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_4,pwm[1]);//PE4——M0PWM4 
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,pwm[2]);//PB4——M0PWM2 
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,pwm[3]);//PB5——M0PWM3 
}      

void Reserved_PWM1_Output(uint16_t us)
{
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,1.25*us);//PB7——M0PWM1——对应扩展板排针PWM端口的标号P3
}

void Reserved_PWM2_Output(uint16_t us)
{
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,1.25*us);//PB6——M0PWM0——对应扩展板排针PWM端口的标号P4
}


#else
void PWM_Init(void)
{
  SysCtlPWMClockSet(SYSCTL_PWMDIV_64); // Set divider to 80M/8=10M=0.1us
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // Enable PWM peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Enable GPIOB peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // Enable GPIOC peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Enable GPIOE peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  // Use alternate function
  GPIOPinConfigure(GPIO_PB6_M0PWM0);//PB6——M0PWM0
  GPIOPinConfigure(GPIO_PB7_M0PWM1);//PB7——M0PWM1
  GPIOPinConfigure(GPIO_PB4_M0PWM2);//PB4——M0PWM2
  GPIOPinConfigure(GPIO_PB5_M0PWM3);//PB5——M0PWM3
  GPIOPinConfigure(GPIO_PC4_M0PWM6);//PB6——M0PWM6
  GPIOPinConfigure(GPIO_PC5_M0PWM7);//PB6——M0PWM7
  // Use pin with PWM peripheral
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);//M0PWM0
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);//M0PWM1
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);//M0PWM2
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);//M0PWM3
  GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);//M0PWM6
  GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);//M0PWM7
	
  // Configure the PWM generator for count down mode with immediate updates to the parameters
  PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	
  // The period is set to 2.5ms (400 Hz)
  period = PWM_PERIOD_MAX; 
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, period); // Set the period
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, period);
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, period);
  // Start the timers in generator 0 and 1
  PWMGenEnable(PWM0_BASE, PWM_GEN_0);
  PWMGenEnable(PWM0_BASE, PWM_GEN_1);
  PWMGenEnable(PWM0_BASE, PWM_GEN_3);
  // Enable the outputs
  PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT 
													| PWM_OUT_2_BIT | PWM_OUT_3_BIT
													| PWM_OUT_6_BIT | PWM_OUT_7_BIT 
													,  true);
  PWM_Output(1000,1000,1000,1000);
	//预留PWM
	Reserved_PWM1_Output(1500);//PB4——M0PWM2——对应扩展板EPWM端口的标号P7
	Reserved_PWM2_Output(1500);//PB5——M0PWM3——对应扩展板EPWM端口的标号P8
}

/***********************************************************
@函数名：PWM_Output
@入口参数：uint16_t width1,uint16_t width2,uint16_t width3,uint16_t width4
@出口参数：无
@功能描述：PWM输出
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void PWM_Output(uint16_t width1,uint16_t width2,uint16_t width3,uint16_t width4)
{
	uint16_t pwm[4]={0};
	pwm[0]=1.25*width1;
	pwm[1]=1.25*width2;
	pwm[2]=1.25*width3;
	pwm[3]=1.25*width4;
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_6,pwm[0]);//PC4——M0PWM6 
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_7,pwm[1]);//PC5——M0PWM7
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,pwm[2]);//PB7——M0PWM1
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,pwm[3]);//PB6——M0PWM0
}      

void Reserved_PWM1_Output(uint16_t us)
{
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,1.25*us);//PB4——M0PWM2——对应扩展板EPWM端口的标号P7
}

void Reserved_PWM2_Output(uint16_t us)
{
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,1.25*us);//PB5——M0PWM3——对应扩展板EPWM端口的标号P8
}


#endif

