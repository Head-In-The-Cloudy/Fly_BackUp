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
#include "Temperature_Ctrl.h"

#define Temperature_Ctrl_Enable   1

#define Temperature_Setpoint   50
const float Temp_Control_Unit[20]={1  ,1 ,1 ,0 ,0 ,0 , 0 ,50    ,10  ,0 ,80,  8.00  ,0.75   ,125.0  ,0    ,0 ,100 , 1 ,  1 ,  1 };
/*
1偏差限幅标志；  2积分限幅标志；3积分分离标志；   4期望；
5反馈            6偏差；        7上次偏差；       8偏差限幅值；
9积分分离偏差值；10积分值       11积分限幅值；    12控制参数Kp；
13控制参数Ki；   14控制参数Kd； 15控制器总输出；  16上次控制器总输出
17总输出限幅度； 18变积分控制时的积分增益
*/		
void Temperature_Ctrl_Init(void)
{
  Total_Controller.IMU_Temperature_Control.Err_Limit_Flag=(uint8)(Temp_Control_Unit[0]);//1偏差限幅标志
  Total_Controller.IMU_Temperature_Control.Integrate_Limit_Flag=(uint8)(Temp_Control_Unit[1]);//2积分限幅标志
  Total_Controller.IMU_Temperature_Control.Integrate_Separation_Flag=(uint8)(Temp_Control_Unit[2]);//3积分分离标志
  Total_Controller.IMU_Temperature_Control.Expect=Temp_Control_Unit[3];//4期望
  Total_Controller.IMU_Temperature_Control.FeedBack=Temp_Control_Unit[4];//5反馈值
  Total_Controller.IMU_Temperature_Control.Err=Temp_Control_Unit[5];//6偏差
  Total_Controller.IMU_Temperature_Control.Last_Err=Temp_Control_Unit[6];//7上次偏差
  Total_Controller.IMU_Temperature_Control.Err_Max=Temp_Control_Unit[7];//8偏差限幅值
  Total_Controller.IMU_Temperature_Control.Integrate_Separation_Err=Temp_Control_Unit[8];//9积分分离偏差值
  Total_Controller.IMU_Temperature_Control.Integrate=Temp_Control_Unit[9];//10积分值
  Total_Controller.IMU_Temperature_Control.Integrate_Max=Temp_Control_Unit[10];//11积分限幅值
  Total_Controller.IMU_Temperature_Control.Kp=Temp_Control_Unit[11];//12控制参数Kp
  Total_Controller.IMU_Temperature_Control.Ki=Temp_Control_Unit[12];//13控制参数Ki
  Total_Controller.IMU_Temperature_Control.Kd=Temp_Control_Unit[13];//14控制参数Ki
  Total_Controller.IMU_Temperature_Control.Control_OutPut=Temp_Control_Unit[14];//15控制器总输出
  Total_Controller.IMU_Temperature_Control.Last_Control_OutPut=Temp_Control_Unit[15];//16上次控制器总输出
  Total_Controller.IMU_Temperature_Control.Control_OutPut_Limit=Temp_Control_Unit[16];//17上次控制器总输出
}

void Simulation_PWM_Init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; 
	HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= GPIO_PIN_2; 
	HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = 0x0;
  
	HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; 
	HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= GPIO_PIN_3; 
	HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = 0x0;	
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);	
  GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2);
	
	GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,0);
}

#define Simulation_PWM_Period_MAX  100//100*1ms=0.1S
void Simulation_PWM_Output(uint16_t width)
{
	if(Sensor_Flag.Mpu_Health==0)
	{
		GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,0);
		return ;
	}
#if Temperature_Ctrl_Enable
	uint16_t static cnt=0;
	cnt++;
	if(cnt>=Simulation_PWM_Period_MAX)  cnt=0;
  if(cnt<=width) GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,GPIO_PIN_2);
	else GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,0);
#else
	GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,0);
#endif
	
}


void Temperature_Ctrl(void)
{
#if Temperature_Ctrl_Enable
 static uint16_t tmp_period_cnt=0;
 tmp_period_cnt++;
 Total_Controller.IMU_Temperature_Control.Expect=Temperature_Setpoint;
 Total_Controller.IMU_Temperature_Control.FeedBack=WP_Sensor._temperature;
 if(tmp_period_cnt>=20)  //8*2.5ms=20ms
 {
	 PID_Control_Div_LPF(&Total_Controller.IMU_Temperature_Control,0.02f);
	 Total_Controller.IMU_Temperature_Control.Control_OutPut=constrain_float(Total_Controller.IMU_Temperature_Control.Control_OutPut,0,Simulation_PWM_Period_MAX);
	 tmp_period_cnt=0;
 }
#else
	Total_Controller.IMU_Temperature_Control.Control_OutPut=0;
#endif
}

uint8_t Temperature_State_Get(void)
{
#if Temperature_Ctrl_Enable
  return (ABS(Total_Controller.IMU_Temperature_Control.Err))<=2.0f?1:0;
#else
	return 1;
#endif	
}

uint8_t Temperature_Stable_Flag=0;
void Temperature_State_Check(void)
{
	static uint16_t _cnt=0;
	static uint16_t temperature_crash_cnt=0;
	if(Temperature_State_Get()==1){
		_cnt++;
		if(_cnt>=400) Temperature_Stable_Flag=1;
	}
	else{
		_cnt/=2;
	}
	
	if(temperature_crash_cnt<400)
	{
		if(WP_Sensor.last_temperature==WP_Sensor.temperature)	temperature_crash_cnt++;
		else temperature_crash_cnt/=2;
		Sensor_Flag.Mpu_Health=1;		
	}
	else
	{
		Sensor_Flag.Mpu_Health=0;
		Controler_State=Lock_Controler;//上锁
	}	
}	
