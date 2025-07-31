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
#include "Bling.h"

/***************************************************
函数名: void Bling_Init(void)
说明:	LED状态指示灯初始化
入口:	无
出口:	无
备注:	上电初始化，运行一次
****************************************************/
void Bling_Init()
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2 |GPIO_PIN_3);	
	
	
//	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_PIN_1);//蓝色
//	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,GPIO_PIN_2);//红色
//	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,GPIO_PIN_3);//绿色
	
	Quad_Start_Bling();            //开机LED预显示
}



Bling_Light rgb_red,rgb_blue,rgb_green,Light_4;
uint16_t Bling_Mode=0;
/***************************************************
函数名: void Bling_Set(Bling_Light *Light,
uint32_t Continue_time,//持续时间
uint16_t Period,//周期100ms~1000ms
float Percent,//0~100%
uint16_t  Cnt,
GPIO_TypeDef* Port,
uint16_t Pin
,uint8_t Flag)
说明:	状态指示灯设置函数
入口:	时间、周期、占空比、端口等
出口:	无
备注:	程序初始化后、始终运行
****************************************************/
void Bling_Set(Bling_Light *Light,
               uint32_t Continue_time,//持续时间
               uint16_t Period,//周期100ms~1000ms
               float Percent,//0~100%
               uint16_t  Cnt,
               uint32_t Port,
               uint16_t Pin
                 ,uint8_t Flag)
{
  Light->Bling_Contiune_Time=(uint16_t)(Continue_time/WP_Duty_Dt_Ms);//持续时间
  Light->Bling_Period=Period;//周期
  Light->Bling_Percent=Percent;//占空比
  Light->Bling_Cnt=Cnt;
  Light->Port=Port;//端口
  Light->Pin=Pin;//引脚
  Light->Endless_Flag=Flag;//无尽模式
}

/***************************************************
函数名: void Bling_Process(Bling_Light *Light)//闪烁运行线程
说明:	状态指示灯实现
入口:	状态灯结构体
出口:	无
备注:	程序初始化后、始终运行
****************************************************/
void Bling_Process(Bling_Light *Light)//闪烁运行线程
{
  if(Light->Bling_Contiune_Time>=1)  Light->Bling_Contiune_Time--;
  else GPIOPinWrite(Light->Port,Light->Pin,0);//置高,亮
  if(Light->Bling_Contiune_Time!=0//总时间未清0
     ||Light->Endless_Flag==1)//判断无尽模式是否开启
  {
    Light->Bling_Cnt++;
    if(5*Light->Bling_Cnt>=Light->Bling_Period) Light->Bling_Cnt=0;//计满清零
    if(5*Light->Bling_Cnt<=Light->Bling_Period*Light->Bling_Percent)
      GPIOPinWrite(Light->Port,Light->Pin,Light->Pin);//置低，亮
    else GPIOPinWrite(Light->Port,Light->Pin,0);//置高，灭
  }
}


void GPIO_SetBits(Bling_Light *Light)
{
  GPIOPinWrite(Light->Port,Light->Pin,Light->Pin);//置高，灭
}

void GPIO_ResetBits(Bling_Light *Light)
{
  GPIOPinWrite(Light->Port,Light->Pin,0);//置低，亮
}
/***************************************************
函数名: Bling_Working(uint16 bling_mode)
说明:	状态指示灯状态机
入口:	当前模式
出口:	无
备注:	程序初始化后、始终运行
****************************************************/
void Bling_Working(uint16 bling_mode)
{
  if(bling_mode==0)//全灭
  {
    Bling_Process(&rgb_red);
    Bling_Process(&rgb_blue);
    Bling_Process(&rgb_green);
  }
  else if(bling_mode==1)//加速度计6面校准模式
  {
    if(flight_direction==0)//第一面校准准备
    {
      Bling_Process(&rgb_blue);
			GPIO_SetBits(&rgb_red);
			GPIO_SetBits(&rgb_green);
    }
    else if(flight_direction==1)//第二面校准准备
    {
      Bling_Process(&rgb_blue);
			GPIO_SetBits(&rgb_red);
			GPIO_SetBits(&rgb_green);
    }
    else if(flight_direction==2)//第三面校准准备
    {
      Bling_Process(&rgb_blue);
			GPIO_SetBits(&rgb_red);
			GPIO_SetBits(&rgb_green);
    }
    else if(flight_direction==3)//第四面校准准备
    {
      Bling_Process(&rgb_blue);
			GPIO_SetBits(&rgb_red);
			GPIO_SetBits(&rgb_green);
    }
    else if(flight_direction==4)//第五面校准准备
    {
      Bling_Process(&rgb_blue);
			GPIO_SetBits(&rgb_red);
			GPIO_SetBits(&rgb_green);
    }
    else if(flight_direction==5)//第六面校准准备
    {
      Bling_Process(&rgb_blue);
			GPIO_SetBits(&rgb_red);
			GPIO_SetBits(&rgb_green);
    }
    else
    {
      Bling_Process(&rgb_red);
      Bling_Process(&rgb_blue);
      Bling_Process(&rgb_green);
    }
  }
  else if(bling_mode==2)//磁力计校准模式
  {
    if(Mag_Calibration_Mode==0)//水平面校准
    {
      Bling_Process(&rgb_blue);
			GPIO_SetBits(&rgb_red);
			GPIO_SetBits(&rgb_green);
    }
    else if(Mag_Calibration_Mode==1)////竖直平面校准
    {
      Bling_Process(&rgb_blue);
			GPIO_SetBits(&rgb_red);
			GPIO_SetBits(&rgb_green);
    }
    else if(Mag_Calibration_Mode==2)////竖直平面校准
    {
      Bling_Process(&rgb_blue);
			GPIO_SetBits(&rgb_red);
			GPIO_SetBits(&rgb_green);
    }
    else
    {
      Bling_Process(&rgb_red);
      Bling_Process(&rgb_blue);
      Bling_Process(&rgb_green);
    }
  }
  else if(bling_mode==3)//全灭
  {
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0);
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,0);
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,0);
  }
    
}


/***************************************************
函数名: void Quad_Start_Bling(void)
说明:	LED初始化后开机闪烁
入口:	无
出口:	无
备注:	上电初始化，运行一次
****************************************************/
void Quad_Start_Bling()
{
  Bling_Set(&rgb_red,3000,200,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_1,0); //红色
  Bling_Set(&rgb_blue,3000,200,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//蓝色
  Bling_Set(&rgb_green,3000,200,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_3,0); //绿色
  //Bling_Set(&Light_4,2000,1000,0.5,0,GPIO_PORTC_BASE,GPIO_PIN_2,1);
}

