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
#include "PPM.h"

uint8_t ppm_update_flag=0;
void PortFIntHandler(void);
/***********************************************************
@函数名：PPM_Init
@入口参数：无
@出口参数：无
功能描述：PPM初始化
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void PPM_Init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//使能GPIO外设
  GPIOIntRegister(GPIO_PORTC_BASE, PortFIntHandler);//GPIO注册中断
  GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_3);//PC3作为中断输入源
  GPIOPadConfigSet(GPIO_PORTC_BASE,GPIO_PIN_3,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);//配置PC3为上拉
  GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_3 , GPIO_FALLING_EDGE);//中断触发类型为下降沿触发
  GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_3);//使能PC3中断
  IntPrioritySet(INT_GPIOC,USER_INT0);
}


uint16_t PPM_Databuf[10]={0};
uint16_t Receiver_PPM_Databuf[10]={0};
static uint16 PPM_buf[10]={0}; 
/***********************************************************
@函数名：PortFIntHandler
@入口参数：无
@出口参数：无
功能描述：PPM中断函数
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void PortFIntHandler(void)//Port3中断程序
{
	static uint32_t last_ppm_time=0,now_ppm_time=0;
	static uint8_t ppm_ready=0,ppm_sample_cnt=0;;
	static uint16_t ppm_time_delta=0;	
  GPIOIntClear(GPIO_PORTC_BASE,GPIO_PIN_3);//清除中断标志	
  last_ppm_time=now_ppm_time;//系统运行时间获取，单位us
  now_ppm_time=micros();//单位us	
  ppm_time_delta=now_ppm_time-last_ppm_time;//获取时间间隔
  //PPM解析开始
	if(ppm_ready==1)
	{
		if(ppm_time_delta>=2200)//帧结束电平至少2ms=2000us，由于部分老版本遥控器、
			//接收机输出PPM信号不标准，当出现解析异常时，尝试改小此值，该情况仅出现一例：使用天地飞老版本遥控器
		{
			//memcpy(PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16));
			ppm_ready = 1;
			ppm_sample_cnt=0;
			ppm_update_flag=1;
		}
		else if(ppm_time_delta>=900&&ppm_time_delta<=2100)
		{         
			PPM_buf[ppm_sample_cnt++]=ppm_time_delta;//对应通道写入缓冲区       
			if(ppm_sample_cnt>=8)//单次解析结束
			{
				memcpy(Receiver_PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16));
				//ppm_ready=0;
				ppm_sample_cnt=0;
			}
		}
		else  ppm_ready=0;
	}
	else if(ppm_time_delta>=2200)//帧结束电平至少2ms=2000us
	{
		ppm_ready=1;
		ppm_sample_cnt=0;
		ppm_update_flag=0;
	}
}
