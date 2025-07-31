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
#include "License.h"
#include "stdlib.h"





void System_Start_Init(void)
{
  SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |SYSCTL_OSC_MAIN);
  FPUEnable();
  FPULazyStackingEnable();
  initTime();
  EEPROM_Init();
	//全局中断使能
	IntMasterEnable();
	IntPriorityGroupingSet(3);	
//	EEPROMMassErase();
}	



void Resume_Factory_Setting(void)
{
	EEPROMMassErase();
}


/***********************************************************
@函数名：ESC_HardWave_Init
@入口参数：无
@出口参数：无
@功能描述：电调校准，随即执行电调模式下相关资源初始化，简单外
芯片资源、必备外设初始化
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void ESC_HardWave_Init()//只初始化校准电调的必要资源
{
  OLED_Init_Fast();//OLED显示屏初始化
  PPM_Init();//PPM解析初始化
  PWM_Init();//PWM初始化
  delay_ms(100);
  WriteFlashParameter(ESC_CALIBRATION_FLAG,0);//写零避免下次上电再次进入
  while(1)
  {
    ESC_Calibration();
    LCD_clear_L(0,0);LCD_clear_L(0,1);LCD_P8x16Str(0,0,"Please Move Thr");
    LCD_clear_L(0,2);LCD_clear_L(0,3);LCD_P8x16Str(0,2,"Down When ESC");
    LCD_clear_L(0,4);LCD_clear_L(0,5);LCD_P8x16Str(0,4,"Beep Beep");
    LCD_P6x8Str(80,4,"Thr:");
    write_6_8_number(80,5,Receiver_PPM_Databuf[2]);
    LCD_clear_L(0,6);LCD_P6x8Str(0,6,"Repower When Set Up");
  }
}



/***********************************************************
@函数名：WP_Init
@入口参数：无
@出口参数：无
功能描述：对系统芯片资源、先执行开机电调校准自检函数，若需要
电调校准，随即执行电调模式下相关资源初始化，否则执行常规硬件
资源初始化。
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void WP_Init(void)
{
  System_Start_Init();//系统启动初始化
	Simulation_PWM_Init();
  ReadFlashParameterOne(ESC_CALIBRATION_FLAG,&ESC_Calibration_Flag);
  if(ESC_Calibration_Flag==1)
  {
    ESC_HardWave_Init();//只初始化校准电调的必要资源 
  }
  else
  {	
		OLED_Init();			      //OLED显示屏初始化
		Init_I2C1(400*100000);			        //硬件I2C初始化
		spl0601_init();         //气压计初始化
  }
}


