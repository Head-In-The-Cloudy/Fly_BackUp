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
*               无名创新开源飞控QQ群：2号群465082224、1号群540707961
*               CSDN博客：http://blog.csdn.net/u011992534
*               B站教学视频：https://space.bilibili.com/67803559/#/video				优酷ID：NamelessCotrun无名小哥
*               无名创新国内首款TI开源飞控设计初衷、知乎专栏:https://zhuanlan.zhihu.com/p/54471146
*               客户使用心得、改进意见征集贴：http://www.openedv.com/forum.php?mod=viewthread&tid=234214&extra=page=1
*               淘宝店铺：https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               公司官网:www.nameless.tech
*               修改日期:2022/03/01                  
*               版本：躺赢者PRO——CarryPilot_V4.0.3
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
#include "myiic.h"


//-----------------------------------------------------
#define ICM20689_ADDRESS 0x68    // AD0 should be low

#define ICM20689_SELF_TEST_X_GYRO  0x00
#define ICM20689_SELF_TEST_Y_GYRO  0x01 
#define ICM20689_SELF_TEST_Z_GYRO  0x02
#define ICM20689_SELF_TEST_X_ACCEL 0x0D	 
#define ICM20689_SELF_TEST_Y_ACCEL 0x0E	 
#define ICM20689_SELF_TEST_Z_ACCEL 0x0F	 
#define ICM20689_XG_OFFS_USRH      0x13	
#define ICM20689_XG_OFFS_USRL      0x14
#define ICM20689_YG_OFFS_USRH      0x15
#define ICM20689_YG_OFFS_USRL      0x16
#define ICM20689_ZG_OFFS_USRH      0x17
#define ICM20689_ZG_OFFS_USRL      0x18
#define ICM20689_SMPLRT_DIV        0x19
#define ICM20689_CONFIG            0x1A
#define ICM20689_GYRO_CONFIG       0x1B
#define ICM20689_ACCEL_CONFIG      0x1C
#define ICM20689_ACCEL_CONFIG_2    0x1D

#define ICM20689_ACCEL_XOUT_H      0x3B
#define ICM20689_ACCEL_XOUT_L      0x3C
#define ICM20689_ACCEL_YOUT_H      0x3D
#define ICM20689_ACCEL_YOUT_L      0x3E
#define ICM20689_ACCEL_ZOUT_H      0x3F
#define ICM20689_ACCEL_ZOUT_L      0x40
#define ICM20689_TEMP_OUT_H        0x41
#define ICM20689_TEMP_OUT_L        0x42
#define ICM20689_GYRO_XOUT_H       0x43
#define ICM20689_GYRO_XOUT_L       0x44
#define ICM20689_GYRO_YOUT_H       0x45
#define ICM20689_GYRO_YOUT_L       0x46
#define ICM20689_GYRO_ZOUT_H       0x47
#define ICM20689_GYRO_ZOUT_L       0x48

#define ICM20689_PWR_MGMT_1        0x6B
#define ICM20689_PWR_MGMT_2        0x6C
#define ICM20689_WHO_AM_I          0x75
/* USER CODE END Private defines */


uint8_t ICM20689_Read_Reg(uint8_t reg)
{												
	return i2cRead(ICM20689_ADDRESS,reg);
}

void ICM20689_Write_Reg(uint8_t reg,uint8_t value)
{
  i2cWrite(ICM20689_ADRESS,reg,value);
}


uint8_t Init_ICM20689(void)
{	
	if(ICM20689_Read_Reg(ICM20689_WHO_AM_I)==0x98)
	{
		//printf("ICM20689 ready\n");
	}
	else
	{
		//printf("ICM20689 error\n");
		return 1;
	}
	ICM20689_Write_Reg(ICM20689_PWR_MGMT_1, 0x00);	//解除休眠状态
	ICM20689_Write_Reg(ICM20689_CONFIG, 0x07);      //低通滤波频率，典型值：0x07(3600Hz)此寄存器内决定Internal_Sample_Rate==8K
	
/*******************Init GYRO and ACCEL******************************/	
	ICM20689_Write_Reg(ICM20689_SMPLRT_DIV, 0x00);  //陀螺仪采样率，典型值：0x07(1kHz) (SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV) )
	ICM20689_Write_Reg(ICM20689_GYRO_CONFIG, 0x18); //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
	ICM20689_Write_Reg(ICM20689_ACCEL_CONFIG, 0x18);//加速计自检、测量范围及高通滤波频率，典型值：0x18(不自检，16G)
	ICM20689_Write_Reg(ICM20689_ACCEL_CONFIG_2, 0x08);//加速计高通滤波频率 典型值 ：0x08  （1.13kHz）	
	return 0;
}

