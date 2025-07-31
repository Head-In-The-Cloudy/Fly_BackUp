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
#include "tfmini.h"



			
					
uint8_t rangefinder_current=TOFSENSE;

tfmini tfdata; 
uint16_t tfmini_tail,last_tfmini_tail,tfmini_offset;
void TFmini_Statemachine(void)
{
  static uint16_t tfmini_cnt=0;
	uint8_t check_sum=0;
	tfmini_cnt++;
  if(tfmini_cnt>=20)//100ms=10hz
  {
		tfmini_cnt=0;
		tfmini_tail=COM7_Rx_Buf.Tail;
		if(tfmini_tail<9)
		{
		  tfmini_offset=9;
		}
    else
		{
		  tfmini_offset=0;
		}			
		for(uint16_t i=tfmini_offset;i<tfmini_offset+9;i++)
		{
			if(COM7_Rx_Buf.Ring_Buff[i]==0x59
				&&COM7_Rx_Buf.Ring_Buff[i+1]==0x59)
			{
				for(uint16_t j=i;j<i+8;j++)  check_sum += COM7_Rx_Buf.Ring_Buff[j];//和校验
				if(check_sum==COM7_Rx_Buf.Ring_Buff[i+8])
				{
					tfdata.pre_last_distance=tfdata.last_distance;//上上次高度
					tfdata.last_distance=tfdata.distance;//上次高度
					tfdata.last_vel=tfdata.vel;//上次速度
					
					tfdata.distance=((COM7_Rx_Buf.Ring_Buff[i+3]*256)+COM7_Rx_Buf.Ring_Buff[i+2])/1.0f;
//					tfdata.distance*=rMat[8];
					tfdata.strength=(COM7_Rx_Buf.Ring_Buff[i+4]*256)+COM7_Rx_Buf.Ring_Buff[i+5];
					tfdata.temperature=((COM7_Rx_Buf.Ring_Buff[i+6]*256)+COM7_Rx_Buf.Ring_Buff[i+7])/8.0f-256;
					
					tfdata.vel=(tfdata.distance-tfdata.last_distance)/0.1f; //观测速度
					tfdata.acc=(tfdata.vel-tfdata.last_vel)/0.1f;					  //观测加速度
					
					ground_distance_cm=tfdata.distance*WP_AHRS.rMat[8];
					ground_distance_div=tfdata.vel;
					ground_distance_acc=tfdata.acc;		
			
					WP_Sensor.tfmini_updtate_flag=1;
				}					
			}
		}
			
		
		if(tfdata.strength!=65535&&tfdata.strength>=100)
		{
			if(tfdata.distance<=1100&&tfdata.distance>1.0f)  Sensor_Flag.Ground_Health=1;
			else Sensor_Flag.Ground_Health=0;
		}			
		else Sensor_Flag.Ground_Health=0;
	}	
}


void Ground_Sensor_Statemachine(void)
{
	if(rangefinder_current==US100)						US_100_Statemachine();	  //超声波传感器状态机更新
	else if(rangefinder_current==TFMINI)			TFmini_Statemachine();		//激光TFMINI_PLUS传感器状态机更新
	else if(rangefinder_current==TOFSENSE)	;			  //激光TOFSensor传感器状态机更新
	else if(rangefinder_current==SMD15)	    ;			  //激光TOFSensor传感器状态机更新
	else {;}
	Check_Front_Tofsense();
	us100_front_statemachine();
}
























//					static uint8_t fault_flag=0;
//					if(ABS(tfdata.vel)<1000&&tfdata.distance>=1.0f&&tfdata.strength!=65535)
//					{
//						if(fault_flag==1&&tfdata.distance==tfdata.pre_last_distance)//本次=上上次
//						{
//							fault_flag=1;
//						}			
//						else if(fault_flag==1&&tfdata.distance==tfdata.last_distance)//本次=上次
//						{
//							fault_flag=1;
//						}
//						else if(fault_flag==1&&tfdata.last_distance==tfdata.pre_last_distance)//上次=上上次
//						{
//							fault_flag=1;		
//						}
//						//以上为传感器异常处理
//						else
//						{
//							TF_Distance=tfdata.distance;
//							TF_Distance_Div=tfdata.vel;
//							TF_Distance_Acc=tfdata.acc;		
//						}
//					}
//					else
//					{
//						fault_flag=1;		
//					}	
					
					
					
					
					
					
					
					




