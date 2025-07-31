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
#include "US_100.h"




float ground_distance_cm=0,ground_distance_div=0,ground_distance_acc=0,_ground_distance_cm=0;
uint16_t Start_Tail=0,Last_Start_Tail;
/***********************************************************
@函数名：US_100_Distance
@入口参数：uint8 MSB,uint8 LSB
@出口参数：无
@功能描述：超声波高度数据合成
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
float US_100_Distance(uint8 MSB,uint8 LSB)
{
  return (256*(MSB)+(LSB))/10.0f;//单位cm
}

/***********************************************************
@函数名：US_100_Temperature
@入口参数：uint8 data
@出口参数：无
@功能描述：超声波温度数据合成
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
float US_100_Temperature(uint8 data)
{
  return (data-45)/1.0;
}

/***********************************************************
@函数名：US_100_Start
@入口参数：无
@出口参数：无
@功能描述：超声波高度数据触发
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void US_100_Start(void)
{	
  UARTCharPut(UART7_BASE, US_100_Distance_CMD);
}


uint8_t US_100_Update_Flag=0;
uint8 US_100_Cnt=0;



us100_data us100;


/***********************************************************
@函数名：US_100_Statemachine
@入口参数：无
@出口参数：无
@功能描述：超声波数据解析状态机
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void US_100_Statemachine(void)
{
	static uint8_t fault_flag=0;
  US_100_Cnt++;
  if(US_100_Cnt>=20)//100ms
  {
    US_100_Start();//超声波触发，接收在外部中断里面处理
    US_100_Cnt=0;
    Start_Tail=COM7_Rx_Buf.Tail;
    if(ABS(Start_Tail-Last_Start_Tail)==2)
    {
			us100.pre_last_distance=us100.last_distance;//上上次高度
			us100.last_distance=us100.distance;//上次高度
						
      if(Start_Tail==0)
      {
        us100.distance=US_100_Distance(COM7_Rx_Buf.Ring_Buff[2],
                                    COM7_Rx_Buf.Ring_Buff[3]);
        US_100_Update_Flag=1;
      }
      else if(Start_Tail==1)
      {
        us100.distance=US_100_Distance(COM7_Rx_Buf.Ring_Buff[3],
                                    COM7_Rx_Buf.Ring_Buff[0]);
        US_100_Update_Flag=1;
      }
      else if(Start_Tail==2)
      {
        us100.distance=US_100_Distance(COM7_Rx_Buf.Ring_Buff[0],
                                    COM7_Rx_Buf.Ring_Buff[1]);
        US_100_Update_Flag=1;
      }
      else if(Start_Tail==3)
      {
        us100.distance=US_100_Distance(COM7_Rx_Buf.Ring_Buff[1],
                                    COM7_Rx_Buf.Ring_Buff[2]);
        US_100_Update_Flag=1;
      }
			
			us100.last_vel=us100.vel;
			us100.vel=(us100.distance-us100.last_distance)/0.1f;
			us100.acc=(us100.vel-us100.last_vel)/0.1f;
			if(ABS(us100.vel)<1000)
			{
				if(fault_flag==1&&us100.distance==us100.pre_last_distance)//本次=上上次
				{
					fault_flag=1;
				}			
				else if(fault_flag==1&&us100.distance==us100.last_distance)//本次=上次
				{
					fault_flag=1;
				}
				else if(fault_flag==1&&us100.last_distance==us100.pre_last_distance)//上次=上上次
				{
					fault_flag=1;		
				}
				//以上为传感器异常处理
				else
				{
					ground_distance_cm=us100.distance;
					ground_distance_div=us100.vel;
					ground_distance_acc=us100.acc;		
				}
			}
			else
			{
				fault_flag=1;		
			}	
			WP_Sensor.us100_updtate_flag=1;
    }
    Last_Start_Tail=Start_Tail;
  }		
	
  if(ground_distance_cm<=User_Height_Max&&ground_distance_cm>0)  Sensor_Flag.Ground_Health=1;
  else  Sensor_Flag.Ground_Health=0; 
}


