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
#include "WP_Ctrl.h"
#include "control_config.h"
/*****************遥控器行程设置**********************/
int16_t M_PWM_1,M_PWM_2,M_PWM_3,M_PWM_4;//4个电机输出PWM
int16_t _M_PWM_1,_M_PWM_2,_M_PWM_3,_M_PWM_4;//上次4个电机输出PWM
uint8_t Controler_High_Mode=1,Last_Controler_High_Mode=1;
uint8_t Controler_Horizontal_Mode=1,Last_Controler_Horizontal_Mode=1;
uint8_t Controler_SDK1_Mode=0,Last_Controler_SDK1_Mode=0;
uint8_t Controler_Land_Mode=1,Last_Controler_Land_Mode=1;


uint8_t Controler_GPS_Mode=1,Last_Controler_GPS_Mode=1;
uint8_t Hover_Point_Set_Flag=0;


uint8_t Control_Mode_Change=0;
uint16_t High_Hold_Throttle=0;
uint16_t Throttle=0;
Controller_Output Flight;



typedef struct
{
	uint32_t switch_time;
	uint32_t release_time; 
	uint32_t switch_in_time;
}gps_switch_state;
gps_switch_state gps_switch;


/***************************************************
函数名: void Controler_Mode_Select(void)
说明:	控制器模式选择函数
入口:	无
出口:	无
备注:	中断任务调度持续运行
****************************************************/
void Controler_Mode_Select()
{
  Last_Controler_High_Mode=Controler_High_Mode;//上次高度控制模式
  Last_Controler_Horizontal_Mode=Controler_Horizontal_Mode;//上次位置控制模式
  Last_Controler_Land_Mode=Controler_Land_Mode;//上次返航模式
	 
  if(PPM_Databuf[4]<=(RC_Data.cal[4].min+RC_Data.cal[4].deadband))  Controler_High_Mode=1;//低位:纯姿态自稳
  else Controler_High_Mode=2;//非低位:气压计、超声波定高
		
  if(PPM_Databuf[5]>=(RC_Data.cal[5].max-RC_Data.cal[5].deadband))       
	{
		Controler_SDK1_Mode=1;//SDK1模式
		Flight.roll_pitch_angle_limit_enable=1;
	}
	else//非SDK1模式	
	{
		NCQ_SDK_Reset();
		flight_subtask_reset();
		Controler_SDK1_Mode=0;
		Flight.roll_pitch_angle_limit_enable=0;
	}
		
	uint8_t ch7_value=RC_Read_Switch(RC_Data.aux[AUX3]);
	uint8_t ch8_value=RC_Read_Switch(RC_Data.aux[AUX4]);

	current_state.last_fault=current_state.fault;
	switch(Optical_Type_Present)
	{
		case 1:
		case 2:
		{
				//定高模式下，第七通道切入中位，并持续保持1s后进入GPS定点模式
				if(ch7_value==0)//遥控器三段开关处于低位  
				{
					Controler_GPS_Mode=1;
					gps_switch.release_time=millis();//单位ms
					//1、切入GPS定点模式，将位置期望给0，当满足定点条件时，在GPS定点控制内，自动刷新当前位置为目标位置
					//2、切出GPS定点模式，将位置期望给0，为下次切入GPS定点模式做准备
					Total_Controller.Latitude_Position_Control.Expect=0;
					Total_Controller.Longitude_Position_Control.Expect=0;
					East_North_Ctrl_Reset();
					Hover_Point_Set_Flag=0;
				}
				else if(ch7_value==1)//遥控器三段开关处于中位     
				{
					 gps_switch.switch_time=millis();//单位ms
					 gps_switch.switch_in_time=(gps_switch.switch_time-gps_switch.release_time);
					 if(gps_switch.switch_in_time>=1000)//持续切入1S中，才认为正常切入
					 {
						 Controler_GPS_Mode=2;
					 }
				} 
				else//高位 
				{
					Controler_GPS_Mode=1;
					gps_switch.release_time=millis();//单位ms
					//1、切入GPS定点模式，将位置期望给0，当满足定点条件时，在GPS定点控制内，自动刷新当前位置为目标位置
					//2、切出GPS定点模式，将位置期望给0，为下次切入GPS定点模式做准备
					Total_Controller.Latitude_Position_Control.Expect=0;
					Total_Controller.Longitude_Position_Control.Expect=0;		
					Hover_Point_Set_Flag=0;
				}
							
				if(ch7_value==2) 				  Controler_Land_Mode=2;//高位——返航模式        
				else if(ch7_value==0)   	Controler_Land_Mode=1;//低位——非返航模式	
				Last_Controler_GPS_Mode=Controler_GPS_Mode;	
				
				//当Optical_Type_Present不为3时，表示不使能slam位置控制，第8通道直接确定光流定点模式，第七通道处于什么状态无关联
				switch(ch8_value)
				{
					case 0:
					{
						Controler_Horizontal_Mode=1;//姿态自稳控制
						//OpticalFlow_SINS_Reset();
						OpticalFlow_Ctrl_Reset();	
					}
					break;
					case 2:
					{
						Controler_Horizontal_Mode=2;//光流水平位置控制
					}
					break;
					default:
					{
						Controler_Horizontal_Mode=1;//姿态自稳控制
						//OpticalFlow_SINS_Reset();
						OpticalFlow_Ctrl_Reset();						
					}
				}		
		}
		break;	
		case 3://此状态下不允许切到GPS模式
		case 4:	
		{
			switch(ch7_value)
			{
				case 0://开关7处于低位
				{
					switch(ch8_value)
					{
						case 0:
						{
							Controler_Horizontal_Mode=1;//姿态自稳控制
							//OpticalFlow_SINS_Reset();
							OpticalFlow_Ctrl_Reset();	
						}
						break;
						case 2:
						{
							Controler_Horizontal_Mode=2;//光流水平位置控制
						}
						break;
						default:
						{
							Controler_Horizontal_Mode=1;//姿态自稳控制
							//OpticalFlow_SINS_Reset();
							OpticalFlow_Ctrl_Reset();						
						}
					}	
					Controler_Land_Mode=1;//非返航模式 
				}
				break;
				case 1://开关7处于中位
				{
					current_state.fault=1;//定位异常，遥控器手动操作设定
					Controler_Horizontal_Mode=1;//强制姿态自稳控制
					Controler_Land_Mode=1;//非返航模式 
				}
				break;
				case 2:
				{
					Controler_Land_Mode=2;//高位——返航模式 
				}
				break;
			}
		}
		break;
		default://其它未实现情形，统一强制切到水平姿态自稳模式
		{
			Controler_Horizontal_Mode=1;//姿态自稳控制
		}		
	}
		
	
//*****************************************************************************************
  if(Unwanted_Lock_Flag==1)//定高模式解锁后，无任何操作
  {
    Thr_Push_Over_State=Thr_Push_Over_Deadband();
    if(Thr_Push_Over_State==2)//只要向上推过了中位死区，即把允许自动上锁操作
    {
      Unwanted_Lock_Flag=0;
    }
    else
    {
      Take_Off_Reset();//清积分
      //Throttle_Control_Reset();//清积分
    }
  }
  
  if(Controler_Land_Mode!=Last_Controler_Land_Mode)
  {
		Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP];//将开关拨动瞬间的惯导高度设置为期望高度
    if(Controler_Land_Mode==1)   land_reset();//返航模式切回正常模式 
  }
  
  if(Controler_High_Mode!=Last_Controler_High_Mode)
  {
    if(Controler_High_Mode==2)  {Control_Mode_Change=1;}//自稳切定高，设置基准油门值，悬停高度
    if(Controler_High_Mode==1)  {Control_Mode_Change=1;}//定高切自稳
  }
  
  if(Controler_Horizontal_Mode!=Last_Controler_Horizontal_Mode)	Control_Mode_Change=2;//位置通道有切换
  
  
  if(Control_Mode_Change==1)//存在定高模式切换，高度只设置一次
  {
    if(Controler_High_Mode==High_Hold_Mode)//本次为定高模式，即自稳切定高
    {
      High_Hold_Throttle=Throttle_Control;//保存当前油门值，只存一次
      /*******************将当前惯导竖直位置估计作为目标高度***************************/
      Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP];//将开关拨动瞬间的惯导高度设置为期望高度
    }
    else//本次为自稳模式，即定高切自稳
    {
      //Throttle_Control_Reset();
    }
    Control_Mode_Change=0;//将模式切换位置0,有且仅处理一次
  }
  else if(Control_Mode_Change==2)//存在定点模式切换，悬停位置只设置一次
  {
    Control_Mode_Change=0;//已响应本次定点档位切换
  }

//*****************************************************************************************
	if(Controler_GPS_Mode==2&&Hover_Point_Set_Flag==0)//GPS挡位下，悬停点未设置
	{
    if(GPS_ok()==TRUE)//首次切定点不满足定点条件，之后又满足定点条件
    {
			/*******************将当前惯导水平位置估计作为目标悬停点************************/
			Total_Controller.Latitude_Position_Control.Expect=NamelessQuad.Position[_NORTH];
			Total_Controller.Longitude_Position_Control.Expect=NamelessQuad.Position[_EAST];			
			East_North_Ctrl_Reset();
			Hover_Point_Set_Flag=1;
		}
		else //定点档位处于定点模式，但未满足定点条件，将Pos_Hold_SetFlag置0，等待满足时再设置悬停点
		{
			Total_Controller.Latitude_Position_Control.Expect=0;
			Total_Controller.Longitude_Position_Control.Expect=0;
			East_North_Ctrl_Reset();
			Hover_Point_Set_Flag=0;//不满足定点条件时，复位位置锁定标志位，等待满足定点条件时，再次锁定		
		}			
	}  
}




void Angle_Control_Target(Controller_Output *_flight_output)//角度环节
{
	//姿态角度期望
	Total_Controller.Pitch_Angle_Control.Expect=_flight_output->pitch_outer_control_output;
	Total_Controller.Roll_Angle_Control.Expect=_flight_output->roll_outer_control_output;
	
//	if(_flight_output->roll_pitch_angle_limit_enable==1)//SDK模式最大倾角输出限制——备用
//	{
//		Total_Controller.Pitch_Angle_Control.Expect=constrain_float(Total_Controller.Pitch_Angle_Control.Expect,-10,10);
//		Total_Controller.Roll_Angle_Control.Expect=constrain_float(Total_Controller.Roll_Angle_Control.Expect,-10,10);
//	}
	
  //姿态角度反馈
  Total_Controller.Roll_Angle_Control.FeedBack =(WP_AHRS.Roll-Roll_Offset);
	Total_Controller.Pitch_Angle_Control.FeedBack=(WP_AHRS.Pitch-Pitch_Offset);
	//运行PID控制器
  PID_Control(&Total_Controller.Roll_Angle_Control,min_ctrl_dt);  
	PID_Control(&Total_Controller.Pitch_Angle_Control,min_ctrl_dt);
	/***************内环角速度期望****************/
	Total_Controller.Pitch_Gyro_Control.Expect=Total_Controller.Pitch_Angle_Control.Control_OutPut;
	Total_Controller.Roll_Gyro_Control.Expect=Total_Controller.Roll_Angle_Control.Control_OutPut;
  switch(_flight_output->yaw_ctrl_mode)
	{
	  case ROTATE:
		{		
			if(_flight_output->yaw_outer_control_output==0)//偏航杆置于中位
			{
				if(Total_Controller.Yaw_Angle_Control.Expect==0)//初次切回中
				{
				  Total_Controller.Yaw_Angle_Control.Expect=WP_AHRS.Yaw;
				}
				Total_Controller.Yaw_Angle_Control.FeedBack=WP_AHRS.Yaw;//偏航角反馈
				PID_Control_Yaw(&Total_Controller.Yaw_Angle_Control,min_ctrl_dt);//偏航角度控制
				Total_Controller.Yaw_Gyro_Control.Expect=Total_Controller.Yaw_Angle_Control.Control_OutPut;//偏航角速度环期望，来源于偏航角度控制器输出
			}
			else//波动偏航方向杆后，只进行内环角速度控制
			{
				Total_Controller.Yaw_Angle_Control.Expect=0;//偏航角期望给0,不进行角度控制
				Total_Controller.Yaw_Gyro_Control.Expect=_flight_output->yaw_outer_control_output;//偏航角速度环期望，直接来源于遥控器打杆量
			}
		}
		break;
	  case AZIMUTH://绝对航向角度
	  {
			if(_flight_output->yaw_ctrl_start==1)//更新偏航角度期望
			{
				float yaw_tmp=_flight_output->yaw_outer_control_output;
				if(yaw_tmp<0) 	yaw_tmp+=360;
				if(yaw_tmp>360) yaw_tmp-=360;
				Total_Controller.Yaw_Angle_Control.Expect=yaw_tmp;
				_flight_output->yaw_ctrl_start=0;
				_flight_output->yaw_ctrl_cnt=0;
				_flight_output->yaw_ctrl_end=0;
			}
			
			if(_flight_output->yaw_ctrl_end==0)//判断偏航角是否控制完毕
			{
				if(ABS(Total_Controller.Yaw_Angle_Control.Err)<3.0f) _flight_output->yaw_ctrl_cnt++;
				else _flight_output->yaw_ctrl_cnt/=2;
				
				if(_flight_output->yaw_ctrl_cnt>=200)  _flight_output->yaw_ctrl_end=1;
			}			
			
			Total_Controller.Yaw_Angle_Control.FeedBack=WP_AHRS.Yaw;//偏航角反馈
			PID_Control_Yaw(&Total_Controller.Yaw_Angle_Control,min_ctrl_dt);//偏航角度控制
			//对最大偏航角速度进行限制
			float tmp=constrain_float(Total_Controller.Yaw_Angle_Control.Control_OutPut,-YAW_GYRO_CTRL_MAX,YAW_GYRO_CTRL_MAX);
			Total_Controller.Yaw_Gyro_Control.Expect=tmp;//偏航角速度环期望，来源于偏航角度控制器输出
		}
		break;
		case CLOCKWISE://顺时针————相对给定时刻的航向角度
		{
			if(_flight_output->yaw_ctrl_start==1)//更新偏航角度期望
			{
				float yaw_tmp=WP_AHRS.Yaw-_flight_output->yaw_outer_control_output;
				if(yaw_tmp<0) 	yaw_tmp+=360;
				if(yaw_tmp>360) yaw_tmp-=360;
				Total_Controller.Yaw_Angle_Control.Expect=yaw_tmp;
				_flight_output->yaw_ctrl_start=0;
				_flight_output->yaw_ctrl_cnt=0;
				_flight_output->yaw_ctrl_end=0;
			}
			
			if(_flight_output->yaw_ctrl_end==0)//判断偏航角是否控制完毕
			{
				if(ABS(Total_Controller.Yaw_Angle_Control.Err)<3.0f) _flight_output->yaw_ctrl_cnt++;
				else _flight_output->yaw_ctrl_cnt/=2;
				
				if(_flight_output->yaw_ctrl_cnt>=200)  _flight_output->yaw_ctrl_end=1;
			}
			
			Total_Controller.Yaw_Angle_Control.FeedBack=WP_AHRS.Yaw;//偏航角反馈
			PID_Control_Yaw(&Total_Controller.Yaw_Angle_Control,min_ctrl_dt);//偏航角度控制
			//对最大偏航角速度进行限制
			float tmp=constrain_float(Total_Controller.Yaw_Angle_Control.Control_OutPut,-YAW_GYRO_CTRL_MAX,YAW_GYRO_CTRL_MAX);
			Total_Controller.Yaw_Gyro_Control.Expect=tmp;//偏航角速度环期望，来源于偏航角度控制器输出
		}
		break;
		case ANTI_CLOCKWISE://逆时针——相对给定时刻的航向角度
		{
			if(_flight_output->yaw_ctrl_start==1)//更新偏航角度期望
			{
				float yaw_tmp=WP_AHRS.Yaw+_flight_output->yaw_outer_control_output;
				if(yaw_tmp<0) 	yaw_tmp+=360;
				if(yaw_tmp>360) yaw_tmp-=360;
				Total_Controller.Yaw_Angle_Control.Expect=yaw_tmp;
				_flight_output->yaw_ctrl_start=0;
				_flight_output->yaw_ctrl_cnt=0;
				_flight_output->yaw_ctrl_end=0;
			}
			
			if(_flight_output->yaw_ctrl_end==0)//判断偏航角是否控制完毕
			{
				if(ABS(Total_Controller.Yaw_Angle_Control.Err)<3.0f) _flight_output->yaw_ctrl_cnt++;
				else _flight_output->yaw_ctrl_cnt/=2;
				
				if(_flight_output->yaw_ctrl_cnt>=200)  _flight_output->yaw_ctrl_end=1;
			}
			
			Total_Controller.Yaw_Angle_Control.FeedBack=WP_AHRS.Yaw;//偏航角反馈
			PID_Control_Yaw(&Total_Controller.Yaw_Angle_Control,min_ctrl_dt);//偏航角度控制
			//对最大偏航角速度进行限制
			float tmp=constrain_float(Total_Controller.Yaw_Angle_Control.Control_OutPut,-YAW_GYRO_CTRL_MAX,YAW_GYRO_CTRL_MAX);
			Total_Controller.Yaw_Gyro_Control.Expect=tmp;//偏航角速度环期望，来源于偏航角度控制器输出
		}
		break;
		case CLOCKWISE_TURN://以某一角速度顺时针旋转多长时间
		{
			uint32_t curr_time_ms=millis();
			
			if(_flight_output->yaw_ctrl_start==1)//更新偏航角度期望
			{
				//对最大偏航角速度进行限制
				float tmp=constrain_float(-_flight_output->yaw_outer_control_output,-YAW_GYRO_CTRL_MAX,YAW_GYRO_CTRL_MAX);
				Total_Controller.Yaw_Gyro_Control.Expect=tmp;//偏航角速度环期望，来源于偏航角度控制器输出
				_flight_output->yaw_ctrl_start=0;
				_flight_output->yaw_ctrl_cnt=0;
				_flight_output->yaw_ctrl_end=0;
				_flight_output->start_time_ms=curr_time_ms;//记录开始转动的时间				
			}
			
			if(_flight_output->yaw_ctrl_end==0)//判断偏航角是否控制完毕
			{
				uint32_t tmp=curr_time_ms-_flight_output->start_time_ms;
				if(tmp>=_flight_output->execution_time_ms)					
					_flight_output->yaw_ctrl_end=1;
			}
			else
			{
				//执行完毕后,
				//1、将偏航角速度期望给0,
				//2、停止旋转并锁定当前偏航角，需要退出CLOCKWISE_TURN模式，角度期望才会有效，因为此模式没有对偏航角度进行控制
				Total_Controller.Yaw_Gyro_Control.Expect=0;
				Total_Controller.Yaw_Angle_Control.Expect=WP_AHRS.Yaw;
			}
		}
		break;
		case ANTI_CLOCKWISE_TURN://以某一角速度逆时针旋转多长时间
		{
			uint32_t curr_time_ms=millis();
			
			if(_flight_output->yaw_ctrl_start==1)//更新偏航角度期望
			{
				//对最大偏航角速度进行限制
				float tmp=constrain_float(_flight_output->yaw_outer_control_output,-YAW_GYRO_CTRL_MAX,YAW_GYRO_CTRL_MAX);
				Total_Controller.Yaw_Gyro_Control.Expect=tmp;//偏航角速度环期望，来源于偏航角度控制器输出
				_flight_output->yaw_ctrl_start=0;
				_flight_output->yaw_ctrl_cnt=0;
				_flight_output->yaw_ctrl_end=0;
				_flight_output->start_time_ms=curr_time_ms;//记录开始转动的时间				
			}
			
			if(_flight_output->yaw_ctrl_end==0)//判断偏航角是否控制完毕
			{
				uint32_t tmp=curr_time_ms-_flight_output->start_time_ms;
				if(tmp>=_flight_output->execution_time_ms)					
					_flight_output->yaw_ctrl_end=1;
			}
			else
			{
				//执行完毕后,
				//1、将偏航角速度期望给0,
				//2、停止旋转并锁定当前偏航角，需要退出CLOCKWISE_TURN模式，角度期望才会有效，因为此模式没有对偏航角度进行控制
				Total_Controller.Yaw_Gyro_Control.Expect=0;
				Total_Controller.Yaw_Angle_Control.Expect=WP_AHRS.Yaw;
			}
		}
		break;
  }
} 

void Yaw_Fault_Check(void)
{
	static uint16_t _cnt=0;
  /*******偏航控制异常情况判断，即偏航控制量很大时，偏航角速度很小，如此时为强外力干扰、已着地等******************************/
  if(ABS(Total_Controller.Yaw_Gyro_Control.Control_OutPut)>Total_Controller.Yaw_Gyro_Control.Control_OutPut_Limit/2//偏航控制输出相对较大
     &&ABS(WP_AHRS.Yaw_Gyro)<=30.0f)//偏航角速度相对很小
  {
    _cnt++;
    if(_cnt>=500) _cnt=500;
  }
  else _cnt/=2;//不满足，快速削减至0
  
  if(_cnt>=400)//持续5ms*400=2S,特殊处理
  {
    PID_Integrate_Reset(&Total_Controller.Yaw_Gyro_Control); //清空偏航角速度控制的积分
    PID_Integrate_Reset(&Total_Controller.Yaw_Angle_Control);//清空偏航角控制的积分
    Total_Controller.Yaw_Angle_Control.Expect=WP_AHRS.Yaw;	 //将当前偏航角，作为期望偏航角
    _cnt=0;
  }
}

void Gyro_Control(void)//角速度环
{	
	//俯仰、横滚方向姿态内环角速度控制器采用PID控制器
	/***************内环角速度期望****************/
	Total_Controller.Pitch_Gyro_Control.Expect=Total_Controller.Pitch_Angle_Control.Control_OutPut;
	Total_Controller.Roll_Gyro_Control.Expect=Total_Controller.Roll_Angle_Control.Control_OutPut;
	/***************内环角速度反馈****************/
	Total_Controller.Pitch_Gyro_Control.FeedBack=WP_AHRS.Pitch_Gyro;
	Total_Controller.Roll_Gyro_Control.FeedBack=WP_AHRS.Roll_Gyro;
	Total_Controller.Yaw_Gyro_Control.FeedBack=Yaw_Gyro_Earth_Frame;//Yaw_Gyro;
	/***************内环角速度控制：微分参数动态调整****************/
	if(WP_AHRS.player_level==0||WP_AHRS.player_level==1)
	{
		PID_Control_Div_LPF_For_Gyro(&Total_Controller.Pitch_Gyro_Control,min_ctrl_dt);
		PID_Control_Div_LPF_For_Gyro(&Total_Controller.Roll_Gyro_Control,min_ctrl_dt);
		PID_Control_Div_LPF_For_Gyro(&Total_Controller.Yaw_Gyro_Control,min_ctrl_dt);	
	}		
	else if(WP_AHRS.player_level==2)//微分先行+一阶低通40hz
	{
		pid_ctrl_rpy_gyro_maple(&Total_Controller.Pitch_Gyro_Control ,min_ctrl_dt,incomplete_diff,first_order_lpf);
		pid_ctrl_rpy_gyro_maple(&Total_Controller.Roll_Gyro_Control  ,min_ctrl_dt,incomplete_diff,first_order_lpf);
		pid_ctrl_rpy_gyro_maple(&Total_Controller.Yaw_Gyro_Control   ,min_ctrl_dt,incomplete_diff,first_order_lpf);		
	}
	else if(WP_AHRS.player_level==3)//微分先行+二阶低通50hz
	{
		pid_ctrl_rpy_gyro_maple(&Total_Controller.Pitch_Gyro_Control ,min_ctrl_dt,incomplete_diff,second_order_lpf);
		pid_ctrl_rpy_gyro_maple(&Total_Controller.Roll_Gyro_Control  ,min_ctrl_dt,incomplete_diff,second_order_lpf);
		pid_ctrl_rpy_gyro_maple(&Total_Controller.Yaw_Gyro_Control   ,min_ctrl_dt,incomplete_diff,second_order_lpf);		
	}
	else if(WP_AHRS.player_level==4)//直接微分+无低通
	{
		pid_ctrl_rpy_gyro_maple(&Total_Controller.Pitch_Gyro_Control ,min_ctrl_dt,direct_diff,noneed_lpf);
		pid_ctrl_rpy_gyro_maple(&Total_Controller.Roll_Gyro_Control  ,min_ctrl_dt,direct_diff,noneed_lpf);
		pid_ctrl_rpy_gyro_maple(&Total_Controller.Yaw_Gyro_Control   ,min_ctrl_dt,direct_diff,noneed_lpf);		
	}	
	/*******偏航控制异常处理******************************/
//  Yaw_Fault_Check();
}






void Main_Leading_Control(void)
{
	if(Flight.init==0)
	{		
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output=0;
		Flight.pitch_outer_control_output=0;
		Flight.roll_outer_control_output=0;
		Flight.init=1;
	}
	
/*********************根据遥控器切换档位，飞控进入不同模式****************************/	
	if(Controler_Land_Mode==2)//一键降落模式
	{
		land_run();
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
		return ;
	}
	else if(Controler_GPS_Mode==2)//GPS定点模式  
	{
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
		ncq_control_poshold();//位置控制
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];		
	  return ;
	}
	
/************************第五通道控制高度模式************************/		
	if(Controler_High_Mode==1)//高度纯手动模式
	{
		Throttle=Throttle_Control;//油门直接来源于遥控器油门给定
		Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
		Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output=RC_Data.rc_rpyt[RC_YAW];		
	}
	else
	{		
		if(Controler_SDK1_Mode==1)//用户SDK开发者自动飞行模式：水平+高度控制
		{
			Auto_Flight_Ctrl(&SDK1_Mode_Setup);
		}
		else//普通定高、定点控制
		{
/************************第八通道控制水平位置模式************************/		
			switch(Controler_Horizontal_Mode)
			{
				case 1://水平自稳
				{
					Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
					Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];
					
					Flight.yaw_ctrl_mode=ROTATE;
					Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];					
				}
				break;
				case 2://水平光流定点模式
				{
					OpticalFlow_Control_Pure(0);//普通光流模式		
					Flight.yaw_ctrl_mode=ROTATE;
					Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];				
				}
				break;				
				case 3://SLAM水平定点模式
				{
					slam_control_poshold(&VIO_SINS);			
					Flight.yaw_ctrl_mode=ROTATE;
					Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];				
				}
				break;
				default:
				{
					Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
					Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];	
					Flight.yaw_ctrl_mode=ROTATE;
					Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];						
				}					
			}
			Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制		
		}
	}
}




/************姿态环控制器：角度+角速度****************/
void Attitude_Control(void)
{ 
	Angle_Control_Target(&Flight);//角度控制	
  Gyro_Control();//角速度控制
}

/***************************************************
函数名: void Total_Control(void)
说明:	总控制器运行，大体分三步：
1、根据遥控器输入、当前状态，给定运行模式（自稳+油门手动、定高+自稳、定高+定点（控速）等）
2、主导上层控制器给定姿态期望，高度控制等
3、自稳（姿态）控制
入口:	无
出口:	无
备注:	上电初始化，运行一次
****************************************************/
void Total_Control(void)
{
	static uint16_t main_ctrl_cnt=0,att_ctrl_cnt=0;
  /*************主导控制器******************/
	main_ctrl_cnt++;
	if(main_ctrl_cnt>=1)
	{
		Main_Leading_Control();
		landon_earth_check();//着陆条件自检
		main_ctrl_cnt=0;
	}		
  /*************姿态环控制器*****************/
	att_ctrl_cnt++;
	if(att_ctrl_cnt>=1)
	{
		Attitude_Control();
		att_ctrl_cnt=0;
	}		
}


void CarryPilot_Control(void)
{
  static uint8_t ctrl_cnt=0;
  ctrl_cnt++;  
  /*************控制器模式选择******************/
  if(ctrl_cnt>=4)//改此计数器值，可以调整控制周期
  {  
    Controler_Mode_Select();
    ctrl_cnt=0;
  }
  Total_Control();//总控制器：水平位置+水平速度+姿态（角度+角速度）控制器，高度位置+高度速度+高度加速度控制器	
  Control_Output();//控制量总输出
}




//特别提醒
//倾覆强制停机检测只适合在各项控制参数合适
//飞行器已经能保持自身姿态稳定飞行的条件下使用
//出于用户安全考虑，设计此安全程序
//方便用户处理在调试无人机飞行任务过程中发生炸鸡、失控等紧急情形时，慌不择路上锁不及时的情形
//开启倾覆检测后，发生上述情形时飞行器会主动上锁
//但用户并不能把所有上锁期望都寄托于此倾覆检测，任何时候都不要丢掉遥控器主动控制权
#define overturn_crash_enable 1//倾覆检测使能
#define overturn_crash_time 100//最大倾覆检测时间*5ms，推荐取值区间100~400
void overturn_check(void)//倾覆强制停机检测，放在5ms执行一次任务中
{
#if	overturn_crash_enable
	static uint32_t crash_cnt=0;
	//温度就位且陀螺仪已标定
	if(Gyro_Safety_Calibration_Flag==0)   return;
	if(crash_cnt==overturn_crash_time+1)  return;

	if(crash_cnt<overturn_crash_time)//持续若干时间满足时,强制上锁飞机
	{
		if(ABS(WP_AHRS.Pitch)>40||ABS(WP_AHRS.Roll)>40)//倾斜超过40度
			crash_cnt++;
		else crash_cnt/=2;	
	}
	else 
	{
		crash_cnt++;
		Controler_State=Lock_Controler;//强制上锁
		//RGB灯闪烁报警
		Bling_Set(&rgb_red,3000,1000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_1,1);//蓝色
		Bling_Set(&rgb_blue,3000,1000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_2,1);//蓝色
		Bling_Set(&rgb_green,3000,1000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_3,1);//绿色	
	}
#endif
}


//#define Thr_Feedforward_Compensation//锂电池电压前馈补偿使能
#ifdef Thr_Feedforward_Compensation
	#define Battery_Cell_Voltage    4.2//1S锂电池充满4.2V
	#define Battery_Cell_Numeber    4//4S锂电池
	#define Battery_Full_Voltage    (Battery_Cell_Voltage*Battery_Cell_Numeber)
#endif

uint16_t Throttle_Output=0;
void Throttle_Angle_Compensate()//油门倾角补偿
{
  float CosPitch_CosRoll=ABS(WP_AHRS.cos_rpy[_PIT]*WP_AHRS.cos_rpy[_ROL]);
  float Throttle_Makeup=0;
  float Temp=0;
  if(CosPitch_CosRoll>=0.999999f)  CosPitch_CosRoll=0.999999f;
  if(CosPitch_CosRoll<=0.000001f)  CosPitch_CosRoll=0.000001f;
  if(CosPitch_CosRoll<=0.50f)  CosPitch_CosRoll=0.50f;//Pitch,Roll约等于30度
  if(Throttle>=Thr_Start)//大于起转油门量
  {
    Temp=(uint16_t)(MAX(ABS(100*WP_AHRS.Pitch),ABS(100*WP_AHRS.Roll)));
    Temp=constrain_float(9000-Temp,0,3000)/(3000*CosPitch_CosRoll);
    Throttle_Makeup=(Throttle-Thr_Start)*Temp;//油门倾角补偿
    Throttle_Output=(uint16_t)(Thr_Start+Throttle_Makeup);
    Throttle_Output=(uint16_t)(constrain_float(Throttle_Output,Thr_Start,2000));
#ifdef Thr_Feedforward_Compensation 
		Throttle_Output+=(Throttle_Output-Thr_Start)*(Battery_Full_Voltage/Battery_Voltage-1.0f);
#endif
  }
  else Throttle_Output=Throttle;	
}


/**************************************************************
***************************************************************
X型安装方式，电机序号与姿态角关系
                   -
                 Pitch
					3#             1#
					   *          *
-   Roll          *         Roll   +
						 *          *
					2#             4#
				         Pitch
								   +
加速度传感器轴向与载体X、Y、Z同轴，沿轴向原点看，逆时针旋转角度为+
Y Aixs
*
*
*
*
*
*
* * * * * * * *   X Axis
(O)
*******************************************************************
******************************************************************/
uint16_t Idel_Cnt=0;
#define Idel_Transition_Gap 4//怠速递增间隔时间 2*5=10ms
#define Idel_Transition_Period (Thr_Idle-Thr_Min)//怠速启动最大计数器  10ms*100=1s
uint16_t Thr_Idle_Transition_Cnt=0;
void Control_Output()
{
	overturn_check();//倾覆检测
  Throttle_Angle_Compensate();//油门倾角补偿
  if(Controler_State==Unlock_Controler)//解锁
  {
    if(Landon_Earth_Flag==1)//检测到着陆条件
    {
      if(_M_PWM_1<=Thr_Min&&_M_PWM_2<=Thr_Min&&_M_PWM_3<=Thr_Min&&_M_PWM_4<=Thr_Min)//只有上锁后再解锁时才会满足
      {
        //如果上次油门输出值为最低位，进入怠速时，安排过渡过程
        Thr_Idle_Transition_Cnt=Idel_Transition_Period;
      }
      else//其他时刻进入着陆条件
      {
        if(Last_Landon_Earth_Flag==0)//上次为起飞状态，本次为着陆状态，上锁电机
        {
          Controler_State=Lock_Controler;
          Bling_Set(&rgb_green,3000,200,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_3,0);
        }
      }
      
      Idel_Cnt++;
      if(Idel_Cnt>=Idel_Transition_Gap)
      {
        if(Thr_Idle_Transition_Cnt>=1)
          Thr_Idle_Transition_Cnt--;
        Idel_Cnt=0;
      }
      M_PWM_1=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;//油门怠速
      M_PWM_2=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;
      M_PWM_3=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;
      M_PWM_4=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;
      Take_Off_Reset();//清积分
      //OpticalFlow_SINS_Reset();
      OpticalFlow_Ctrl_Reset();
    }
    else  //解锁后不满足着陆条件，默认起飞
    {
			switch(Controler_High_Mode)
			{
				case 2://定高模式
			  {
					if(Auto_Relock_Flag_Set==1)//解锁后在偏航回中之后
					{
						M_PWM_1=Int_Sort(Moter1_Thr_Scale  *Throttle_Output
																+Moter1_Roll_Scale *Total_Controller.Roll_Gyro_Control.Control_OutPut
																+Moter1_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
																+Moter1_Yaw_Scale  *Total_Controller.Yaw_Gyro_Control.Control_OutPut);
						M_PWM_2=Int_Sort(Moter2_Thr_Scale  *Throttle_Output
																+Moter2_Roll_Scale *Total_Controller.Roll_Gyro_Control.Control_OutPut
																+Moter2_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
																+Moter2_Yaw_Scale  *Total_Controller.Yaw_Gyro_Control.Control_OutPut);
						M_PWM_3=Int_Sort(Moter3_Thr_Scale  *Throttle_Output
																+Moter3_Roll_Scale *Total_Controller.Roll_Gyro_Control.Control_OutPut
																+Moter3_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
																+Moter3_Yaw_Scale  *Total_Controller.Yaw_Gyro_Control.Control_OutPut);
						M_PWM_4=Int_Sort(Moter4_Thr_Scale  *Throttle_Output
																+Moter4_Roll_Scale *Total_Controller.Roll_Gyro_Control.Control_OutPut
																+Moter4_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
																+Moter4_Yaw_Scale  *Total_Controller.Yaw_Gyro_Control.Control_OutPut);
					}
					else//解锁后在偏航回中之前
					{
						M_PWM_1=(int16_t)(Throttle_Output);	
						M_PWM_2=(int16_t)(Throttle_Output);	
						M_PWM_3=(int16_t)(Throttle_Output);	
						M_PWM_4=(int16_t)(Throttle_Output);	
					}	
					M_PWM_1=Value_Limit(Thr_Idle,2500,M_PWM_1);//总输出限幅
					M_PWM_2=Value_Limit(Thr_Idle,2500,M_PWM_2);
					M_PWM_3=Value_Limit(Thr_Idle,2500,M_PWM_3);
					M_PWM_4=Value_Limit(Thr_Idle,2500,M_PWM_4);
				}
				break;
				default://姿态模式
				{
					if(Throttle>=Thr_Fly_Start)//大于起飞油门
					{
							M_PWM_1=Int_Sort(Moter1_Thr_Scale  *Throttle_Output
																	+Moter1_Roll_Scale *Total_Controller.Roll_Gyro_Control.Control_OutPut
																	+Moter1_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
																	+Moter1_Yaw_Scale  *Total_Controller.Yaw_Gyro_Control.Control_OutPut);
							M_PWM_2=Int_Sort(Moter2_Thr_Scale  *Throttle_Output
																	+Moter2_Roll_Scale *Total_Controller.Roll_Gyro_Control.Control_OutPut
																	+Moter2_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
																	+Moter2_Yaw_Scale  *Total_Controller.Yaw_Gyro_Control.Control_OutPut);
							M_PWM_3=Int_Sort(Moter3_Thr_Scale  *Throttle_Output
																	+Moter3_Roll_Scale *Total_Controller.Roll_Gyro_Control.Control_OutPut
																	+Moter3_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
																	+Moter3_Yaw_Scale  *Total_Controller.Yaw_Gyro_Control.Control_OutPut);
							M_PWM_4=Int_Sort(Moter4_Thr_Scale  *Throttle_Output
																	+Moter4_Roll_Scale *Total_Controller.Roll_Gyro_Control.Control_OutPut
																	+Moter4_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
																	+Moter4_Yaw_Scale  *Total_Controller.Yaw_Gyro_Control.Control_OutPut);
							
					}
					else//小于起飞油门
					{
						M_PWM_1=Int_Sort(Throttle_Output);
						M_PWM_2=Int_Sort(Throttle_Output);
						M_PWM_3=Int_Sort(Throttle_Output);
						M_PWM_4=Int_Sort(Throttle_Output);
						Take_Off_Reset();//清积分
					}
					M_PWM_1=Value_Limit(Thr_Idle,2500,M_PWM_1);//总输出限幅
					M_PWM_2=Value_Limit(Thr_Idle,2500,M_PWM_2);
					M_PWM_3=Value_Limit(Thr_Idle,2500,M_PWM_3);
					M_PWM_4=Value_Limit(Thr_Idle,2500,M_PWM_4);		
					Throttle_Control_Reset();				
				}
				break;				
			}
		}
	}	
  else//未解锁，油门置于最低位，停转
  {
    M_PWM_1=Thr_Min;
    M_PWM_2=Thr_Min;
    M_PWM_3=Thr_Min;
    M_PWM_4=Thr_Min;
    Take_Off_Reset();//清积分
		//OpticalFlow_SINS_Reset();
		OpticalFlow_Ctrl_Reset();
//    Throttle_Control_Reset();
	}
  _M_PWM_1=M_PWM_1;
  _M_PWM_2=M_PWM_2;
  _M_PWM_3=M_PWM_3;
  _M_PWM_4=M_PWM_4;  
  M_PWM_1=Value_Limit(0,2200,M_PWM_1);//总输出限幅
  M_PWM_2=Value_Limit(0,2200,M_PWM_2);
  M_PWM_3=Value_Limit(0,2200,M_PWM_3);
  M_PWM_4=Value_Limit(0,2200,M_PWM_4);
	PWM_Output(M_PWM_1,M_PWM_2,M_PWM_3,M_PWM_4);
}









/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/



