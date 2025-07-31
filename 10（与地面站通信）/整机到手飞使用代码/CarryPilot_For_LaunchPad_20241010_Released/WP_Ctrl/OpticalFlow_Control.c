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
#include "OpticalFlow_Control.h"


extern Vector2f OpticalFlow_Position;
extern Vector2f OpticalFlow_Speed;
extern Vector2f OpticalFlow_Speed_Err;
extern Vector2f OpticalFlow_Position_Err;

uint8_t  OpticalFlow_Pos_Ctrl_Recode=1;
Vector2f OpticalFlow_Pos_Ctrl_Expect={0};
Vector2f OpticalFlow_Pos_Ctrl_Err={0};
Vector2f OpticalFlow_Pos_Ctrl_Integrate={0};
Vector2f OpticalFlow_Pos_Ctrl_Output={0};

Vector2f OpticalFlow_Vel_Ctrl_Expect={0};
Vector2f OpticalFlow_Vel_Ctrl_Err={0};
Vector2f OpticalFlow_Vel_Ctrl_Integrate={0};
Vector2f OpticalFlow_Vel_Ctrl_Output={0};

void OpticalFlow_SINS_Reset(void)
{ 
  OpticalFlow_SINS.Position[_EAST]=0;
  OpticalFlow_SINS.Speed[_EAST]=0;
  OpticalFlow_SINS.Position[_NORTH]=0;
  OpticalFlow_SINS.Speed[_NORTH]=0;
  
	OpticalFlow_Position.x=0;
	OpticalFlow_Position.y=0;

	VIO_SINS.Position[_EAST] = current_state.position_x;
	VIO_SINS.Position[_NORTH]= current_state.position_y;
	
	VIO_SINS.Speed[_EAST]=0;
	VIO_SINS.Speed[_NORTH]=0;
	
	correct[0].acc=0;
	correct[1].acc=0;
	correct[0].vel=0;
	correct[1].vel=0;
	correct[0].pos=0;
	correct[1].pos=0;
}


void SDK_Pos_Ctrl_Reset(void)
{ 
	Total_Controller.SDK_Pitch_Position_Control.Integrate=0;
	Total_Controller.SDK_Pitch_Position_Control.Last_Err=0;
	Total_Controller.SDK_Pitch_Position_Control.Err=0;
	Total_Controller.SDK_Pitch_Position_Control.Last_Err_LPF=0;		

	Total_Controller.SDK_Roll_Position_Control.Integrate=0;
	Total_Controller.SDK_Roll_Position_Control.Last_Err=0;
	Total_Controller.SDK_Roll_Position_Control.Err=0;
	Total_Controller.SDK_Roll_Position_Control.Last_Err_LPF=0;
	
	PID_LPF_Reset(&Total_Controller.SDK_Roll_Position_Control,SDK_Roll_Position_Controler);
	PID_LPF_Reset(&Total_Controller.SDK_Pitch_Position_Control,SDK_Pitch_Position_Controler);
}

void OpticalFlow_Ctrl_Reset(void)
{ 
  OpticalFlow_Vel_Ctrl_Integrate.x=0.0f;
  OpticalFlow_Vel_Ctrl_Integrate.y=0.0f;
  OpticalFlow_Pos_Ctrl_Integrate.x=0.0f;
  OpticalFlow_Pos_Ctrl_Integrate.y=0.0f;
  OpticalFlow_Pos_Ctrl_Expect.x=0;
  OpticalFlow_Pos_Ctrl_Expect.y=0;
	OpticalFlow_Pos_Ctrl_Recode=1;
	OpticalFlow_Vel_Ctrl_Expect.x= 0;//速度期望
	OpticalFlow_Vel_Ctrl_Expect.y= 0;
	
	SDK_Pos_Ctrl_Reset();
}




float OpticalFlow_Expect_Speed_Mapping(float input,uint16_t input_max,float output_max)
{
  float output_speed=0;
  float temp_scale=(float)(input/input_max);
  temp_scale=constrain_float(temp_scale,-1.0f, 1.0f);
  if(temp_scale>=0) output_speed=(float)(output_max*temp_scale*temp_scale);
  else output_speed=(float)(-output_max*temp_scale*temp_scale); 
  return output_speed;
}

void OpticalFlow_Set_Target_Point(Vector2f target)
{
  OpticalFlow_Pos_Ctrl_Expect.x=target.x;
  OpticalFlow_Pos_Ctrl_Expect.y=target.y;
}

void OpticalFlow_Set_Target_Vel(Vector2f target)
{
  OpticalFlow_Vel_Ctrl_Expect.x=target.x;
  OpticalFlow_Vel_Ctrl_Expect.y=target.y;
}

//////////////////////////////////////////////////////////////////////////


bool OPT_Is_Fix(void)
{
 if(Sensor_Flag.Ground_Health==1
	 &&(opt_data.valid==1||current_state.valid==1))
 {
	 return true;
 }
 else 
 {
	 return false;
 }
}



void OpticalFlow_Pos_Control(void)
{
	if(OPT_Is_Fix()==false)//OPT定位状态未锁定，姿态期望直接来源于遥控器给定
	{
		Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
		Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];	
		return ;
	}
	
  static uint16_t OpticalFlow_Pos_Ctrl_Cnt=0;
  OpticalFlow_Pos_Ctrl_Cnt++;
  if(OpticalFlow_Pos_Ctrl_Cnt>=4)//20ms控制一次速度，避免输入频率过大，系统响应不过来
  {
    //计算位置偏差  
    OpticalFlow_Pos_Ctrl_Err.x=OpticalFlow_Pos_Ctrl_Expect.x-OpticalFlow_SINS.Position[_EAST];
    OpticalFlow_Pos_Ctrl_Err.y=OpticalFlow_Pos_Ctrl_Expect.y-OpticalFlow_SINS.Position[_NORTH];
		
    OpticalFlow_Pos_Ctrl_Err.x=constrain_float(OpticalFlow_Pos_Ctrl_Err.x,-Total_Controller.Optical_Position_Control.Err_Max,Total_Controller.Optical_Position_Control.Err_Max);//100
    OpticalFlow_Pos_Ctrl_Err.y=constrain_float(OpticalFlow_Pos_Ctrl_Err.y,-Total_Controller.Optical_Position_Control.Err_Max,Total_Controller.Optical_Position_Control.Err_Max);		
		
    //计算位置控制输出
    OpticalFlow_Pos_Ctrl_Output.x=Total_Controller.Optical_Position_Control.Kp*OpticalFlow_Pos_Ctrl_Err.x;
    OpticalFlow_Pos_Ctrl_Output.y=Total_Controller.Optical_Position_Control.Kp*OpticalFlow_Pos_Ctrl_Err.y;
    OpticalFlow_Pos_Ctrl_Cnt=0;
  }
}

void OpticalFlow_Pos_Control_VIO(void)
{
	if(OPT_Is_Fix()==false)//OPT定位状态未锁定，姿态期望直接来源于遥控器给定
	{
		Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
		Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];	
		return ;
	}
	
  static uint16_t OpticalFlow_Pos_Ctrl_Cnt=0;
  OpticalFlow_Pos_Ctrl_Cnt++;
  if(OpticalFlow_Pos_Ctrl_Cnt>=4)//20ms控制一次速度，避免输入频率过大，系统响应不过来
  {
    //计算位置偏差  
    OpticalFlow_Pos_Ctrl_Err.x=OpticalFlow_Pos_Ctrl_Expect.x-VIO_SINS.Position[_EAST];
    OpticalFlow_Pos_Ctrl_Err.y=OpticalFlow_Pos_Ctrl_Expect.y-VIO_SINS.Position[_NORTH];
		
    OpticalFlow_Pos_Ctrl_Err.x=constrain_float(OpticalFlow_Pos_Ctrl_Err.x,-Total_Controller.Optical_Position_Control.Err_Max,Total_Controller.Optical_Position_Control.Err_Max);//100
    OpticalFlow_Pos_Ctrl_Err.y=constrain_float(OpticalFlow_Pos_Ctrl_Err.y,-Total_Controller.Optical_Position_Control.Err_Max,Total_Controller.Optical_Position_Control.Err_Max);		

		//导航坐标系下机体Pitch、Roll方向上位置偏差
		from_vio_to_body_frame(Total_Controller.Optical_Position_Control.Kp*OpticalFlow_Pos_Ctrl_Err.x,
													 Total_Controller.Optical_Position_Control.Kp*OpticalFlow_Pos_Ctrl_Err.y,
													 &OpticalFlow_Pos_Ctrl_Output.x,
													 &OpticalFlow_Pos_Ctrl_Output.y,
													 WP_AHRS.Yaw);
    OpticalFlow_Pos_Ctrl_Cnt=0;
  }
}


systime opt_speed_dt;
void OpticalFlow_Vel_Control(Vector2f target)
{
	Get_Systime(&opt_speed_dt);
	//如果本函数运行周期大于控制周期10倍
	if(0.001f*opt_speed_dt.period>=20*WP_Duty_Dt)
	{
		//情况1:初次从其它模式切入本模式
		//情况2:系统调度超时，在系统设计合理情况下，本情况不可能发生
	}
	
	
	if(OPT_Is_Fix()==false)//OPT定位状态未锁定，姿态期望直接来源于遥控器给定
	{
		Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
		Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];	
		return ;
	}	
	
  static uint16_t OpticalFlow_Vel_Ctrl_Cnt=0;
  OpticalFlow_Vel_Ctrl_Expect.x= target.x;//速度期望
  OpticalFlow_Vel_Ctrl_Expect.y= target.y;
  OpticalFlow_Vel_Ctrl_Cnt++;
  if(OpticalFlow_Vel_Ctrl_Cnt>=2)//10ms控制一次速度，避免输入频率过大，系统响应不过来
  {
    OpticalFlow_Vel_Ctrl_Err.x=constrain_float(OpticalFlow_Vel_Ctrl_Expect.x-OpticalFlow_SINS.Speed[_EAST],-Total_Controller.Optical_Speed_Control.Err_Max,Total_Controller.Optical_Speed_Control.Err_Max);//30
    OpticalFlow_Vel_Ctrl_Err.y=constrain_float(OpticalFlow_Vel_Ctrl_Expect.y-OpticalFlow_SINS.Speed[_NORTH] ,-Total_Controller.Optical_Speed_Control.Err_Max,Total_Controller.Optical_Speed_Control.Err_Max);
    
    //if(ABS(OpticalFlow_Vel_Ctrl_Err.x)<=Total_Controller.Optical_Speed_Control.Integrate_Separation_Err)  
    OpticalFlow_Vel_Ctrl_Integrate.x+=Total_Controller.Optical_Speed_Control.Ki*OpticalFlow_Vel_Ctrl_Err.x;//0.1  15
    //if(ABS(OpticalFlow_Vel_Ctrl_Err.y)<=Total_Controller.Optical_Speed_Control.Integrate_Separation_Err)  
    OpticalFlow_Vel_Ctrl_Integrate.y+=Total_Controller.Optical_Speed_Control.Ki*OpticalFlow_Vel_Ctrl_Err.y;
    
    OpticalFlow_Vel_Ctrl_Integrate.x=constrain_float(OpticalFlow_Vel_Ctrl_Integrate.x,-Total_Controller.Optical_Speed_Control.Integrate_Max,Total_Controller.Optical_Speed_Control.Integrate_Max);
    OpticalFlow_Vel_Ctrl_Integrate.y=constrain_float(OpticalFlow_Vel_Ctrl_Integrate.y,-Total_Controller.Optical_Speed_Control.Integrate_Max,Total_Controller.Optical_Speed_Control.Integrate_Max);
    
    OpticalFlow_Vel_Ctrl_Output.x=OpticalFlow_Vel_Ctrl_Integrate.x+Total_Controller.Optical_Speed_Control.Kp*OpticalFlow_Vel_Ctrl_Err.x;//4.5
    OpticalFlow_Vel_Ctrl_Output.y=OpticalFlow_Vel_Ctrl_Integrate.y+Total_Controller.Optical_Speed_Control.Kp*OpticalFlow_Vel_Ctrl_Err.y;
    
    accel_target.y=-constrain_float(OpticalFlow_Vel_Ctrl_Output.y,-Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);//450
    accel_target.x=-constrain_float(OpticalFlow_Vel_Ctrl_Output.x,-Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);//期望运动加速度
    
		ENU_Desired_Accel_Transform_Angle(accel_target,&angle_target);//期望运动加速度转期望姿态倾角
		
		Flight.roll_outer_control_output =angle_target.x;
		Flight.pitch_outer_control_output=angle_target.y;
				
    OpticalFlow_Vel_Ctrl_Cnt=0;
  }
}


void Horizontal_Control(uint8_t force_brake_flag)
{
	/////////////////////////////////SDK位置控制开始////////////////////////////////////////						
	if(ngs_sdk.update_flag==true)
	{
			if(ngs_sdk.move_flag.sdk_front_flag==true||ngs_sdk.move_flag.sdk_behind_flag==true)//前进/后退
			{
					OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_EAST];										  //左右保持
					OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_NORTH]+ngs_sdk.f_distance;	//前后调整		
					Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP];	 				  //上下保持																
			}
			
			if(ngs_sdk.move_flag.sdk_left_flag==true||ngs_sdk.move_flag.sdk_right_flag==true)//向左/向右
			{
				  OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_EAST]+ngs_sdk.f_distance;   //左右调整
					OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_NORTH];										 //前后保持			
					Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP];			 	     //上下保持														
			}
			
			if(ngs_sdk.move_flag.sdk_up_flag==true||ngs_sdk.move_flag.sdk_down_flag==true)//上升/下降
			{
					OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_EAST];															//前后保持
					OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_NORTH];																//左右保持
				  Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP]+ngs_sdk.f_distance; //上下调整										
			}
			ngs_sdk.move_flag.sdk_front_flag=false;
			ngs_sdk.move_flag.sdk_behind_flag=false;
			ngs_sdk.move_flag.sdk_left_flag=false;
			ngs_sdk.move_flag.sdk_right_flag=false;
			ngs_sdk.move_flag.sdk_up_flag=false;
			ngs_sdk.move_flag.sdk_down_flag=false;							
			ngs_sdk.update_flag=false;
	}

	
	if(Roll_Control==0&&Pitch_Control==0)//无水平遥控量给定
	{ 
		/**************************光流位置控制器************************************/
		if(OpticalFlow_Pos_Ctrl_Recode==1||force_brake_flag==1)
		{
			OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_EAST];
			OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_NORTH];
			OpticalFlow_Pos_Ctrl_Recode=0;
		}	
		OpticalFlow_Pos_Control();
		/***********当只需要速度控制时，开启以下注释，仅限调试时用*************/
		 //OpticalFlow_Pos_Ctrl_Output.x=0;
		 //OpticalFlow_Pos_Ctrl_Output.y=0;
	}
	else
	{
		OpticalFlow_Pos_Ctrl_Output.x=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_ROLL],Pit_Rol_Max,OpticalFlow_Speed_Control_Max);
		OpticalFlow_Pos_Ctrl_Output.y=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_PITCH],Pit_Rol_Max,OpticalFlow_Speed_Control_Max); 
		OpticalFlow_Pos_Ctrl_Expect.x=0;
		OpticalFlow_Pos_Ctrl_Expect.y=0;
		OpticalFlow_Pos_Ctrl_Recode=1;
	}
	/**************************基于模型的加速度-姿态角映射，相比直接给姿态，参数差异大概在20倍左右************************************/
	OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度期望
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Horizontal_Control_VIO_Pure(uint8_t force_brake_flag)
{
	if(Roll_Control==0
		 &&Pitch_Control==0)//无水平遥控量给定
	{ 
		/**************************光流位置控制器************************************/
		if(OpticalFlow_Pos_Ctrl_Recode==1||force_brake_flag==1)
		{
			OpticalFlow_Pos_Ctrl_Expect.x=VIO_SINS.Position[_EAST];
			OpticalFlow_Pos_Ctrl_Expect.y=VIO_SINS.Position[_NORTH];
			OpticalFlow_Pos_Ctrl_Recode=0;
		}
		OpticalFlow_Pos_Control_VIO();
	}
	else
	{
		OpticalFlow_Pos_Ctrl_Output.x=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_ROLL],Pit_Rol_Max,OpticalFlow_Speed_Control_Max);
		OpticalFlow_Pos_Ctrl_Output.y=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_PITCH],Pit_Rol_Max,OpticalFlow_Speed_Control_Max); 				
		OpticalFlow_Pos_Ctrl_Expect.x=0;
		OpticalFlow_Pos_Ctrl_Expect.y=0;
		OpticalFlow_Pos_Ctrl_Recode=1;
	}
	/**************************基于模型的加速度-姿态角映射，相比直接给姿态，参数差异大概在20倍左右************************************/
	OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度期望
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ngs_nav_ctrl_finish_predict(void)
{
	//判断是否到达目标航点位置
	if(ngs_nav_ctrl.ctrl_finish_flag==0)
	{
		if(ngs_nav_ctrl.cnt<10)//持续50ms满足
		{
			ngs_nav_ctrl.dis_cm=pythagorous3(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y,Total_Controller.Height_Position_Control.Err);
			if(ngs_nav_ctrl.dis_cm<=ngs_nav_ctrl.dis_limit_cm)	ngs_nav_ctrl.cnt++;
			else ngs_nav_ctrl.cnt/=2;
		}
		else
		{
			ngs_nav_ctrl.ctrl_finish_flag=1;
			ngs_nav_ctrl.cnt=0;
			NCLink_Send_Check_Flag[14]=1;//控制完毕后，返回应答给上位机
			send_check_back=3;//控制完毕后，返回应答给ROS
		}
	}
}

void Horizontal_Control_VIO(uint8_t force_brake_flag)
{
	/////////////////////////////////SDK位置控制开始////////////////////////////////////////						
	if(ngs_sdk.update_flag==true)
	{
		if(ngs_sdk.move_flag.sdk_front_flag==true||ngs_sdk.move_flag.sdk_behind_flag==true)//前进/后退
		{
				OpticalFlow_Pos_Ctrl_Expect.x=VIO_SINS.Position[_EAST];										  //左右保持
				OpticalFlow_Pos_Ctrl_Expect.y=VIO_SINS.Position[_NORTH]+ngs_sdk.f_distance;	//前后调整		
				Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP];	 				  //上下保持																
		}
		
		if(ngs_sdk.move_flag.sdk_left_flag==true||ngs_sdk.move_flag.sdk_right_flag==true)//向左/向右
		{
				OpticalFlow_Pos_Ctrl_Expect.x=VIO_SINS.Position[_EAST]-ngs_sdk.f_distance;   //左右调整
				OpticalFlow_Pos_Ctrl_Expect.y=VIO_SINS.Position[_NORTH];										 //前后保持			
				Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP];			 	     //上下保持														
		}
		
		if(ngs_sdk.move_flag.sdk_up_flag==true||ngs_sdk.move_flag.sdk_down_flag==true)//上升/下降
		{
				OpticalFlow_Pos_Ctrl_Expect.x=VIO_SINS.Position[_EAST];															//前后保持
				OpticalFlow_Pos_Ctrl_Expect.y=VIO_SINS.Position[_NORTH];																//左右保持
				Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP]+ngs_sdk.f_distance; //上下调整										
		}
		ngs_sdk.move_flag.sdk_front_flag=false;
		ngs_sdk.move_flag.sdk_behind_flag=false;
		ngs_sdk.move_flag.sdk_left_flag=false;
		ngs_sdk.move_flag.sdk_right_flag=false;
		ngs_sdk.move_flag.sdk_up_flag=false;
		ngs_sdk.move_flag.sdk_down_flag=false;							
		ngs_sdk.update_flag=false;
	}
	
	if(ngs_nav_ctrl.update_flag==1)//地面站/ROS发送串口控制指令
	{
		Horizontal_Navigation(ngs_nav_ctrl.x,
												  ngs_nav_ctrl.y,
											    ngs_nav_ctrl.z,
													ngs_nav_ctrl.nav_mode,
													ngs_nav_ctrl.frame_id);
		ngs_nav_ctrl.update_flag=0;
		ngs_nav_ctrl.ctrl_finish_flag=0;
		ngs_nav_ctrl.cnt=0;
	}
	
	switch(ngs_nav_ctrl.nav_mode)
	{
		case RELATIVE_MODE:
		case GLOBAL_MODE://导航位置控制
		{
			Horizontal_Control_VIO_Pure(force_brake_flag);
			ngs_nav_ctrl_finish_predict();//导航位置控制结束判定	
		}
		break;
		case CMD_VEL_MODE://速度控制模式
		{
			if(ngs_nav_ctrl.x!=0||ngs_nav_ctrl.y!=0)//当水平速度不为0时，速度控制才有效
			{	
					Vector2f target;		
					target.x=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_ROLL],Pit_Rol_Max,OpticalFlow_Speed_Control_Max);
					target.y=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_PITCH],Pit_Rol_Max,OpticalFlow_Speed_Control_Max);
					if(Roll_Control==0&&Pitch_Control==0)//无水平遥控量给定
					{
						if(ngs_nav_ctrl.cmd_vel_update==1)//速度控制执行时间控制
						{
							if(ngs_nav_ctrl.cmd_vel_during_cnt>0)
							{
								ngs_nav_ctrl.cmd_vel_during_cnt=ngs_nav_ctrl.cmd_vel_during_cnt-1;
								Vector2f target_tmp;
								target_tmp.x=ngs_nav_ctrl.cmd_vel_x;
								target_tmp.y=ngs_nav_ctrl.cmd_vel_y;
								OpticalFlow_Vel_Control(target_tmp);
								ngs_nav_ctrl.cmd_vel_suspend_flag=1;
							}
							else	
							{
								ngs_nav_ctrl.cmd_vel_update=0;//速度控制完毕
								ngs_nav_ctrl.nav_mode=TRANSITION_MODE;//控制完毕后，进入过渡模式
								ngs_nav_ctrl.cmd_vel_suspend_flag=0;//正常中止
								NCLink_Send_Check_Flag[14]=1;//控制完毕后，返回应答给上位机
								send_check_back=3;//控制完毕后，返回应答给ROS
							}						
						}
						else OpticalFlow_Vel_Control(target);
					}
					else 
					{
						OpticalFlow_Vel_Control(target);
						//存在手动打杆操作，同样视作任务终止
						ngs_nav_ctrl.cmd_vel_update=0;//只要存在遥控器打杆操作，强制结束本次速度控制
						ngs_nav_ctrl.nav_mode=TRANSITION_MODE;//控制完毕后，进入过渡模式
						ngs_nav_ctrl.cmd_vel_during_cnt=0;
						ngs_nav_ctrl.cmd_vel_suspend_flag=0;//视同为正常中止
					}
			}
			else//否则水平仍然采用位置控制，只响应偏航控制
			{
					if(ngs_nav_ctrl.cmd_vel_update==1)//速度控制执行时间控制
					{
						if(ngs_nav_ctrl.cmd_vel_during_cnt>0)
						{
							ngs_nav_ctrl.cmd_vel_during_cnt=ngs_nav_ctrl.cmd_vel_during_cnt-1;
							//程序在此空转，实际不执行具体任务
							//只在外部控制中，响应偏航角速度控制
							//待计数器归0后，退出速度控制模式
						}
						else	
						{
							ngs_nav_ctrl.cmd_vel_update=0;//速度控制完毕
							ngs_nav_ctrl.nav_mode=TRANSITION_MODE+1;//控制完毕后，进入过渡的下一模式，本任务全程没有刷新水平悬停点
							NCLink_Send_Check_Flag[14]=1;//控制完毕后，返回应答给上位机
							send_check_back=3;//控制完毕后，返回应答给ROS
							ngs_nav_ctrl.cmd_vel_suspend_flag=0;//正常中止
						}						
					}
					
					if(ngs_nav_ctrl.cmd_vel_suspend_flag==1)//因后续指令被覆盖，提前终止上条指令的速度控制时，再次进入后会首选刷新悬停点
					{
						Horizontal_Control_VIO_Pure(1);
						ngs_nav_ctrl.cmd_vel_suspend_flag=0;
					}
					else Horizontal_Control_VIO_Pure(force_brake_flag);
					
					//存在手动打杆操作，同样视作任务终止
					if(Roll_Control!=0||Pitch_Control!=0)//存在遥控量给定
					{
							ngs_nav_ctrl.cmd_vel_update=0;//速度控制完毕
							ngs_nav_ctrl.nav_mode=TRANSITION_MODE+1;//控制完毕后，进入过渡的下一模式，本任务全程没有刷新水平悬停点
							NCLink_Send_Check_Flag[14]=1;//控制完毕后，返回应答给上位机
							send_check_back=3;//控制完毕后，返回应答给ROS
							ngs_nav_ctrl.cmd_vel_suspend_flag=0;//视同为正常中止					
					}
			}
		}
		break;
		case TRANSITION_MODE://安排过渡模式，只会执行一次，用于刷新悬停位置
		{
			Horizontal_Control_VIO_Pure(1);  //进入过渡模式后，刷新悬停位置期望
			ngs_nav_ctrl.nav_mode++;			   //自加后下次会进入普通光流定点模式
		}
		break;
		default:Horizontal_Control_VIO_Pure(force_brake_flag);//普通光流控制
	}
}




void OpticalFlow_X_Vel_Control(float target_x)//机头左侧为X+
{ 
  static uint16_t OpticalFlow_Vel_Ctrl_Cnt=0;
  OpticalFlow_Vel_Ctrl_Expect.x= target_x;//速度期望
  OpticalFlow_Vel_Ctrl_Cnt++;
  if(OpticalFlow_Vel_Ctrl_Cnt>=2)//10ms控制一次速度，避免输入频率过大，系统响应不过来
  {
    OpticalFlow_Vel_Ctrl_Err.x=constrain_float(OpticalFlow_Vel_Ctrl_Expect.x-OpticalFlow_SINS.Speed[_EAST],-Total_Controller.Optical_Speed_Control.Err_Max,Total_Controller.Optical_Speed_Control.Err_Max);//30

    //if(ABS(OpticalFlow_Vel_Ctrl_Err.x)<=Total_Controller.Optical_Speed_Control.Integrate_Separation_Err)  
    OpticalFlow_Vel_Ctrl_Integrate.x+=Total_Controller.Optical_Speed_Control.Ki*OpticalFlow_Vel_Ctrl_Err.x;//0.1  15

    OpticalFlow_Vel_Ctrl_Integrate.x=constrain_float(OpticalFlow_Vel_Ctrl_Integrate.x,-Total_Controller.Optical_Speed_Control.Integrate_Max,Total_Controller.Optical_Speed_Control.Integrate_Max);

    OpticalFlow_Vel_Ctrl_Output.x=OpticalFlow_Vel_Ctrl_Integrate.x+Total_Controller.Optical_Speed_Control.Kp*OpticalFlow_Vel_Ctrl_Err.x;//4.5

		accel_target.x=-constrain_float(OpticalFlow_Vel_Ctrl_Output.x,-Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);//期望运动加速度
    accel_target.y=0;
		
		ENU_Desired_Accel_Transform_Angle(accel_target,&angle_target);//期望运动加速度转期望姿态倾角
		Flight.roll_outer_control_output=angle_target.x;
    OpticalFlow_Vel_Ctrl_Cnt=0;
  }
}


void OpticalFlow_Y_Vel_Control(float target_y)//机头前侧为Y+
{ 
  static uint16_t OpticalFlow_Vel_Ctrl_Cnt=0;
  OpticalFlow_Vel_Ctrl_Expect.y= target_y;
  OpticalFlow_Vel_Ctrl_Cnt++;
  if(OpticalFlow_Vel_Ctrl_Cnt>=2)//10ms控制一次速度，避免输入频率过大，系统响应不过来
  {
    OpticalFlow_Vel_Ctrl_Err.y=constrain_float(OpticalFlow_Vel_Ctrl_Expect.y-OpticalFlow_SINS.Speed[_NORTH] ,-Total_Controller.Optical_Speed_Control.Err_Max,Total_Controller.Optical_Speed_Control.Err_Max);
    
    OpticalFlow_Vel_Ctrl_Integrate.y+=Total_Controller.Optical_Speed_Control.Ki*OpticalFlow_Vel_Ctrl_Err.y;
    
    OpticalFlow_Vel_Ctrl_Integrate.y=constrain_float(OpticalFlow_Vel_Ctrl_Integrate.y,-Total_Controller.Optical_Speed_Control.Integrate_Max,Total_Controller.Optical_Speed_Control.Integrate_Max);
    
    OpticalFlow_Vel_Ctrl_Output.y=OpticalFlow_Vel_Ctrl_Integrate.y+Total_Controller.Optical_Speed_Control.Kp*OpticalFlow_Vel_Ctrl_Err.y;
    
		accel_target.x=0;
    accel_target.y=-constrain_float(OpticalFlow_Vel_Ctrl_Output.y,-Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);//450
    
		ENU_Desired_Accel_Transform_Angle(accel_target,&angle_target);//期望运动加速度转期望姿态倾角

		Flight.pitch_outer_control_output=angle_target.y;
		
    OpticalFlow_Vel_Ctrl_Cnt=0;
  }
}


///////////////////////////////////////////////
void OpticalFlow_Control(uint8_t force_brake_flag)
{
	if(OPT_Is_Fix()==false)//OPT定位状态未锁定，姿态期望直接来源于遥控器给定
	{
		Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
		Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];	
		return ;
	}

	static uint8_t miss_fixed_flag=0;
	static systime opt_poshold_dt;
	Get_Systime(&opt_poshold_dt);
	//如果本函数运行周期大于控制周期10倍
	if(0.001f*opt_poshold_dt.period>=20*WP_Duty_Dt)
	{
		//情况1:初次从其它模式切入本模式
		//情况2:系统调度超时，在系统设计合理情况下，本情况不可能发生
		miss_fixed_flag=1;
	}
	
	if(miss_fixed_flag==1)//之前未满足定位条件,未锁定目标点
	{
		OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_EAST];
		OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_NORTH];
		//控制器复位
		OpticalFlow_Ctrl_Reset();
		miss_fixed_flag=0;
	}
	
    if(Roll_Control==0
       &&Pitch_Control==0)//无水平遥控量给定
    {
      /**************************光流位置控制器************************************/
      if(OpticalFlow_Pos_Ctrl_Recode==1)
      {
        if(force_brake_flag==1||(WP_AHRS.rMat[8]>=0.95f
                                 &&pythagorous2(OpticalFlow_SINS.Speed[_EAST],OpticalFlow_SINS.Speed[_NORTH])<=40))//打杆回中后，根据当前速度、倾角判断是否进行悬停
        {
          OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_EAST];
          OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_NORTH];
					OpticalFlow_Pos_Ctrl_Recode=0;
        }
        else  //打杆回中未满足悬停条件时，只进行速度控制 
        {
          OpticalFlow_Pos_Ctrl_Output.x=0;
          OpticalFlow_Pos_Ctrl_Output.y=0;
        }
      }
      else  OpticalFlow_Pos_Control();
      
      /***********当只需要速度控制时，开启以下注释，仅限调试时用*************/
      //OpticalFlow_Pos_Ctrl_Output.x=0;
      //OpticalFlow_Pos_Ctrl_Output.y=0;
    }
    else
    {
      OpticalFlow_Pos_Ctrl_Output.x=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_ROLL],Pit_Rol_Max,OpticalFlow_Speed_Control_Max);
      OpticalFlow_Pos_Ctrl_Output.y=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_PITCH],Pit_Rol_Max,OpticalFlow_Speed_Control_Max); 
      OpticalFlow_Pos_Ctrl_Expect.x=0;
      OpticalFlow_Pos_Ctrl_Expect.y=0;
			OpticalFlow_Pos_Ctrl_Recode=1;
    }
		/**************************基于模型的加速度-姿态角映射，相比直接给姿态，参数差异大概在20倍左右************************************/
		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度期望
}



void OpticalFlow_Control_Pure(uint8_t force_brake_flag)
{
  if(OPT_Is_Fix()==false)//OPT定位状态未锁定，姿态期望直接来源于遥控器给定
	{
		Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
		Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];	
		return ;
	}
	
	if(Optical_Type_Present==3||Optical_Type_Present==4)
	{
		switch(current_state.slam_sensor)
		{
			case NO_SLAM:
			{
				Horizontal_Control(force_brake_flag);
			}
			break;
			case LIDAR_2D_SLAM:
			case T265_SLAM:
			case LOAM:
			{
				Horizontal_Control_VIO(force_brake_flag);	
			};
			break;
			default:			
			{
				Horizontal_Control(force_brake_flag);			
			}		
		}
	}
	else
	{
		Horizontal_Control(force_brake_flag);
	}
}



void Color_Block_Control_Pilot(void)
{
	static uint8_t miss_cnt=1;
	static uint16_t _cnt=0;
	static uint8_t miss_flag=0;
	_cnt++;
	if(_cnt>=20)//100ms
	{	
		_cnt=0;		
	  if(Opv_Top_View_Target.target_ctrl_enable==1)//目标点检测跟踪
		{			
			miss_cnt=1;
			miss_flag=0;			
			Opv_Top_View_Target.target_ctrl_enable=0;
			Total_Controller.SDK_Roll_Position_Control.Expect=0;
			Total_Controller.SDK_Roll_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.x;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Roll_Position_Control,Opv_Top_View_Target.trust_flag,0.1f);
			
			Total_Controller.SDK_Pitch_Position_Control.Expect=0;
			Total_Controller.SDK_Pitch_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.y;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Pitch_Position_Control,Opv_Top_View_Target.trust_flag,0.1f);
			
			OpticalFlow_Pos_Ctrl_Output.x=-Total_Controller.SDK_Roll_Position_Control.Control_OutPut;
			OpticalFlow_Pos_Ctrl_Output.y=-Total_Controller.SDK_Pitch_Position_Control.Control_OutPut;	
		}
		else//丢失目标
		{
		  miss_flag=1;
		}		
	}

	
	if(miss_flag==1)//目标丢失
	{
		if(miss_cnt==1)//初始丢失跟踪目标后，锁定当前位置后，进行普通光流控制
		{
			miss_cnt=2;
			OpticalFlow_Control_Pure(1);//20ms		
		}
		else if(miss_cnt==2)//丢失跟踪目标后，进行普通光流控制
		{
			OpticalFlow_Control_Pure(0);//20ms		
		}
	}
  else//目标未丢失,10ms
	{
		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度控制周期20ms
	}			
}


void Top_APrilTag_Control_Pilot(void)
{
	static uint8_t miss_cnt=1;
	static uint16_t _cnt=0;
	static uint8_t miss_flag=0;
	_cnt++;
	if(_cnt>=20)//100ms
	{	
		_cnt=0;		
	  if(Opv_Top_View_Target.target_ctrl_enable==1)//目标点检测跟踪
		{			
			miss_cnt=1;
			miss_flag=0;			
			Opv_Top_View_Target.target_ctrl_enable=0;
			Total_Controller.SDK_Roll_Position_Control.Expect=0;
			Total_Controller.SDK_Roll_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.x;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Roll_Position_Control,Opv_Top_View_Target.trust_flag,0.1f);
			
			Total_Controller.SDK_Pitch_Position_Control.Expect=0;
			Total_Controller.SDK_Pitch_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.y;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Pitch_Position_Control,Opv_Top_View_Target.trust_flag,0.1f);
			
			OpticalFlow_Pos_Ctrl_Output.x=-Total_Controller.SDK_Roll_Position_Control.Control_OutPut;
			OpticalFlow_Pos_Ctrl_Output.y=-Total_Controller.SDK_Pitch_Position_Control.Control_OutPut;	
		}
		else//丢失目标
		{
		  miss_flag=1;
		}		
	}

	
	if(miss_flag==1)//目标丢失
	{
		if(miss_cnt==1)//初始丢失跟踪目标后，锁定当前位置后，进行普通光流控制
		{
			miss_cnt=2;
			OpticalFlow_Control_Pure(1);//20ms		
		}
		else if(miss_cnt==2)//丢失跟踪目标后，进行普通光流控制
		{
			OpticalFlow_Control_Pure(0);//20ms		
		}
	}
  else//目标未丢失,10ms
	{
		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度控制周期20ms
	}			
}


#define Forward_Keep_Distance   100//cm
#define Deadband_Keep_Distance  10
void Front_AprilTag_Control_Pilot(void)
{
	static uint8_t miss_cnt=1;
	static uint16_t _cnt=0;
	static uint8_t miss_flag=0;
	_cnt++;
	if(_cnt>=20)//100ms
	{	
		_cnt=0;		
	  if(Opv_Front_View_Target.target_ctrl_enable==1)//目标点检测跟踪
		{			
			miss_cnt=1;
			miss_flag=0;			
			Opv_Front_View_Target.target_ctrl_enable=0;

			Total_Controller.SDK_Roll_Position_Control.Expect=0;
			Total_Controller.SDK_Roll_Position_Control.FeedBack=Opv_Front_View_Target.sdk_target.x;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Roll_Position_Control,Opv_Front_View_Target.trust_flag,0.1f);
			OpticalFlow_Pos_Ctrl_Output.x=-Total_Controller.SDK_Roll_Position_Control.Control_OutPut;
			

			Total_Controller.SDK_Pitch_Position_Control.Expect=Forward_Keep_Distance;//期望为前向保持距离，具体可根据实际自行定义
			Total_Controller.SDK_Pitch_Position_Control.FeedBack=Opv_Front_View_Target.apriltag_distance;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Pitch_Position_Control,Opv_Front_View_Target.trust_flag,0.1f);
			OpticalFlow_Pos_Ctrl_Output.y=-Total_Controller.SDK_Pitch_Position_Control.Control_OutPut;	
		}
		else//丢失目标
		{
		  miss_flag=1;
		}		
	}

	
	if(miss_flag==1)//目标丢失
	{
		if(miss_cnt==1)//初始丢失跟踪目标后，锁定当前位置后，进行普通光流控制
		{
			miss_cnt=2;
			OpticalFlow_Pos_Ctrl_Output.x=0;
			OpticalFlow_Pos_Ctrl_Output.y=0;
			OpticalFlow_Control_Pure(1);		
		}
		else if(miss_cnt==2)//丢失跟踪目标后，进行普通光流控制
		{
			OpticalFlow_Control_Pure(0);		
		}
	}
  else//目标未丢失
	{
		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度控制周期20ms
	}			
}








void Self_Track_Control_Pilot(void)
{
	static uint8_t miss_cnt=1;
	static uint16_t _cnt=0;
	static uint8_t miss_flag=0;
	_cnt++;
	if(_cnt>=20)//100ms
	{	
		_cnt=0;		
	  if(Opv_Top_View_Target.target_ctrl_enable==1)//目标点检测跟踪
		{			
			miss_cnt=1;
			miss_flag=0;			
			Opv_Top_View_Target.target_ctrl_enable=0;

			Total_Controller.SDK_Roll_Position_Control.Expect=0;
			Total_Controller.SDK_Roll_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.x;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Roll_Position_Control,Opv_Top_View_Target.trust_flag,0.1f);
			OpticalFlow_Pos_Ctrl_Output.x=-Total_Controller.SDK_Roll_Position_Control.Control_OutPut;
			
			static float target_speed;//5 3 2 1
			if(ABS(Opv_Top_View_Target.sdk_angle)<=10)      target_speed=8;//15
			else if(ABS(Opv_Top_View_Target.sdk_angle)<=20) target_speed=5;//10
			else if(ABS(Opv_Top_View_Target.sdk_angle)<=50) target_speed=3;//5
			else target_speed=1;//5 3 2 1
					
			OpticalFlow_Pos_Ctrl_Output.y=target_speed;	
		}
		else//丢失目标
		{
		  miss_flag=1;
		}		
	}

	
	if(miss_flag==1)//目标丢失
	{
		if(miss_cnt==1)//初始丢失跟踪目标后，锁定当前位置后，进行普通光流控制
		{
			miss_cnt=2;
			OpticalFlow_Pos_Ctrl_Output.x=0;
			OpticalFlow_Pos_Ctrl_Output.y=0;
			OpticalFlow_Control_Pure(1);		
		}
		else if(miss_cnt==2)//丢失跟踪目标后，进行普通光流控制
		{
			OpticalFlow_Control_Pure(0);		
			Flight.yaw_ctrl_mode=ROTATE;
			Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];			
		}
	}
  else//目标未丢失
	{
		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度控制周期20ms		
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output  =-3.0f*Opv_Top_View_Target.sdk_angle;
	}			
}


#define Pole_Keep_Distance   50//cm
#define Deadband_Pole_Keep_Distance  10
void Front_Surround_Pole_Control_Pilot(void)
{
	static uint8_t miss_cnt=1;
	static uint16_t _cnt=0;
	static uint8_t miss_flag=0;
	static uint8_t gap_miss_cnt=1;
	_cnt++;
	if(_cnt>=20)//100ms
	{	
		_cnt=0;		
	  if(Opv_Front_View_Target.target_ctrl_enable==1)//目标点检测跟踪
		{			
			miss_cnt=1;
			miss_flag=0;			
			Opv_Front_View_Target.target_ctrl_enable=0;
			//第一优先级：当左右方向存在偏移时，调整偏航角度，使得机头对准塔杆
			
			Flight.yaw_ctrl_mode=ROTATE;
			Flight.yaw_outer_control_output  =5.0f*Opv_Front_View_Target.sdk_target.x;
			if(ABS(Opv_Front_View_Target.sdk_target.x)<5.0f)//机头已经对准塔杆，可进行下一步调整飞机与塔杆的距离
			{
				gap_miss_cnt=1;
				//第二优先级：当前后方向存在偏移时，调整俯仰方向位置，使得飞机与塔杆保持一定距离
				if(front_tofsense_distance_valid_flag==1)//机头已经对准塔杆，且测距传感器距离有效时
				{
					//实时调整对杆距离
					Total_Controller.SDK_Pitch_Position_Control.Expect=Pole_Keep_Distance;//期望为前向保持距离，具体可根据实际自行定义
					Total_Controller.SDK_Pitch_Position_Control.FeedBack=front_tofsense_distance;
					PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Pitch_Position_Control,front_tofsense_distance_valid_flag,0.1f);//Opv_Front_View_Target.trust_flag
					OpticalFlow_Pos_Ctrl_Output.y=-Total_Controller.SDK_Pitch_Position_Control.Control_OutPut;
					OpticalFlow_Pos_Ctrl_Output.y=constrain_float(OpticalFlow_Pos_Ctrl_Output.y,-10,10);
					
				}
				else//机头已经对准塔杆，但是测距传感器的距离无效时
				{
					//可能情况：1距离塔杆太远，给定机头前向速度，靠进塔杆
					OpticalFlow_Pos_Ctrl_Output.y=10;
				}

				
				if(ABS(Total_Controller.SDK_Pitch_Position_Control.Err)<10.0f)//机头已经对准塔杆，且飞机与塔杆的距离已经调整完毕，可以执行横向速度控制
				{
					//第三优先级：当左右方向存在偏移误差小，并且距离塔杆距离误差小时，移动横滚方向速度，实现绕杆飞行
					OpticalFlow_Pos_Ctrl_Output.x=-5;//机头右侧方向速度为负，逆时针转动				
				}
				else//机头已经对准塔杆，但是飞机与塔杆之间距离尚未调整完毕，等待距离调整完毕再执行横向速度控制
				{
					OpticalFlow_Pos_Ctrl_Output.x=0;//横向位置保持
				}
			}
			else//视野能识别到塔杆，但是机头还未对准塔杆时，原地悬停，等待机头对准后再调整前后距离和绕杆飞行
			{
				miss_flag=2;
			}
		}
		else//丢失目标
		{
		  miss_flag=1;
		}		
	}

	
	if(miss_flag==1)//目标丢失
	{
		if(miss_cnt==1)//初始丢失跟踪目标后，锁定当前位置后，进行普通光流控制
		{
			miss_cnt=2;
			OpticalFlow_Pos_Ctrl_Output.x=0;
			OpticalFlow_Pos_Ctrl_Output.y=0;
			OpticalFlow_Control_Pure(1);		
		}
		else if(miss_cnt==2)//丢失跟踪目标后，进行普通光流控制
		{
			OpticalFlow_Control_Pure(0);	
			Flight.yaw_ctrl_mode=ROTATE;
			Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];			
		}
	}
	else if(miss_flag==2)//视野能识别到塔杆，但是机头还未对准塔杆时
	{
		//偏航控制权限仍然来源于塔杆位置反馈，前后、横向位置保持原地
		if(gap_miss_cnt==1)//初始丢失跟踪目标后，锁定当前位置后，进行普通光流控制
		{
			gap_miss_cnt=2;
			OpticalFlow_Pos_Ctrl_Output.x=0;
			OpticalFlow_Pos_Ctrl_Output.y=0;
			OpticalFlow_Control_Pure(1);		
		}
		else if(gap_miss_cnt==2)//丢失跟踪目标后，进行普通光流控制
		{
			OpticalFlow_Control_Pure(0);	
		}	
	}
  else//目标未丢失
	{
		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度控制周期20ms
	}			
}



//适用于激光雷达SLAM定位条件下，普通光流（LC307、LC302）定位条件下无效
void Horizontal_Navigation(float x,float y,float z,uint8_t nav_mode,uint8_t frame_id)
{	
	if(nav_mode==RELATIVE_MODE)//相对模式
	{
		switch(frame_id)
		{
			case BODY_FRAME://机体坐标系下
			{
				float map_x=0,map_y=0;
				from_body_to_nav_frame(x,y,&map_x,&map_y,WP_AHRS.Yaw);
				OpticalFlow_Pos_Ctrl_Expect.x=VIO_SINS.Position[_EAST]+map_x;
				OpticalFlow_Pos_Ctrl_Expect.y=VIO_SINS.Position[_NORTH]+map_y;
				Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP]+z;			
			}
			break;
			case MAP_FRAME://导航坐标系下
			{
				OpticalFlow_Pos_Ctrl_Expect.x=VIO_SINS.Position[_EAST]+x;
				OpticalFlow_Pos_Ctrl_Expect.y=VIO_SINS.Position[_NORTH]+y;
				Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP]+z;
			}
			break;		
		}	
	}
	else if(nav_mode==GLOBAL_MODE)//全局模式
	{
		switch(frame_id)
		{
			case MAP_FRAME://导航坐标系下
			{
				OpticalFlow_Pos_Ctrl_Expect.x=x;
				OpticalFlow_Pos_Ctrl_Expect.y=y;
				Total_Controller.Height_Position_Control.Expect=z;
			}
			break;
			default://原地保持
			{
				OpticalFlow_Pos_Ctrl_Expect.x=VIO_SINS.Position[_EAST];
				OpticalFlow_Pos_Ctrl_Expect.y=VIO_SINS.Position[_NORTH];
				Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP];
			}				
		}	
	}
}








