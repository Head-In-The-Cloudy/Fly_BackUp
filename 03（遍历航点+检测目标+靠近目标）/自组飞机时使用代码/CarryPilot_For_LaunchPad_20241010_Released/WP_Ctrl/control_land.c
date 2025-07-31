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
#include "control_config.h"
#include "control_land.h"



enum fixed_state{Faraway,Near,Arrive};


Vector2_Nav land_pos_target,land_pos_now,land_home_pos;
float target_yaw_rate=0,target_yaw_alt=0;
bool land_althold(float taret_climb_rate,float target_climb_alt)
{
  static uint8_t move_flag=0;
  bool handmove_flag=FALSE;
//  float step_mapping=0,scale_mapping=0;
  Thr_Scale_Set(&RC_Data.cal[2]);
  High_Hold_Throttle=Thr_Hover_Default;
  /*高度控制器第1步*/
  /********
      **
      **
      **
      **
      **
  ********/
  /*******************************高度控制器开始****************************************/
  /****************定高：高度位置环+速度环+加速度环，控制周期分别为8ms、4ms、4ms*******************/
  if(target_climb_alt==0)
  {
    Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP];//一直更新高度期望
  }
  else
  {
    Total_Controller.Height_Position_Control.Expect=target_climb_alt;//更新高度期望
  }
	if(RC_Data.rc_rpyt[RC_THR]==0)//油门杆位于中位死区内，进行高度保持
  {
    //高度位置环输出给定速度期望
		Total_Controller.Height_Position_Control.FeedBack=NamelessQuad.Position[_UP];//反馈
		PID_Control(&Total_Controller.Height_Position_Control,0.005f);//海拔高度位置控制器
		//内环速度期望
		if(target_climb_alt==0) Alt_Vel_Target=taret_climb_rate;//本次速度期望来源于外部直接给定
		else Alt_Vel_Target=Total_Controller.Height_Position_Control.Control_OutPut;//本次速度期望来源于位置控制器输出
#ifdef ALT_VEL_FEEDFORWARD_ENABLE  //速度前馈控制器
		ALT_VEL_FEEDFORWARD_ENABLE_Delta=(Alt_Vel_Target-Last_Alt_Vel_Target)/(1*Delta);//速度期望变化率
		ALT_VEL_FEEDFORWARD_ENABLE_Output=ALT_VEL_FEEDFORWARD_ENABLE_Rate*ALT_VEL_FEEDFORWARD_ENABLE_Delta;
		Last_Alt_Vel_Target=Alt_Vel_Target;//上次速度期望
#endif
		Total_Controller.Height_Speed_Control.Expect=Alt_Vel_Target;
    handmove_flag=FALSE;
    move_flag=0;
  }
  else if(RC_Data.rc_rpyt[RC_THR]>0)//给定上升速度期望
  {
    //油门杆上推、给定速度期望
    Total_Controller.Height_Speed_Control.Expect=RC_Data.rc_rpyt[RC_THR];//最大上升速度50cm/s
    handmove_flag=TRUE;
    move_flag=1;
  }
  else if(RC_Data.rc_rpyt[RC_THR]<0)//给定下降速度期望
  {
    Total_Controller.Height_Speed_Control.Expect=RC_Data.rc_rpyt[RC_THR];//最大下降速度40cm/s
    handmove_flag=TRUE;
    move_flag=2;
  }
  /*高度控制器第2步*/
  /********
          *
          *
   * * * *
  *
  *
  ********/
  /*******************************竖直速度控制器开始*******************************************************************/
	Total_Controller.Height_Speed_Control.FeedBack=NamelessQuad.Speed[_UP];//惯导速度估计给速度反馈
	PID_Control(&Total_Controller.Height_Speed_Control,0.005f);//海拔高度速度控制
	//ADRC_TD_Control(&ADRC_Alt_Vel_Controller,Total_Controller.Height_Speed_Control.Expect,Total_Controller.Height_Speed_Control.FeedBack);
	//Total_Controller.Height_Speed_Control.Control_OutPut=ADRC_Alt_Vel_Controller.u;
  /*******************************上升下降过程中期望加速度限幅单独处理*******************************************************************/     
  if(move_flag==0)
	{
    Total_Controller.Height_Speed_Control.Control_OutPut=constrain_float(Total_Controller.Height_Speed_Control.Control_OutPut,
                                                                       -SDK_Mild_Acceleration_Max,SDK_Mild_Acceleration_Max);  
	}		
  else if(move_flag==1)//在上下推杆时对速度控制器输出，对期望上升、下降加速度进行分别限幅，确保快速下降时姿态平稳
  {
    Total_Controller.Height_Speed_Control.Control_OutPut=constrain_float(Total_Controller.Height_Speed_Control.Control_OutPut,
                                                                       -Climb_Up_Acceleration_Max,Climb_Up_Acceleration_Max);                                                                
  }
  else if(move_flag==2)
  {
    Total_Controller.Height_Speed_Control.Control_OutPut=constrain_float(Total_Controller.Height_Speed_Control.Control_OutPut,
                                                                       -Climb_Down_Acceleration_Max,Climb_Down_Acceleration_Max);
  }    
  /*******************************竖直速度控制器结束******************************************************************/
  /*高度控制器第3步*/
  /********
         **
         **
       **
         **
         **
  ********/
  
  /*******************************竖直加速度控制器开始******************************************************************/
  Total_Controller.Height_Acce_Control.Expect=Total_Controller.Height_Speed_Control.Control_OutPut;//加速度期望
#ifdef ALT_VEL_FEEDFORWARD_ENABLE  //速度前馈控制器
  Total_Controller.Height_Acce_Control.Expect+=ALT_VEL_FEEDFORWARD_ENABLE_Output;//加上前馈控制器输出
#endif
  Total_Controller.Height_Acce_Control.FeedBack=Acceleration_Feedback[_UP];//加速度反馈
	PID_Control(&Total_Controller.Height_Acce_Control,0.005f);//海拔高度加速度控制
  /**************************************
  加速度环前馈补偿，引用时请注明出处
  悬停油门 = 加速度环积分值 + 基准悬停油门
  此时输出力 F = mg
  当需要输出a的加速度时，输出力 F1=m(g+a)
  F1/F = 1 + a/g
  因此此时应输出：悬停油门*(1 + a/g)
  **************************************/
  Total_Controller.Height_Acce_Control.Control_OutPut+=(High_Hold_Throttle+Total_Controller.Height_Acce_Control.Integrate-Thr_Start)*
    (Total_Controller.Height_Acce_Control.Expect/980);
  /*******************************竖直加速度控制器结束******************************************************************/
	Throttle=Int_Sort(High_Hold_Throttle+Total_Controller.Height_Acce_Control.Control_OutPut);//油门来源于高度加速度控制器输出
  /*****************************************高度控制器结束，给定油门控制量***********************************************************/
  return handmove_flag;
}



Vector2f_Nav land_pos_sort(Vector2_Nav target_pos,Vector2_Nav now_pos)//得到相对目标点机体Pit、Rol方向偏移
{
  Vector2f_Nav earth_pos_delta;
  //Vector2_Body body_pos_delta;
  static int32_t last_land_lat;
  static float land_scale = 1.0;
  //比较两次纬度相差值，避免重复运算余弦函数
  if(ABS(last_land_lat - target_pos.lat) < 100000)
  {
    // we are within 0.01 degrees (about 1km) of the
    // same latitude. We can avoid the cos() and return
    // the same scale factor.
    //land_scale;
  }
  else
  {
    land_scale = cosf(target_pos.lat * 1.0e-7f * DEG_TO_RAD);
    land_scale = constrain_float(land_scale, 0.01f, 1.0f);
  }
  last_land_lat= target_pos.lat;
  earth_pos_delta.N=(target_pos.lat-now_pos.lat) * LOCATION_SCALING_FACTOR;//距离单位为m
  earth_pos_delta.E=(target_pos.lng-now_pos.lng) * LOCATION_SCALING_FACTOR * land_scale;//距离单位为m
  /***********************************************************************************
  明确下导航系方向，这里正北、正东为正方向:
  沿着正东，经度增加,当无人机相对home点，往正东向移动时，此时GPS_Present.lng>GPS_Home.lng，得到的location_delta.x大于0;
  沿着正北，纬度增加,当无人机相对home点，往正北向移动时，此时GPS_Present.lat>GPS_Home.lat，得到的location_delta.y大于0;
  ******************************************************************************/
  earth_pos_delta.N*=100;//沿地理坐标系，正北方向位置偏移,单位为CM
  earth_pos_delta.E*=100;//沿地理坐标系，正东方向位置偏移,单位为CM
  return earth_pos_delta;
}

bool alt_handmove_check(void)
{
  bool alt_handmove=FALSE;
	if(ABS(RC_Data.rc_rpyt[RC_THR])!=0) alt_handmove=TRUE;
  return alt_handmove;
}

bool pos_handmove_check(void)
{
  bool pos_handmove=FALSE;
  pos_handmove=(bool)(Roll_Control!=0||Pitch_Control!=0);//无水平遥控量给定
  return pos_handmove;
}

bool land_poshold(Vector2_Nav target_pos,Vector2_Nav now_pos,uint8_t fixed_state)
{
  static uint16 position_cnt=0;//高度速度控制计数器
  static uint16 speed_cnt=0;//高度速度控制计数器
  bool handmove_flag=FALSE;
  float max_nav_speed=0;
  Vector2f_Nav earth_frame_distance={0,0};
  earth_frame_distance=land_pos_sort(target_pos,land_home_pos);//得到相对目标点N、E方向偏移，即期望位置偏移
  if(Landon_Earth_Flag==0)
  {
    /*******************************水平位置控制器开始***********************************************************/
    //技术讲解见博客：四旋翼GPS定点流程梳理与原理浅析 http://blog.csdn.net/u011992534/article/details/79408187
    if(GPS_ok()==TRUE&&Sensor_Flag.Mag_Health==TRUE)//磁力计正常连接
    {
      if(Roll_Control==0
         &&Pitch_Control==0)//无水平遥控量给定
      {
        position_cnt++;
        if(position_cnt>8)//20ms控制一次
        {
          //位置期望,经纬、航行速度、高度
          Total_Controller.Latitude_Position_Control.Expect=NamelessQuad.Position[_NORTH];//移植更新水平目标位置，目的是切回定点模式赋悬停点
          Total_Controller.Longitude_Position_Control.Expect=NamelessQuad.Position[_EAST];
          //导航坐标系下E、N方向上位置偏差
          Earth_Frame_Pos_Err.N=earth_frame_distance.N-NamelessQuad.Position[_NORTH];
          Earth_Frame_Pos_Err.E=earth_frame_distance.E-NamelessQuad.Position[_EAST];
          //导航坐标系下机体Pitch、Roll方向上位置偏差
          Body_Frame_Pos_Err.Pit=-Earth_Frame_Pos_Err.E*WP_AHRS.sin_rpy[_YAW]+Earth_Frame_Pos_Err.N*WP_AHRS.cos_rpy[_YAW];
          Body_Frame_Pos_Err.Rol=Earth_Frame_Pos_Err.E*WP_AHRS.cos_rpy[_YAW]+Earth_Frame_Pos_Err.N*WP_AHRS.sin_rpy[_YAW];
          //导航坐标系下机体Pitch、Roll方向上期望刹车速度，这里为单比例运算不调用PID_Control()函数
          if(fixed_state==Faraway)   max_nav_speed=First_Nav_Rate;
          else if(fixed_state==Near)   max_nav_speed=Second_Nav_Rate;
          else if(fixed_state==Arrive)   max_nav_speed=Third_Nav_Rate;
          else   max_nav_speed=Default_Nav_rate;
          Body_Frame_Brake_Speed.Pit=constrain_float(Total_Controller.Latitude_Position_Control.Kp*Body_Frame_Pos_Err.Pit,-max_nav_speed, max_nav_speed);//位置偏差限幅，单位cm
          Body_Frame_Brake_Speed.Rol=constrain_float(Total_Controller.Latitude_Position_Control.Kp*Body_Frame_Pos_Err.Rol,-max_nav_speed, max_nav_speed);//位置偏差限幅，单位cm
          
          //速度控制器期望
          Total_Controller.Latitude_Speed_Control.Expect =Body_Frame_Brake_Speed.Pit;
          Total_Controller.Longitude_Speed_Control.Expect=Body_Frame_Brake_Speed.Rol;
          position_cnt=0;//位置控制器结束，给出期望刹车速度
        }
        //导航系的水平速度，转化到机体坐标系X-Y方向上
        //沿载体Pitch、Roll方向水平速度控制
        speed_cnt++;
        if(speed_cnt>=4)//10ms控制一次位置
        {
          Body_Frame_Speed_Feedback.Pit=-NamelessQuad.Speed[_EAST]*WP_AHRS.sin_rpy[_YAW]+NamelessQuad.Speed[_NORTH]*WP_AHRS.cos_rpy[_YAW];
          Body_Frame_Speed_Feedback.Rol=NamelessQuad.Speed[_EAST]*WP_AHRS.cos_rpy[_YAW]+NamelessQuad.Speed[_NORTH]*WP_AHRS.sin_rpy[_YAW];
          //沿载体方向速度反馈量
          Total_Controller.Latitude_Speed_Control.FeedBack=Body_Frame_Speed_Feedback.Pit;//机头Pitch方向，Y轴正向
          Total_Controller.Longitude_Speed_Control.FeedBack=Body_Frame_Speed_Feedback.Rol;//横滚Roll方向，X轴正向
          //沿载体方向速度控制器
          PID_Control_Div_LPF(&Total_Controller.Latitude_Speed_Control,0.02f);
          PID_Control_Div_LPF(&Total_Controller.Longitude_Speed_Control,0.02f);
          
          accel_target.y=-Total_Controller.Latitude_Speed_Control.Control_OutPut;
          accel_target.x=Total_Controller.Longitude_Speed_Control.Control_OutPut;//期望运动加速度
          ENU_Desired_Accel_Transform_Angle(accel_target,&angle_target);//期望运动加速度转期望姿态倾角
					Flight.roll_outer_control_output =angle_target.x;
					Flight.pitch_outer_control_output=angle_target.y;
				
          //Total_Controller.Pitch_Angle_Control.Expect=-Total_Controller.Latitude_Speed_Control.Control_OutPut;
          //Total_Controller.Roll_Angle_Control.Expect=Total_Controller.Longitude_Speed_Control.Control_OutPut;
          speed_cnt=0;
        }
        /*******************************水平位置控制器结束***********************************************************/
        handmove_flag=FALSE;
      }
      else //只进行水平速度控制，无水平位置控制
      {
        speed_cnt++;
        if(speed_cnt>=4)//10ms控制一次速度
        {
          Total_Controller.Latitude_Speed_Control.Expect=ncq_speed_mapping(-Target_Angle[0],Pit_Rol_Max,Max_Horvel);
          Total_Controller.Longitude_Speed_Control.Expect=ncq_speed_mapping(Target_Angle[1],Pit_Rol_Max,Max_Horvel);
          //导航系的水平速度，转化到机体坐标系X-Y方向上
          //沿载体Pitch、Roll方向水平速度控制
          Body_Frame_Speed_Feedback.Pit=-NamelessQuad.Speed[_EAST]*WP_AHRS.sin_rpy[_YAW]+NamelessQuad.Speed[_NORTH]*WP_AHRS.cos_rpy[_YAW];
          Body_Frame_Speed_Feedback.Rol=NamelessQuad.Speed[_EAST]*WP_AHRS.cos_rpy[_YAW]+NamelessQuad.Speed[_NORTH]*WP_AHRS.sin_rpy[_YAW];
          
          Total_Controller.Latitude_Speed_Control.FeedBack=Body_Frame_Speed_Feedback.Pit;//机头Pitch方向，Y轴正向
          Total_Controller.Longitude_Speed_Control.FeedBack=Body_Frame_Speed_Feedback.Rol;//横滚Roll方向，X轴正向
          
          PID_Control_Div_LPF(&Total_Controller.Latitude_Speed_Control,0.02f);
          PID_Control_Div_LPF(&Total_Controller.Longitude_Speed_Control,0.02f);
          
          accel_target.y=-Total_Controller.Latitude_Speed_Control.Control_OutPut;
          accel_target.x=Total_Controller.Longitude_Speed_Control.Control_OutPut;//期望运动加速度
          ENU_Desired_Accel_Transform_Angle(accel_target,&angle_target);//期望运动加速度转期望姿态倾角
					Flight.roll_outer_control_output =angle_target.x;
					Flight.pitch_outer_control_output=angle_target.y;
          //Total_Controller.Pitch_Angle_Control.Expect=-Total_Controller.Latitude_Speed_Control.Control_OutPut;
          //Total_Controller.Roll_Angle_Control.Expect=Total_Controller.Longitude_Speed_Control.Control_OutPut;
          speed_cnt=0;
        }
        handmove_flag=TRUE;
      }
    }
  }
  else
  {
		Flight.roll_outer_control_output =0;
		Flight.pitch_outer_control_output=0;
    Take_Off_Reset();//清空积分控制器
  }
  return handmove_flag;
}





#define Minimal_Thrust_Threshold 1150//着陆检测油门最小值
uint16_t Landon_Earth_Cnt=0;
uint8_t Last_Landon_Earth_Flag=0,Landon_Earth_Flag=0;
uint8_t Landon_Earth_Flag_Set=0;

void landon_earth_check(void)//自检触地进入怠速模式
{
  Last_Landon_Earth_Flag=Landon_Earth_Flag;
  //油门控制处于较低行程：//1、姿态模式下，油门杆处于低位
  //2、定高模式下，期望速度向下，单加速度环反馈为角小值，
  //加速度控制输出由于长时间积分，到负的较大值，使得油门控制较低
  if(Throttle_Output<=Minimal_Thrust_Threshold
     &&Gyro_Length<=50.0f//触地后无旋转，合角速度小于30deg/s
       &&ABS(NamelessQuad.Speed[_UP])<=40.0f//惯导竖直轴速度+-40cm/s
         &&Unwanted_Lock_Flag==0)
    Landon_Earth_Cnt++;
  else Landon_Earth_Cnt/=2;
  
  if(Landon_Earth_Cnt>=1500)  Landon_Earth_Cnt=1500;//防止溢出
  if(Landon_Earth_Cnt>=200*1.0)//持续1.0S
  {
    Landon_Earth_Flag=1;//着陆标志位
    //Landon_Earth_Cnt=0;
  }
  else
  {
    Landon_Earth_Flag=0;//着陆标志位
  }
  /*
  if(ABS(Throttle_Rate)>=1.0E-5f)//只要油门变化率不为0，即清空着陆标志位
  {
  Landon_Earth_Flag=0;
  Landon_Earth_Cnt=0;
	}
  */
  
  
}

// land_nogps_run - runs the land controller
//      pilot controls roll and pitch angles
//      should be called at 100hz or more
void land_nogps_run()
{
  if(Landon_Earth_Flag==0)//未触地，有姿态控制参与
  {
		Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
		Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];
		
    if(NamelessQuad.Position[_UP]<=Near_Ground_Height)//接近地面，下降速度最慢
			target_yaw_rate=Nav_Near_Ground_Rate;
    else if(NamelessQuad.Position[_UP]<=Safe_Height)//相对初始高度10M以内,暂时忽略漂移
      target_yaw_rate=Nav_Decline_Rate;//以Nav_Decline_rate速度下降
    else target_yaw_rate=Nav_Rapid_Decline_Rate;//以Nav_Rapid_Decline_rate速度下降   
		target_yaw_alt=0;
		Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL,NUL,target_yaw_rate);//高度控制
  }
  else
  {
		Flight.roll_outer_control_output =0;
		Flight.pitch_outer_control_output=0;
    Take_Off_Reset();//清空积分控制器
    target_yaw_rate=2*Nav_Rapid_Decline_Rate;//以2*Nav_Rapid_Decline_rate速度下降,使得油门量迅速满足怠速条件
    target_yaw_alt=0;
		Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL,NUL,target_yaw_rate);//高度控制
  }
}



uint8_t last_handmove_change_mode=1,handmove_change_mode=1;
uint16 nav_transition_cnt=0;
uint16 nav_transition_flag=0;
uint16 nav_climb_rate_cnt=0;
int16 nav_decline_rate_cnt=0;
uint16 first_switch_flag=1;
uint8_t distance_land_mode=0,last_distance_land_mode=0;
uint8_t climb_safe_alt_flag=0,home_fixed_flag=0;
bool check_pos_handmove=FALSE;
bool check_alt_handmove=FALSE;

void land_reset()//着陆过程中存在各种标志位，在一次飞行过程中，可能会频繁切换，需要清空上次未进行完毕的标志位，在切换时会全部清空
{
  //复位过程中的所有标志位，对于应对较为复杂的逻辑过程的必备编程技巧，一定要掌握好
  last_handmove_change_mode=0;
  
  nav_transition_cnt=0;
  nav_transition_flag=0;
  nav_climb_rate_cnt=0;
  distance_land_mode=0;
  last_distance_land_mode=0;//将自动降落模式标志位置0，下一次循环会重新判断
  climb_safe_alt_flag=0;
  first_switch_flag=1;
  home_fixed_flag=0;
  handmove_change_mode=2;//水平位置手动模式
  first_switch_flag=1;//准备好下次回中时刻的切换
  
  /*
  last_handmove_change_mode=1;
  handmove_change_mode=1;
  nav_transition_cnt=0;
  nav_transition_flag=0;
  nav_climb_rate_cnt=0;
  nav_decline_rate_cnt=0;
  first_switch_flag=1;
  distance_land_mode=0;
  last_distance_land_mode=0;
  climb_safe_alt_flag=0;
  home_fixed_flag=0;
  check_pos_handmove=FALSE;
  check_alt_handmove=FALSE;
  */
}


void land_state_check(void)
{
  float distance_to_home=100*get_distance(GPS_Home,GPS_Present);//单位cm
  last_distance_land_mode=distance_land_mode;
  if(distance_to_home>=Faraway_Distance)//离起飞点水平距离超过Faraway_Distance半径
  {
    distance_land_mode=1;//先原地上升至安全高度再返航到水平点正上方
    home_fixed_flag=Faraway;//远离home点
  }
  else if(distance_to_home>=Near_Distance)//离起飞点水平距离Near_Distance~Faraway_Distance半径
  {
    distance_land_mode=2;//先保持当前高度，水平运动至home点正上方，再下降
    home_fixed_flag=Near;//接近home点
  }
  else//与home点距离小于等于Near_Distance半径，已到达home点
  {
    distance_land_mode=3;//水平位置保持为home点，并直接下降
    home_fixed_flag=Arrive;//已锁定home点
  }
  
  check_pos_handmove=pos_handmove_check();//水平位置方向手动
  check_alt_handmove=alt_handmove_check();//竖直高度方向手动
  
  last_handmove_change_mode=handmove_change_mode;
  if(check_pos_handmove==TRUE)//若水平位置自动控制过程中，有手动操作
  {
    last_distance_land_mode=0;//将自动降落模式标志位置0，下一次循环会重新判断
    //distance_land_mode=0;
    climb_safe_alt_flag=0;
    home_fixed_flag=0;
    handmove_change_mode=2;//水平位置手动模式
    first_switch_flag=1;//准备好下次回中时刻的切换
    /**************希望返航过程中，水平打时无高度上升、下降操作，即水平打杆时，高度保持即可**************/
  }
  else handmove_change_mode=1;
  
  if(check_alt_handmove==TRUE)//若高度自动控制过程中，有手动操作
  {
    last_distance_land_mode=0;//将自动降落模式标志位置0，下一次循环会重新判断
    //distance_land_mode=0;
    climb_safe_alt_flag=0;
    home_fixed_flag=0;
    first_switch_flag=1;//准备好下次回中时刻的切换
  }
}
// land_run - runs the land controller
//      horizontal position controlled with loiter controller
//      should be called at 100hz or more
void land_gps_run()
{
  static uint16_t land_gap_cnt=0;
  land_home_pos.lat=GPS_Home.lat;
  land_home_pos.lng=GPS_Home.lng;
  land_pos_now.lat=GPS_Present.lat;
  land_pos_now.lng=GPS_Present.lng;
  
  land_state_check();
  
  if(Landon_Earth_Flag==0)//未触地，有姿态控制参与
  {
    if(check_pos_handmove==FALSE)//无水平打杆动作位
    {
      if(distance_land_mode==1&&first_switch_flag==1)//首次切返航模式，距离home很远时
      {
        if(NamelessQuad.Position[_UP]<Safe_Height)//当切返航瞬间的高度小于安全高度时，保持当前水平位置，攀升至安全高度再返航
        {
          land_pos_target.lat=GPS_Present.lat;
          land_pos_target.lng=GPS_Present.lng;
          //target_yaw_rate=Nav_climb_rate;//以Nav_climb_rate速度上升到Nav_Safety_Height安全高度
          nav_climb_rate_cnt=Nav_Climb_Rate;
          target_yaw_rate=Nav_Climb_Rate-nav_climb_rate_cnt;//速度依次递增，避免油门控制量突变
          land_gap_cnt=0;//递增时间步长计数器
          //这里可简单计算一下当微分参数为1.5时
          //1.5*50=75油门控制量瞬间会有75的突变，可能导致飞机不稳定
          target_yaw_alt=0;//目标高度置0
          climb_safe_alt_flag=0;
        }
        else//当切返航瞬间的高度大于安全高度时，保持当前高度，执行返航
        {
          land_pos_target.lat=GPS_Home.lat;
          land_pos_target.lng=GPS_Home.lng;
          target_yaw_rate=0;//目标速度置0
          target_yaw_alt=NamelessQuad.Position[_UP];//目标保持当前高度
          climb_safe_alt_flag=1;//已经达到安全高度
        }
        first_switch_flag=0;
        last_distance_land_mode=1;//确保能顺利进入下一阶段
      }
      else if(distance_land_mode==1&&last_distance_land_mode==1)//持续处于远离home点较远位置
      {
        if(NamelessQuad.Position[_UP]>=Safe_Height//当返航的高度超过安全高度时，保持当前高度，飞至home正上方即可
           &&climb_safe_alt_flag==0)//还未到达安全高度
        {
          climb_safe_alt_flag=1;//已到达安全高度
          nav_transition_cnt=Nav_Transition_Period;//设置过渡缓冲时间Nav_Transition_Period个周期
          nav_transition_flag=0;
        }
        else if(climb_safe_alt_flag==0)//离home较远，未达到安全高度，仍然处于保持原地位置，需继续上升
        {
          //水平fixed位置不用再次设置，因为在初始切返航时已设置
          //target_yaw_rate=Nav_climb_rate;//以Nav_climb_rate速度上升到Nav_Safety_Height安全高度
          land_gap_cnt++;
          if(land_gap_cnt>=8)//每8*5=40ms递减
          {
            if(nav_climb_rate_cnt>=1)  nav_climb_rate_cnt--;
            else nav_climb_rate_cnt=0;
            land_gap_cnt=0;
          }
          
          target_yaw_rate=Nav_Climb_Rate-nav_climb_rate_cnt;//速度依次递增，避免油门控制量突变
          //这里可简单计算一下当微分参数为1.5时
          //1.5*50=75油门控制量瞬间会有75的突变，飞机导致不稳定
          target_yaw_alt=0;//目标高度置0
        }
        
        if(climb_safe_alt_flag==1)//只设置一次
        {
          if(nav_transition_cnt>=1)  nav_transition_cnt--;//衰减至0
          else nav_transition_cnt=0;
          
          if(nav_transition_cnt==0)//缓冲时间已结束，设置返航点
          {
            land_pos_target.lat=GPS_Home.lat;
            land_pos_target.lng=GPS_Home.lng;
            target_yaw_rate=0;//目标速度置0
            target_yaw_alt=NamelessQuad.Position[_UP];//目标保持当前高度
            climb_safe_alt_flag++;//climb_safe_alt_flag==2
          }
          else//缓冲期间
          {
            if(nav_transition_flag==0)//未设置缓冲状态
            {
              nav_transition_flag=1;//有且仅设置1次，设置为在原地悬停nav_transition_cnt个周期
              land_pos_target.lat=GPS_Present.lat;
              land_pos_target.lng=GPS_Present.lng;
              target_yaw_rate=0;
              target_yaw_alt=NamelessQuad.Position[_UP];
            }
          }
        }
      }
      else if(distance_land_mode==2&&last_distance_land_mode!=2)//首次切返航模式，距离home较近时，保持当前高度，将Home点作为目标点，飞至home正上方
      {
        land_pos_target.lat=GPS_Home.lat;
        land_pos_target.lng=GPS_Home.lng;
        target_yaw_rate=0;//目标速度置0
        target_yaw_alt=NamelessQuad.Position[_UP];//将目标高度设置为当前高度
        climb_safe_alt_flag=2;
        //这里包含两种情况：1、A->B  2、C->B
        //执行的操作相同，即：保持当前高度，向home靠近
        last_distance_land_mode=2;//确保能顺利进入下一阶段
      }
      else if(distance_land_mode==2&&last_distance_land_mode==2)
      {
        //水平fixed位置不用再次设置，因为在上一状态已设置
        land_pos_target.lat=GPS_Home.lat;
        land_pos_target.lng=GPS_Home.lng;
        target_yaw_rate=0;//目标速度置0
        climb_safe_alt_flag=2;
      }
      else if(distance_land_mode==3&&last_distance_land_mode!=3)//首次进入home点正上方
      {
        land_pos_target.lat=GPS_Home.lat;
        land_pos_target.lng=GPS_Home.lng;
        //target_yaw_rate=Nav_Decline_Rate;//以Nav_Decline_rate目标速度下降
        nav_decline_rate_cnt=Nav_Decline_Rate;
        target_yaw_rate=Nav_Decline_Rate-nav_decline_rate_cnt;
        land_gap_cnt=0;//递增时间步长计数器
        
        target_yaw_alt=0;//将目标高度设置为当前高度
        climb_safe_alt_flag=2;
        
        //这里有且仅有一种情况：1、B->A
        //执行的操作为：水平期望设置为home点，并下降
      }
      else if(distance_land_mode==3&&last_distance_land_mode==3)//处于home点正上方
      {
        land_pos_target.lat=GPS_Home.lat;
        land_pos_target.lng=GPS_Home.lng;
        //target_yaw_rate=Nav_Decline_Rate;//以Nav_Decline_rate目标速度下降
        
        land_gap_cnt++;
        if(land_gap_cnt>=8)//每8*5=40ms递减
        {
          if(nav_decline_rate_cnt<=-1)  nav_decline_rate_cnt++;
          else nav_decline_rate_cnt=0;
          land_gap_cnt=0;
        }
        target_yaw_rate=Nav_Decline_Rate-nav_decline_rate_cnt;//下降速度依次递增，避免油门控制量突变
        
        target_yaw_alt=0;//将目标高度设置为当前高度
        climb_safe_alt_flag=2;
      }
      /*************************************************************/
      land_poshold(land_pos_target,land_pos_now,home_fixed_flag);
      land_althold(target_yaw_rate,target_yaw_alt);
    }
    else//存在水平手动操作
    {
      land_poshold(land_pos_target,land_pos_now,home_fixed_flag);
      if(handmove_change_mode==2&&last_handmove_change_mode==1)
      {
        target_yaw_rate=0;//目标速度置0
        target_yaw_alt=NamelessQuad.Position[_UP];//将目标高度设置为当前高度
      }
      else if(handmove_change_mode==2&&last_handmove_change_mode==2)
      {
        target_yaw_rate=0;//目标速度置0
        //target_yaw_alt=NamelessQuad.Position[_UP];//将目标高度设置为当前高度
      }
      land_althold(target_yaw_rate,target_yaw_alt);
    }
  }
  else
  {
		Flight.roll_outer_control_output =0;
		Flight.pitch_outer_control_output=0;
    Take_Off_Reset();//清空积分控制器
    target_yaw_rate=2*Nav_Rapid_Decline_Rate;//以2*Nav_Rapid_Decline_rate速度下降,使得油门量迅速满足怠速条件
    target_yaw_alt=0;
    land_althold(target_yaw_rate,target_yaw_alt);
  }
}



bool GPS_ok()// returns true if the GPS is ok and home position is set
{
  if(GPS_Sate_Num>=7//定位卫星超过7个
     &&GPS_Quality<=3.5f//水平精度因子大于6不可用
       &&GPS_Home_Set==1
         &&Filter_Defeated_Flag==0)
  {
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}


// land_run - runs the land controller
// should be called at 100hz or more
void land_run()
{
  if (GPS_ok()==TRUE&&Sensor_Flag.Mag_Health==TRUE)//磁力计正常连接
  {
    land_gps_run();//GPS有效时，执行一键返航操作
  }
  else
  {
    land_nogps_run();//GPS无效时，执行一键着陆操作
  }
}

/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/



