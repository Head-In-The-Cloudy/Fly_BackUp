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
#include "control_poshold.h"

Vector3_Nav Earth_Frame_Accel_Target={0,0,0};   //导航（地理）坐标系，正东、正北方向目标运动加速度期望
Vector3_Nav Earth_Frame_Pos_Err={0,0,0};        //导航（地理）坐标系，正东、正北方向w位置偏差
Vector2_Ang Body_Frame_Accel_Target={0,0};      //导航（地理）坐标系，机体横滚(Y正)、俯仰(X正)方向目标运动加速度期望
Vector2_Ang Body_Frame_Speed_Feedback={0,0};    //导航（地理）坐标系，机体横滚(Y正)、俯仰(X正)方向目标运动速度反馈
Vector2_Ang Body_Frame_Pos_Err={0,0};           //机体方向上位置偏差
Vector2_Ang Body_Frame_Brake_Speed={0,0};       //机体方向上刹车速度
uint8 GPS_Speed_Control_Mode=0;


//将NED坐标系下的期望加速度转换成机体坐标系下的倾斜角度(pitch,roll)
void NED_Desired_Accel_Transform_Angle(Vector2f _accel_target,Vector2f *target_angle)
{
	float accel_right, accel_forward;
	float lean_angle_max = 30.0f;	
	accel_right  =-_accel_target.x;//cm/s^2
	accel_forward= _accel_target.y;//cm/s^2
	//update angle targets that will be passed to stabilize controller
	//计算期望横滚角
	target_angle->x=RAD_TO_DEG*atanf((-accel_right/(GRAVITY_MSS*100))*WP_AHRS.cos_rpy[_PIT]);
	//计算期望俯仰角
	target_angle->y=RAD_TO_DEG*atanf(accel_forward/(GRAVITY_MSS*100));
	
	target_angle->x=constrain_float(target_angle->x,-lean_angle_max,lean_angle_max);//roll
	target_angle->y=constrain_float(target_angle->y,-lean_angle_max,lean_angle_max);//pitch
}


void ENU_Desired_Accel_Transform_Angle(Vector2f _accel_target,Vector2f *target_angle)
{
	float accel_right, accel_forward;
	float lean_angle_max = 30.0f;	
	accel_right  =  _accel_target.x;//cm/s^2
	accel_forward= -_accel_target.y;//cm/s^2
	//update angle targets that will be passed to stabilize controller	
	target_angle->x=RAD_TO_DEG*atanf( accel_right/GRAVITY_CMSS);													//计算期望横滚角
	target_angle->y=RAD_TO_DEG*atanf(-accel_forward*WP_AHRS.cos_rpy[_ROL]/GRAVITY_CMSS); //计算期望俯仰角
	
	target_angle->x=constrain_float(target_angle->x,-lean_angle_max,lean_angle_max);//roll
	target_angle->y=constrain_float(target_angle->y,-lean_angle_max,lean_angle_max);//pitch
}


float ncq_speed_mapping(float input,uint16_t input_max,float output_max)
{
  float output_speed=0;
  float temp_scale=(float)(input/input_max);
  temp_scale=constrain_float(temp_scale,-1.0f, 1.0f);
  if(temp_scale>=0) output_speed=(float)(output_max*temp_scale*temp_scale);
  else output_speed=(float)(-output_max*temp_scale*temp_scale); 
  return output_speed;
}

Vector2f accel_target={0},angle_target={0};
void ncq_control_poshold()
{
  static uint16 position_cnt=0;//高度速度控制计数器
  static uint16 speed_cnt=0;//高度速度控制计数器
  static uint8_t miss_fixed_flag=0;
	static systime poshold_dt;
	if(GPS_ok()==FALSE)//不满足定点条件，控制量给水平姿态
  {
    /********对GPS定点模式位置0，直接进入姿态模式，等待GPS信号再次满足条件时，***********
    *********自动切换至GPS定点模式，结合Controler_Mode_Select函数理解运行过程**********/
    //在控制模式里面自检是否允许再次进入GPS定点模式
		Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];
		Flight.roll_outer_control_output=RC_Data.rc_rpyt[RC_ROLL];
		miss_fixed_flag=1;		
		return ;		
  }
	
	Get_Systime(&poshold_dt);
	//如果本函数运行周期大于控制周期10倍
	if(0.001f*poshold_dt.period>=20*WP_Duty_Dt)
	{
		//情况1:初次从其它模式切入本模式
	  //情况2:系统调度超时，在系统设计合理情况下，本情况不可能发生
    miss_fixed_flag=1;
	}	
	
	if(miss_fixed_flag==1)//之前未满足定位条件,未锁定目标点
	{
		miss_fixed_flag=0;
		Total_Controller.Latitude_Position_Control.Expect=NamelessQuad.Position[_NORTH];
		Total_Controller.Longitude_Position_Control.Expect=NamelessQuad.Position[_EAST];
		Total_Controller.Latitude_Speed_Control.Expect =0;
		Total_Controller.Longitude_Speed_Control.Expect=0;
		
		East_North_Ctrl_Reset();
	}	
	
  /*******************************水平位置控制器开始***********************************************************/
  //技术讲解见博客：四旋翼GPS定点流程梳理与原理浅析 http://blog.csdn.net/u011992534/article/details/79408187
    if(Roll_Control==0
       &&Pitch_Control==0)//无水平遥控量给定
    {
      position_cnt++;
      if(position_cnt>8)//20ms控制一次
      {
        //位置期望,经纬、航行速度、高度
        if(Total_Controller.Latitude_Position_Control.Expect==0
           &&Total_Controller.Longitude_Position_Control.Expect==0)//方向杆回中后，只设置一次
        {
          if(get_stopping_point_xy(&UAV_Cushion_Stop_Point)==1)
          {
            Total_Controller.Latitude_Position_Control.Expect=UAV_Cushion_Stop_Point.y;
            Total_Controller.Longitude_Position_Control.Expect=UAV_Cushion_Stop_Point.x;
          }
          else//只采用速度刹车
          {
            //速度控制器期望
            Total_Controller.Latitude_Speed_Control.Expect =0;
            Total_Controller.Longitude_Speed_Control.Expect=0;  
          }
        }
        else
        {
          //位置反馈，来源于当前惯导的位置估计
          Total_Controller.Latitude_Position_Control.FeedBack=NamelessQuad.Position[_NORTH];
          Total_Controller.Longitude_Position_Control.FeedBack=NamelessQuad.Position[_EAST];
          //导航坐标系下E、N方向上位置偏差
          Earth_Frame_Pos_Err.N=Total_Controller.Latitude_Position_Control.Expect-Total_Controller.Latitude_Position_Control.FeedBack;
          Earth_Frame_Pos_Err.E=Total_Controller.Longitude_Position_Control.Expect-Total_Controller.Longitude_Position_Control.FeedBack;
          //导航坐标系下机体Pitch、Roll方向上位置偏差
          Body_Frame_Pos_Err.Pit=-Earth_Frame_Pos_Err.E*WP_AHRS.sin_rpy[_YAW]+Earth_Frame_Pos_Err.N*WP_AHRS.cos_rpy[_YAW];
          Body_Frame_Pos_Err.Rol=Earth_Frame_Pos_Err.E*WP_AHRS.cos_rpy[_YAW]+Earth_Frame_Pos_Err.N*WP_AHRS.sin_rpy[_YAW];
          //导航坐标系下机体Pitch、Roll方向上期望刹车速度，这里为单比例运算不调用PID_Control()函数
          Body_Frame_Pos_Err.Pit=constrain_float(Body_Frame_Pos_Err.Pit,-Total_Controller.Latitude_Position_Control.Err_Max, Total_Controller.Latitude_Position_Control.Err_Max);//位置偏差限幅，单位cm
          Body_Frame_Pos_Err.Rol=constrain_float(Body_Frame_Pos_Err.Rol,-Total_Controller.Longitude_Position_Control.Err_Max,Total_Controller.Longitude_Position_Control.Err_Max);//位置偏差限幅，单位cm
          
          Body_Frame_Brake_Speed.Pit=Total_Controller.Latitude_Position_Control.Kp*Body_Frame_Pos_Err.Pit;
          Body_Frame_Brake_Speed.Rol=Total_Controller.Longitude_Position_Control.Kp*Body_Frame_Pos_Err.Rol;
          //速度控制器期望
          Total_Controller.Latitude_Speed_Control.Expect =Body_Frame_Brake_Speed.Pit;
          Total_Controller.Longitude_Speed_Control.Expect=Body_Frame_Brake_Speed.Rol;  
        }
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
				Flight.pitch_outer_control_output=angle_target.y;;	
				
        speed_cnt=0;
      }
      /*******************************水平位置控制器结束***********************************************************/
    }
    else //只进行水平速度控制，无水平位置控制
    {
      //分两种情况，1、导航坐标系的航向速度控制；
      //            2、载体坐标系方向上的速度控制
      if(GPS_Speed_Control_Mode==Angle_Mode)//推动方向杆，对应期望角度
      {
				Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
				Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];	
      }
      else//推动方向杆，对应给定载体坐标系的沿Pitch,Roll方向运动速度
      {
        speed_cnt++;
        if(speed_cnt>=4)//10ms控制一次速度
        {
          Total_Controller.Latitude_Speed_Control.Expect =ncq_speed_mapping(-RC_Data.rc_rpyt[RC_PITCH],Pit_Rol_Max,Max_Horvel);
          Total_Controller.Longitude_Speed_Control.Expect=ncq_speed_mapping(RC_Data.rc_rpyt[RC_ROLL],Pit_Rol_Max,Max_Horvel);
													
          //导航系的水平速度，转化到机体坐标系X-Y方向上
          //沿载体Pitch、Roll方向水平速度控制
          Body_Frame_Speed_Feedback.Pit=-NamelessQuad.Speed[_EAST]*WP_AHRS.sin_rpy[_YAW]+NamelessQuad.Speed[_NORTH]*WP_AHRS.cos_rpy[_YAW];
          Body_Frame_Speed_Feedback.Rol= NamelessQuad.Speed[_EAST]*WP_AHRS.cos_rpy[_YAW]+NamelessQuad.Speed[_NORTH]*WP_AHRS.sin_rpy[_YAW];
          
          Total_Controller.Latitude_Speed_Control.FeedBack=Body_Frame_Speed_Feedback.Pit;//机头Pitch方向，Y轴正向
          Total_Controller.Longitude_Speed_Control.FeedBack=Body_Frame_Speed_Feedback.Rol;//横滚Roll方向，X轴正向
          
          PID_Control_Div_LPF(&Total_Controller.Latitude_Speed_Control,0.02f);
          PID_Control_Div_LPF(&Total_Controller.Longitude_Speed_Control,0.02f);//水平速度控制器得到期望运动加速度
          
          
          accel_target.y=-Total_Controller.Latitude_Speed_Control.Control_OutPut;
          accel_target.x= Total_Controller.Longitude_Speed_Control.Control_OutPut;//期望运动加速度
          ENU_Desired_Accel_Transform_Angle(accel_target,&angle_target);//期望运动加速度转期望姿态倾角
          Flight.pitch_outer_control_output=angle_target.y;
          Flight.roll_outer_control_output=angle_target.x;           
          speed_cnt=0;
        }
      }
      
      Total_Controller.Latitude_Position_Control.Expect=0;
      Total_Controller.Longitude_Position_Control.Expect=0;
    }
}



//定点模式下，遥杆回中后，先用水平速度控制刹车，待刹停后再赋值位置选点
uint8_t get_stopping_point_xy(Vector3f *stopping_point)
{
  Vector2f curr_pos,curr_vel,curr_accel;
  float vel_total=0,accel_total=0;   
  curr_pos.x=NamelessQuad.Position[_EAST];
  curr_pos.y=NamelessQuad.Position[_NORTH];
  curr_vel.x=NamelessQuad.Speed[_EAST];
  curr_vel.y=NamelessQuad.Speed[_NORTH];   
  curr_accel.x=NamelessQuad.Acceleration[_EAST];
  curr_accel.y=NamelessQuad.Acceleration[_NORTH];
  
  vel_total=pythagorous2(curr_vel.x,curr_vel.y);
  accel_total=pythagorous2(curr_accel.x,curr_accel.y);
  
  if(vel_total <= 20.0f //合水平速度的小于等于20cm/s
     && accel_total<=40 //合水平加速度的小于等于40cm/s^2
     &&WP_AHRS.rMat[8]>=0.97f)//WP_AHRS.cos_rpy[_PIT]*WP_AHRS.cos_rpy[_ROL]、单个方向水平姿态约为15deg，两个方向水平姿态角约为10deg  
  {
    stopping_point->x = curr_pos.x;
    stopping_point->y = curr_pos.y;
    return 1;
  }
  return 0;
}




bool slam_ok()// returns true if the GPS is ok and home position is set
{
  if(current_state.fault!=1)
  {
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}



void slam_control_poshold(SINS_Lite *_ins)
{
  static uint16 position_cnt=0;//高度速度控制计数器
  static uint16 speed_cnt=0;//高度速度控制计数器
  static uint8_t miss_fixed_flag=0;
	static systime poshold_dt;
	if(slam_ok()==FALSE)//不满足定点条件，控制量给水平姿态
  {
    /********对GPS定点模式位置0，直接进入姿态模式，等待GPS信号再次满足条件时，***********
    *********自动切换至GPS定点模式，结合Controler_Mode_Select函数理解运行过程**********/
    //在控制模式里面自检是否允许再次进入GPS定点模式
		Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];
		Flight.roll_outer_control_output=RC_Data.rc_rpyt[RC_ROLL];
		miss_fixed_flag=1;		
		return ;		
  }
	
	Get_Systime(&poshold_dt);
	//如果本函数运行周期大于控制周期10倍
	if(0.001f*poshold_dt.period>=20*WP_Duty_Dt)
	{
		//情况1:初次从其它模式切入本模式
	  //情况2:系统调度超时，在系统设计合理情况下，本情况不可能发生
    miss_fixed_flag=1;
	}	
	
	if(miss_fixed_flag==1)//之前未满足定位条件,未锁定目标点
	{
		miss_fixed_flag=0;
		Total_Controller.Latitude_Position_Control.Expect=_ins->Position[_NORTH];
		Total_Controller.Longitude_Position_Control.Expect=_ins->Position[_EAST];

		Total_Controller.Latitude_Speed_Control.Expect =0;
		Total_Controller.Longitude_Speed_Control.Expect=0;	
		East_North_Ctrl_Reset();		
	}	
	
  /*******************************水平位置控制器开始***********************************************************/
  //技术讲解见博客：四旋翼GPS定点流程梳理与原理浅析 http://blog.csdn.net/u011992534/article/details/79408187
    if(Roll_Control==0
       &&Pitch_Control==0)//无水平遥控量给定
    {
      position_cnt++;
      if(position_cnt>8)//20ms控制一次
      {
        //位置期望,经纬、航行速度、高度
        if(Total_Controller.Latitude_Position_Control.Expect==0
           &&Total_Controller.Longitude_Position_Control.Expect==0)//方向杆回中后，只设置一次
        {
					Total_Controller.Latitude_Position_Control.Expect=_ins->Position[_NORTH];
					Total_Controller.Longitude_Position_Control.Expect=_ins->Position[_EAST];
        }
        else
        {
          //位置反馈，来源于当前惯导的位置估计
          Total_Controller.Latitude_Position_Control.FeedBack=_ins->Position[_NORTH];
          Total_Controller.Longitude_Position_Control.FeedBack=_ins->Position[_EAST];
          //导航坐标系下E、N方向上位置偏差
          Earth_Frame_Pos_Err.N=Total_Controller.Latitude_Position_Control.Expect-Total_Controller.Latitude_Position_Control.FeedBack;
          Earth_Frame_Pos_Err.E=Total_Controller.Longitude_Position_Control.Expect-Total_Controller.Longitude_Position_Control.FeedBack;
          //导航坐标系下机体Pitch、Roll方向上位置偏差
          Body_Frame_Pos_Err.Pit=-Earth_Frame_Pos_Err.E*WP_AHRS.sin_rpy[_YAW]+Earth_Frame_Pos_Err.N*WP_AHRS.cos_rpy[_YAW];
          Body_Frame_Pos_Err.Rol=Earth_Frame_Pos_Err.E*WP_AHRS.cos_rpy[_YAW]+Earth_Frame_Pos_Err.N*WP_AHRS.sin_rpy[_YAW];
          //导航坐标系下机体Pitch、Roll方向上期望刹车速度，这里为单比例运算不调用PID_Control()函数
          Body_Frame_Pos_Err.Pit=constrain_float(Body_Frame_Pos_Err.Pit,-Total_Controller.Latitude_Position_Control.Err_Max, Total_Controller.Latitude_Position_Control.Err_Max);//位置偏差限幅，单位cm
          Body_Frame_Pos_Err.Rol=constrain_float(Body_Frame_Pos_Err.Rol,-Total_Controller.Longitude_Position_Control.Err_Max,Total_Controller.Longitude_Position_Control.Err_Max);//位置偏差限幅，单位cm
          
          Body_Frame_Brake_Speed.Pit=Total_Controller.Latitude_Position_Control.Kp*Body_Frame_Pos_Err.Pit;
          Body_Frame_Brake_Speed.Rol=Total_Controller.Longitude_Position_Control.Kp*Body_Frame_Pos_Err.Rol;
          //速度控制器期望
          Total_Controller.Latitude_Speed_Control.Expect =Body_Frame_Brake_Speed.Pit;
          Total_Controller.Longitude_Speed_Control.Expect=Body_Frame_Brake_Speed.Rol;  
        }
        position_cnt=0;//位置控制器结束，给出期望刹车速度
      }
      //导航系的水平速度，转化到机体坐标系X-Y方向上
      //沿载体Pitch、Roll方向水平速度控制
      speed_cnt++;
      if(speed_cnt>=4)//10ms控制一次位置
      {
        Body_Frame_Speed_Feedback.Pit=-_ins->Speed[_EAST]*WP_AHRS.sin_rpy[_YAW]+_ins->Speed[_NORTH]*WP_AHRS.cos_rpy[_YAW];
        Body_Frame_Speed_Feedback.Rol= _ins->Speed[_EAST]*WP_AHRS.cos_rpy[_YAW]+_ins->Speed[_NORTH]*WP_AHRS.sin_rpy[_YAW];
        //沿载体方向速度反馈量
        Total_Controller.Latitude_Speed_Control.FeedBack =Body_Frame_Speed_Feedback.Pit;//机头Pitch方向，Y轴正向
        Total_Controller.Longitude_Speed_Control.FeedBack=Body_Frame_Speed_Feedback.Rol;//横滚Roll方向，X轴正向
        //沿载体方向速度控制器
        PID_Control_Div_LPF(&Total_Controller.Latitude_Speed_Control,0.02f);
        PID_Control_Div_LPF(&Total_Controller.Longitude_Speed_Control,0.02f);
        
        accel_target.y=-Total_Controller.Latitude_Speed_Control.Control_OutPut;
        accel_target.x= Total_Controller.Longitude_Speed_Control.Control_OutPut;//期望运动加速度
        ENU_Desired_Accel_Transform_Angle(accel_target,&angle_target);//期望运动加速度转期望姿态倾角
				Flight.roll_outer_control_output =angle_target.x;
				Flight.pitch_outer_control_output=angle_target.y;;			
        speed_cnt=0;
      }
      /*******************************水平位置控制器结束***********************************************************/
    }
    else //只进行水平速度控制，无水平位置控制
    {	    
			//推动方向杆，对应给定载体坐标系的沿Pitch,Roll方向运动速度					
			Total_Controller.Latitude_Speed_Control.Expect =ncq_speed_mapping(-RC_Data.rc_rpyt[RC_PITCH],Pit_Rol_Max,Max_Horvel);
			Total_Controller.Longitude_Speed_Control.Expect=ncq_speed_mapping(RC_Data.rc_rpyt[RC_ROLL],Pit_Rol_Max,Max_Horvel);
			speed_cnt++;
			if(speed_cnt>=4)//10ms控制一次速度
			{							
				//导航系的水平速度，转化到机体坐标系X-Y方向上
				//沿载体Pitch、Roll方向水平速度控制
				Body_Frame_Speed_Feedback.Pit=-_ins->Speed[_EAST]*WP_AHRS.sin_rpy[_YAW]+_ins->Speed[_NORTH]*WP_AHRS.cos_rpy[_YAW];
				Body_Frame_Speed_Feedback.Rol= _ins->Speed[_EAST]*WP_AHRS.cos_rpy[_YAW]+_ins->Speed[_NORTH]*WP_AHRS.sin_rpy[_YAW];
				
				Total_Controller.Latitude_Speed_Control.FeedBack=Body_Frame_Speed_Feedback.Pit;//机头Pitch方向，Y轴正向
				Total_Controller.Longitude_Speed_Control.FeedBack=Body_Frame_Speed_Feedback.Rol;//横滚Roll方向，X轴正向
				
				PID_Control_Div_LPF(&Total_Controller.Latitude_Speed_Control,0.02f);
				PID_Control_Div_LPF(&Total_Controller.Longitude_Speed_Control,0.02f);//水平速度控制器得到期望运动加速度
						 
				accel_target.y=-Total_Controller.Latitude_Speed_Control.Control_OutPut;
				accel_target.x= Total_Controller.Longitude_Speed_Control.Control_OutPut;//期望运动加速度
				ENU_Desired_Accel_Transform_Angle(accel_target,&angle_target);//期望运动加速度转期望姿态倾角
				Flight.pitch_outer_control_output=angle_target.y;
				Flight.roll_outer_control_output=angle_target.x;           
				speed_cnt=0;
			}		
      Total_Controller.Latitude_Position_Control.Expect=0;
      Total_Controller.Longitude_Position_Control.Expect=0;
    }
}

/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/


