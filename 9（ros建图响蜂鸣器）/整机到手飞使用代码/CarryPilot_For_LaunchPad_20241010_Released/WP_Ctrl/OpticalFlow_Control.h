#ifndef _OPTICALFLOW_CONTROL_H
#define _OPTICALFLOW_CONTROL_H


#define OpticalFlow_Speed_Control_Max  250


void OpticalFlow_Control_Pure(uint8_t force_brake_flag);
void OpticalFlow_Control(uint8_t force_brake_flag);
void OpticalFlow_SINS_Reset(void);
void OpticalFlow_Ctrl_Reset(void);
void OpticalFlow_Vel_Control(Vector2f target);
void OpticalFlow_Pos_Control(void);
void OpticalFlow_Y_Vel_Control(float target_y);
void OpticalFlow_X_Vel_Control(float target_x);


void Color_Block_Control_Pilot(void);
void Top_APrilTag_Control_Pilot(void);
void Self_Track_Control_Pilot(void);
void Front_AprilTag_Control_Pilot(void);
void Front_Surround_Pole_Control_Pilot(void);



void Horizontal_Navigation(float x,float y,float z,uint8_t nav_mode,uint8_t frame_id);
	

extern Vector2f OpticalFlow_Pos_Ctrl_Expect;
extern Vector2f OpticalFlow_Pos_Ctrl_Err;
extern Vector2f OpticalFlow_Pos_Ctrl_Integrate;
extern Vector2f OpticalFlow_Pos_Ctrl_Output;




#endif















//SDK接收模式
//  {
//    if(Roll_Control==0&&Pitch_Control==0)//无水平遥控量给定
//    {  
//      if(Opv_Top_View_Target.line_ctrl_enable==1)//线检测
//      {
//        /***********当只需要速度控制时，开启以下注释，仅限调试时用*************/
//        sdk_ctrl_cnt++;
//        if(sdk_ctrl_cnt>=4)//20ms
//        {
//          Total_Controller.SDK_Roll_Position_Control.Expect=0;
//          Total_Controller.SDK_Roll_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.x;
//          PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Roll_Position_Control,1);   
//          accel_target.x=constrain_float(Total_Controller.SDK_Roll_Position_Control.Control_OutPut,
//                                         -Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,
//                                         Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);                             
//          Controller.roll_outer_control_output=constrain_float(fast_atan(accel_target.x/(GRAVITY_MSS*100))*RAD2DEG,-30,30);//roll                                
//          sdk_ctrl_cnt=0;
//        }
//				
//        if(ABS(SDK_Target_Yaw_Gyro)<=10) OpticalFlow_Y_Vel_Control(5);
//        else if(ABS(SDK_Target_Yaw_Gyro)<=20) OpticalFlow_Y_Vel_Control(3);
//        else if(ABS(SDK_Target_Yaw_Gyro)<=50) OpticalFlow_Y_Vel_Control(2);
//        else OpticalFlow_Y_Vel_Control(1);
//        //OpticalFlow_Y_Vel_Control(0);
//        OpticalFlow_Pos_Ctrl_Expect.x=0;
//        OpticalFlow_Pos_Ctrl_Expect.y=0;
//        force_brake_flag=1;
//      }
//      else if(Opv_Top_View_Target.target_ctrl_enable==1)//目标点检测跟踪
//      {
//        sdk_ctrl_cnt++;
//        if(sdk_ctrl_cnt>=10)//50ms
//        {				
//					Opv_Top_View_Target.target_ctrl_enable=0;
//					Total_Controller.SDK_Roll_Position_Control.Expect=0;
//					Total_Controller.SDK_Roll_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.x;
//					PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Roll_Position_Control,Opv_Top_View_Target.trust_flag);
//					
//					Total_Controller.SDK_Pitch_Position_Control.Expect=0;
//					Total_Controller.SDK_Pitch_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.y;
//					PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Pitch_Position_Control,Opv_Top_View_Target.trust_flag);
//					
//					accel_target.x=constrain_float(Total_Controller.SDK_Roll_Position_Control.Control_OutPut,
//																				 -Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,
//																				 Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);                             
//					Controller.roll_outer_control_output=constrain_float(fast_atan(accel_target.x/(GRAVITY_MSS*100))*RAD2DEG,-30,30);//roll
//					
//					
//					accel_target.y=constrain_float(Total_Controller.SDK_Pitch_Position_Control.Control_OutPut,
//																				 -Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,
//																				 Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);//450
//					Controller.pitch_outer_control_output=constrain_float(fast_atan(accel_target.y*WP_AHRS.cos_rpy[_ROL]/(GRAVITY_MSS*100))*RAD2DEG,-30,30);//pitch 
//					sdk_ctrl_cnt=0;
//        }				
//        OpticalFlow_Pos_Ctrl_Expect.x=0;
//        OpticalFlow_Pos_Ctrl_Expect.y=0;
//        force_brake_flag=1;
//      }
//      else//SDK模式下，未检测对于特征
//      {
//        /**************************光流位置控制器************************************/
//        if(OpticalFlow_Pos_Ctrl_Expect.x==0
//           &&OpticalFlow_Pos_Ctrl_Expect.y==0)
//        {
//          if(force_brake_flag==1||(rMat[2][2]>=0.95f
//                                   &&pythagorous2(OpticalFlow_SINS.Speed[_EAST],OpticalFlow_SINS.Speed[_NORTH])<=40))//打杆回中后，根据当前速度、倾角判断是否进行悬停
//          {
//            OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_EAST];
//            OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_NORTH];
//          }
//          else  //打杆回中未满足悬停条件时，只进行速度控制 
//          {
//            OpticalFlow_Pos_Ctrl_Output.x=0;
//            OpticalFlow_Pos_Ctrl_Output.y=0;
//          }
//        }
//        else  OpticalFlow_Pos_Control();
//        /**************************基于模型的加速度-姿态角映射，相比直接给姿态，参数差异大概在20倍左右************************************/
//        OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度期望        
//        PID_LPF_Reset(&Total_Controller.SDK_Roll_Position_Control,SDK_Roll_Position_Controler);
//        PID_LPF_Reset(&Total_Controller.SDK_Pitch_Position_Control,SDK_Pitch_Position_Controler);
//				SDK_Pos_Ctrl_Reset();
//			}
//    }
//    else//SDK模式下存在手动打杆操作时，控制量来源于遥控器给定
//    {
//			Controller.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
//			Controller.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];		
//      OpticalFlow_Pos_Ctrl_Expect.x=0;
//      OpticalFlow_Pos_Ctrl_Expect.y=0;
//      force_brake_flag=1;
//			PID_LPF_Reset(&Total_Controller.SDK_Roll_Position_Control,SDK_Roll_Position_Controler);
//			PID_LPF_Reset(&Total_Controller.SDK_Pitch_Position_Control,SDK_Pitch_Position_Controler);
//			SDK_Pos_Ctrl_Reset();
//    }  
//    OpticalFlow_Pos_Ctrl_Expect.x=0;
//    OpticalFlow_Pos_Ctrl_Expect.y=0;
//  }
/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/

//			if(ABS(Opv_Front_View_Target.sdk_target.x)<5.0f)//机头已经对准塔杆，可进行下一步调整飞机与塔杆的距离
//			{
//				gap_miss_cnt=1;
//				//第二优先级：当前后方向存在偏移时，调整俯仰方向位置，使得飞机与塔杆保持一定距离
//				Total_Controller.SDK_Pitch_Position_Control.Expect=Pole_Keep_Distance;//期望为前向保持距离，具体可根据实际自行定义
//				Total_Controller.SDK_Pitch_Position_Control.FeedBack=front_tofsense_distance;
//				PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Pitch_Position_Control,Opv_Front_View_Target.trust_flag);
//				OpticalFlow_Pos_Ctrl_Output.y=-Total_Controller.SDK_Pitch_Position_Control.Control_OutPut;				
//				if(ABS(Total_Controller.SDK_Pitch_Position_Control.Err)<5.0f)//机头已经对准塔杆，切飞机与塔杆的距离已经调整完毕，可以执行横向速度控制
//				{
//					//第三优先级：当左右方向存在偏移误差小，并且距离塔杆距离误差小时，移动横滚方向速度，实现绕杆飞行
//					OpticalFlow_Pos_Ctrl_Output.x=-5;//机头右侧方向速度为负，逆时针转动				
//				}
//				else//机头已经对准塔杆，但是飞机与塔杆之间距离尚未调整完毕，等待距离调整完毕再执行横向速度控制
//				{
//					OpticalFlow_Pos_Ctrl_Output.x=0;//横向位置保持
//				}
//			}
//			else//视野能识别到塔杆，但是机头还未对准塔杆时，原地悬停，等待机头对准后再调整前后距离和绕杆飞行
//			{
//				miss_flag=2;
//			}












