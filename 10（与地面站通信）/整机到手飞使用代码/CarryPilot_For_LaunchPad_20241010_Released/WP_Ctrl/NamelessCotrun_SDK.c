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
#include "NamelessCotrun_SDK.h"

bool auto_altland(float taret_climb_rate,float target_climb_alt)
{
  return land_althold(taret_climb_rate,target_climb_alt);
}
//SDK本模式需要在光流模式下使用
uint8_t move_with_speed_target(float x_target,float y_target,float delta,SDK_Status *Status,uint16_t number)
{
  static float end_time=0;
  Vector2f vel_target;
  Testime dt;
  vel_target.x=x_target;
  vel_target.y=y_target;
  Test_Period(&dt);
  ncq_control_althold();//高度控制依然进行
  if(Status->Status[number].Start_Flag==1
     &&Status->Status[number].Execute_Flag==1
       &&Status->Status[number].End_flag==1)
  {
    OpticalFlow_Control_Pure(0);//完成之后，进行光流悬停
    return 1;
  }
  else
  {
    if(Status->Status[number].Start_Flag==0) 
    {
      end_time=dt.Now_Time+delta;//单位ms 
      Status->Status[number].Start_Flag=1;
    } 
    if(dt.Now_Time>end_time)
    {
      Status->Status[number].Execute_Flag=1;
      Status->Status[number].End_flag=1;
      
      OpticalFlow_Pos_Ctrl_Expect.x=0;
      OpticalFlow_Pos_Ctrl_Expect.y=0;
			OpticalFlow_Control_Pure(1);//完成之后，进行光流悬停
			
      end_time=0;
      Status->Transition_Time[number]=400;//400*5ms=2s
      return 1;//返回完成
    }
    else
    { 
      OpticalFlow_Pos_Ctrl_Expect.x=0;
      OpticalFlow_Pos_Ctrl_Expect.y=0;
      Status->Status[number].Execute_Flag=1;
      OpticalFlow_Vel_Control(vel_target);//给定速度期望
      return 0;
    }
  }
}


uint8_t move_with_xy_target(float pos_x_target,float pos_y_target,SDK_Status *Status,uint16_t number)
{
  ncq_control_althold();//高度控制依然进行
  if(Status->Status[number].Start_Flag==0) 
  {
    OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_EAST]+pos_x_target;
    OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_NORTH]+pos_y_target;
    Status->Status[number].Start_Flag=1;
  }
  
  if(Status->Status[number].Start_Flag==1
     &&Status->Status[number].Execute_Flag==1
       &&Status->Status[number].End_flag==1)
  {
    OpticalFlow_Control_Pure(0);//完成之后，进行光流悬停
    return 1;
  }
  else
  {    
    if(pythagorous2(OpticalFlow_Pos_Ctrl_Expect.x-OpticalFlow_SINS.Position[_EAST],
                    OpticalFlow_Pos_Ctrl_Expect.y-OpticalFlow_SINS.Position[_NORTH])<=10.0f)
    {
      Status->Status[number].Execute_Flag=1;
      Status->Status[number].End_flag=1;
			Status->Transition_Time[number]=200;
			
      OpticalFlow_Pos_Ctrl_Expect.x=0;
      OpticalFlow_Pos_Ctrl_Expect.y=0;
      OpticalFlow_Control_Pure(1);//完成之后，进行光流悬停
      return 1;
    }
    else
    { 
      Status->Status[number].Execute_Flag=1; 
      OpticalFlow_Pos_Control();//光流位置控制
      OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//光流速度控制 
      return 0;
    }
  }
}

uint8_t move_with_target(float pos_x_target,float pos_y_target,Duty_Status *Status,uint8_t *Start_flag)
{
  //static Vector2f pos_base;
  ncq_control_althold();//高度控制依然进行
  if(*Start_flag==1)
  {
    //pos_base.x=OpticalFlow_SINS.Position[_EAST];
    //pos_base.y=OpticalFlow_SINS.Position[_NORTH];
    *Start_flag=0;
  }
  
  if(Status->Start_Flag==0) 
  {
    OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_EAST]+pos_x_target;
    OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_NORTH]+pos_y_target;
    Status->Start_Flag=1;
  }
  
  if(Status->Start_Flag==1
     &&Status->Execute_Flag==1
       &&Status->End_flag==1)
  {
    OpticalFlow_Control_Pure(0);//完成之后，进行光流悬停
    return 1;
  }
  else
  {    
    if(pythagorous2(OpticalFlow_Pos_Ctrl_Expect.x-OpticalFlow_SINS.Position[_EAST],
                    OpticalFlow_Pos_Ctrl_Expect.y-OpticalFlow_SINS.Position[_NORTH])<=8.0f)
    {
      Status->Execute_Flag=1;
      Status->End_flag=1;
      //pos_base.x=0;
      //pos_base.y=0;
      
      OpticalFlow_Pos_Ctrl_Expect.x=0;
      OpticalFlow_Pos_Ctrl_Expect.y=0;
			OpticalFlow_Control_Pure(1);//完成之后，进行光流悬停
			
      return 1;
    }
    else
    { 
      Status->Execute_Flag=1; 
      OpticalFlow_Pos_Control();//光流位置控制
      OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//光流速度控制 
      return 0;
    }
  }
}


uint8_t move_with_z_target(float z_target,float z_vel,float delta,SDK_Status *Status,uint16_t number)
{
  static float target_rate=0;
  static float target_alt=0;
  static uint8_t end_flag=0;
  //static float z_base;
  static float end_time=0;
  Testime dt;
  Test_Period(&dt);
  OpticalFlow_Control_Pure(0);//水平位置控制依然进行
  
  if(Status->Status[number].Start_Flag==1
     &&Status->Status[number].Execute_Flag==1
       &&Status->Status[number].End_flag==1)
  {   
    ncq_control_althold();
    return 1;
  }
  else
  {
    if(Status->Status[number].Start_Flag==0) 
    {  
      if(z_target==0)//速度控制，一次任务中只设置一次
      {
        //z_base=NamelessQuad.Position[_UP];
        target_rate=z_vel;
        target_alt=0;
        end_flag=1;
      }
      else//位置控制，一次任务中只设置一次
      {
        //z_base=NamelessQuad.Position[_UP];
        target_rate=0;
        target_alt=NamelessQuad.Position[_UP]+z_target;
        end_flag=2;
      }
      end_time=dt.Now_Time+delta;//单位ms 
      Status->Status[number].Start_Flag=1;
      
      Unwanted_Lock_Flag=0;//允许自动上锁
      //OpticalFlow_Pos_Ctrl_Expect.x=0;
      //OpticalFlow_Pos_Ctrl_Expect.y=0;
    }
    
    
    if(end_flag==1)
    {  
      auto_altland(target_rate,target_alt);
      if(dt.Now_Time>end_time)
      { 
        end_flag=0;
        target_rate=0;
        target_alt=0;
        //z_base=0;
        end_time=0;
        Status->Status[number].Execute_Flag=1;
        Status->Status[number].End_flag=1;
        Status->Transition_Time[number]=200;
        
        OpticalFlow_Pos_Ctrl_Expect.x=0;
        OpticalFlow_Pos_Ctrl_Expect.y=0;
        OpticalFlow_Control_Pure(1);//完成之后，进行光流悬停
				
        Total_Controller.Height_Position_Control.Expect=0;
        return 1;//返回完成
      }
      Status->Status[number].Execute_Flag=1;
    }
    else if(end_flag==2)
    {
      auto_altland(target_rate,target_alt);    
      if(ABS(target_alt-NamelessQuad.Position[_UP])<=5.0f)
      {
        end_flag=0;
        target_rate=0;
        target_alt=0;
        //z_base=0;
        end_time=0;
        Status->Status[number].Execute_Flag=1;
        Status->Status[number].End_flag=1;
        Status->Transition_Time[number]=200;
        
        OpticalFlow_Pos_Ctrl_Expect.x=0;
        OpticalFlow_Pos_Ctrl_Expect.y=0;
				OpticalFlow_Control_Pure(1);//完成之后，进行光流悬停
        
        Total_Controller.Height_Position_Control.Expect=0;
        return 1;
      }
      Status->Status[number].Execute_Flag=1;
    } 
  }
  return 0;
}

//#define NCQ_SDK_DUTY1 move_with_speed_target(10,0,2000 ,&SDK_Duty_Status,1-1)//左
//#define NCQ_SDK_DUTY2 move_with_speed_target(0,10,2000 ,&SDK_Duty_Status,2-1)//前
//#define NCQ_SDK_DUTY3 move_with_speed_target(-10,0,2000,&SDK_Duty_Status,3-1)//右
//#define NCQ_SDK_DUTY4 move_with_speed_target(0,-10,2000,&SDK_Duty_Status,4-1)//后


#define NCQ_SDK_DUTY_MAX   3
#define NCQ_SDK_DUTY1 move_with_z_target(120,0,0,&SDK_Duty_Status,1-1)
#define NCQ_SDK_DUTY2 move_with_xy_target(0,100,&SDK_Duty_Status,2-1)
#define NCQ_SDK_DUTY3 move_with_z_target(0,-50,1000*100,&SDK_Duty_Status,3-1)//(-150,0,0,&SDK_Duty_Status,3-1)


SDK_Status SDK_Duty_Status;
uint16_t SDK_Duty_Cnt=0;
uint16_t SDK_Transition_Time=0;
void NCQ_SDK_Run(void)
{
  if(SDK_Duty_Status.Transition_Time[SDK_Duty_Cnt]>=1) 
    SDK_Duty_Status.Transition_Time[SDK_Duty_Cnt]--;//安排过渡时间
  
  if(SDK_Duty_Status.Status[SDK_Duty_Cnt].Start_Flag==1
     &&SDK_Duty_Status.Status[SDK_Duty_Cnt].Execute_Flag==1
       &&SDK_Duty_Status.Status[SDK_Duty_Cnt].End_flag==1
         &&SDK_Duty_Status.Transition_Time[SDK_Duty_Cnt]==0)
    SDK_Duty_Cnt++;
  
  if(SDK_Duty_Cnt>=NCQ_SDK_DUTY_MAX) SDK_Duty_Cnt=NCQ_SDK_DUTY_MAX;
  
  if(SDK_Duty_Cnt==0)        NCQ_SDK_DUTY1;
  else if(SDK_Duty_Cnt==1)   NCQ_SDK_DUTY2;
  else if(SDK_Duty_Cnt==2)   NCQ_SDK_DUTY3;
  //else if(SDK_Duty_Cnt==3)   NCQ_SDK_DUTY4;
  //else if(SDK_Duty_Cnt==4)   NCQ_SDK_DUTY5;
  //else if(SDK_Duty_Cnt==5)   NCQ_SDK_DUTY6;
  //else if(SDK_Duty_Cnt==6)   NCQ_SDK_DUTY7;
  else
  {
    ncq_control_althold();//高度控制
    OpticalFlow_Control(0);//位置控制
  }
}


void NCQ_SDK_Reset(void)
{
  uint16_t i=0;
  for(i=0;i<SDK_Duty_Max;i++)
  {
    SDK_Duty_Status.Status[i].Start_Flag=0;
    SDK_Duty_Status.Status[i].Execute_Flag=0;
    SDK_Duty_Status.Status[i].End_flag=0;
  }
  SDK_Duty_Cnt=0;
}






unsigned char sdk_data_to_send[10];
void SDK_DT_Send_Data(unsigned char *dataToSend , unsigned char length)
{
  USART3_Send(sdk_data_to_send, length);
}

void SDK_DT_Send_Check(unsigned char mode,COM_SDK com)
{
  sdk_data_to_send[0]=0xFF;
  sdk_data_to_send[1]=0xFC;
  sdk_data_to_send[2]=0xA0;
  sdk_data_to_send[3]=2;
  sdk_data_to_send[4]=mode;
  sdk_data_to_send[5]=0
    ;
  u8 sum = 0;
  for(u8 i=0;i<6;i++) sum += sdk_data_to_send[i];
  sdk_data_to_send[6]=sum;
	if(com==UART3_SDK) USART3_Send(sdk_data_to_send, 7);
	if(com==UART0_SDK) USART0_Send(sdk_data_to_send, 7); 
}

void SDK_DT_Send_Check_Search_LandColor(unsigned char mode,uint8_t color,uint8_t shape,COM_SDK com)
{
  sdk_data_to_send[0]=0xFF;
  sdk_data_to_send[1]=0xFC;
  sdk_data_to_send[2]=0xA0;
  sdk_data_to_send[3]=4;
  sdk_data_to_send[4]=mode;
  sdk_data_to_send[5]=color;    //颜色
  sdk_data_to_send[6]=shape;   //形状
  sdk_data_to_send[7]=0;        //保留


  u8 sum = 0;
  for(u8 i=0;i<8;i++) sum += sdk_data_to_send[i];
  sdk_data_to_send[8]=sum;
	if(com==UART3_SDK) USART3_Send(sdk_data_to_send, 9);
	if(com==UART0_SDK) USART0_Send(sdk_data_to_send, 9); 
}









#define SDK_TARGET_X_OFFSET  0
#define SDK_TARGET_Y_OFFSET  0//-12


Target_Check Opv_Top_View_Target,Opv_Front_View_Target;
float SDK_Target_Yaw_Gyro=0;

/*************************************************************************/
#define  Pixel_Size_MV    0.0024f//6um=0.000006m=0.0006cm
                       //320---0.0012
                       //160---0.0024
                       //80 ---0.0048
#define  Focal_Length_MV  0.42f

#define OV7725_Sensor_Width_MM    		3.984f//3984um
#define OV7725_Sensor_Height_MM   		2.952f//2952um
#define Pixel_Image_Width_MV    		160//320
#define Pixel_Image_Height_MV   		120//240
#define Pixel_Image_Focal_MM_MV 		4.2f
#define Pixel_Image_View_Angle_X_MV  (56.72/2)//deg(50.75/2)
#define Pixel_Image_View_Angle_Y_MV  (44.07/2)//deg(38.72/2)
/*************************************************************************/
#define  Pixel_Size_CV    0.00056f//cm
													//2592:1944——0.00014cm
													//640:480——0.00056cm
#define  Focal_Length_CV  0.36f  //焦距3.6mm

#define OV5647_Sensor_Width_MM    		3.674f//3674um
#define OV5647_Sensor_Height_MM   		2.738f//2738.4um#
#define Pixel_Image_Width_CV    			640//640
#define Pixel_Image_Height_CV   			480//480
#define Pixel_Image_Focal_MM_CV 			3.6f
#define Pixel_Image_View_Angle_X_CV  (53.5/2)//约为66deg广角
#define Pixel_Image_View_Angle_Y_CV  (41.4/2)


#define AprilTag_Side_Length  13.6f//cm13.6

float _Pixel_Image_View_Angle_X,_Pixel_Image_View_Angle_Y;
void Get_Camera_Wide_Angle(float view_angle)
{
	float fh=5.0f/FastTan(0.5f*view_angle*DEG2RAD);
  _Pixel_Image_View_Angle_X=2*RAD2DEG*fast_atan(4/fh);
  _Pixel_Image_View_Angle_Y=2*RAD2DEG*fast_atan(3/fh);
}
	
float _P1=0,_P2=0;
uint16_t _CX=0,_CY=0;
float _TX=0,_TY=0;
float _DX=0,_DY=0;
void Sensor_Parameter_Sort(uint16_t tx,uint16_t ty,float pitch,float roll,float alt)
{
//	Get_Camera_Wide_Angle(68);//根据摄像头广角，计算得到X、Y轴视角
	float theta_x_max=0,theta_y_max=0;
	theta_x_max=Pixel_Image_View_Angle_X_MV;
	theta_y_max=Pixel_Image_View_Angle_Y_MV;
  _P1=0.5f*Pixel_Image_Width_MV/FastTan(theta_x_max*DEG2RAD);	
	_P2=0.5f*Pixel_Image_Height_MV/FastTan(theta_y_max*DEG2RAD);
	
	_CX=Pixel_Image_Width_MV/2;
	_CY=Pixel_Image_Height_MV/2;
	
	float tmp_x=0,tmp_y=0;
	tmp_x=fast_atan((_CX-tx)/_P1);
	tmp_y=fast_atan((_CY-ty)/_P2);
	
	_TX= FastTan(tmp_x+roll*DEG2RAD) *_P1;
	_TY= FastTan(tmp_y+pitch*DEG2RAD)*_P2;
	
//	_DX=alt*_TX/_P1;
//	_DY=alt*_TY/_P2;
	_DX=0.5f*alt*_TX/_P1;
	_DY=0.5f*alt*_TY/_P2;

	Opv_Top_View_Target.sdk_target.x=_DX;
    Opv_Top_View_Target.sdk_target.y=_DY;
}
















void SDK_DT_Reset()
{
  Opv_Top_View_Target.x=0;
  Opv_Top_View_Target.y=0; 
  Opv_Top_View_Target.pixel=0;
  Opv_Top_View_Target.flag=0;
	Opv_Top_View_Target.state=0;
	Opv_Top_View_Target.angle=0;
	Opv_Top_View_Target.distance=0;
	Opv_Top_View_Target.apriltag_id=0;
	Opv_Top_View_Target.width=0;
	Opv_Top_View_Target.height=0;
	Opv_Top_View_Target.fps=0;
	Opv_Top_View_Target.reserved1=0;
	Opv_Top_View_Target.reserved2=0;
	Opv_Top_View_Target.reserved3=0;
	Opv_Top_View_Target.reserved4=0;
	Opv_Top_View_Target.range_sensor1=0;
	Opv_Top_View_Target.range_sensor2=0;
	Opv_Top_View_Target.range_sensor3=0;
	Opv_Top_View_Target.range_sensor4=0;
	
	Opv_Front_View_Target.x=0;
  Opv_Front_View_Target.y=0; 
  Opv_Front_View_Target.pixel=0;
  Opv_Front_View_Target.flag=0;
	Opv_Front_View_Target.state=0;
	Opv_Front_View_Target.angle=0;
	Opv_Front_View_Target.distance=0;
	Opv_Front_View_Target.apriltag_id=0;
	Opv_Front_View_Target.width=0;
	Opv_Front_View_Target.height=0;
	Opv_Front_View_Target.fps=0;
	Opv_Front_View_Target.reserved1=0;
	Opv_Front_View_Target.reserved2=0;
	Opv_Front_View_Target.reserved3=0;
	Opv_Front_View_Target.reserved4=0;
	Opv_Front_View_Target.range_sensor1=0;
	Opv_Front_View_Target.range_sensor2=0;
	Opv_Front_View_Target.range_sensor3=0;
	Opv_Front_View_Target.range_sensor4=0;
}




uint16_t err_cnt[2]={0};
void Openmv_Data_Receive_Anl_1(uint8_t *data_buf,uint8_t num,Target_Check *target)
{
  uint8_t sum = 0;
  for(uint8_t i=0;i<(num-1);i++)  sum+=*(data_buf+i);
  if(!(sum==*(data_buf+num-1))) 	return;//不满足和校验条件
  if(!(*(data_buf)==0xFF && *(data_buf+1)==0xFC))return;//不满足帧头条件
	target->x				    =*(data_buf+4)<<8|*(data_buf+5);
    target->y					=*(data_buf+6)<<8|*(data_buf+7);
	target->pixel			=*(data_buf+8)<<8|*(data_buf+9);  
	target->flag			=*(data_buf+10);
	target->state		  =*(data_buf+11);		
	target->angle		  =*(data_buf+12)<<8|*(data_buf+13);
	target->distance  =*(data_buf+14)<<8|*(data_buf+15);
	target->apriltag_id=*(data_buf+16)<<8|*(data_buf+17);
	target->width	    =*(data_buf+18)<<8|*(data_buf+19);
	target->height	  =*(data_buf+20)<<8|*(data_buf+21);
	target->fps  		  =*(data_buf+22);
	target->reserved1 =*(data_buf+23);
	target->reserved2 =*(data_buf+24);
	target->reserved3 =*(data_buf+25);
	target->reserved4 =*(data_buf+26);
	//扩展距离传感器
	target->range_sensor1 =*(data_buf+27)<<8|*(data_buf+28);
	target->range_sensor2 =*(data_buf+29)<<8|*(data_buf+30);
	target->range_sensor3 =*(data_buf+31)<<8|*(data_buf+32);
	target->range_sensor4 =*(data_buf+33)<<8|*(data_buf+34);
	target->camera_id=*(data_buf+35);
	target->reserved1_int32=*(data_buf+36)<<24|*(data_buf+37)<<16|*(data_buf+38)<<8|*(data_buf+39);
	target->reserved2_int32=*(data_buf+40)<<24|*(data_buf+41)<<16|*(data_buf+42)<<8|*(data_buf+43);
	target->reserved3_int32=*(data_buf+44)<<24|*(data_buf+45)<<16|*(data_buf+46)<<8|*(data_buf+47);
	target->reserved4_int32=*(data_buf+48)<<24|*(data_buf+49)<<16|*(data_buf+50)<<8|*(data_buf+51);
	
	target->x_pixel_size=Pixel_Size_MV*(Pixel_Image_Width_MV/target->width);
	target->y_pixel_size=Pixel_Size_MV*(Pixel_Image_Height_MV/target->height);
  target->apriltag_distance=AprilTag_Side_Length*Focal_Length_MV/(target->x_pixel_size*FastSqrt(target->pixel));
  target->sdk_mode=*(data_buf+2);
	
	if(target->camera_id==0x01)//摄像头id为OPENMV
	{
		switch(target->sdk_mode)
		{
			case 0xA1://点检测
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<20)	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else target->trust_flag=1;
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;	
				Sensor_Parameter_Sort(target->x,target->y,WP_AHRS.Pitch,WP_AHRS.Roll,NamelessQuad.Position[_UP]);		
			}
			break;
			case 0xA2://AprilTag检测
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<20)	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else target->trust_flag=1;
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;		
				Sensor_Parameter_Sort(target->x,target->y,WP_AHRS.Pitch,WP_AHRS.Roll,NamelessQuad.Position[_UP]);		
			}
			break;
			case 0xA3://线检测
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<20)	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else target->trust_flag=1;
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;
				Sensor_Parameter_Sort(target->x,target->y,WP_AHRS.Pitch,WP_AHRS.Roll,NamelessQuad.Position[_UP]);
				if(target->angle>90) Opv_Top_View_Target.sdk_angle=target->angle-180;
				else Opv_Top_View_Target.sdk_angle=target->angle;			
			}
			break;
			case 0xA4://AprilTag检测
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<20)	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else target->trust_flag=1;
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;				
				Sensor_Parameter_Sort(target->x,target->y,WP_AHRS.Pitch,WP_AHRS.Roll,NamelessQuad.Position[_UP]);
			}
			break;			
			case 0xA7:
			case 0xAB://农作物检测
			{		
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<3)//连续三次检测到	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else 
					{
						target->trust_flag=1;//此标志作为判断底部是否为农作物的标志位

						//buzzer_setup(1000,0.2f,1);//蜂鸣器提示，飞行过程中飞控板载蜂鸣器提示会产生噪声导致飞行不稳定故关闭
						Bling_Set(&rgb_green,2000,1000,0.5,0,GPIO_PORTF_BASE,BLING_G_PIN,0);//RGB绿色指示灯提示
					}
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;	
				Sensor_Parameter_Sort(target->x,target->y,WP_AHRS.Pitch,WP_AHRS.Roll,NamelessQuad.Position[_UP]);		
			}
			break;
			case 0xB0://送货无人机——目标检测
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<5)//连续检测5次	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else target->trust_flag=1;
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;	
				Sensor_Parameter_Sort(target->x,target->y,WP_AHRS.Pitch,WP_AHRS.Roll,NamelessQuad.Position[_UP]);	
			}
			break;	

/************************************************************************************************/
/************************************************************************************************/
/************************************************************************************************/
/*************************************遍历航点 遇到目标  进行靠近 然后降落*********************************/
			
			case (0xA0+0x23):
			{




				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<20)	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else target->trust_flag=1;
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;		
				Sensor_Parameter_Sort(target->x,target->y,WP_AHRS.Pitch,WP_AHRS.Roll,NamelessQuad.Position[_UP]);		
			
			}
			break;
/************************************************************************************************/
/************************************************************************************************/
/************************************************************************************************/
/************************************************************************************************/			
			default:
			{
				target->target_ctrl_enable=0;
				target->trust_flag=0;
				target->x=0;
				target->y=0;
			}
		}
	}
}



void Openmv_Data_Receive_Anl_2(uint8_t *data_buf,uint8_t num,Target_Check *target)   //前视  摄像头  的串口  
{
  uint8_t sum = 0;
	if(!(*(data_buf)==0xFF && *(data_buf+1)==0xFC))return;//不满足帧头条件
	
  for(uint8_t i=0;i<(num-1);i++)  sum+=*(data_buf+i);
  if(!(sum==*(data_buf+num-1))) 	return;//不满足和校验条件
  	
	target->x				  	=*(data_buf+4)<<8|*(data_buf+5);
  target->y						=*(data_buf+6)<<8|*(data_buf+7);
	target->pixel				=*(data_buf+8)<<8|*(data_buf+9);  
	target->flag				=*(data_buf+10);
	target->state		  	=*(data_buf+11);		
	target->angle		  	=*(data_buf+12)<<8|*(data_buf+13);
	target->distance  	=*(data_buf+14)<<8|*(data_buf+15);
	target->apriltag_id	=*(data_buf+16)<<8|*(data_buf+17);
	target->width	    	=*(data_buf+18)<<8|*(data_buf+19);
	target->height	  	=*(data_buf+20)<<8|*(data_buf+21);
	target->fps  		  	=*(data_buf+22);
	target->reserved1 	=*(data_buf+23);
	target->reserved2 	=*(data_buf+24);
	target->reserved3 	=*(data_buf+25);
	target->reserved4 	=*(data_buf+26);
	//扩展距离传感器
	target->range_sensor1 =*(data_buf+27)<<8|*(data_buf+28);
	target->range_sensor2 =*(data_buf+29)<<8|*(data_buf+30);
	target->range_sensor3 =*(data_buf+31)<<8|*(data_buf+32);
	target->range_sensor4 =*(data_buf+33)<<8|*(data_buf+34);
	target->camera_id=*(data_buf+35);
	target->reserved1_int32=*(data_buf+36)<<24|*(data_buf+37)<<16|*(data_buf+38)<<8|*(data_buf+39);
	target->reserved2_int32=*(data_buf+40)<<24|*(data_buf+41)<<16|*(data_buf+42)<<8|*(data_buf+43);
	target->reserved3_int32=*(data_buf+44)<<24|*(data_buf+45)<<16|*(data_buf+46)<<8|*(data_buf+47);
	target->reserved4_int32=*(data_buf+48)<<24|*(data_buf+49)<<16|*(data_buf+50)<<8|*(data_buf+51);	
	
	target->sdk_mode=*(data_buf+2);
	
	if(target->camera_id==0x01)//摄像头id为OPENMV
	{
		target->x_pixel_size=Pixel_Size_MV*(Pixel_Image_Width_MV/target->width);
		target->y_pixel_size=Pixel_Size_MV*(Pixel_Image_Height_MV/target->height);
		target->apriltag_distance=AprilTag_Side_Length*Focal_Length_MV/(target->x_pixel_size*FastSqrt(target->pixel));
		
		switch(target->sdk_mode)
		{
			case 0xA4://AprilTag检测
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<20)	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else target->trust_flag=1;    //这里可以查看 april tag id
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;				
				target->sdk_target.x=target->x_cm=(target->x_pixel_size*(0.5*target->width-target->x )*target->apriltag_distance)/Focal_Length_MV;
				target->sdk_target.y=target->y_cm=(target->y_pixel_size*(0.5*target->height-target->y)*target->apriltag_distance)/Focal_Length_MV;		
			}
			break;
			case 0xA5://红色塔杆检测
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<20)	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else target->trust_flag=1;
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}	
				//front_tofsense_distance
				target->sdk_target.x=target->x_cm=(target->x_pixel_size*(0.5*target->width-target->x )*front_tofsense_distance)/Focal_Length_MV;
				target->sdk_target.y=target->y_cm=(target->y_pixel_size*(0.5*target->height-target->y)*front_tofsense_distance)/Focal_Length_MV;		
			}
			break;
			case 0xA6:
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<20)	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else 
					{
						target->trust_flag=1;
						barcode_id=target->apriltag_id;
					  barcode_flag=1;
					}
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;				
				target->sdk_target.x=target->x_cm=(target->x_pixel_size*(0.5*target->width-target->x )*target->apriltag_distance)/Focal_Length_MV;
				target->sdk_target.y=target->y_cm=(target->y_pixel_size*(0.5*target->height-target->y)*target->apriltag_distance)/Focal_Length_MV;	
			}
			break;
			default:
			{
				target->target_ctrl_enable=0;
				target->trust_flag=0;
				target->x=0;
				target->y=0;		
			}
		}
	}
	else if(target->camera_id==0x02)//摄像头id为OPENCV
	{	
		switch(target->sdk_mode)
		{
			case 0xA4://AprilTag检测
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<20)	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else target->trust_flag=1;
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;							
//				target->x_cm=-target->reserved1_int32/10.0f;
//				target->y_cm=-target->reserved2_int32/10.0f;
//				target->z_cm= target->reserved3_int32/10.0f;
//				target->sdk_target.x=target->x_cm;
//				target->sdk_target.y=target->y_cm;			
//				target->sdk_target.z=target->z_cm;
//				target->apriltag_distance=target->sdk_target.z;
				
				target->x_pixel_size=Pixel_Size_CV*(Pixel_Image_Width_CV/target->width);
				target->y_pixel_size=Pixel_Size_CV*(Pixel_Image_Height_CV/target->height);
				target->apriltag_distance=target->z_cm=AprilTag_Side_Length*Focal_Length_CV/(target->x_pixel_size*target->pixel);
				target->sdk_target.x=target->x_cm=(target->x_pixel_size*(0.5*target->width-target->x )*target->apriltag_distance)/Focal_Length_CV;
				target->sdk_target.y=target->y_cm=(target->y_pixel_size*(0.5*target->height-target->y)*target->apriltag_distance)/Focal_Length_CV;					
			}
			break;
			case 0xA6:
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<5)//连续5次检测到	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else 
					{
						target->trust_flag=1;
						barcode_id=target->apriltag_id;
					  barcode_flag=1;
					}
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				//target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				//target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;				
				//target->sdk_target.x=target->x_cm=(target->x_pixel_size*(0.5*target->width-target->x )*target->apriltag_distance)/Focal_Length_CV;
				//target->sdk_target.y=target->y_cm=(target->y_pixel_size*(0.5*target->height-target->y)*target->apriltag_distance)/Focal_Length_CV;	
			}
			break;
			
			/***********************************************************************************************************************/
			/***********************************************************************************************************************/
			/*******************************************二维码识别测试*******************************************************************/
			case 0xC2:
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<5)//连续5次检测到	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else 
					{
						target->trust_flag=1;
						barcode_id=target->apriltag_id;
					  barcode_flag=1;
					}
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}				
			}
			break;
			/***********************************************************************************************************************/
			/***********************************************************************************************************************/
			/***********************************************************************************************************************/
			/***********************************************************************************************************************/
			default:
			{
				target->target_ctrl_enable=0;
				target->trust_flag=0;
				target->x=0;
				target->y=0;	
				target->sdk_target.x=target->x_cm=0;
				target->sdk_target.y=target->y_cm=0;		
				target->sdk_target.z=target->z_cm=0;
				target->apriltag_distance=0;				
			}
		}	
	}
}


static uint8_t state[3] = {0};
static uint8_t _data_len[2] = {0},_data_cnt[2] = {0};
uint8_t _buf[3][SDK_Target_Length];
void SDK_Data_Receive_Prepare(uint8_t data,uint8_t label)
{
  if(state[label]==0&&data==0xFF)//帧头1
  {
    state[label]=1;
    _buf[label][0]=data;
  }
  else if(state[label]==1&&data==0xFC)//帧头2
  {
    state[label]=2;
    _buf[label][1]=data;
  }
  else if(state[label]==2&&data<0XFF)//功能字节
  {
    state[label]=3;
    _buf[label][2]=data;
  }
  else if(state[label]==3&&data<50)//数据长度
  {
    state[label] = 4;
    _buf[label][3]=data;
    _data_len[label] = data;
    _data_cnt[label] = 0;
  }
  else if(state[label]==4&&_data_len>0)//有多少数据长度，就存多少个
  {
    _data_len[label]--;
    _buf[label][4+_data_cnt[label]++]=data;
    if(_data_len[label]==0) state[label] = 5;
  }
  else if(state[label]==5)//最后接收数据校验和
  {
    state[label] = 0;
    _buf[label][4+_data_cnt[label]]=data;
		if(label==0)  Openmv_Data_Receive_Anl_1(_buf[label],_data_cnt[label]+5,&Opv_Top_View_Target);
		else Openmv_Data_Receive_Anl_2(_buf[label],_data_cnt[label]+5,&Opv_Front_View_Target);
  }
  else state[label] = 0;
}


//SDK1数据解析
void SDK_Data_Receive_Prepare_1(uint8_t data)
{
	uint8_t label=0;
  if(state[label]==0&&data==0xFF)//帧头1
  {
    state[label]=1;
    _buf[label][0]=data;
  }
  else if(state[label]==1&&data==0xFC)//帧头2
  {
    state[label]=2;
    _buf[label][1]=data;
  }
  else if(state[label]==2&&data<0XFF)//功能字节
  {
    state[label]=3;
    _buf[label][2]=data;
  }
  else if(state[label]==3&&data<50)//数据长度
  {
    state[label] = 4;
    _buf[label][3]=data;
    _data_len[label] = data;
    _data_cnt[label] = 0;
  }
  else if(state[label]==4&&_data_len>0)//有多少数据长度，就存多少个
  {
    _data_len[label]--;
    _buf[label][4+_data_cnt[label]++]=data;
    if(_data_len[label]==0) state[label] = 5;
  }
  else if(state[label]==5)//最后接收数据校验和
  {
    state[label] = 0;
    _buf[label][4+_data_cnt[label]]=data;
		Openmv_Data_Receive_Anl_1(_buf[label],_data_cnt[label]+5,&Opv_Top_View_Target);
  }
  else 
	{
		state[label] = 0;
	}
}


//SDK2数据解析
void SDK_Data_Receive_Prepare_2(uint8_t data)
{
	uint8_t label=1;
  if(state[label]==0&&data==0xFF)//帧头1
  {
    state[label]=1;
    _buf[label][0]=data;
  }
  else if(state[label]==1&&data==0xFC)//帧头2
  {
    state[label]=2;
    _buf[label][1]=data;
  }
  else if(state[label]==2&&data<0XFF)//功能字节
  {
    state[label]=3;
    _buf[label][2]=data;
  }
  else if(state[label]==3&&data<50)//数据长度
  {
    state[label] = 4;
    _buf[label][3]=data;
    _data_len[label] = data;
    _data_cnt[label] = 0;
  }
  else if(state[label]==4&&_data_len>0)//有多少数据长度，就存多少个
  {
    _data_len[label]--;
    _buf[label][4+_data_cnt[label]++]=data;
    if(_data_len[label]==0) state[label] = 5;
  }
  else if(state[label]==5)//最后接收数据校验和
  {
    state[label] = 0;
    _buf[label][4+_data_cnt[label]]=data;
		Openmv_Data_Receive_Anl_2(_buf[label],_data_cnt[label]+5,&Opv_Front_View_Target);
  }
  else state[label] = 0;
}



/***********************************************************/
/***********************************************************/
/***********************************************************/
/***********************************************************/
/***********************************************************/
/*******************地面站**********************************/

GroundStation_To_F4_Target To_F4_Target;
GroundStation_To_TIVA_Target To_TIVA_Target;
//把 找到 动物的信息  告知 地面站
void SDK_DT_Send_To_GroundStation(uint8_t animal_type,uint8_t animal_num,vector2f Rigion)
{
  sdk_data_to_send[0]=0xFF;
  sdk_data_to_send[1]=0xFC;
  sdk_data_to_send[2]=0xA0;
  sdk_data_to_send[3]=4;    // 发送给 地面站的 是固定的数据
  sdk_data_to_send[4]=animal_type;
  sdk_data_to_send[5]=animal_num;
  sdk_data_to_send[6]=Rigion.A;
  sdk_data_to_send[7]=Rigion.B;
  u8 sum = 0;
  for(u8 i=0;i<8;i++) sum += sdk_data_to_send[i];
  sdk_data_to_send[8]=sum;  //校验和
  USART2_Send(sdk_data_to_send, 9);    //打包好之后发送
	 
}









//地面站数据解析  两边公用的部分
void SDK_Data_Receive_Prepare_GroundStation(uint8_t data)
{
	uint8_t label=2;   //3是用来存放地点站消的 缓冲区
  if(state[label]==0&&data==0xFF)//帧头1
  {
    state[label]=1;
    _buf[label][0]=data;
  }
  else if(state[label]==1&&data==0xFC)//帧头2
  {
    state[label]=2;
    _buf[label][1]=data;
  }
  else if(state[label]==2&&data<0XFF)//功能字节
  {
    state[label]=3;
    _buf[label][2]=data;
  }
  else if(state[label]==3&&data<50)//数据长度
  {
    state[label] = 4;
    _buf[label][3]=data;
    _data_len[label] = data;
    _data_cnt[label] = 0;
  }
  else if(state[label]==4&&_data_len[label]>0)//有多少数据长度，就存多少个
  {
    _data_len[label]--;
    _buf[label][4+_data_cnt[label]++]=data;
    if(_data_len[label]==0) state[label] = 5;
  }
  else if(state[label]==5)//最后接收数据校验和
  {
    state[label] = 0;
    _buf[label][4+_data_cnt[label]]=data;
		Data_Receive_GroundStation(_buf[label],_data_cnt[label]+5,&To_TIVA_Target);
  }
  else 
	{
		state[label] = 0;
	}
}


//把地面站传过来的 禁区信息 存进 To_TIVA_Target的内容当中   
//在串口数据解析完毕且正确 之后 会置位 一个标志位，Get_Forbidden_Rigion=1
//在主函数当中会 对这个变量To_TIVA_Target->Get_Forbidden_Rigion 进行轮询  当发现变量置位  把这个变量置为0  并进行路线的规划生成

void Data_Receive_GroundStation(uint8_t *data_buf,uint8_t num,GroundStation_To_TIVA_Target* target)
{
  uint8_t sum = 0;
	if(!(*(data_buf)==0xFF && *(data_buf+1)==0xFC))return;//不满足帧头条件
	
  for(uint8_t i=0;i<(num-1);i++)  sum+=*(data_buf+i);
  if(!(sum==*(data_buf+num-1))) 	return;//不满足和校验条件
  	
	target->Forbidden_Rigion_Num				  	=*(data_buf+3);
    for(int tem=0;tem<target->Forbidden_Rigion_Num;tem++)
    {
        target->Forbidden_Rigion[tem].A=*(data_buf+4+2*tem);
        target->Forbidden_Rigion[tem].B=*(data_buf+4+2*tem+1);
        
    }
	target->Get_Forbidden_Rigion=1;

}


void GroundStation_Begin_MAP(void)  //主函数中会 一直调用他
{
    static uint8_t sta_cnt=0;
    if(sta_cnt==0)
    {
        if(To_TIVA_Target.Get_Forbidden_Rigion==1)
        {
            To_TIVA_Target.Get_Forbidden_Rigion=0;
			To_TIVA_Target.Forbidden_Rigion_Num/=2; //每一组数据是两个，所以要/2
            sta_cnt=1;  //说明收集到了 若干个Forbidden_Rigion 的 信息 
        }
        else return;
    }
    else if(sta_cnt==1)
    {
        sta_cnt=2; 


    }
    else 
    {
        //建完了 啥也不用做了
    }
    return ;


}



/***********************************************************/
/***********************************************************//***********************************************************/
/***********************************************************/
/***********************************************************/



uint16_t sdk_offset_cnt1=0;
void SDK_Data_Prase_1(void)
{
  static uint16_t sdk_prase_cnt=0;
  uint16_t i=0;
  sdk_prase_cnt++;
  if(sdk_prase_cnt>=2)//5*2=10ms
  {
    if(COM3_Rx_Buf.Tail<SDK_Target_Length)//0-11数据位正在传输
    {
      sdk_offset_cnt1=SDK_Target_Length;
    }
    else//12-23数据位正在传输
    {
      sdk_offset_cnt1=0;
    }
    for(i=0;i<SDK_Target_Length;i++)
    {
      SDK_Data_Receive_Prepare(COM3_Rx_Buf.Ring_Buff[sdk_offset_cnt1+i],0);
    }
    sdk_prase_cnt=0;
  }
}


uint16_t sdk_offset_cnt2=0;
void SDK_Data_Prase_2(void)
{
  static uint16_t sdk_prase_cnt=0;
  uint16_t i=0;
  sdk_prase_cnt++;
  if(sdk_prase_cnt>=2)//5*2=10ms
  {
    if(COM0_Rx_Buf.Tail<SDK_Target_Length)//0-11数据位正在传输
    {
      sdk_offset_cnt2=SDK_Target_Length;
    }
    else//12-23数据位正在传输
    {
      sdk_offset_cnt2=0;
    }
    for(i=0;i<SDK_Target_Length;i++)
    {
      SDK_Data_Receive_Prepare(COM0_Rx_Buf.Ring_Buff[sdk_offset_cnt2+i],1);
    }
    sdk_prase_cnt=0;
  }
}





void SDK_Init(void)
{
  float sdk_mode_default1=0,sdk_mode_default2=0,task_select=0;
  SDK_DT_Reset();//复位SDK线检测数据
  ReadFlashParameterOne(SDK1_MODE_DEFAULT,&sdk_mode_default1);
  if(isnan(sdk_mode_default1)==0)
  {
    SDK1_Mode_Setup=(uint8_t)(sdk_mode_default1);//俯视
    SDK_DT_Send_Check(SDK1_Mode_Setup,UART3_SDK);//初始化俯视opemmv工作模式，默认以上次工作状态配置
  }
	
  ReadFlashParameterOne(FORWARD_VISION_MODE_DEFAULT,&sdk_mode_default2);
  if(isnan(sdk_mode_default2)==0)
  {
    Forward_Vision_Mode_Setup=(uint8_t)(sdk_mode_default2);//前视
    SDK_DT_Send_Check(Forward_Vision_Mode_Setup,UART0_SDK);//初始化前视opemmv工作模式，默认以上次工作状态配置
  } 

  ReadFlashParameterOne(TASK_SELECT_AFTER_TAKEOFF,&task_select);
  if(isnan(task_select)==0) task_select_cnt=(int16_t)(task_select);//自动起飞后任务选择	
	else task_select=1;
}





