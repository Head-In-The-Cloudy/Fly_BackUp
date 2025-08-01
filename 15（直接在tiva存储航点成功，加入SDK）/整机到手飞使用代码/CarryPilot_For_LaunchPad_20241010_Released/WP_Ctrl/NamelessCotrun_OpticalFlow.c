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
#include "NamelessCotrun_OpticalFlow.h"



#include <Time.h>
void Third_Order_Complementarity(float flow_height,Vector2f accel,Vector2f flow);

#define OpticalFlow_Ringbuf  COM2_Rx_Buf



uint8_t OpticalFlow_Is_Work=0;
float opticalflow_high=1000;//默认1m=100cm=1000mm
flow_float opt_data;
flow_integral_frame opt_origin_data;
Vector2f opt_filter_data; //滤波后的像素位移
Vector2f opt_gyro_data;   //光流角速度
Vector2f opt_gyro_filter_data;//光流经过旋转互补滤波后的角速度
Vector2f gyro_filter_data;//低通同步相位的角速度
Vector2f gyro_filter_data_backup[20];//低通同步相位的角速度
lpf_param OpticalFlow_Parameter,OpticalFlow_gyro_lpf_param;
lpf_buf Buffer_OpticalFlow[2],Buffer_OpticalFlow_Gyro[2];
uint8_t Optical_Type_Present=1;



void OpticalFlow_Init()
{
  set_cutoff_frequency(50, 20,&OpticalFlow_Parameter);
  set_cutoff_frequency(50, 6,&OpticalFlow_gyro_lpf_param);//同步相位

  float optical_type_default=0;
  ReadFlashParameterOne(OPTICAL_TYPE,&optical_type_default);
  if(isnan(optical_type_default)==0)
  {
		Optical_Type_Present=(uint8_t)(optical_type_default);
  } 
	
  //根据不同的光流配置，设置对应长度的数据缓冲区	
	if(Optical_Type_Present==1)//LC307
	{
		OpticalFlow_Is_Work=Config_Init_Uart();//光流传感器初始化
	}
	else if(Optical_Type_Present==2)//LC302
	{
	  OpticalFlow_Is_Work=Config_Init_Uart();//光流传感器初始化;
	}
	else if(Optical_Type_Present==3||Optical_Type_Present==4)//lidar or T265
	{
	  OpticalFlow_Is_Work=1;
	}
}



systime optical_t;
uint8_t lc30x_buf[20];
void LC30X_Data_Receive_Anl(uint8_t *data_buf);//50hz
void LC30X_OpticalFlow_Sense_Prase(uint8_t data,uint8_t *buf)
{
  static uint8_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==0xfe)//判断帧头1
  {
    state=1;
    buf[0]=data;
  }
  else if(state==1&&data==0x0a)//判断帧头2
  {
    state=2;
    buf[1]=data;
		data_len = 10;
		data_cnt = 0;
  }
  else if(state==2&&data_len>0)//数据接收
  {
    data_len--;
    buf[2+data_cnt++]=data;
    if(data_len==0)  state = 3;
  }
  else if(state==3)//异或校验
  {
    state = 4;
    buf[2+data_cnt++]=data;
  }
  else if(state==4)//判断帧头尾
  {
		static unsigned char last_valid=0;
		static uint32_t start_t,end_t;
		last_valid=opt_data.valid;
    state = 0;
    buf[2+data_cnt++]=data;
		
		
		LC30X_Data_Receive_Anl(buf);
		Get_Systime(&optical_t);	
		if(last_valid==1&&opt_data.valid==0)//记录首次失效时刻
		{
			start_t=millis();
			opt_data.ctrl_valid=1;
		}
		else if(last_valid==0&&opt_data.valid==0)//持续记录失效时刻
		{
			end_t=millis();					
			if(end_t-start_t>=1000)//持续失效1000ms，认为光流已故障，放弃光流控制
			{
				opt_data.ctrl_valid=0;
			}
		}
		else//光流数据正常
		{
			opt_data.ctrl_valid=1;
		}
  }
  else state = 0;
}




void opticalflow_pretreat(void)
{
	opt_filter_data.x=LPButterworth(opt_origin_data.pixel_flow_x_integral,&Buffer_OpticalFlow[0],&OpticalFlow_Parameter);
	opt_filter_data.y=LPButterworth(opt_origin_data.pixel_flow_y_integral,&Buffer_OpticalFlow[1],&OpticalFlow_Parameter);   
	opt_data.x=(opt_origin_data.pixel_flow_x_integral*opticalflow_high)/10000.0f;//单位:乘以高度单位mm后为实际位移mm
	opt_data.y=(opt_origin_data.pixel_flow_y_integral*opticalflow_high)/10000.0f;//单位:乘以高度单位mm后为实际位移mm     
	opt_data.dt=(int16_t)(opt_origin_data.integration_timespan*0.001f);//单位ms
	opt_data.qual=opt_origin_data.qual;
	if(opt_data.qual==0xF5) opt_data.valid=1;
	else opt_data.valid=0;	

	opt_gyro_data.x=(opt_filter_data.x)/200.0f;//光流角速度rad/s
	opt_gyro_data.y=(opt_filter_data.y)/200.0f;//光流角速度rad/s         
	gyro_filter_data.x=LPButterworth(WP_AHRS.Roll_Gyro,&Buffer_OpticalFlow_Gyro[0],&OpticalFlow_gyro_lpf_param)/57.3f;//陀螺仪相位同步角速度
	gyro_filter_data.y=LPButterworth(WP_AHRS.Pitch_Gyro,&Buffer_OpticalFlow_Gyro[1],&OpticalFlow_gyro_lpf_param)/57.3f;//陀螺仪相位同步角速度
	
	for(uint16_t i=19;i>0;i--)
	{
		gyro_filter_data_backup[i].x=gyro_filter_data_backup[i-1].x;
		gyro_filter_data_backup[i].y=gyro_filter_data_backup[i-1].y;
	} 
	gyro_filter_data_backup[0].x=gyro_filter_data.x;
	gyro_filter_data_backup[0].y=gyro_filter_data.y;
	
	
	opt_gyro_filter_data.x=OpticalFlow_Rotate_Complementary_Filter(opt_gyro_data.x,gyro_filter_data_backup[0].x,'x',opt_data.valid);//光流角速度与陀螺仪角速度融合 
	opt_gyro_filter_data.y=OpticalFlow_Rotate_Complementary_Filter(opt_gyro_data.y,gyro_filter_data_backup[0].y,'y',opt_data.valid); //光流角速度与陀螺仪角速度融合 
}
	
	

uint8_t Optflow_Is_Okay=0;
float rpi_opticalflow_scale=0.3f;
void LC30X_Data_Receive_Anl(uint8_t *data_buf)//50hz
{	
	uint8_t xor_check = *(data_buf+2);
  for(uint8_t i=3;i<12;i++)  xor_check^=*(data_buf+i);
	if(!(*(data_buf)==0xfe&&*(data_buf+1)==0x0a))	return;//不满足帧头条件
	if(!(*(data_buf+13)==0x55)) 	return;
  //if(!(xor_check==*(data_buf+12))) return;		
	Optflow_Is_Okay=1;
	switch(Optical_Type_Present)
	{
		case 1://LC307
		{
			opt_origin_data.pixel_flow_x_integral=(int16_t)(data_buf[3]<<8)|data_buf[2];
			opt_origin_data.pixel_flow_y_integral=(int16_t)(data_buf[5]<<8)|data_buf[4];			
			opt_origin_data.pixel_flow_x_integral*=(-1);					
			opt_origin_data.pixel_flow_y_integral*=(-1);				
			opt_origin_data.integration_timespan= (int16_t)(data_buf[7]<<8)|data_buf[6];
			opt_origin_data.qual=data_buf[10]; 
		}
		break;
		case 2://LC302
		{
			opt_origin_data.pixel_flow_x_integral=(int16_t)(data_buf[3]<<8)|data_buf[2];
			opt_origin_data.pixel_flow_y_integral=(int16_t)(data_buf[5]<<8)|data_buf[4];						
			opt_origin_data.integration_timespan= (int16_t)(data_buf[7]<<8)|data_buf[6];
			opt_origin_data.qual=data_buf[10]; 	
		}
		break;	
		default://LC302
		{
			opt_origin_data.pixel_flow_x_integral=(int16_t)(data_buf[3]<<8)|data_buf[2];
			opt_origin_data.pixel_flow_y_integral=(int16_t)(data_buf[5]<<8)|data_buf[4];						
			opt_origin_data.integration_timespan= (int16_t)(data_buf[7]<<8)|data_buf[6];
			opt_origin_data.qual=data_buf[10];      
		}
	}
	opticalflow_pretreat();
}


void Optflow_Statemachine(void)
{
	if(Temperature_Stable_Flag==0||Gyro_Safety_Calibration_Flag==0) return;//温控就位后才进行融合
  if(OpticalFlow_Is_Work==1)//若初始化时，存在光流外设
  {
		switch(Optical_Type_Present)
		{
			case 1:
			case 2:Third_Order_Complementarity(NamelessQuad.Position[_UP],SINS_Accel_Body,opt_gyro_filter_data);break;
			case 3:VIO_SLAM_INS_CF();break;
			case 4:altitude_kalman_filter(&slam_kf,&VIO_SINS,0.005f);break;
			default:	;	
		}
  }
}



float OpticalFlow_Rotate_Complementary_Filter(float optflow_gyro,float gyro,uint8_t axis,uint8_t valid)
{
  float optflow_gyro_filter=0;
  if(Optical_Type_Present==1||Optical_Type_Present==2)
  {
		optflow_gyro_filter=optflow_gyro-constrain_float(gyro,-ABS(optflow_gyro),ABS(optflow_gyro));
		//optflow_gyro_filter=optflow_gyro-constrain_float(gyro,-5.0f,5.0f);		
  }
  return optflow_gyro_filter;
}



SINS_Lite OpticalFlow_SINS,VIO_SINS;
Testime Optical_Delta;
Vector2f OpticalFlow_Position={0};
Vector2f OpticalFlow_Speed={0};
Vector2f OpticalFlow_Speed_Err={0};
Vector2f OpticalFlow_Position_Err={0};
uint16_t Optflow_Sync_Cnt=5;
float CF_Parameter[2]={0.05f,0.1f};//光流互补滤波权重 0.1  0.1   0.08  0
//光流位置融合权重给为0，表示不加入修正位置修正，因为低成本光流模块漂移较大，亦可以给较小值如0.2f
#define Optical_Output_Dt  0.02f//50hz
void  OpticalFlow_INS_CF(float flow_height,Vector2f accel,Vector2f flow)
{
  float use_height=0;
  float optical_dt=0;
  Vector2f speed_delta={0};
  Test_Period(&Optical_Delta);
  optical_dt=Optical_Delta.Time_Delta/1000.0f;
	if(optical_dt>1.05f*WP_Duty_Dt||optical_dt<0.95f*WP_Duty_Dt||isnan(optical_dt)!=0)   optical_dt=WP_Duty_Dt; 
	use_height = constrain_float(flow_height,5,10000);
  if(Optflow_Is_Okay==1&&opt_data.valid==1)//存在数据光流更新时，20ms一次
  {  
    OpticalFlow_Speed.x=flow.x*use_height;//光流速度
    OpticalFlow_Speed.y=flow.y*use_height;//光流速度
    OpticalFlow_Position.x+=OpticalFlow_Speed.x*Optical_Output_Dt;//光流位移
    OpticalFlow_Position.y+=OpticalFlow_Speed.y*Optical_Output_Dt;//光流位移

    Optflow_Is_Okay=0;
    OpticalFlow_Position_Err.x=OpticalFlow_Position.x-OpticalFlow_SINS.Pos_Backups[_EAST][Optflow_Sync_Cnt];
    OpticalFlow_Position_Err.y=OpticalFlow_Position.y-OpticalFlow_SINS.Pos_Backups[_NORTH][Optflow_Sync_Cnt];
		OpticalFlow_Speed_Err.x=OpticalFlow_Speed.x-OpticalFlow_SINS.Vel_Backups[_EAST][Optflow_Sync_Cnt];
    OpticalFlow_Speed_Err.y=OpticalFlow_Speed.y-OpticalFlow_SINS.Vel_Backups[_NORTH][Optflow_Sync_Cnt];
		
		OpticalFlow_Speed_Err.x=constrain_float(OpticalFlow_Speed_Err.x,-200,200);
		OpticalFlow_Speed_Err.y=constrain_float(OpticalFlow_Speed_Err.y,-200,200);
  }
  else
  {
    OpticalFlow_Speed_Err.x=0;
    OpticalFlow_Speed_Err.y=0;
    OpticalFlow_Position_Err.x=0;
    OpticalFlow_Position_Err.y=0;
  }
  
  OpticalFlow_SINS.Acceleration[_EAST]=-accel.x;//惯导加速度沿载体横滚,机头左侧为正
  OpticalFlow_SINS.Acceleration[_NORTH] = accel.y;//惯导加速度沿载体机头,机头前向为正
  
	speed_delta.x=OpticalFlow_SINS.Acceleration[_EAST]*optical_dt;
  speed_delta.y=OpticalFlow_SINS.Acceleration[_NORTH]*optical_dt;    
  
	OpticalFlow_SINS.Position[_EAST]+=OpticalFlow_SINS.Speed[_EAST]*optical_dt
    +0.5f*speed_delta.x*optical_dt+CF_Parameter[1]*OpticalFlow_Position_Err.x;
  OpticalFlow_SINS.Position[_NORTH]+=OpticalFlow_SINS.Speed[_NORTH]*optical_dt
    +0.5f*speed_delta.y*optical_dt+CF_Parameter[1]*OpticalFlow_Position_Err.y;

  OpticalFlow_SINS.Speed[_EAST]+=OpticalFlow_SINS.Acceleration[_EAST]*optical_dt+CF_Parameter[0]*OpticalFlow_Speed_Err.x;
  OpticalFlow_SINS.Speed[_NORTH]+=OpticalFlow_SINS.Acceleration[_NORTH]*optical_dt+CF_Parameter[0]*OpticalFlow_Speed_Err.y; 
   	
	for(uint16_t i=Num-1;i>0;i--)
	{
		OpticalFlow_SINS.Pos_Backups[_NORTH][i]=OpticalFlow_SINS.Pos_Backups[_NORTH][i-1];
		OpticalFlow_SINS.Pos_Backups[_EAST][i]=OpticalFlow_SINS.Pos_Backups[_EAST][i-1];
		OpticalFlow_SINS.Vel_Backups[_NORTH][i]=OpticalFlow_SINS.Vel_Backups[_NORTH][i-1];
		OpticalFlow_SINS.Vel_Backups[_EAST][i]=OpticalFlow_SINS.Vel_Backups[_EAST][i-1]; 		
	}   
	OpticalFlow_SINS.Pos_Backups[_NORTH][0]=OpticalFlow_SINS.Position[_NORTH];
  OpticalFlow_SINS.Pos_Backups[_EAST][0]=OpticalFlow_SINS.Position[_EAST]; 
  OpticalFlow_SINS.Vel_Backups[_NORTH][0]=OpticalFlow_SINS.Speed[_NORTH];
  OpticalFlow_SINS.Vel_Backups[_EAST][0]=OpticalFlow_SINS.Speed[_EAST];  	 
}







Vector3s correct[2];
float K_ACC_OPT=1.0f,K_VEL_OPT=3.0f,K_POS_OPT=5.0f;
void Third_Order_Complementarity(float flow_height,Vector2f accel,Vector2f flow)
{	
	float use_height=constrain_float(flow_height,5,10000);
	float obs_err[2];
	Vector2f acc,speed_delta={0};
	if(Optflow_Is_Okay==1&&opt_data.valid==1)//存在数据光流更新时，20ms一次
  {  
    OpticalFlow_Speed.x=flow.x*use_height;//光流速度
    OpticalFlow_Speed.y=flow.y*use_height;//光流速度
    OpticalFlow_Position.x+=OpticalFlow_Speed.x*Optical_Output_Dt;//光流位移
    OpticalFlow_Position.y+=OpticalFlow_Speed.y*Optical_Output_Dt;//光流位移
		
    Optflow_Is_Okay=0;
    OpticalFlow_Position_Err.x=OpticalFlow_Position.x-OpticalFlow_SINS.Pos_Backups[_EAST][Optflow_Sync_Cnt];
    OpticalFlow_Position_Err.y=OpticalFlow_Position.y-OpticalFlow_SINS.Pos_Backups[_NORTH][Optflow_Sync_Cnt];

		OpticalFlow_Speed_Err.x=OpticalFlow_Speed.x-OpticalFlow_SINS.Vel_Backups[_EAST][Optflow_Sync_Cnt];
    OpticalFlow_Speed_Err.y=OpticalFlow_Speed.y-OpticalFlow_SINS.Vel_Backups[_NORTH][Optflow_Sync_Cnt];		
		OpticalFlow_Speed_Err.x=constrain_float(OpticalFlow_Speed_Err.x,-200,200);
		OpticalFlow_Speed_Err.y=constrain_float(OpticalFlow_Speed_Err.y,-200,200);	
	}
//	else
//	{
//    OpticalFlow_Position_Err.x=0;
//    OpticalFlow_Position_Err.y=0;
//		OpticalFlow_Speed_Err.x=0;
//    OpticalFlow_Speed_Err.y=0;			
//	}
	obs_err[0]=OpticalFlow_Position_Err.x;
	obs_err[1]=OpticalFlow_Position_Err.y;
	//三路积分反馈量修正惯导
	correct[0].acc +=obs_err[0]* K_ACC_OPT*0.005f;//加速度矫正量
	correct[1].acc +=obs_err[1]* K_ACC_OPT*0.005f;//加速度矫正
	correct[0].acc=constrain_float(correct[0].acc,-50,50);
	correct[1].acc=constrain_float(correct[1].acc,-50,50);
	correct[0].vel  =OpticalFlow_Speed_Err.x*K_VEL_OPT*0.005f;//速度矫正量
	correct[1].vel  =OpticalFlow_Speed_Err.y*K_VEL_OPT*0.005f;//速度矫正量
	correct[0].pos  =obs_err[0]* K_POS_OPT*0.005f;//位置矫正量	
	correct[1].pos  =obs_err[1]* K_POS_OPT*0.005f;//位置矫正量
	
	acc.x=-accel.x;//惯导加速度沿载体横滚,机头左侧为正
	acc.y= accel.y;//惯导加速度沿载体机头,机头前向为正
	
  OpticalFlow_SINS.Acceleration[_EAST]= acc.x+correct[0].acc;//惯导加速度沿载体横滚,机头左侧为正
  OpticalFlow_SINS.Acceleration[_NORTH] = acc.y+correct[1].acc;//惯导加速度沿载体机头,机头前向为正
  
	speed_delta.x=(OpticalFlow_SINS.Acceleration[_EAST])*0.005f;
  speed_delta.y=(OpticalFlow_SINS.Acceleration[_NORTH])*0.005f; 
 	
	OpticalFlow_SINS.Position[_EAST]+=OpticalFlow_SINS.Speed[_EAST]*0.005f+0.5f*speed_delta.x*0.005f+correct[0].pos;
	OpticalFlow_SINS.Position[_NORTH] +=OpticalFlow_SINS.Speed[_NORTH] *0.005f+0.5f*speed_delta.y*0.005f+correct[1].pos;
	OpticalFlow_SINS.Speed[_EAST]+=speed_delta.x+correct[0].vel;
	OpticalFlow_SINS.Speed[_NORTH] +=speed_delta.y+correct[1].vel;	
	
	for(uint16_t i=Num-1;i>0;i--)
	{
		OpticalFlow_SINS.Pos_Backups[_NORTH][i]=OpticalFlow_SINS.Pos_Backups[_NORTH][i-1];
		OpticalFlow_SINS.Pos_Backups[_EAST][i]=OpticalFlow_SINS.Pos_Backups[_EAST][i-1];
		OpticalFlow_SINS.Vel_Backups[_NORTH][i]=OpticalFlow_SINS.Vel_Backups[_NORTH][i-1];
		OpticalFlow_SINS.Vel_Backups[_EAST][i]=OpticalFlow_SINS.Vel_Backups[_EAST][i-1]; 		
	}   
	OpticalFlow_SINS.Pos_Backups[_NORTH][0]=OpticalFlow_SINS.Position[_NORTH];
  OpticalFlow_SINS.Pos_Backups[_EAST][0]=OpticalFlow_SINS.Position[_EAST]; 
  OpticalFlow_SINS.Vel_Backups[_NORTH][0]=OpticalFlow_SINS.Speed[_NORTH];
  OpticalFlow_SINS.Vel_Backups[_EAST][0]=OpticalFlow_SINS.Speed[_EAST];  
}



void from_vio_to_body_frame(float map_x,float map_y,float *bod_x,float *bod_y,float _yaw)
{
	float _cos=FastCos(_yaw*DEG2RAD);
	float _sin=FastSin(_yaw*DEG2RAD);
	*bod_x= -map_x*_cos-map_y*_sin;
	*bod_y= -map_x*_sin+map_y*_cos;
}


void from_body_to_nav_frame(float bod_x,float bod_y,float *map_x,float *map_y,float _yaw)
{
	float _cos=FastCos(_yaw*DEG2RAD);
	float _sin=FastSin(_yaw*DEG2RAD);
	*map_x= bod_x*_cos-bod_y*_sin;
	*map_y= bod_x*_sin+bod_y*_cos;
}

void from_slam_to_nav_frame(float slam_e,float slam_n,float *map_e,float *map_n,float _yaw)
{
	float _cos=FastCos(_yaw*DEG2RAD);
	float _sin=FastSin(_yaw*DEG2RAD);
	*map_e= slam_e*_cos+slam_n*_sin;
	*map_n=-slam_e*_sin+slam_n*_cos;
}

//30  300  100  10
#define K_ACC_RPLIDAR 30.0f
#define K_VEL_RPLIDAR 400.0f//range(300,1000)500
#define K_POS_RPLIDAR 100.0f//range(50,150)100
#define SYNC_CNT_RPLIDAR 5//10  5
//5  120  30 5
#define K_ACC_T265    5.0f
#define K_VEL_T265    120.0f
#define K_POS_T265    30.0f
#define SYNC_CNT_T265 5
/*********************************************/
//30  100  150  10
//50  150  100  10
//50  150  100  5
float K_ACC_LOAM    =20.0f;
float K_VEL_LOAM    =100.0f;
float K_POS_LOAM    =80.0f;
float SYNC_CNT_LOAM =15;
/*********************************************/
float slam_fusion_param[3]={0};
uint16_t slam_sync_cnt=0;
void  VIO_SLAM_INS_CF(void)
{
  Vector2f acc={0,0},speed_delta={0,0};
	float obs_err[2];
	switch(current_state.slam_sensor)
	{
		case NO_SLAM:
		{
			return ;
		}
		case LIDAR_2D_SLAM:
		{
			slam_fusion_param[0]=K_ACC_RPLIDAR;
			slam_fusion_param[1]=K_VEL_RPLIDAR;
			slam_fusion_param[2]=K_POS_RPLIDAR;	
			slam_sync_cnt=SYNC_CNT_RPLIDAR;
			acc.x= NamelessQuad.Inertial_Acceleration[_EAST];//惯导加速度沿载体横滚,机头右侧为正
			acc.y= NamelessQuad.Inertial_Acceleration[_NORTH];//惯导加速度沿载体机头,机头前向为正			
		}
		break;
		case T265_SLAM:
		{
			slam_fusion_param[0]=K_ACC_T265;
			slam_fusion_param[1]=K_VEL_T265;
			slam_fusion_param[2]=K_POS_T265;	
			slam_sync_cnt=SYNC_CNT_T265;
			acc.x= NamelessQuad.Inertial_Acceleration[_EAST];//惯导加速度沿载体横滚,机头右侧为正
			acc.y= NamelessQuad.Inertial_Acceleration[_NORTH];//惯导加速度沿载体机头,机头前向为正			
		}
		break;
		case LOAM:
		{
			slam_fusion_param[0]=K_ACC_LOAM;
			slam_fusion_param[1]=K_VEL_LOAM;
			slam_fusion_param[2]=K_POS_LOAM;	
			slam_sync_cnt=SYNC_CNT_LOAM;
			acc.x= NamelessQuad.Inertial_Acceleration[_EAST];//惯导加速度沿载体横滚,机头右侧为正
			acc.y= NamelessQuad.Inertial_Acceleration[_NORTH];//惯导加速度沿载体机头,机头前向为正			
		}
		break;
		default:	return;		
	}

  if(current_state.update_flag==1)//存在数据SLAM更新时，100ms一次
  { 
		current_state.valid=1;
		current_state.update_flag=0; 
		
		OpticalFlow_Position.x= current_state.position_x;
		OpticalFlow_Position.y= current_state.position_y;	
		
    OpticalFlow_Position_Err.x=OpticalFlow_Position.x-VIO_SINS.Pos_Backups[_EAST][slam_sync_cnt];
    OpticalFlow_Position_Err.y=OpticalFlow_Position.y-VIO_SINS.Pos_Backups[_NORTH][slam_sync_cnt];
  }
	else
	{
		OpticalFlow_Position_Err.x=0;
		OpticalFlow_Position_Err.y=0;
	}
	obs_err[0]=OpticalFlow_Position_Err.x;
	obs_err[1]=OpticalFlow_Position_Err.y;
	//三路反馈量修正惯导
	correct[0].acc +=obs_err[0]* slam_fusion_param[0]*0.005f;//加速度矫正量
	correct[1].acc +=obs_err[1]* slam_fusion_param[0]*0.005f;//加速度矫正
	correct[0].acc=constrain_float(correct[0].acc,-50,50);
	correct[1].acc=constrain_float(correct[1].acc,-50,50);
	correct[0].vel  =obs_err[0]* slam_fusion_param[1]*0.005f;//速度矫正量
	correct[1].vel  =obs_err[1]* slam_fusion_param[1]*0.005f;//速度矫正量
	correct[0].pos  =obs_err[0]* slam_fusion_param[2]*0.005f;//位置矫正量	
	correct[1].pos  =obs_err[1]* slam_fusion_param[2]*0.005f;//位置矫正量
	
  VIO_SINS.Acceleration[_EAST] = acc.x+correct[0].acc;//惯导加速度沿载体横滚,机头左侧为正
  VIO_SINS.Acceleration[_NORTH]= acc.y+correct[1].acc;//惯导加速度沿载体机头,机头前向为正
  
	speed_delta.x=VIO_SINS.Acceleration[_EAST] *0.005f;
  speed_delta.y=VIO_SINS.Acceleration[_NORTH]*0.005f; 
 	
	VIO_SINS.Position[_EAST]  +=VIO_SINS.Speed[_EAST] *0.005f+0.5f*speed_delta.x*0.005f+correct[0].pos;
	VIO_SINS.Position[_NORTH] +=VIO_SINS.Speed[_NORTH]*0.005f+0.5f*speed_delta.y*0.005f+correct[1].pos;
	VIO_SINS.Speed[_EAST]  +=speed_delta.x+correct[0].vel;
	VIO_SINS.Speed[_NORTH] +=speed_delta.y+correct[1].vel;	
	
	for(uint16_t i=Num-1;i>0;i--)
	{
		VIO_SINS.Pos_Backups[_NORTH][i]=VIO_SINS.Pos_Backups[_NORTH][i-1];
		VIO_SINS.Pos_Backups[_EAST][i]=VIO_SINS.Pos_Backups[_EAST][i-1];
		VIO_SINS.Vel_Backups[_NORTH][i]=VIO_SINS.Vel_Backups[_NORTH][i-1];
		VIO_SINS.Vel_Backups[_EAST][i]=VIO_SINS.Vel_Backups[_EAST][i-1]; 		
	}   
	VIO_SINS.Pos_Backups[_NORTH][0]=VIO_SINS.Position[_NORTH];
  VIO_SINS.Pos_Backups[_EAST][0]=VIO_SINS.Position[_EAST]; 
  VIO_SINS.Vel_Backups[_NORTH][0]=VIO_SINS.Speed[_NORTH];
  VIO_SINS.Vel_Backups[_EAST][0]=VIO_SINS.Speed[_EAST];
	
	from_vio_to_body_frame(VIO_SINS.Position[_EAST],
												 VIO_SINS.Position[_NORTH],
												 &OpticalFlow_SINS.Position[_EAST],
												 &OpticalFlow_SINS.Position[_NORTH],
												 WP_AHRS.Yaw);
	
	from_vio_to_body_frame(VIO_SINS.Speed[_EAST],
												 VIO_SINS.Speed[_NORTH],
												 &OpticalFlow_SINS.Speed[_EAST],
												 &OpticalFlow_SINS.Speed[_NORTH],
												 WP_AHRS.Yaw);	
}
