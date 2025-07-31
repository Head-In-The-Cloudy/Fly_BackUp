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
#include "Sensor.h"
#include "Filter.h"


static float q0=1.0f,q1=0,q2=0,q3=0;																	
float Pitch_Observation,Roll_Observation,Yaw_Observation;//俯仰观测角、横滚观测角、偏航观测角
Vector3f_Body Circle_Angle;
float Gyro_Length=0;//陀螺仪模长
float q_backup[4][10]={0};
float K[3]={1.0,1.0,1.0};//默认标度(量程)误差
float B[3]={0,0,0};//默认零位误差
Sensor_Health Sensor_Flag;
float yaw_angle_deg_enu=0;
float yaw_angle_deg_enu_backups[30]={0};
float euler_rpy_init[3],q_init[4];

static float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;  
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

float sqf(float x) {return ((x)*(x));}
void Compute_Rotation_Matrix_From_Euler(float *rmat_btn)
{
  WP_AHRS.sin_rpy[_PIT]=FastSin(WP_AHRS.Pitch*DEG2RAD);
  WP_AHRS.cos_rpy[_PIT]=FastCos(WP_AHRS.Pitch*DEG2RAD);
  WP_AHRS.sin_rpy[_ROL] =FastSin(WP_AHRS.Roll*DEG2RAD);
  WP_AHRS.cos_rpy[_ROL] =FastCos(WP_AHRS.Roll*DEG2RAD);
  WP_AHRS.sin_rpy[_YAW] =FastSin(WP_AHRS.Yaw *DEG2RAD);
  WP_AHRS.cos_rpy[_YAW] =FastCos(WP_AHRS.Yaw *DEG2RAD);
	
  rmat_btn[0]=WP_AHRS.cos_rpy[_YAW] * WP_AHRS.cos_rpy[_ROL];
  rmat_btn[1]=WP_AHRS.sin_rpy[_PIT] * WP_AHRS.sin_rpy[_ROL]*WP_AHRS.cos_rpy[_YAW]-WP_AHRS.cos_rpy[_PIT] * WP_AHRS.sin_rpy[_YAW];
  rmat_btn[2]=WP_AHRS.sin_rpy[_PIT] * WP_AHRS.sin_rpy[_YAW]+WP_AHRS.cos_rpy[_PIT] * WP_AHRS.sin_rpy[_ROL] * WP_AHRS.cos_rpy[_YAW];  
  rmat_btn[3]=WP_AHRS.sin_rpy[_YAW] * WP_AHRS.cos_rpy[_ROL];
  rmat_btn[4]=WP_AHRS.sin_rpy[_PIT] * WP_AHRS.sin_rpy[_ROL] * WP_AHRS.sin_rpy[_YAW] +WP_AHRS.cos_rpy[_PIT] * WP_AHRS.cos_rpy[_YAW];
  rmat_btn[5]=WP_AHRS.cos_rpy[_PIT] * WP_AHRS.sin_rpy[_ROL] * WP_AHRS.sin_rpy[_YAW] - WP_AHRS.sin_rpy[_PIT] * WP_AHRS.cos_rpy[_YAW];
  rmat_btn[6]=-WP_AHRS.sin_rpy[_ROL];
  rmat_btn[7]= WP_AHRS.sin_rpy[_PIT]* WP_AHRS.cos_rpy[_ROL];
  rmat_btn[8]= WP_AHRS.cos_rpy[_PIT]* WP_AHRS.cos_rpy[_ROL];
}

void Compute_Rotation_Matrix_From_Quad(float *rmat_btn)
{
	float a=WP_AHRS.quaternion[0];
	float b=WP_AHRS.quaternion[1];
	float c=WP_AHRS.quaternion[2];
	float d=WP_AHRS.quaternion[3];
	float bc=b*c;
	float ad=a*d;
	float bd=b*d;
	float ac=a*c;
	float cd=c*d;
  float ab=a*b;
	float a2=a*a;
	float b2=b*b;
	float c2=c*c;
	float d2=d*d;	
	
  rmat_btn[0]=a2+b2-c2-d2;
  rmat_btn[1]=2*(bc-ad);
  rmat_btn[2]=2*(bd+ac);  
  rmat_btn[3]=2*(bc+ad);
  rmat_btn[4]=a2-b2+c2-d2;
  rmat_btn[5]=2*(cd-ab);
  rmat_btn[6]=2*(bd-ac);
  rmat_btn[7]=2*(cd+ab);
  rmat_btn[8]=a2-b2-c2+d2;
	
	WP_AHRS.sin_rpy[_PIT]=FastSin(WP_AHRS.Pitch*DEG2RAD);
  WP_AHRS.cos_rpy[_PIT]=FastCos(WP_AHRS.Pitch*DEG2RAD);
  WP_AHRS.sin_rpy[_ROL] =FastSin(WP_AHRS.Roll*DEG2RAD);
  WP_AHRS.cos_rpy[_ROL] =FastCos(WP_AHRS.Roll*DEG2RAD);
  WP_AHRS.sin_rpy[_YAW] =FastSin(WP_AHRS.Yaw *DEG2RAD);
  WP_AHRS.cos_rpy[_YAW] =FastCos(WP_AHRS.Yaw *DEG2RAD);
}


uint8_t Get_Baro_Offset()
{
  static uint16_t baro_sample_cnt=0;
  if(baro_sample_cnt<=100) 
  {
    baro_sample_cnt++;
  }    
  if(baro_sample_cnt==100) 
  {
    WP_Sensor.baro_presure_offset=WP_Sensor.baro_pressure_raw;
    return 1;
  }
  else if(baro_sample_cnt==101) 
  {
    float Tempbaro=(float)(WP_Sensor.baro_pressure_raw/WP_Sensor.baro_presure_offset)*1.0f;
    WP_Sensor.baro_altitude = 4433000.0f * (1 - FastPow((float)(Tempbaro),0.190295f));
		if(baro_flag==1)
		{
			WP_Sensor.baro_altitude_div=(WP_Sensor.baro_altitude-WP_Sensor.last_baro_altitude)/(Baro_Update_Dt*0.001f);			
		  WP_Sensor.last_baro_altitude=WP_Sensor.baro_altitude;			
			WP_Sensor.baro_altitude_acc=(WP_Sensor.baro_altitude_div-WP_Sensor.last_baro_altitude_div)/(Baro_Update_Dt*0.001f);
			WP_Sensor.last_baro_altitude_div=WP_Sensor.baro_altitude_div;
			baro_flag=0;
		}
    return 1;
  }
  return 0;
}

ButterFilterStruct accel_filter4[3],gyro_filter4[3];
void sensor_filter_init(void)
{
	//陀螺仪高阶低通滤波参数(30,95,1,80)
	//时延大概(1-1.4)*5ms,58hz处-3dB
	iir_high_order_filter_init(&gyro_filter4[0],30,0,95,0,1,80,200,FILTER_IIR_BUTTER_LOW);//最大1dB衰减通带，阻带至少80DB衰减
	iir_high_order_filter_init(&gyro_filter4[1],30,0,95,0,1,80,200,FILTER_IIR_BUTTER_LOW);//最大1dB衰减通带，阻带至少80DB衰减
	iir_high_order_filter_init(&gyro_filter4[2],30,0,95,0,1,80,200,FILTER_IIR_BUTTER_LOW);//最大1dB衰减通带，阻带至少80DB衰减

	//加速度计高阶低通滤波参数(25,80,2,40)
	//时延大概(1.5-2.25)*5ms,37hz处-3dB
	iir_high_order_filter_init(&accel_filter4[0],25,0,80,0,2,40,200,FILTER_IIR_BUTTER_LOW);//最大1dB衰减通带，阻带至少20DB衰减
	iir_high_order_filter_init(&accel_filter4[1],25,0,80,0,2,40,200,FILTER_IIR_BUTTER_LOW);//最大1dB衰减通带，阻带至少20DB衰减
	iir_high_order_filter_init(&accel_filter4[2],25,0,80,0,2,40,200,FILTER_IIR_BUTTER_LOW);//最大1dB衰减通带，阻带至少20DB衰减
	
	for(uint16_t i=0;i<1000;i++)
	{	
		IIR_High_Order_Filter(&gyro_filter4[0], 0.0f);
		IIR_High_Order_Filter(&gyro_filter4[1], 0.0f);
		IIR_High_Order_Filter(&gyro_filter4[2], 0.0f);
		IIR_High_Order_Filter(&accel_filter4[0],0.0f);	
		IIR_High_Order_Filter(&accel_filter4[1],0.0f);	
		IIR_High_Order_Filter(&accel_filter4[2],1.0f);			
	}	
}



Vector3f gyro,accel,mag;
Vector3f gyro_filter,accel_filter,mag_filter,ins_accel_filter;
lpf_buf ins_accel_filter_buf[3],gyro_filter_buf_bug[3],gyro_filter_buf[3],accel_filter_buf[3],accel_cal_filter_buf[3],accel_feedback_filter_buf[3];
lpf_buf loam_accel_buf[3],loam_gyro_buf[3];
uint8_t mag_update_flag=0;

void imu_sense_sampling(void)
{
  //传感器数据0.5~0.8ms采集完毕
	WP_Sensor.last_temperature=WP_Sensor.temperature;
  ICM20689_Read_Data(&WP_Sensor.gyro_raw,&WP_Sensor.accel_raw,&WP_Sensor.temperature);
  WP_Sensor.imu_updtate_flag=1;
	static Vector3f _gyro,_accel;  
  //得到校准后的角速度、加速度、磁力计数据
  _gyro.x=WP_Sensor.gyro_raw.x-gyro_offset.x;
  _gyro.y=WP_Sensor.gyro_raw.y-gyro_offset.y;
  _gyro.z=WP_Sensor.gyro_raw.z-gyro_offset.z;
  
  _accel.x=K[0]*WP_Sensor.accel_raw.x-B[0]*G_TO_RAW;
  _accel.y=K[1]*WP_Sensor.accel_raw.y-B[1]*G_TO_RAW;
  _accel.z=K[2]*WP_Sensor.accel_raw.z-B[2]*G_TO_RAW;
/************************************************************************************************/
	//当启用宇树科技4D激光雷达时,引入带阻滤波器对雷达转动的引入噪声,特定频率段数据进行滤波
//	if(current_state.slam_sensor==LOAM)
//	{
//		gyro.x =LPButterworth(_gyro.x,&loam_gyro_buf[0],&loam_ft);
//		gyro.y =LPButterworth(_gyro.y,&loam_gyro_buf[1],&loam_ft);
//		gyro.z =LPButterworth(_gyro.z,&loam_gyro_buf[2],&loam_ft);
//		accel.x=LPButterworth(_accel.x,&loam_accel_buf[0],&loam_ft);
//		accel.y=LPButterworth(_accel.y,&loam_accel_buf[1],&loam_ft);
//		accel.z=LPButterworth(_accel.z,&loam_accel_buf[2],&loam_ft);
//	}
//	else 
	{
		gyro=_gyro;
		accel=_accel;
	}
/************************************************************************************************/	
	//原始数据滤波处理	
  static float last_temperature;
	last_temperature=WP_Sensor._temperature;
  WP_Sensor._temperature=last_temperature+0.1f*(WP_Sensor.temperature-last_temperature);
/************************************************************************************************/
	accel_filter.x=LPButterworth(accel.x,&accel_filter_buf[0],&accel_lpf_param);
	accel_filter.y=LPButterworth(accel.y,&accel_filter_buf[1],&accel_lpf_param);
	accel_filter.z=LPButterworth(accel.z,&accel_filter_buf[2],&accel_lpf_param);		
	gyro_filter.x =LPButterworth(gyro.x,&gyro_filter_buf_bug[0],&gyro_lpf_param);
  gyro_filter.y =LPButterworth(gyro.y,&gyro_filter_buf_bug[1],&gyro_lpf_param);
  gyro_filter.z =LPButterworth(gyro.z,&gyro_filter_buf_bug[2],&gyro_lpf_param);	
/************************************************************************************************/
  ins_accel_filter.x=LPButterworth(accel.x,&ins_accel_filter_buf[0],&ins_lpf_param);
  ins_accel_filter.y=LPButterworth(accel.y,&ins_accel_filter_buf[1],&ins_lpf_param);
  ins_accel_filter.z=LPButterworth(accel.z,&ins_accel_filter_buf[2],&ins_lpf_param);
  WP_Sensor.acce_filter[0]=LPButterworth(WP_Sensor.accel_raw.x,&accel_cal_filter_buf[0],&cal_lpf_param);
  WP_Sensor.acce_filter[1]=LPButterworth(WP_Sensor.accel_raw.y,&accel_cal_filter_buf[1],&cal_lpf_param);
  WP_Sensor.acce_filter[2]=LPButterworth(WP_Sensor.accel_raw.z,&accel_cal_filter_buf[2],&cal_lpf_param);
	
  WP_Sensor.acce_filter_fb[0]=LPButterworth(accel.x,&accel_feedback_filter_buf[0],&accel_fb_lpf_param);//5
  WP_Sensor.acce_filter_fb[1]=LPButterworth(accel.y,&accel_feedback_filter_buf[1],&accel_fb_lpf_param);
  WP_Sensor.acce_filter_fb[2]=LPButterworth(accel.z,&accel_feedback_filter_buf[2],&accel_fb_lpf_param);
}



systime ins_sense_t0,ins_sense_t1;
float ins_read_dt=0;
void INS_Sensor_Update(void)
{
	imu_sense_sampling();//每1ms执行一次
	
	static uint16_t cnt=0;	cnt++;
	if(cnt<5)  return;cnt=0;//每5ms执行一次
	Get_Systime(&ins_sense_t0);	
  //计算三轴角速度，用于姿态内环控制
  WP_AHRS.Pitch_Gyro=gyro_filter.x*GYRO_CALIBRATION_COFF;
  WP_AHRS.Roll_Gyro =gyro_filter.y*GYRO_CALIBRATION_COFF;
  WP_AHRS.Yaw_Gyro  =gyro_filter.z*GYRO_CALIBRATION_COFF;
	
  WP_AHRS.Pitch_Gyro_Rad=DEG2RAD* WP_AHRS.Pitch_Gyro;
  WP_AHRS.Roll_Gyro_Rad =DEG2RAD* WP_AHRS.Roll_Gyro;
  WP_AHRS.Yaw_Gyro_Rad  =DEG2RAD* WP_AHRS.Yaw_Gyro;
	WP_AHRS.Accel_X_MPSS=accel.x*RAW_TO_G;
	WP_AHRS.Accel_Y_MPSS=accel.y*RAW_TO_G;
	WP_AHRS.Accel_Z_MPSS=accel.z*RAW_TO_G;
	Gyro_Length=FastSqrt(WP_AHRS.Yaw_Gyro*WP_AHRS.Yaw_Gyro+WP_AHRS.Pitch_Gyro*WP_AHRS.Pitch_Gyro+WP_AHRS.Roll_Gyro*WP_AHRS.Roll_Gyro);//单位deg/s
  
  Circle_Angle.Pit+=WP_AHRS.Pitch_Gyro*min_ctrl_dt;
  Circle_Angle.Rol+=WP_AHRS.Roll_Gyro*min_ctrl_dt;
  Circle_Angle.Yaw+=WP_AHRS.Yaw_Gyro*min_ctrl_dt;
  if(Circle_Angle.Pit<0.0f)   Circle_Angle.Pit+=360.0f;
  if(Circle_Angle.Pit>360.0f) Circle_Angle.Pit-=360.0f;
  if(Circle_Angle.Rol<0.0f)   Circle_Angle.Rol+=360.0f;
  if(Circle_Angle.Rol>360.0f) Circle_Angle.Rol-=360.0f;
  if(Circle_Angle.Yaw<0.0f)   Circle_Angle.Yaw+=360.0f;
  if(Circle_Angle.Yaw>360.0f) Circle_Angle.Yaw-=360.0f;
	
  Madgwick_AHRS_Update_IMU(gyro_filter.x,gyro_filter.y,gyro_filter.z,accel_filter.x,accel_filter.y,accel_filter.z,Gyro_Length);	
	
	
	WP_AHRS.Pitch= atan2f(2.0f * q2 * q3 + 2.0f * q0 * q1, -2.0f *q1 *q1 - 2.0f * q2* q2 + 1.0f) * RAD2DEG;		// Pitch
	WP_AHRS.Roll = asinf( 2.0f * q0 * q2 - 2.0f * q1 * q3) * RAD2DEG;																					// Roll

  WP_AHRS.q[0]=q0;
	WP_AHRS.q[1]=q1;
  WP_AHRS.q[2]=q2;
	WP_AHRS.q[3]=q3;
	
	SPL06_Read_Data(&WP_Sensor.baro_temp_raw,&WP_Sensor.baro_pressure_raw);
	Get_Baro_Offset();		
	
	Get_Systime(&ins_sense_t1);
	float tmp=ins_sense_t1.current_time-ins_sense_t0.current_time;
	if(ins_read_dt<tmp) ins_read_dt=tmp;
}


void OBS_Sensor_Update(void)
{
	Gyro_Calibration_Check(&WP_Sensor.gyro_raw);
  mag_update_flag=Compass_Read_Data(&WP_Sensor.mag_raw);
	Compass_Fault_Check();  
  mag.x=WP_Sensor.mag_raw.x-mag_offset.x;
  mag.y=WP_Sensor.mag_raw.y-mag_offset.y;
  mag.z=WP_Sensor.mag_raw.z-mag_offset.z;
  //原始数据滤波处理 
	WP_Sensor.mag_intensity=pythagorous3(mag.x,mag.y,mag.z);

	Observation_Angle_Calculate();//观测量计算
}



void Observation_Angle_Calculate(void)
{
  float ACCE_X_TEMP,ACCE_Y_TEMP,ACCE_Z_TEMP;
  ACCE_X_TEMP=accel.x;
  ACCE_Y_TEMP=accel.y;
  ACCE_Z_TEMP=accel.z;
  Roll_Observation=-57.3*atan(ACCE_X_TEMP*invSqrt(ACCE_Y_TEMP*ACCE_Y_TEMP+ACCE_Z_TEMP*ACCE_Z_TEMP));//横滚角
  Pitch_Observation=57.3*atan(ACCE_Y_TEMP*invSqrt(ACCE_X_TEMP*ACCE_X_TEMP+ACCE_Z_TEMP*ACCE_Z_TEMP));//俯仰角
  /************磁力计倾角补偿*****************/
	if(mag_update_flag==1)
	{
		vector3f mb=mag;  			
		/***********反正切得到磁力计观测角度*********/
    //东北天ENU+右前上xyz,按ZYX顺规,新版
		magn.x=  mb.x * WP_AHRS.cos_rpy[_ROL]
						+mb.y * WP_AHRS.sin_rpy[_ROL] * WP_AHRS.sin_rpy[_PIT]
						+mb.z * WP_AHRS.sin_rpy[_ROL] * WP_AHRS.cos_rpy[_PIT];		
		magn.y=  mb.y * WP_AHRS.cos_rpy[_PIT]- mb.z * WP_AHRS.sin_rpy[_PIT];
//		//东北天ENU+右前上xyz,按ZXY顺规,旧版
//		magn.x=  mb.x * WP_AHRS.cos_rpy[_ROL]+ mb.z * WP_AHRS.sin_rpy[_ROL];
//		magn.y=  mb.x * WP_AHRS.sin_rpy[_PIT]*WP_AHRS.sin_rpy[_ROL]
//						+mb.y * WP_AHRS.cos_rpy[_PIT]
//						-mb.z * WP_AHRS.sin_rpy[_PIT]*WP_AHRS.cos_rpy[_ROL];
		Yaw_Observation=FastAtan2(magn.x,magn.y)*57.296f;	
		if(Yaw_Observation<0) Yaw_Observation=Yaw_Observation+360;
		Yaw_Observation=constrain_float(Yaw_Observation,0,360);
	}
}


void Euler_Angle_Init()
{
  float ACCE_X_TEMP,ACCE_Y_TEMP,ACCE_Z_TEMP;    
  ICM20689_Read_Data(&WP_Sensor.gyro_raw,&WP_Sensor.accel_raw,&WP_Sensor.temperature);
  accel.x=K[0]*WP_Sensor.accel_raw.x-B[0]*G_TO_RAW;
  accel.y=K[1]*WP_Sensor.accel_raw.y-B[1]*G_TO_RAW;
  accel.z=K[2]*WP_Sensor.accel_raw.z-B[2]*G_TO_RAW;
  ACCE_X_TEMP=accel.x;
  ACCE_Y_TEMP=accel.y;
  ACCE_Z_TEMP=accel.z;
  Roll_Observation=-57.3*atan(ACCE_X_TEMP*invSqrt(ACCE_Y_TEMP*ACCE_Y_TEMP+ACCE_Z_TEMP*ACCE_Z_TEMP));//横滚角
  Pitch_Observation=57.3*atan(ACCE_Y_TEMP*invSqrt(ACCE_X_TEMP*ACCE_X_TEMP+ACCE_Z_TEMP*ACCE_Z_TEMP));//俯仰角
  
  WP_AHRS.sin_rpy[_PIT]=sin(Pitch_Observation* DEG2RAD);
  WP_AHRS.cos_rpy[_PIT]=cos(Pitch_Observation* DEG2RAD);
  WP_AHRS.sin_rpy[_ROL]=sin(Roll_Observation* DEG2RAD);
  WP_AHRS.cos_rpy[_ROL]=cos(Roll_Observation* DEG2RAD);
	
	if(Sensor_Flag.Mag_Health==TRUE)
	{
		for(uint16_t i=0;i<20;i++)
		{
			delay_ms(5);
			Compass_Read_Data(&WP_Sensor.mag_raw);
		}
		/************磁力计倾角补偿*****************/
		mag.x=WP_Sensor.mag_raw.x-mag_offset.x;
		mag.y=WP_Sensor.mag_raw.y-mag_offset.y;
		mag.z=WP_Sensor.mag_raw.z-mag_offset.z;  
		/***********反正切得到磁力计观测角度*********/
		vector3f mb=mag; 
		magn.x=  mb.x * WP_AHRS.cos_rpy[_ROL]
						+mb.y * WP_AHRS.sin_rpy[_ROL] * WP_AHRS.sin_rpy[_PIT]
						+mb.z * WP_AHRS.sin_rpy[_ROL] * WP_AHRS.cos_rpy[_PIT];		
		magn.y=  mb.y * WP_AHRS.cos_rpy[_PIT]- mb.z * WP_AHRS.sin_rpy[_PIT];	
		Yaw_Observation=FastAtan2(magn.x,magn.y)*57.296f;	
		if(Yaw_Observation<0) Yaw_Observation=Yaw_Observation+360;
		Yaw_Observation=constrain_float(Yaw_Observation,0,360);	
		yaw_angle_deg_enu=Yaw_Observation;
	}
	else
	{
		Yaw_Observation=0;	
		yaw_angle_deg_enu=0;	
	}
	
  euler_rpy_init[0]=Roll_Observation;  //初始化欧拉翻滚角
  euler_rpy_init[1]=Pitch_Observation; //初始化欧拉俯仰角
  euler_rpy_init[2]  =Yaw_Observation ;
}



void Euler_Angle_Init_Again(void)
{
  float ACCE_X_TEMP,ACCE_Y_TEMP,ACCE_Z_TEMP; 
  accel.x=K[0]*WP_Sensor.accel_raw.x-B[0]*G_TO_RAW;
  accel.y=K[1]*WP_Sensor.accel_raw.y-B[1]*G_TO_RAW;
  accel.z=K[2]*WP_Sensor.accel_raw.z-B[2]*G_TO_RAW;
  ACCE_X_TEMP=accel.x;
  ACCE_Y_TEMP=accel.y;
  ACCE_Z_TEMP=accel.z;
  Roll_Observation=-57.3*atan(ACCE_X_TEMP*invSqrt(ACCE_Y_TEMP*ACCE_Y_TEMP+ACCE_Z_TEMP*ACCE_Z_TEMP));//横滚角
  Pitch_Observation=57.3*atan(ACCE_Y_TEMP*invSqrt(ACCE_X_TEMP*ACCE_X_TEMP+ACCE_Z_TEMP*ACCE_Z_TEMP));//俯仰角  
  WP_AHRS.sin_rpy[_PIT]=sin(Pitch_Observation* DEG2RAD);
  WP_AHRS.cos_rpy[_PIT]=cos(Pitch_Observation* DEG2RAD);
  WP_AHRS.sin_rpy[_ROL]=sin(Roll_Observation* DEG2RAD);
  WP_AHRS.cos_rpy[_ROL]=cos(Roll_Observation* DEG2RAD);
	
	if(Sensor_Flag.Mag_Health==TRUE)
	{
		/************磁力计倾角补偿*****************/
		mag.x=WP_Sensor.mag_raw.x-mag_offset.x;
		mag.y=WP_Sensor.mag_raw.y-mag_offset.y;
		mag.z=WP_Sensor.mag_raw.z-mag_offset.z;  
		/***********反正切得到磁力计观测角度*********/
		vector3f mb=mag; 
		magn.x=  mb.x * WP_AHRS.cos_rpy[_ROL]
						+mb.y * WP_AHRS.sin_rpy[_ROL] * WP_AHRS.sin_rpy[_PIT]
						+mb.z * WP_AHRS.sin_rpy[_ROL] * WP_AHRS.cos_rpy[_PIT];		
		magn.y=  mb.y * WP_AHRS.cos_rpy[_PIT]- mb.z * WP_AHRS.sin_rpy[_PIT];
		/***********反正切得到磁力计观测角度*********/
		Yaw_Observation=FastAtan2(magn.x,magn.y)*57.296f;
		if(Yaw_Observation<0) Yaw_Observation=Yaw_Observation+360;
		Yaw_Observation=constrain_float(Yaw_Observation,0,360);
		yaw_angle_deg_enu=Yaw_Observation;			
	}
	else
	{
		Yaw_Observation=0;	
		yaw_angle_deg_enu=0;	
	}

  euler_rpy_init[0]=Roll_Observation;  //初始化欧拉翻滚角
  euler_rpy_init[1]=Pitch_Observation; //初始化欧拉俯仰角
  euler_rpy_init[2]=Yaw_Observation ;
	quad_from_euler(q_init,euler_rpy_init[0],euler_rpy_init[1],euler_rpy_init[2]);
  q0=q_init[0];
  q1=q_init[1];
  q2=q_init[2];
  q3=q_init[3];
	
	for(int16_t i=9;i>0;i--)
	{ 
	  q_backup[0][i]=q0;
		q_backup[1][i]=q1;
		q_backup[2][i]=q2;
		q_backup[3][i]=q3;
	}
		q_backup[0][0]=q0;
		q_backup[1][0]=q1;
		q_backup[2][0]=q2;
		q_backup[3][0]=q3;
}



void quad_from_euler(float *q, float roll, float pitch, float yaw)
{
	float sPitch2, cPitch2; // sin(phi/2) and cos(phi/2)
	float sRoll2 , cRoll2;  // sin(theta/2) and cos(theta/2)
	float sYaw2  , cYaw2;   // sin(psi/2) and cos(psi/2)
	//calculate sines and cosines
	
	FastSinCos(0.5f * roll*DEG2RAD, &sRoll2, &cRoll2);//roll
	FastSinCos(0.5f * pitch*DEG2RAD,&sPitch2,&cPitch2);//pitch
	FastSinCos(0.5f * yaw*DEG2RAD,  &sYaw2,  &cYaw2);//yaw
	

	// compute the quaternion elements
	q[0] = cPitch2*cRoll2*cYaw2+sPitch2*sRoll2*sYaw2;
	q[1] = sPitch2*cRoll2*cYaw2-cPitch2*sRoll2*sYaw2;
	q[2] = cPitch2*sRoll2*cYaw2+sPitch2*cRoll2*sYaw2;
	q[3] = cPitch2*cRoll2*sYaw2-sPitch2*sRoll2*cYaw2;

  // Normalise quaternion
  float recipNorm = invSqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
  q[0] *= recipNorm;
  q[1] *= recipNorm;
  q[2] *= recipNorm;
  q[3] *= recipNorm;
}


/****************** 根据初始化欧拉角初始化四元数 *****************************/
void Quaternion_Init()
{
  Euler_Angle_Init();
	quad_from_euler(q_init,euler_rpy_init[0],euler_rpy_init[1],euler_rpy_init[2]);
}

void WP_Quad_Init(void)//初始四元数初始化
{
  Quaternion_Init();
  q0=q_init[0];
  q1=q_init[1];
  q2=q_init[2];
  q3=q_init[3];
	
	for(int16_t i=9;i>0;i--)
	{ 
	  q_backup[0][i]=q0;
		q_backup[1][i]=q1;
		q_backup[2][i]=q2;
		q_backup[3][i]=q3;
	}
		q_backup[0][0]=q0;
		q_backup[1][0]=q1;
		q_backup[2][0]=q2;
		q_backup[3][0]=q3;
}
Testime Imu_Delta;
float IMU_Dt=0.0f;
float Yaw_Gyro_Earth_Frame=0;
#define ahrs_sync 0
uint8_t ahrs_sync_flag=0;

volatile float beta=0.01f;//0.0175
float yaw_fus_beta=0.025f;//0.025f
float slam_yaw_fus_beta[3]={0.025f,0.15f,0.10f};//0.025,0.15,0.15
uint16_t slam_yaw_fus_sync[3]={10,20,5};//10,20,10
float accel_mode=0;
void Madgwick_AHRS_Update_IMU(float gx, float gy, float gz, 
                              float ax, float ay, float az,
                              float gyro_mold) 
{
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2;
  float q0q0, q1q1, q2q2, q3q3;  
  float beta_temp=0;
	static uint16_t ahrs_sync_cnt=0;
  Test_Period(&Imu_Delta);
  IMU_Dt=(float)(Imu_Delta.Time_Delta/1000.0f);
	if(IMU_Dt>1.05f*min_ctrl_dt||IMU_Dt<0.95f*min_ctrl_dt||isnan(IMU_Dt)!=0)   IMU_Dt=min_ctrl_dt;
  gx*=GYRO_CALIBRATION_COFF;
  gy*=GYRO_CALIBRATION_COFF;
  gz*=GYRO_CALIBRATION_COFF;
  //{-sinθ          cosθsin Φ                          cosθcosΦ                   }
  Yaw_Gyro_Earth_Frame=-WP_AHRS.sin_rpy[_ROL]*gx+ WP_AHRS.cos_rpy[_ROL]*WP_AHRS.sin_rpy[_PIT] *gy+ WP_AHRS.cos_rpy[_PIT] * WP_AHRS.cos_rpy[_ROL] *gz;
  
  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz)*DEG2RAD;
  qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy)*DEG2RAD;
  qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx)*DEG2RAD;
  qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx)*DEG2RAD;
  
	ahrs_sync_cnt++;
	if(ahrs_sync_cnt>=2)//10ms
	{
		for(int16_t i=9;i>0;i--)
		{ 
			q_backup[0][i]=q_backup[0][i-1];
			q_backup[1][i]=q_backup[1][i-1];
			q_backup[2][i]=q_backup[2][i-1];
			q_backup[3][i]=q_backup[3][i-1];
		}
		ahrs_sync_cnt=0;
		ahrs_sync_flag=1;
	}
		q_backup[0][0]=q0;
		q_backup[1][0]=q1;
		q_backup[2][0]=q2;
		q_backup[3][0]=q3;
		
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))&&ahrs_sync_flag==1) 
  {
		accel_mode=GRAVITY_MSS*((safe_sqrt(sq(ax)+sq(ay)+sq(az))/GRAVITY_RAW)-1.0f);
    ahrs_sync_flag=0;		
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;     
    
    // Auxiliary variables to avoid repeated arithmetic  
		_2q0=2.0f *q_backup[0][ahrs_sync];//_2q0 = 2.0f * q0;
		_2q1=2.0f *q_backup[1][ahrs_sync];//_2q1 = 2.0f * q1;
		_2q2=2.0f *q_backup[2][ahrs_sync];//_2q2 = 2.0f * q2;
		_2q3=2.0f *q_backup[3][ahrs_sync];//_2q3 = 2.0f * q3;	
		_4q0=4.0f *q_backup[0][ahrs_sync];//_4q0 = 4.0f * q0;
		_4q1=4.0f *q_backup[1][ahrs_sync];//_4q1 = 4.0f * q1;
		_4q2=4.0f *q_backup[2][ahrs_sync];//_4q2 = 4.0f * q2;
		_8q1=8.0f *q_backup[1][ahrs_sync];//_8q1 = 8.0f * q1;
		_8q2=8.0f *q_backup[2][ahrs_sync];//_8q2 = 8.0f * q2;
		q0q0 = q_backup[0][ahrs_sync] * q_backup[0][ahrs_sync];//		 q0q0 = q0 * q0;
    q1q1 = q_backup[1][ahrs_sync] * q_backup[1][ahrs_sync];//    q1q1 = q1 * q1;
    q2q2 = q_backup[2][ahrs_sync] * q_backup[2][ahrs_sync];//    q2q2 = q2 * q2;
    q3q3 = q_backup[3][ahrs_sync] * q_backup[3][ahrs_sync];//    q3q3 = q3 * q3;		  
    
    
    
    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q_backup[1][ahrs_sync] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q_backup[2][ahrs_sync] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q_backup[3][ahrs_sync] - _2q1 * ax + 4.0f * q2q2 * q_backup[3][ahrs_sync] - _2q2 * ay;
    
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;
    
    beta_temp=beta+0.01f*IMU_Dt*constrain_float(gyro_mold,0,500);//0.0175f  0.025 
		beta_temp-=0.0005f*(accel_mode/GRAVITY_MSS);//0.005
    beta_temp=constrain_float(beta_temp,beta,0.06f);
    
    // Apply feedback step
    qDot1 -= beta_temp * s0;
    qDot2 -= beta_temp * s1;
    qDot3 -= beta_temp * s2;
    qDot4 -= beta_temp * s3;
  }
  
  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * IMU_Dt;
  q1 += qDot2 * IMU_Dt;
  q2 += qDot3 * IMU_Dt;
  q3 += qDot4 * IMU_Dt;
  
  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}


Testime yaw_fus_time;
void Get_Status_Feedback(void)
{
	float yaw_obs_err=0;
	Test_Period(&yaw_fus_time);
	float yaw_fus_dt=(float)(yaw_fus_time.Time_Delta/1000.0f);
  if(yaw_fus_dt>1.05f*WP_Duty_Dt||yaw_fus_dt<0.95f*WP_Duty_Dt||isnan(yaw_fus_dt)!=0)   yaw_fus_dt=WP_Duty_Dt;
	
	if(current_state.rec_update_flag==1
	 &&current_state.fault==0//定位异常，遥控器手动操作设定
	 &&Optical_Type_Present==3)//设置的定位类型为RPLIDAR/T265
	{
		uint8_t yaw_fusion_label=0;
		//偏航角一阶互补融合
		yaw_angle_deg_enu+=Yaw_Gyro_Earth_Frame*yaw_fus_dt;
		if(yaw_angle_deg_enu<0)   yaw_angle_deg_enu+=360;
		if(yaw_angle_deg_enu>360) yaw_angle_deg_enu-=360;
		
		static uint16_t _cnt=0;	_cnt++;
		if(_cnt>=2)
		{
			_cnt=0;
			for(uint16_t i=29;i>0;i--)	yaw_angle_deg_enu_backups[i]=yaw_angle_deg_enu_backups[i-1];
		}
		yaw_angle_deg_enu_backups[0]=yaw_angle_deg_enu;
		
		yaw_obs_err=current_state.rpy[2]-yaw_angle_deg_enu_backups[slam_yaw_fus_sync[yaw_fusion_label]];
		if(current_state.rpy[2]>270&&yaw_angle_deg_enu_backups[slam_yaw_fus_sync[yaw_fusion_label]]<90)      
			yaw_obs_err=yaw_obs_err-360;
		else if(current_state.rpy[2]<90&&yaw_angle_deg_enu_backups[slam_yaw_fus_sync[yaw_fusion_label]]>270) 
			yaw_obs_err=360+yaw_obs_err;
				
		if(current_state.rec_head_update_flag==1)
		{
			current_state.rec_head_update_flag=0;
			quad_getangle(current_state.q,current_state.rpy);
			if(current_state.rpy[2]<0.0f)   current_state.rpy[2]+=360.0f;
			if(current_state.rpy[2]>360.0f) current_state.rpy[2]-=360.0f;		
			yaw_angle_deg_enu +=yaw_obs_err* slam_yaw_fus_beta[yaw_fusion_label];
			//偏航角数据量化到0~360
		}
		if(yaw_angle_deg_enu<0)   WP_AHRS.Yaw=yaw_angle_deg_enu+360;
		else WP_AHRS.Yaw=yaw_angle_deg_enu;
	}
	else if(current_state.rec_update_flag==1
		    &&current_state.fault==0//定位异常，遥控器手动操作设定
		    &&Optical_Type_Present==4)//设置的定位类型为3D_LIDAR
	{
		uint8_t yaw_fusion_label=0;
		if(current_state.slam_sensor==LOAM) yaw_fusion_label=2;//根据slam定位传感器类型，选择与之匹配的融合参数
		//偏航角一阶互补融合
		yaw_angle_deg_enu+=Yaw_Gyro_Earth_Frame*yaw_fus_dt;
		if(yaw_angle_deg_enu<0)   yaw_angle_deg_enu+=360;
		if(yaw_angle_deg_enu>360) yaw_angle_deg_enu-=360;
		
		static uint16_t _cnt=0;
		_cnt++;
		if(_cnt>=2)
		{
			_cnt=0;
			for(uint16_t i=29;i>0;i--)	yaw_angle_deg_enu_backups[i]=yaw_angle_deg_enu_backups[i-1];
		}
		yaw_angle_deg_enu_backups[0]=yaw_angle_deg_enu;
						
		if(current_state.rec_head_update_flag==1)//slam输出的观测偏航角更新时去修正偏航
		{
			current_state.rec_head_update_flag=0;
			quad_getangle(current_state.q,current_state.rpy);
			if(current_state.rpy[2]<0.0f)   current_state.rpy[2]+=360.0f;
			if(current_state.rpy[2]>360.0f) current_state.rpy[2]-=360.0f;
			
			//减去初始slam偏航角偏置
			WP_Sensor.slam_yaw=current_state.rpy[2];

			//if(WP_Sensor.slam_yaw_setup==1) WP_Sensor.slam_yaw-=WP_Sensor.slam_yaw_init;//初始航向对准
			
		  if(WP_Sensor.slam_yaw<0)   WP_Sensor.slam_yaw+=360;
		  if(WP_Sensor.slam_yaw>360) WP_Sensor.slam_yaw-=360;	
			
			//偏航误差
			yaw_obs_err=WP_Sensor.slam_yaw-yaw_angle_deg_enu_backups[slam_yaw_fus_sync[yaw_fusion_label]];
			if(WP_Sensor.slam_yaw>270&&yaw_angle_deg_enu_backups[slam_yaw_fus_sync[yaw_fusion_label]]<90)      	yaw_obs_err=yaw_obs_err-360;
			else if(WP_Sensor.slam_yaw<90&&yaw_angle_deg_enu_backups[slam_yaw_fus_sync[yaw_fusion_label]]>270) 	yaw_obs_err=360+yaw_obs_err;
			//误差修正
			yaw_angle_deg_enu +=yaw_obs_err* slam_yaw_fus_beta[yaw_fusion_label];
		}
		//偏航角数据量化到0~360
		if(yaw_angle_deg_enu<0)   WP_AHRS.Yaw=yaw_angle_deg_enu+360;
		else WP_AHRS.Yaw=yaw_angle_deg_enu;
		
		//slam偏航角偏置提取
		static uint16_t slam_yaw_cnt=0;
		if(slam_yaw_cnt<200&&WP_Sensor.slam_yaw_setup==0)
		{
			if(ABS(yaw_obs_err)<1.0f) slam_yaw_cnt++;
			else slam_yaw_cnt/=2;
		}
		else if(WP_Sensor.slam_yaw_setup==0)
		{	
			WP_Sensor.slam_yaw_setup=1;
			WP_Sensor.slam_yaw_init=WP_AHRS.Yaw;
			if(WP_Sensor.slam_yaw_init>180) WP_Sensor.slam_yaw_init-=360.0f;	
			//slam偏航偏置记录完毕蜂鸣器提示
			buzzer_setup(500,0.25f,2);
		}
	}
	else if(Sensor_Flag.Mag_Health==TRUE)//磁力计参与下的偏航角融合
	{		
		//偏航角一阶互补融合
		yaw_angle_deg_enu+=Yaw_Gyro_Earth_Frame*yaw_fus_dt;
		if(yaw_angle_deg_enu<0)   yaw_angle_deg_enu+=360;
		if(yaw_angle_deg_enu>360) yaw_angle_deg_enu-=360;
		
	  yaw_obs_err=Yaw_Observation-yaw_angle_deg_enu;   
		if(yaw_obs_err>180) yaw_obs_err=yaw_obs_err-360;
		else if(yaw_obs_err<-180) yaw_obs_err=yaw_obs_err+360;
	
		if(mag_update_flag==1)
		{
			mag_update_flag=0;			
			yaw_angle_deg_enu +=yaw_obs_err* yaw_fus_beta;
			//偏航角数据量化到0~360
	  }
		if(yaw_angle_deg_enu<0)   WP_AHRS.Yaw=yaw_angle_deg_enu+360;
		else WP_AHRS.Yaw=yaw_angle_deg_enu;
		if(GPS_Home_Set==1)  WP_AHRS.Yaw=WP_AHRS.Yaw-Declination;//如果GPS home点已设置，获取当地磁偏角，得到地理真北
	}
	else
	{
		WP_AHRS.Yaw = FastAtan2(2.0f * q1 * q2 + 2.0f * q0 * q3, -2.0f * q3 *q3 - 2.0f * q2 * q2 + 1.0f) * RAD2DEG;		// Yaw
	}
	
  if(WP_AHRS.Yaw<0.0f)   WP_AHRS.Yaw+=360.0f;
  if(WP_AHRS.Yaw>360.0f) WP_AHRS.Yaw-=360.0f;
	
	if(current_state.last_fault!=current_state.fault)	Total_Controller.Yaw_Angle_Control.Expect=WP_AHRS.Yaw;

	quad_from_euler(WP_AHRS.quaternion,WP_AHRS.Roll,WP_AHRS.Pitch,WP_AHRS.Yaw);	
	Compute_Rotation_Matrix_From_Quad(WP_AHRS.rMat);//Compute_Rotation_Matrix_From_Euler(WP_AHRS.rMat);
	
  //更新位置、速度、加速度
  SINS_Prepare();//惯导加速度更新
  Strapdown_INS_High_Kalman();//高度方向卡尔曼滤波估计竖直速度、位置

/*****************************************************************************/
	//计算航程
	static uint16_t cnt=0;cnt++;
	static vector3f last_pos_3d;
	int16_t pos_delta=0;
	if(cnt>=100)//每500ms计算一次位置增量
	{
		cnt=0;	
		pos_delta=pythagorous3(VIO_SINS.Position[_EAST]-last_pos_3d.x,
												   VIO_SINS.Position[_NORTH]-last_pos_3d.y,
												   NamelessQuad.Position[_UP]-last_pos_3d.z);
		//保持历史值
		last_pos_3d.x=VIO_SINS.Position[_EAST];
		last_pos_3d.y=VIO_SINS.Position[_NORTH];
		last_pos_3d.z=NamelessQuad.Position[_UP];
	}
	WP_Sensor.distance_3d_cm+=pos_delta;
/*****************************************************************************/	
}




