#ifndef __SENSOR_H
#define __SENSOR_H

extern float K[3];
extern float B[3];

#define RtA         57.324841
#define AtR    	    0.0174533
#define Acc_G 	    0.0000610351
#define Gyro_G 	    0.0610351
#define Gyro_Gr	    0.0010653
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)
//#define GYRO_CALIBRATION_COFF 0.060976f  //2000
#define GYRO_CALIBRATION_COFF 0.030488f; //1000
//#define GYRO_CALIBRATION_COFF 0.0152672f;  //500



void INS_Sensor_Update(void);
void OBS_Sensor_Update(void);
void quad_from_euler(float *q, float roll, float pitch, float yaw);
void WP_Quad_Init(void);
void Senosr_Update(void);	
void Observation_Angle_Calculate(void);
void Madgwick_AHRS_Update_IMU(float gx, float gy, float gz, 
															float ax, float ay, float az,
															float gyro_mold);
void Get_Status_Feedback(void);
void Euler_Angle_Init_Again(void);
void sensor_filter_init(void);

															
extern float Yaw_Gyro_Earth_Frame;
extern float Pitch_Observation,Roll_Observation,Yaw_Observation,Altitude_Observation;
extern Sensor WP_Sensor;
extern Vector3f gyro,accel,mag;
extern Vector3f gyro_filter,accel_filter,mag_filter,ins_accel_filter;
extern Sensor_Health Sensor_Flag;
extern Vector3f_Body Circle_Angle;
extern float Gyro_Length;
extern float IMU_Dt;
extern float yaw_angle_deg_enu; 
extern float Sensor_Time;

#endif

//		uint8_t yaw_fusion_label=0;
//		if(current_state.slam_sensor==LOAM) yaw_fusion_label=1;//根据slam定位传感器类型，选择与之匹配的融合参数
//		//偏航角一阶互补融合
//		yaw_angle_deg_enu+=Yaw_Gyro_Earth_Frame*yaw_fus_dt;
//		if(yaw_angle_deg_enu<0)   yaw_angle_deg_enu+=360;
//		if(yaw_angle_deg_enu>360) yaw_angle_deg_enu-=360;
//		
//		static uint16_t _cnt=0;
//		_cnt++;
//		if(_cnt>=2)
//		{
//			_cnt=0;
//			for(uint16_t i=29;i>0;i--)	yaw_angle_deg_enu_backups[i]=yaw_angle_deg_enu_backups[i-1];
//		}
//		yaw_angle_deg_enu_backups[0]=yaw_angle_deg_enu;
//						
//		if(current_state.rec_head_update_flag==1)//slam输出的观测偏航角更新时去修正偏航
//		{
//			current_state.rec_head_update_flag=0;
//			quad_getangle(current_state.q,current_state.rpy);
//			if(current_state.rpy[2]<0.0f)   current_state.rpy[2]+=360.0f;
//			if(current_state.rpy[2]>360.0f) current_state.rpy[2]-=360.0f;
//			
//			//减去初始slam偏航角偏置
//			WP_Sensor.slam_yaw=current_state.rpy[2];
//			if(RC_Data.aux[AUX3]>1200) 
//			{
//				if(WP_Sensor.slam_yaw_setup==1) WP_Sensor.slam_yaw-=WP_Sensor.slam_yaw_init;
//			}
//		  if(WP_Sensor.slam_yaw<0)   WP_Sensor.slam_yaw+=360;
//		  if(WP_Sensor.slam_yaw>360) WP_Sensor.slam_yaw-=360;	
//			
//			//偏航误差
//			yaw_obs_err=WP_Sensor.slam_yaw-yaw_angle_deg_enu_backups[slam_yaw_fus_sync[yaw_fusion_label]];
//			if(WP_Sensor.slam_yaw>270&&yaw_angle_deg_enu_backups[slam_yaw_fus_sync[yaw_fusion_label]]<90)      	yaw_obs_err=yaw_obs_err-360;
//			else if(WP_Sensor.slam_yaw<90&&yaw_angle_deg_enu_backups[slam_yaw_fus_sync[yaw_fusion_label]]>270) 	yaw_obs_err=360+yaw_obs_err;
//			//误差修正
//			yaw_angle_deg_enu +=yaw_obs_err* slam_yaw_fus_beta[yaw_fusion_label];
//			//偏航角数据量化到0~360
//		}
//		
//		if(yaw_angle_deg_enu<0)   WP_AHRS.Yaw=yaw_angle_deg_enu+360;
//		else WP_AHRS.Yaw=yaw_angle_deg_enu;
//		
//		//slam偏航角速度偏执提取
//		static uint16_t slam_yaw_cnt=0;
//		if(slam_yaw_cnt<200&&WP_Sensor.slam_yaw_setup==0)
//		{
//			if(ABS(yaw_obs_err)<1.0f) slam_yaw_cnt++;
//			else slam_yaw_cnt/=2;
//		}
//		else if(WP_Sensor.slam_yaw_setup==0)
//		{	
//			WP_Sensor.slam_yaw_setup=1;
//			WP_Sensor.slam_yaw_init=WP_AHRS.Yaw;
//			if(WP_Sensor.slam_yaw_init>180) WP_Sensor.slam_yaw_init-=360.0f;	
//			//slam偏航偏置记录完毕蜂鸣器提示
//			buzzer_setup(500,0.25f,2);
//		}


//	accel_filter.x=iir_high_order_filter_both_export(&accel_filter4[0],accel.x);
//	accel_filter.y=iir_high_order_filter_both_export(&accel_filter4[1],accel.y);
//	accel_filter.z=iir_high_order_filter_both_export(&accel_filter4[2],accel.z);
//	gyro_filter.x =iir_high_order_filter_both_export(&gyro_filter4[0],gyro.x);
//	gyro_filter.y =iir_high_order_filter_both_export(&gyro_filter4[1],gyro.y);
//	gyro_filter.z =iir_high_order_filter_both_export(&gyro_filter4[2],gyro.z);


