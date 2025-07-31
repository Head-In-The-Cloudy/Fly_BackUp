#ifndef __NCLINK_H__
#define __NCLINK_H__


#define RESERVED_PARAM_NUM 200
extern float param_value[RESERVED_PARAM_NUM];


#define BYTE0(dwTemp)  (*((char *)(&dwTemp)))
#define BYTE1(dwTemp)  (*((char *)(&dwTemp)+1))
#define BYTE2(dwTemp)  (*((char *)(&dwTemp)+2))
#define BYTE3(dwTemp)  (*((char *)(&dwTemp)+3))
	
#define NCLINK_STATUS         0x01//发送飞控状态数据，功能字节
#define NCLINK_SENSOR         0x02//发送传感器原始数据，功能字节
#define NCLINK_RCDATA         0x03//发送遥控器解析数据，功能字节
#define NCLINK_GPS 		        0x04//发送GPS经纬度、海拔数据，功能字节
#define NCLINK_OBS_NE         0x05//发送NE方向观测数据，功能字节
#define NCLINK_OBS_UOP        0x06//发送U方向观测数据，功能字节
#define NCLINK_FUS_U          0x07//发送U方向融合数据，功能字节
#define NCLINK_FUS_NE         0x08//发送NE方向融合数据，功能字节
#define NCLINK_USER           0x09//发送用户数据，功能字节

#define NCLINK_SEND_PID1_3    0x0A//PID01-03数据，功能字节+应答字节
#define NCLINK_SEND_PID4_6    0x0B//PID04-06数据，功能字节+应答字节
#define NCLINK_SEND_PID7_9    0x0C//PID07-09数据，功能字节+应答字节
#define NCLINK_SEND_PID10_12  0x0D//PID10-12数据，功能字节+应答字节
#define NCLINK_SEND_PID13_15  0x0E//PID13-15数据，功能字节+应答字节
#define NCLINK_SEND_PID16_18  0x0F//PID16-18数据，功能字节+应答字节

#define NCLINK_SEND_PARA      0x10//飞控发送其它参数，功能字节+应答字节
#define NCLINK_SEND_CAL_RAW1  0x11//发送校准原始数据1，功能字节
#define NCLINK_SEND_CAL_RAW2  0x12//发送校准原始数据2，功能字节
#define NCLINK_SEND_CAL_PARA1 0x13//发送校准结果数据1，功能字节
#define NCLINK_SEND_CAL_PARA2 0x14//发送校准结果数据2，功能字节
#define NCLINK_SEND_CAL_PARA3 0x15//发送校准结果数据3，功能字节

#define NCLINK_SEND_PARA_RESERVED		0x16//飞控发送预留参数，功能字节+应答字节
#define NCLINK_SEND_3D_TRACK				0x17//飞控3D位姿数据发送参数



#define NCLINK_SEND_CHECK     					0xF0//发送应答数据，功能字节
//应答数据
#define NCLINK_SEND_RC        					0xF1//飞控解析遥控器控制指令，应答字节
#define NCLINK_SEND_DIS       					0xF2//飞控解析位移控制指令，应答字节
#define NCLINK_SEND_CAL       					0xF3//飞控传感器校准完毕，应答字节
#define NCLINK_SEND_CAL_READ  					0xF4//读取校准参数成功，应答字节
#define NCLINK_SEND_FACTORY   					0xF5//恢复出厂设置成功，应答字节
#define NCLINK_SEND_NAV_CTRL  					0xF6//飞控解析导航控制指令，应答字节
#define NCLINK_SEND_NAV_CTRL_FINISH     0xF7//飞控导航控制完毕，应答字节
#define NCLINK_SEND_SLAM_SYSTEM_RESET   0xF8//SLAM导航复位完毕，控制字节
#define NCLINK_SEND_SLAM_STOP_MOTOR     0xF9//SLAM控制电机停止，控制字节
#define NCLINK_SEND_SLAM_START_MOTOR    0xFA//SLAM控制电机开始，控制字节
//数据结构声明
typedef enum
{
	SDK_FRONT = 0x00,
	SDK_BEHIND,
	SDK_LEFT,
	SDK_RIGHT,
	SDK_UP,
	SDK_DOWN
}SDK_MODE;

typedef struct
{
  bool sdk_front_flag;
	bool sdk_behind_flag;
	bool sdk_left_flag;
	bool sdk_right_flag;
	bool sdk_up_flag;
	bool sdk_down_flag;
}sdk_mode_flag;


typedef struct
{
  uint8_t move_mode;//SDK移动标志位
  uint8_t mode_order;//SDK移动顺序
  uint16_t move_distance;//SDK移动距离
	bool update_flag;
	sdk_mode_flag move_flag;
	float f_distance;
}ngs_sdk_control;

extern ngs_sdk_control ngs_sdk;

extern uint8_t NCLink_Head[2];
extern uint8_t NCLink_End[2];
extern uint8_t NCLink_Send_Ask_Flag[10],NCLink_Send_Check_Flag[20];
extern uint8_t unlock_flag,takeoff_flag;
extern uint8_t rc_update_flag;

extern uint8_t cal_flag,cal_step,cal_cmd,shutdown_now_cal_flag;

extern uint32_t Guide_Flight_Lng,Guide_Flight_Lat,Guide_Flight_Cnt;
extern uint8_t Guide_Flight_Flag;
extern uint16_t NCLINK_PPM_Databuf[10];
extern third_party_state current_state;
extern nav_ctrl ngs_nav_ctrl;



void Serial_Data_Send(uint8_t *buf, uint32_t cnt);
void Float2Byte(float *FloatValue, unsigned char *Byte, unsigned char Subscript);
void Byte2Float(unsigned char *Byte, unsigned char Subscript, float *FloatValue);
void NCLink_Send_Status(float roll,float pitch,float yaw,
											  float roll_gyro,float pitch_gyro,float yaw_gyro,
												float imu_temp,float vbat,uint8_t fly_model,uint8_t armed);												
void NCLink_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,
												int16_t g_x,int16_t g_y,int16_t g_z,
												int16_t m_x,int16_t m_y,int16_t m_z);
void NCLink_Send_RCData(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4,
	                      uint16_t ch5,uint16_t ch6,uint16_t ch7,uint16_t ch8);
void NCLink_Send_GPSData(int32_t lng,
                         int32_t lat,
                         int32_t alt,
                         int16_t pdop,
                         uint8_t fixstate,
                         uint8_t numsv);
void NCLink_Send_Obs_NE(float lat_pos_obs,float lng_pos_obs,
												float lat_vel_obs,float lng_vel_obs);
void NCLink_Send_Obs_UOP(float alt_obs_baro,float alt_obs_ult,float opt_vel_p,float opt_vel_r);
void NCLink_Send_Fusion_U(float alt_pos_fus,float alt_vel_fus,float alt_accel_fus);
void NCLink_Send_Fusion_NE(float lat_pos_fus	,float lng_pos_fus,
											     float lat_vel_fus  ,float lng_vel_fus,
											     float lat_accel_fus,float lng_accel_fus);
void NCLink_Send_PID(uint8_t group,float pid1_kp,float pid1_ki,float pid1_kd,
																	 float pid2_kp,float pid2_ki,float pid2_kd,
																	 float pid3_kp,float pid3_ki,float pid3_kd);	

void NCLink_Send_Parameter(uint16_t targeheight,uint16_t safeheight,uint16_t safevbat,uint16_t maxheight,
													 uint16_t maxradius,uint16_t maxupvel,uint16_t maxdownvel,uint16_t maxhorvel,
												   uint16_t reserveduart,uint16_t neargroundheight,uint16_t uart2_mode,uint16_t avoid_obstacle);

void NCLink_Send_Userdata(float userdata1	 ,float userdata2,
											    float userdata3  ,float userdata4,
											    float userdata5  ,float userdata6);
													
void NCLink_Send_To_Firetruck(float x,float y,float z,float dis,int16_t xf,int16_t yf,uint8_t update);
													
uint8_t NCLink_Send_Check_Status_Parameter(void);
void NCLink_Data_Prase_Prepare(uint8_t data);
void NCLink_Data_Prase_Process(uint8_t *data_buf,uint8_t num);
void NCLink_SEND_StateMachine(void);			
void Pilot_Status_Tick(void);													
#endif




