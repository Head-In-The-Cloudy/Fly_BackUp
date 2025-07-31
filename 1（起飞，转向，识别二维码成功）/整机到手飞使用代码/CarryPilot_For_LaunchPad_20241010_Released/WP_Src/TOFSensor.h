#ifndef _TOFSENSOR_H_
#define _TOFSENSOR_H_

#define TOFSENSE_MAX_NUMBER 	10//最大运行的TOF传感器组
#define TOFSENSE_CURRENT_WORK 2//当前运行的TOF传感器组


typedef struct 
{
	uint8_t reserved1;
	uint8_t id;
	uint32_t system_time;
	int32_t dis;
	uint8_t dis_status;
	uint16_t signal_strength;
	uint8_t reserved2;
	
	uint32_t last_system_time;
	
	uint8_t range_precision;
	uint8_t update_flag;
	uint8_t valid;
	uint16_t period_ms;
	
	float distance;
	float last_distance;
	float pre_last_distance;
	float vel;
	float acc;
	float last_vel;
}TOFSensor_Data;


typedef struct 
{
	uint8_t id;
	uint32_t system_time;
	int32_t dis;
	uint8_t dis_status;
	uint16_t signal_strength;
	uint8_t range_precision;

	
	uint32_t last_system_time;	
	float distance;
	float last_distance;
	float pre_last_distance;
	float vel;
	float acc;
	float last_vel;
	
}TOFSense_F;



#pragma pack(1)
  typedef struct
  {
    uint8_t byteArray[3];
  } nint24_t;

  typedef struct
  {
    uint8_t byteArray[3];
  } nuint24_t;
#pragma pack()


#pragma pack(1)
typedef struct
{
  nint24_t dis;
  uint8_t dis_status;
  uint16_t signal_strength;
} ntsm_frame0_pixel_raw_t;


typedef struct
{
  uint8_t frame_header;
  uint8_t function_mark;
  uint8_t reserved;
  uint8_t id;
  uint32_t system_time;
  uint8_t pixel_count;
  ntsm_frame0_pixel_raw_t pixels[16];
  uint8_t reserved1[6];
  uint8_t sum;
} ntsm_frame0_raw_t_4x4;
#pragma pack()

typedef struct
{
  float dis_mm;
  uint8_t dis_status;
  uint16_t signal_strength;
}ntsm_frame0_pixel;



extern ntsm_frame0_pixel tofsense_m[16];

extern TOFSense_F TOFSense_F_Data;



void TOF_Statemachine(void);
void TOFSense_Prase(uint8_t data);
void TOFSense_M_Prase(uint8_t data);
void SMD15_init(uint32_t baudrate);
void tofsense_init(void);
void sdm15_prase(uint8_t data);

void Check_Front_Tofsense(void);
extern TOFSensor_Data tofdata[TOFSENSE_MAX_NUMBER]; 

extern float front_tofsense_distance;
extern uint8_t front_tofsense_distance_valid_cnt,front_tofsense_distance_valid_flag;

#endif






//						uint8_t id=0;
//						
//						id=COM7_Rx_Buf.Ring_Buff[i+3];
//						
//						tofdata[id].pre_last_distance=tofdata[id].last_distance;//上上次高度
//						tofdata[id].last_distance=tofdata[id].distance;//上次高度
//						tofdata[id].last_vel=tofdata[id].vel;//上次速度
//						tofdata[id].last_system_time=tofdata[id].system_time;
//						
//						
//						tofdata[id].id=COM7_Rx_Buf.Ring_Buff[i+3];
//						tofdata[id].system_time=COM7_Rx_Buf.Ring_Buff[i+4]|(COM7_Rx_Buf.Ring_Buff[i+5]<<8)|(COM7_Rx_Buf.Ring_Buff[i+6]<<16)|(COM7_Rx_Buf.Ring_Buff[i+7]<<24);
//					
//  					//if(tofdata.last_system_time==tofdata.system_time)//数据未更新 
//			
//						Get_Systime(&tofsensor);
//						
//						tofdata[id].dis=(int32_t)(COM7_Rx_Buf.Ring_Buff[8]<< 8|COM7_Rx_Buf.Ring_Buff[9]<<16|COM7_Rx_Buf.Ring_Buff[10]<<24)/256;
//						tofdata[id].dis_status=COM7_Rx_Buf.Ring_Buff[i+11];
//						tofdata[id].signal_strength=COM7_Rx_Buf.Ring_Buff[i+12]|(COM7_Rx_Buf.Ring_Buff[i+13]<<8);

//				
//						tofdata[id].distance=tofdata[id].dis/10.0f;//cm
//						tofdata[id].vel=(tofdata[id].distance-tofdata[id].last_distance)/0.1f; //观测速度
//						tofdata[id].acc=(tofdata[id].vel-tofdata[id].last_vel)/0.1f;					  //观测加速度








