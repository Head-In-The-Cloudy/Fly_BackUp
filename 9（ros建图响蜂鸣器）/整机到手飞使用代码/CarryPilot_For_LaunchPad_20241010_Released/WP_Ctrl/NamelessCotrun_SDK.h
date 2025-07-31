#ifndef __NAMELESSCOTRUN_SDK_H
#define __NAMELESSCOTRUN_SDK_H

#define SDK_Duty_Max 10

typedef enum 
{
  UART3_SDK=0,
	UART0_SDK,
}COM_SDK;



typedef struct 
{
  uint8_t Start_Flag;
  uint8_t Execute_Flag;
  uint8_t End_flag;
}Duty_Status;

typedef struct 
{
  Duty_Status Status[SDK_Duty_Max];
  uint16_t Transition_Time[SDK_Duty_Max];
}SDK_Status;



#define SDK_Target_Length  53//45

typedef struct
{
  uint16_t x;
  uint16_t y; 
  uint16_t pixel;  
	uint8_t flag;
	uint8_t state;		
	int16_t angle;
	uint16_t distance;
	uint16_t apriltag_id;
	uint16_t width;
	uint16_t height;
	uint8_t fps;
	uint8_t reserved1;
	uint8_t reserved2;
	uint8_t reserved3;
	uint8_t reserved4;
	uint16_t range_sensor1;
	uint16_t range_sensor2;
	uint16_t range_sensor3;
	uint16_t range_sensor4;
	uint8_t camera_id;
	int32_t reserved1_int32;
	int32_t reserved2_int32;
	int32_t reserved3_int32;
	int32_t reserved4_int32;
	//
	uint8_t sdk_mode;
	float x_cm;
	float y_cm;
	float z_cm;	
	float x_pixel_size;
	float y_pixel_size;
	float z_pixel_size;
	float apriltag_distance;
	uint16_t trust_cnt;
	uint16_t trust_flag;
	uint8_t line_ctrl_enable;
	uint8_t target_ctrl_enable;
	Vector3f sdk_target,sdk_target_offset;
	float sdk_angle;
}Target_Check;//Ä¿±ê¼ì²â



extern Target_Check Opv_Top_View_Target,Opv_Front_View_Target;
extern float SDK_Target_Yaw_Gyro;

extern uint16_t SDK_Transition_Time;
extern SDK_Status SDK_Duty_Status;
void NCQ_SDK_Reset(void);
uint8_t move_with_speed_target(float x_target,float y_target,float delta,SDK_Status *Status,uint16_t number);
uint8_t move_with_xy_target(float pos_x_target,float pos_y_target,SDK_Status *Status,uint16_t number);
uint8_t move_with_z_target(float z_target,float z_vel,float delta,SDK_Status *Status,uint16_t number);
void NCQ_SDK_Run(void);
void SDK_DT_Send_Check(unsigned char mode,COM_SDK com);
void SDK_Init(void);
void SDK_Data_Prase_1(void);
void SDK_Data_Prase_2(void);
void SDK_Data_Receive_Prepare(u8 data,uint8_t label);
void SDK_DT_Send_Check_Search_LandColor(unsigned char mode,uint8_t color,uint8_t shape,COM_SDK com);


void SDK_Data_Receive_Prepare_1(uint8_t data);
void SDK_Data_Receive_Prepare_2(uint8_t data);
		
	
#endif

