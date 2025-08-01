#ifndef __NAMELESSCOTRUN_SDK_H
#define __NAMELESSCOTRUN_SDK_H

#define SDK_Duty_Max 10

typedef enum 
{
  UART3_SDK=0,
	UART0_SDK,
}COM_SDK;

/**********************************/


#define  FORBIDDEN_RIGION_MAX_NUM 30 //最多可以承载的 禁飞点的数量

extern uint8_t FLY_PORT[100][2];
extern uint8_t if_FLY[7][9];

typedef  struct
{
    uint8_t A;
    uint8_t B;
}vector2f;

typedef struct
{
	uint8_t animal_type;
	uint8_t animal_num;	
}Animal;



typedef  struct
{
    uint8_t animal_type_num;
    Animal animal_all[5];
	vector2f Find_Animal_Rigion;//一个二维坐标用来存储 发现动物的位置
}GroundStation_To_F4_Target;


typedef  struct
{
    uint8_t Forbidden_Rigion_Num;
    vector2f Forbidden_Rigion[FORBIDDEN_RIGION_MAX_NUM];
    uint8_t Get_Forbidden_Rigion;  //初始化之后是要 赋值 为0 比较保险
}GroundStation_To_TIVA_Target;


/***********************************/



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
}Target_Check;//目标检测



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
		

void Write_In_FLY_PORT(void);
	
	
/****************地面站******************/
void SDK_Data_Receive_Prepare_GroundStation(uint8_t data);
void Data_Receive_GroundStation(uint8_t *data_buf,uint8_t num,GroundStation_To_TIVA_Target* target);
void SDK_DT_Send_To_GroundStation(GroundStation_To_F4_Target* target);
void GroundStation_Begin_MAP(void) ; //主函数中会 一直调用他
uint8_t count_FLY_PORT(void);//计算航点数
void SDK_DT_Send_To_GroundStation_MAP_BUG(void);
/*********MAXI CAM*********/

void SDK_DT_Send_To_Maxicam_Mode_1(void);
void SDK_DT_Send_To_Maxicam_Mode_1_REP(void);  //应答
void SDK_DT_Send_To_GroundStation_MAP(void);

	
#endif

