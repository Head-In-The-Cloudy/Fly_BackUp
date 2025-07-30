#ifndef _WP_CTRL_
#define _WP_CTRL_

#define Aerocraft_Axis_Mode_Default  4
#define Aerocraft_Axis_Mode Aerocraft_Axis_Mode_Default
          
#if (Aerocraft_Axis_Mode==6)//六轴控制器输出映射表
/*
     机头
    3   1
    *   *   
      
5 *   *   * 6

    *   *
    2   4
其中：1、4、5逆时针转动
     2、3、6逆时针转动
*/
#define Moter1_Thr_Scale 1.0f
#define Moter2_Thr_Scale 1.0f
#define Moter3_Thr_Scale 1.0f
#define Moter4_Thr_Scale 1.0f
#define Moter5_Thr_Scale 1.0f
#define Moter6_Thr_Scale 1.0f


#define Moter1_Roll_Scale -0.75f//-0.5f
#define Moter2_Roll_Scale  0.75f//0.5f
#define Moter3_Roll_Scale  0.75f//0.5f
#define Moter4_Roll_Scale -0.75f//-0.5f
#define Moter5_Roll_Scale  0.75f//1.0f
#define Moter6_Roll_Scale -0.75f//-1.0f
          
#define Moter1_Pitch_Scale  0.866025f
#define Moter2_Pitch_Scale -0.866025f
#define Moter3_Pitch_Scale  0.866025f
#define Moter4_Pitch_Scale -0.866025f
#define Moter5_Pitch_Scale  0.0f
#define Moter6_Pitch_Scale  0.0f
          
#define Moter1_Yaw_Scale -1.0f
#define Moter2_Yaw_Scale 1.0f
#define Moter3_Yaw_Scale 1.0f
#define Moter4_Yaw_Scale -1.0f
#define Moter5_Yaw_Scale -1.0f
#define Moter6_Yaw_Scale 1.0f

#else//四轴控制器输出映射表
/*
      机头
    3     1
    *     *   
       * 
    *     *
    2     4
其中：1、2逆时针转动
      3、4逆时针转动
*/
#define Moter1_Thr_Scale 1.0f
#define Moter2_Thr_Scale 1.0f
#define Moter3_Thr_Scale 1.0f
#define Moter4_Thr_Scale 1.0f
#define Moter5_Thr_Scale 0.0f
#define Moter6_Thr_Scale 0.0f

#define Moter1_Roll_Scale -1.0f
#define Moter2_Roll_Scale 1.0f
#define Moter3_Roll_Scale 1.0f
#define Moter4_Roll_Scale -1.0f
#define Moter5_Roll_Scale 0.0f
#define Moter6_Roll_Scale 0.0f
          
#define Moter1_Pitch_Scale 1.0f
#define Moter2_Pitch_Scale -1.0f
#define Moter3_Pitch_Scale 1.0f
#define Moter4_Pitch_Scale -1.0f
#define Moter5_Pitch_Scale 0.0f
#define Moter6_Pitch_Scale 0.0f
          
#define Moter1_Yaw_Scale -1.0f
#define Moter2_Yaw_Scale -1.0f
#define Moter3_Yaw_Scale 1.0f
#define Moter4_Yaw_Scale 1.0f
#define Moter5_Yaw_Scale 0.0f
#define Moter6_Yaw_Scale 0.0f
#endif
          
          
enum YAW_CTRL_MODE
{
	ROTATE=0,							//手动偏航控制模式
  AZIMUTH=1,						//绝对偏航角度控制模式
	CLOCKWISE=2,					//相对偏航角度顺时针控制模式	
	ANTI_CLOCKWISE=3,			//相对偏航角度逆时针控制模式	
	CLOCKWISE_TURN=4,			//角速度控制顺时针模式
	ANTI_CLOCKWISE_TURN=5,//角速度控制逆时针模式
};

typedef enum
{
	MOTOR1=0,
	MOTOR2,
	MOTOR3,
	MOTOR4,
	MOTOR5,
	MOTOR6,
	MOTOR7,
	MOTOR8,
	MOTOR_NUM
}motor;

typedef struct
{
	float throttle_control_output;	  	//油门控制器最终输出，变量暂未使用
	float roll_control_output;			  	//横滚姿态控制器最终输出，变量暂未使用
	float pitch_control_output;			  	//俯仰姿态控制器最终输出，变量暂未使用
	float yaw_control_output;				  	//偏航姿态控制器最终输出，变量暂未使用
	float roll_outer_control_output;  	//横滚姿态控制器输入
	float pitch_outer_control_output; 	//俯仰姿态控制器输入
  float yaw_outer_control_output;	  	//偏航姿态控制器输入
  uint16_t motor_output[MOTOR_NUM];		//电机映射输出值，变量暂未使用
  uint16_t temperature_control_output;//温度控制器输出值，变量暂未使用
	uint16_t yaw_ctrl_cnt;							//偏航控制计数器
	uint8_t yaw_ctrl_mode;							//偏航控制模式
	uint8_t yaw_ctrl_start;							//偏航控制开始标志位
	uint8_t yaw_ctrl_end;								//偏航控制结束标志位
	uint32_t start_time_ms;							//偏航控制开始时间
	uint32_t execution_time_ms;					//偏航控制执行时间
	uint8_t init;												//偏航控制初始化标志位
	//uint8_t yaw_least_cost_enable;      //偏航控制最小代价使能标志位
	uint8_t roll_pitch_angle_limit_enable;
}Controller_Output;


extern Controller_Output Flight;




#define Optical_Enable  1//光流辅助悬停标志
#define ADRC_MODE  0
#define PID_MODE   1
#define TEST_MODE  2
//#define GYRO_CONTROL_MODE  TEST_MODE
#define GYRO_CONTROL_MODE  PID_MODE
//#define GYRO_CONTROL_MODE  ADRC_MODE



void Total_Control(void);
void Control_Output(void);
void Landon_Earth_Check(void);//自检触地进入怠速模式
void CarryPilot_Control(void);
	

extern float LPButter_Vel_Error(float curr_input);
extern uint8_t Controler_High_Mode,Controler_SDK1_Mode;
extern uint8_t Controler_Horizontal_Mode,Last_Controler_Horizontal_Mode;
extern uint16_t High_Hold_Throttle;
extern uint8_t Landon_Earth_Flag;
extern uint16 Throttle;
extern uint8_t Last_Landon_Earth_Flag,Landon_Earth_Flag;
extern uint16_t Landon_Earth_Cnt;
extern uint16_t Throttle_Output;
extern uint8_t Landon_Earth_Flag;
extern int16_t M_PWM_1,M_PWM_2,M_PWM_3,M_PWM_4,M_PWM_5,M_PWM_6;//四个电机输出PWM
extern Vector2_Ang Body_Frame_Pos_Err;
extern Vector3_Nav Earth_Frame_Pos_Err;

#endif

