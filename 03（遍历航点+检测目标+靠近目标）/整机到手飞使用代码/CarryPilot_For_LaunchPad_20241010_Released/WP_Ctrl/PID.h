#ifndef _PID_H_
#define _PID_H_



#include "Filter.h"

typedef struct
{
 float p;
 float i;
 float d;
}Vector3f_pid;


typedef struct
{
    uint8 Err_Limit_Flag :1;//偏差限幅标志
    uint8 Integrate_Limit_Flag :1;//积分限幅标志
    uint8 Integrate_Separation_Flag :1;//积分分离标志
    float Expect;//期望
    float FeedBack;//反馈值
    float Err;//偏差
    float Last_Err;//上次偏差
    float Err_Max;//偏差限幅值
    float Integrate_Separation_Err;//积分分离偏差值
    float Integrate;//积分值
    float Integrate_Max;//积分限幅值
    float Kp;//控制参数Kp
    float Ki;//控制参数Ki
    float Kd;//控制参数Kd
    float Control_OutPut,Raw_Control_OutPut;//控制器总输出
    float Last_Control_OutPut;//上次控制器总输出
    float Control_OutPut_Limit;//输出限幅
    /***************************************/
    float Pre_Last_Err;//上上次偏差
    float Adaptable_Kd;//自适应微分参数
    float Last_FeedBack;//上次反馈值
    float Dis_Err;//微分量
    float Dis_Error_History[5];//历史微分量
    float Err_LPF;
    float Last_Err_LPF;
    float Dis_Err_LPF;
    float Last_Dis_Err_LPF;
    float Pre_Last_Dis_Err_LPF;
    lpf_buf _lpf_buf;//控制器低通输入输出缓冲
		systime Systime_t;	
		//
		float d_lpf_alpha;
		float derivative,last_derivative;//上次微分量	
		
		float Last_Expect;
		float Expect_Div;		//期望的微分
		float Feedback_Div;	//反馈信号的微分
		float Combine_Div;	//组合微分
}PID_Controler;

typedef struct
{
     PID_Controler Pitch_Angle_Control;
     PID_Controler Pitch_Gyro_Control;
     PID_Controler Roll_Angle_Control;
     PID_Controler Roll_Gyro_Control;
     PID_Controler Yaw_Angle_Control;
     PID_Controler Yaw_Gyro_Control;
     PID_Controler Height_Position_Control;
     PID_Controler Height_Speed_Control;
     PID_Controler Longitude_Position_Control;
     PID_Controler Longitude_Speed_Control;
     PID_Controler Latitude_Position_Control;
     PID_Controler Latitude_Speed_Control;
     /*************加速度控制器，新加****************/
     PID_Controler Height_Acce_Control;
     PID_Controler Reserved_User1_Control;
     PID_Controler Reserved_User2_Control;
     /*************光流控制器，新加****************/
     PID_Controler Optical_Position_Control;
     PID_Controler Optical_Speed_Control;
     PID_Controler SDK_Roll_Position_Control;
     PID_Controler SDK_Pitch_Position_Control;
		 PID_Controler IMU_Temperature_Control;
}AllControler;

typedef enum
{
     Pitch_Angle_Controler=0,
     Pitch_Gyro_Controler=1,
     Roll_Angle_Controler=2,
     Roll_Gyro_Controler=3,
     Yaw_Angle_Controler=4,
     Yaw_Gyro_Controler=5,
     Height_Position_Controler=6,
     Height_Speed_Controler=7,
		 
     Longitude_Position_Controler=8,
     Longitude_Speed_Controler=9,
     Latitude_Position_Controler=10,
     Latitude_Speed_Controler=11,
     Height_Acce_Controler=12,
     Reserved_User1_Controler=13,
     Reserved_User2_Controler=14,
     Optical_Position_Controler=15,
     Optical_Speed_Controler=16,
     SDK_Roll_Position_Controler=17,
     SDK_Pitch_Position_Controler=18,
     IMU_Temperature_Controler=19	
}Controler_Label;


typedef enum 
{
	direct_diff=0,		//直接微分
	interval_diff=1,	//区间微分
	incomplete_diff=2,//微分先行
}diff_mode;

typedef enum 
{
	noneed_lpf=0,		    //微分无需低通
	first_order_lpf=1,	//微分一阶低通
	second_order_lpf=2, //微分二阶低通
}lpf_mode;


extern AllControler Total_Controller;


void PID_Paramter_Init_With_Flash(void);
void  Total_PID_Init(void);
void  PID_Init(PID_Controler *Controler,Controler_Label Label);

float PID_Control(PID_Controler *Controler,float period_second);
float PID_Control_Yaw(PID_Controler *Controler,float period_second);
float PID_Control_Div_LPF(PID_Controler *Controler,float period_second);
float PID_Control_Err_LPF(PID_Controler *Controler,float period_second);
float PID_Control_SDK_Err_LPF(PID_Controler *Controler,uint8_t Differential_Enable_Flag,float period_second);
float Differential_Forward_PID_Control_Div_LPF(PID_Controler *Controler,float period_second);
float PID_Control_Div_LPF_For_Gyro(PID_Controler *Controler,float period_second);

float pid_ctrl_rpy_gyro_maple(PID_Controler *ctrl,float period_second,diff_mode _diff_mode,lpf_mode _lpf_mode);;


void  PID_LPF_Reset(PID_Controler *Controler,Controler_Label Label);
void  PID_Integrate_Reset(PID_Controler *Controler);
void  Take_Off_Reset(void);
void East_North_Ctrl_Reset(void);

void Throttle_Control_Reset(void);
void Save_Or_Reset_PID_Parameter(void);
void Save_PID_Parameter(void);
extern uint8_t Sort_PID_Flag;
#endif


