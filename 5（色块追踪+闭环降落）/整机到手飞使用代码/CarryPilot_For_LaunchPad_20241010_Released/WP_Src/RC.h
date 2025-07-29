#ifndef __RC_H__
#define __RC_H__

#define  Scale_Pecent_Max  0.75f   //最大解锁幅值量程因子
#define  Pit_Rol_Max 35.0f           //最大俯仰、横滚期望
#define  Yaw_Max     200.0f          //最大偏航期望
#define  Buttom_Safe_Deadband  50.0f //油门底部安全死区



#define Lock_Controler  0
#define Unlock_Controler  1


typedef enum 
{
	RC_ROLL_CHANNEL=0,
	RC_PITCH_CHANNEL,
	RC_THR_CHANNEL,
	RC_YAW_CHANNEL,
	RC_AUX1_CHANNEL,
	RC_AUX2_CHANNEL,
	RC_AUX3_CHANNEL,
	RC_AUX4_CHANNEL,
	RC_CHL_MAX
}RC_CHL_MAP;

typedef enum 
{
  RC_ROLL=0,
	RC_PITCH,
	RC_YAW,
	RC_THR
}RC_RPYT;

typedef enum 
{
  AUX1=0,
	AUX2,
	AUX3,
	AUX4
}AUX;


typedef enum 
{
  HAND_THR=0,//手动控制油门
	AUTO_THR,	 //自动控制油门
}Thr_Mode;


typedef enum 
{
  STABILIZE=0,//姿态自稳模式
	POS_GUIDE,	//位置导航模式
	POS_HOLD,   //GPS定点模式
}Flight_Mode;


typedef struct
{
  uint16_t max;		//最大值
	uint16_t min;		//最小值
	uint16_t middle;//中间值
	uint16_t deadband;    //中位死区值
	bool reverse_flag;    //反向标志位
	float scale;
}rc_calibration;


typedef struct
{
  uint16_t rcdata[RC_CHL_MAX];
	rc_calibration cal[RC_CHL_MAX];
  float rc_rpyt[4];
  uint16_t thr;
  uint16_t aux[4];
	Thr_Mode thr_mode;
	Thr_Mode last_thr_mode;
	Flight_Mode flight_mode;
	Flight_Mode last_flight_mode;
	uint8_t mode_change_flag;
	uint8_t poshold_set_flag;
}rc;

void Remote_Control(void);
uint8_t RC_Read_Switch(uint16_t ch);

extern rc RC_Data;
extern uint16 Controler_State;
extern uint8_t Auto_Relock_Flag_Set;
extern int16_t Throttle_Rate;
extern uint16_t Auto_ReLock_Cnt;//自动上锁计数器
extern uint16_t Unlock_Makesure_Cnt,Lock_Makesure_Cnt,Forced_Lock_Makesure_Cnt;
extern uint16_t Unwanted_Lock_Flag;
extern uint16 Throttle_Control;
extern float Pitch_Control,Roll_Control,Yaw_Control;
extern uint16_t PPM_LPF_Databuf[8];
extern float Target_Angle[2];
extern int16_t TRPY[4];
#endif

