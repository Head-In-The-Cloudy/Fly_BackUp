#ifndef _CONTROL_ALTHOLD_H
#define _CONTROL_ALTHOLD_H



//#define ALT_VEL_FEEDFORWARD_ENABLE 		 1 //速度前馈控制使能


typedef enum 
{
  ALTHOLD_MANUAL_CTRL=0,		//高度手动控制
  ALTHOLD_AUTO_POS_CTRL,		//高度直接位置控制
	ALTHOLD_AUTO_VEL_CTRL,    //高度直接速度控制
}ALTHOLD_CTRL_MODE;

#define NUL 0


void Flight_Alt_Hold_Control(uint8_t mode,float target_alt,float target_vel);



void Thr_Scale_Set(rc_calibration *rc_date);
void ncq_control_althold(void);
float get_stopping_point_z(Vector3f *stopping_point);
uint8_t Thr_Push_Over_Deadband(void);


extern float ALT_VEL_FEEDFORWARD_ENABLE_Output;//竖直速度前馈控制器输出;
extern float ALT_VEL_FEEDFORWARD_ENABLE_Rate;//竖直速度前馈控制器，APM里面为1、0.45;
extern float ALT_VEL_FEEDFORWARD_ENABLE_Delta;//竖直期望速度变化率;
extern float Last_Alt_Vel_Target;
extern float Alt_Vel_Target;
extern Vector3f UAV_Cushion_Stop_Point;

extern uint16_t  Deadband;//油门中位死区
extern uint16_t  Deadzone_Min;
extern uint16_t  Deadzone_Max;
extern uint16_t  Thr_Top;//油门最大上行程
extern uint16_t  Thr_Buttom;//油门最大下行程
extern uint8_t Thr_Push_Over_State;
#endif

