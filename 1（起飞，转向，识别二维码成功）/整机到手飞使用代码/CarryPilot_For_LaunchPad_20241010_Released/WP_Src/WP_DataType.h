#ifndef __WP_DATATYPE_H
#define __WP_DATATYPE_H

#include "math.h"
#include "WP_Math.h"

enum
{
	_UP = 0,
	_EAST,
	_NORTH
};

enum
{
	_ROL = 0,
	_PIT,
	_YAW
};

#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)

#define int16 short
#define uint16 unsigned short
#define int32 int
#define uint32 unsigned int
#define uint8 unsigned char
#define s32 int32

#define Int_Sort (int16_t)
#define TRUE true
#define FALSE false

typedef enum
{
	RESET = 0,
	SET = !RESET
} FlagStatus,
	ITStatus;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
} vector3i;

typedef struct
{
	volatile float Last_Time;
	volatile float Now_Time;
	volatile float Time_Delta;
	volatile uint16_t Time_Delta_INT; // 单位ms
} Testime;

typedef struct
{
	volatile float last_time;
	volatile float current_time;
	volatile float period;
	volatile uint16_t period_int; // 单位ms
} systime;

typedef struct
{
	int32_t x;
	int32_t y;
} Vector2i;

typedef struct
{
	float x;
	float y;
} Vector2f;

typedef struct
{
	float x;
	float y;
	float z;
} Vector3f;

typedef struct
{
	float pos;
	float vel;
	float acc;
} Vector3s;

typedef struct
{
	float q0;
	float q1;
	float q2;
	float q3;
} Vector4q;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
} Vector3i;

typedef struct
{
	float E;
	float N;
	float U;
} Vector3_Nav;

typedef struct
{
	float E;
	float N;
} Vector2f_Nav;

typedef struct
{
	int32_t lat;
	int32_t lng;
} Vector2_Nav;

typedef struct
{
	float x;
	float y;
	float z;
} Vector3_Body;

typedef struct
{
	float Pit;
	float Rol;
} Vector2_Ang;

typedef struct
{
	float Pit;
	float Rol;
} Vector2_Body;

typedef struct
{
	float Pit;
	float Rol;
	float Yaw;
} Vector3f_Body;

typedef struct
{
	float North;
	float East;
} Vector2_Earth;

typedef struct
{
	Vector3f a;
	Vector3f b;
	Vector3f c;
} Matrix3f;

typedef struct
{
	Vector3f gyro_raw;
	Vector3f accel_raw;
	Vector3f mag_raw;
	Vector3f last_mag_raw;
	float mag_intensity;
	float acce_filter[3];
	float acce_filter_fb[3];
	float temperature;
	float last_temperature;
	float baro_temp_raw;
	float baro_pressure_raw;
	float baro_presure_offset;
	float baro_altitude;
	float last_baro_altitude;
	float baro_altitude_div;
	float last_baro_altitude_div;
	float baro_altitude_acc;
	uint8_t imu_updtate_flag;
	uint8_t baro_updtate_flag;
	uint8_t us100_updtate_flag;
	uint8_t tfmini_updtate_flag;
	uint8_t tofsensor_updtate_flag;
	uint8_t vl53l1x_updtate_flag;
	float _temperature;
	float distance_3d_cm;
	//
	float slam_yaw_init, slam_yaw; // slam定位初始偏航角零偏
	uint8_t slam_yaw_setup;		   // slam定位初始偏航角设定标志位
} Sensor;

typedef struct
{
	float rMat[9];
	float sin_rpy[3];
	float cos_rpy[3];
	float q[4];			 // 不含磁力计修正的四元数
	float quaternion[4]; // 含磁力计修正的四元数
	float Pitch, Roll, Yaw;
	float Yaw_Gyro, Pitch_Gyro, Roll_Gyro;
	float Yaw_Gyro_Rad, Pitch_Gyro_Rad, Roll_Gyro_Rad;
	float Accel_X_MPSS, Accel_Y_MPSS, Accel_Z_MPSS;

	/***************************************************/
	int8_t player_level;
} AHRS;

#define vector3f Vector3f
#define Axis_Num 3
#define Num 30
typedef struct
{
	float Position[Axis_Num];		   // 位置估计量
	float Speed[Axis_Num];			   // 速度估计量
	float Acceleration[Axis_Num];	   // 加速度估计量
	float Pos_Backups[Axis_Num][Num];  // 历史惯导位置
	float Vel_Backups[Axis_Num][Num];  // 历史惯导速度
	float Acce_Backups[Axis_Num][Num]; // 历史惯导速度
	float Acce_Bias[Axis_Num];		   // 惯导加速度漂移量估计量
	float Acce_Bias_All[Axis_Num];	   // 惯导加速度漂移量估计量
	float Last_Acceleration[Axis_Num];
	// float Last_Speed[Axis_Num];
	float Inertial_Acceleration[Axis_Num];
} SINS;

typedef struct
{
	float Position[Axis_Num];		  // 位置估计量
	float Speed[Axis_Num];			  // 速度估计量
	float Acceleration[Axis_Num];	  // 加速度估计量
	float Pos_Backups[Axis_Num][Num]; // 历史惯导位置
	float Vel_Backups[Axis_Num][Num]; // 历史惯导速度
	// float Acce_Backups[Axis_Num][Num];//历史惯导速度
	float Acce_Bias[Axis_Num]; // 惯导加速度漂移量估计量
							   // float Acce_Bias_All[Axis_Num];//惯导加速度漂移量估计量
							   // float Last_Acceleration[Axis_Num];
							   // float Last_Speed[Axis_Num];
} SINS_Lite;

typedef struct
{
	uint8_t Mpu_Health;
	uint8_t Baro_Health;
	uint8_t Mag_Health;
	uint8_t Gps_Health;
	uint8_t Ground_Health;
} Sensor_Health;

typedef enum
{
	NO_SLAM = 0,
	LIDAR_2D_SLAM, // 位置导航模式
	T265_SLAM,	   // T265定点模式
	LOAM,
} slam_sensor_mode;

typedef struct
{
	float position_x;
	float position_y;
	float position_z;
	float velocity_x;
	float velocity_y;
	float velocity_z;
	float q[4];
	float quality;
	uint8_t update_flag;
	uint8_t byte[8];
	float rpy[3];

	uint8_t rec_update_flag;
	uint8_t rec_head_update_flag;
	uint8_t valid;
	uint8_t fault, last_fault;
	slam_sensor_mode slam_sensor;
	uint8_t loam_update_flag;
	uint8_t alt_update_flag;
} third_party_state;

typedef enum
{
	BODY_FRAME = 0, // 机体坐标系
	MAP_FRAME,		// 导航坐标系
} Navigation_Frame;

typedef enum
{
	RELATIVE_MODE = 0, // 相对模式
	GLOBAL_MODE,	   // 全局模式
	CMD_VEL_MODE,	   // 速度模式
	TRANSITION_MODE	   // 过渡模式
} Navigation_Mode;

typedef struct
{
	uint16_t number;  // 任务编号
	float x;		  // x方向位置/速度期望
	float y;		  // y方向位置/速度期望
	float z;		  // z方向位置/偏航角速度期望
	uint8_t nav_mode; // 导航模式
	uint8_t frame_id; // 坐标系类型

	uint8_t update_flag;	  // 指令更新标志位
	uint8_t ctrl_finish_flag; // 控制完毕标志位

	uint16_t cnt;				 // 位置控制完毕判断计数器
	float dis_cm;				 // 总位置偏差
	const float dis_limit_cm;	 // 位置阈值
	const float cmd_vel_max;	 // 最大线速度限制
	const float cmd_angular_max; // 最大角速度限制

	uint8_t cmd_vel_update;		  // 速度控制指令更新标志位
	float cmd_vel_x;			  // 期望x方向速度
	float cmd_vel_y;			  // 期望y方向速度
	float cmd_vel_angular_z;	  // 期望偏航方向角速度
	uint32_t cmd_vel_during_cnt;  // 速度控制计数器
	uint8_t cmd_vel_suspend_flag; // 控制中止标志位
} nav_ctrl;

typedef struct
{
	float p[3][3];			   // 协方差矩阵
	float qp, qv;			   // 加速度计过程噪声
	float qb;				   // 加速度计过程偏移噪声
	float rp, rv;			   // 位置观测噪声、速度观测噪声
	float cp[2], cv[2], cb[2]; // 修正量
	float k[3][2];			   ////增益矩阵
	float err[2][2];
	uint8_t init;
} kalman_filter;

#endif
