#ifndef __SINS_H_
#define __SINS_H_



#define US100  	  		 0x01
#define TFMINI 	  		 0x02
#define TOFSENSE  		 0x03
#define SMD15    			 0x04
#define TOFSENSE_M     0x05
#define LIDAR3D        0x06
#define  Observation_Err_Max   1000000//cm


typedef struct{
   // by making alt 24 bit we can make p1 in a command 16 bit,
    // allowing an accurate angle in centi-degrees. This keeps the
    // storage cost per mission item at 15 bytes, and allows mission
    // altitudes of up to +/- 83km
    int32_t alt:24; ///< param 2 - Altitude in centimeters (meters * 100)
    int32_t lat;    ///< param 3 - Lattitude * 10**7
    int32_t lng;    ///< param 4 - Longitude * 10**7
}Location;



float longitude_scale(Location loc);
float get_distance(Location loc1,Location loc2);
void  GPSData_Sort(void);
void  Set_GPS_Home(void);
void  SINS_Prepare(void);
void Strapdown_INS_High_Kalman(void);
void Strapdown_INS_Reset(SINS *Ins,uint8_t Axis,float Pos_Target,float Vel_Target);
void KalmanFilter_Horizontal(void);
void altitude_kalman_filter(kalman_filter *kf,SINS_Lite *_ins,float dt);

Vector2f location_diff(Location loc1,Location loc2);
void from_enu_to_body_frame(float e,float n,float *right,float *forward,float _yaw);
void from_body_to_enu_frame(float right,float forward,float *e,float *n,float _yaw);

void sins_params_init(void);
	

extern SINS NamelessQuad;
extern uint8_t Filter_Defeated_Flag;
extern Vector2f SINS_Accel_Body;
extern uint8 GPS_Home_Set;
extern Location GPS_Home;//初始定位成功点信息
extern Location GPS_Present;//当前位置点信息
extern float Acceleration_Feedback[3];
extern float user_alt_obs_cm;
extern Vector3_Nav Earth_Frame_To_XYZ;

extern uint8_t GPS_Home_Set;
extern float Declination;//地球磁偏角
extern uint16 High_Delay_Cnt;

extern float observation_div,last_observation,observation_acc;
extern float height_zero_cm_offset,distance_tof_to_3dlidar;

extern float Acce_Bias[2];


extern float kalman_k[2];
extern kalman_filter slam_kf;
#endif

