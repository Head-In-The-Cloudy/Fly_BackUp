#ifndef _NAMELESSCOTRUN_OPTICALFLOW_
#define _NAMELESSCOTRUN_OPTICALFLOW_



typedef struct{
  unsigned short frame_count_since_last_readout;
  float  pixel_flow_x_integral;
  float  pixel_flow_y_integral;
  signed short gyro_x_rate_integral;
  signed short gyro_y_rate_integral;
  signed short gyro_z_rate_integral;
  unsigned int integration_timespan;
  unsigned int sonar_timestamp;
  unsigned short ground_distance;
  signed short gyro_temperature;
  unsigned char qual;
	unsigned char sum;
	unsigned char version;
}flow_integral_frame;


typedef struct{
  float x;
  float y;
  unsigned short dt;
  unsigned char qual;
  unsigned char update;
	unsigned char valid;
	unsigned char ctrl_valid;
}flow_float;


void OpticalFlow_Init(void);
void LC30X_OpticalFlow_Sense_Prase(uint8_t data,uint8_t *buf);
void Optflow_Statemachine(void);
float OpticalFlow_Rotate_Complementary_Filter(float optflow_gyro,float gyro,uint8_t axis,uint8_t valid);
void  OpticalFlow_INS_CF(float flow_height,Vector2f accel,Vector2f flow);
void  VIO_SLAM_INS_CF(void);

void from_vio_to_body_frame(float map_x,float map_y,float *bod_x,float *bod_y,float _yaw);
void from_body_to_nav_frame(float bod_x,float bod_y,float *map_x,float *map_y,float _yaw);


extern flow_float opt_data;
extern uint8_t OpticalFlow_Is_Work;
extern Vector2f OpticalFlow_Speed,OpticalFlow_Speed_Err,OpticalFlow_Position;
extern uint8_t Optical_Type_Present;

extern  flow_integral_frame opt_origin_data;
extern Vector2f opt_filter_data,opt_gyro_filter_data,opt_gyro_data,gyro_filter_data;
extern SINS_Lite OpticalFlow_SINS,VIO_SINS;
extern uint8_t lc30x_buf[20];
extern Vector3s correct[2];
extern Vector2f gyro_filter_data_backup[20];



#endif


//		from_body_to_nav_frame(current_state.position_x,
//													 current_state.position_y,
//													&OpticalFlow_Position.x,
//													&OpticalFlow_Position.y,
//													 WP_Sensor.slam_yaw_init);
//		from_slam_to_nav_frame(current_state.position_x,
//													 current_state.position_y,
//													&OpticalFlow_Position.x,
//													&OpticalFlow_Position.y,
//													 WP_Sensor.slam_yaw_init);





