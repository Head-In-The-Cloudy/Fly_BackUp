#ifndef __LSN10_H__
#define __LSN10_H__

#define SCAN_FRQ_HZ   4500
#define OUTPUT_FRQ_HZ 10
#define POINT_CLOUD_CNT (SCAN_FRQ_HZ/OUTPUT_FRQ_HZ)
#define FRAME_POINT_CNT 16


#define min_dis_mm_limit  100
#define max_dis_mm_limit  11990


typedef struct
{
	uint8_t distance_hsb;
  uint8_t distance_lsb;
	uint8_t peak;
}point;

typedef struct
{
	uint8_t header[2];
	uint8_t length;
	uint8_t speed_hsb;
	uint8_t speed_lsb;
	uint8_t start_angle_hsb;
  uint8_t start_angle_lsb;
	point data[16];
	uint8_t end_angle_hsb;
	uint8_t end_angle_lsb;
	uint8_t crc;
}lsn10;


typedef struct
{
	uint16_t distance_mm;
	uint8_t strength;
}point_cloud;



typedef struct
{
	float dis_min;
	float angle_min;
	uint8_t strength_min;
	float dis_max;
	float angle_max;
	uint8_t strength_max;
	uint8_t motor_start_flag;
}laser_state;



void lsn10_data_prase(void);	
void laser_state_sort(void);
void lsn10_motor_ctrl(uint8_t cmd);

extern point_cloud _point_cloud[POINT_CLOUD_CNT+10];
extern laser_state _laser_state;
#endif


