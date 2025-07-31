#include "Headfile.h"
#include "LSN10.h"



lsn10 laser_point_cloud_raw;
point_cloud _point_cloud[POINT_CLOUD_CNT+10];

laser_state _laser_state=
{
	.dis_min=12000,
	.angle_min=360,	
	.dis_max=0,
	.angle_max=0,
	.motor_start_flag=0
};


float start_angle,end_angle,gap_angle;
void lsn10_data_prase(void)
{
	static uint16_t offset_cnt=0;
  static uint16_t prase_cnt=0;
  prase_cnt++;
  if(prase_cnt>=3)//3ms
  {
    if(COM2_Rx_Buf.Tail<58)//0-58数据位正在传输
    {
      offset_cnt=58;
    }
    else//58-116数据位正在传输
    {
      offset_cnt=0;
    }
		
		uint8_t sum=0;
		for(uint16_t i=offset_cnt;i<offset_cnt+57;i++)//和校验
		{
			sum+=COM2_Rx_Buf.Ring_Buff[i];	
		}
		
		if(sum==COM2_Rx_Buf.Ring_Buff[offset_cnt+57])
		{
			memcpy(&laser_point_cloud_raw,&COM2_Rx_Buf.Ring_Buff[offset_cnt],58*sizeof(unsigned char));
		}
			
		start_angle=(uint16_t)((laser_point_cloud_raw.start_angle_hsb<<8)|laser_point_cloud_raw.start_angle_lsb)/100.0f;
		end_angle  =(uint16_t)((laser_point_cloud_raw.end_angle_hsb  <<8)|laser_point_cloud_raw.end_angle_lsb  )/100.0f;
		gap_angle  =(end_angle-start_angle)/(FRAME_POINT_CNT-1);
		
		if(start_angle>360) start_angle-=360;
		
		uint16_t _cnt=(start_angle/360.0f)*POINT_CLOUD_CNT;
		for(uint16_t i=0;i<FRAME_POINT_CNT;i++)
		{		
			uint16_t _distance=(uint16_t)((laser_point_cloud_raw.data[i].distance_hsb<<8)|laser_point_cloud_raw.data[i].distance_lsb);
			uint16_t j=_cnt+i;
			if(j<POINT_CLOUD_CNT)
			{
				_point_cloud[j].distance_mm=_distance;
				_point_cloud[j].strength=laser_point_cloud_raw.data[i].peak;
			}
			else
			{
				j=constrain_int16(j-POINT_CLOUD_CNT,0,POINT_CLOUD_CNT-1);
				_point_cloud[j].distance_mm=_distance;
				_point_cloud[j].strength=laser_point_cloud_raw.data[i].peak;			
			}	
		}
    prase_cnt=0;
  }
}


void laser_state_sort(void)
{	
	if(Uart2_Mode==0x00)  return;//激光雷达使能时才处理数据
	if(_laser_state.motor_start_flag==0x00)
	{
		if(Gyro_Safety_Calibration_Flag==1) 
		{
			lsn10_motor_ctrl(0x01);//启动电机
			_laser_state.motor_start_flag=0x01;
			send_check_back=6;//启动电机
		}
	}
		
	//提取最大距离、最小距离，以及其所在的角度
	static uint16_t cnt=0;
	cnt++;
	if(cnt==10)
	{
		_laser_state.dis_min=12000;
		_laser_state.angle_min=360;	
		_laser_state.dis_max=0;
		_laser_state.angle_max=0;
		
		for(uint16_t k=0;k<POINT_CLOUD_CNT;k++)
		{
			if(_point_cloud[k].distance_mm<min_dis_mm_limit||_point_cloud[k].distance_mm>max_dis_mm_limit) continue;//剔除掉量程临界数据
			
			//记录最近距离处的角度与信号强度
			if(_point_cloud[k].distance_mm<_laser_state.dis_min)
			{
				_laser_state.dis_min  =_point_cloud[k].distance_mm;
				_laser_state.angle_min=(k*360.0f)/POINT_CLOUD_CNT;
				_laser_state.strength_min=_point_cloud[k].strength;
			}
			//记录最远距离处的角度与信号强度
			if(_point_cloud[k].distance_mm>_laser_state.dis_max)
			{
				_laser_state.dis_max  =_point_cloud[k].distance_mm;
				_laser_state.angle_max=(k*360.0f)/POINT_CLOUD_CNT;
				_laser_state.strength_max=_point_cloud[k].strength;
			}	
		}
		cnt=0;
	}
}

static void lsn10_uart_send(u8 dat)
{
  UARTCharPut(UART2_BASE, dat);
}


void lsn10_motor_ctrl(uint8_t cmd)
{
	lsn10_uart_send(0xa5);
	lsn10_uart_send(0x5a);
	lsn10_uart_send(0x55);
	for(uint16_t i=0;i<181;i++) lsn10_uart_send(0x00);
	lsn10_uart_send(0x01);
	lsn10_uart_send(cmd);
	lsn10_uart_send(0xfa);	
	lsn10_uart_send(0xfb);
}


//uint16_t _distance=(uint16_t)((laser_point_cloud_raw.data[i].distance_hsb<<8)|laser_point_cloud_raw.data[i].distance_lsb);
//uint16_t _distance=(uint16_t)((laser_point_cloud_raw.data[i].distance.distance_byte[0]<<8)|laser_point_cloud_raw.data[i].distance.distance_byte[1]);
