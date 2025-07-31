/* Copyright (c)  2019-2030 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
/*----------------------------------------------------------------------------------------------------------------------/
																									开源并不等于免费
																									开源并不等于免费
																									开源并不等于免费
																									重要的事情说三遍
								先驱者的历史已经证明，在当前国内略浮躁+躺平+内卷的大环境下，对于毫无收益的开源项目，单靠坊间飞控爱好者、
								个人情怀式、自发地主动输出去参与开源项目的方式行不通，好的开源项目需要请专职人员做好售后技术服务、配套
								手册和视频教程要覆盖新手入门到进阶阶段，使用过程中对用户反馈问题和需求进行统计、在实践中完成对产品的一
								次次完善与迭代升级。
-----------------------------------------------------------------------------------------------------------------------
*                                                 为什么选择无名创新？
*                                         感动人心价格厚道，最靠谱的开源飞控；
*                                         国内业界良心之作，最精致的售后服务；
*                                         追求极致用户体验，高效进阶学习之路；
*                                         萌新不再孤单求索，合理把握开源尺度；
*                                         响应国家扶贫号召，促进教育体制公平；
*                                         新时代奋斗最出彩，建人类命运共同体。 
-----------------------------------------------------------------------------------------------------------------------
*               生命不息、奋斗不止；前人栽树，后人乘凉！！！
*               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
*               学习优秀者，简历可推荐到DJI、ZEROTECH、XAG、AEE、GDU、AUTEL、EWATT、HIGH GREAT等公司就业
*               求职简历请发送：15671678205@163.com，需备注求职意向单位、岗位、待遇等
*               无名创新开源飞控QQ群：2号群465082224、1号群540707961、TI MSPM0学习中心828746221
*               CSDN博客：http://blog.csdn.net/u011992534
*               B站教学视频：https://space.bilibili.com/67803559/#/video				优酷ID：NamelessCotrun无名小哥
*               无名创新国内首款TI开源飞控设计初衷、知乎专栏:https://zhuanlan.zhihu.com/p/54471146
*               TI教育无人机品质供应商，开源-教学-培养-竞赛,盘古 TI MCU系统板 NController多功能控制器https://item.taobao.com/item.htm?abbucket=19&id=697442280363 
*               淘宝店铺：https://namelesstech.taobao.com/
*               公司官网:www.nameless.tech
*               修改日期:2024/10/01                  
*               版本：躺赢者PRO_V3——CarryPilot_V7.0.1
*               版权所有，盗版必究。
*               Copyright(C) 2019-2030 武汉无名创新科技有限公司 
*               All rights reserved
-----------------------------------------------------------------------------------------------------------------------
*               重要提示：
*               正常淘宝咸鱼转手的飞控、赠送朋友、传给学弟的都可以进售后群学习交流，
*               不得在网上销售无名创新资料，公司开放代码有软件著作权保护版权，他人不得将
*               资料代码传网上供他人下载，不得以谋利为目去销售资料代码，发现有此类操作者，
*               公司会提前告知，请1天内及时处理，否则你的侵权违规行为会被贴出在抖音、
*               今日头条、百家号、公司官网、微信公众平台、技术博客、知乎等平台予以公示曝光
*               此种侵权所为会成为个人终身污点，影响升学、找工作、社会声誉、很快就很在无人机界出名，后果很严重。
*               因此行为给公司造成重大损失者，会以法律途径解决，感谢您的合作，谢谢！！！
----------------------------------------------------------------------------------------------------------------------*/
#include "Headfile.h"
#include "Reserved_Serialport.h"


void NCLink_Data_Prase_Process_Lite(uint8_t *data_buf,uint8_t num);

void quad_getangle(float *q,float* rpy)
{
		rpy[1]= RADTODEG(FastAtan2(q[0]*q[1] + q[2]*q[3], 0.5f - q[1]*q[1] - q[2]*q[2]));
		rpy[0]= RADTODEG(FastAsin(-2.0f * (q[1]*q[3] - q[0]*q[2])));
		rpy[2]= RADTODEG(FastAtan2(q[1]*q[2] + q[0]*q[3], 0.5f - q[2]*q[2] - q[3]*q[3]));
}


void Reserved_Serialport_Init()
{
	switch(Reserved_Uart)
	{
		case FRONT_RANGE_FINDER:
			ConfigureUART6(9600,10);
		break;
		case GPS_M8N:
			Set_GPS_USART();
		break;
		case THIRD_PARTY_STATE:
			ConfigureUART6(460800,60*2);	//460800	
		break;
		default:Set_GPS_USART();
	}
}

static uint8_t NCLink_Head[2]={0xFF,0xFC};//数据帧头
static uint8_t NCLink_End[2] ={0xA1,0xA2};//数据帧尾
static uint8_t nclink_buf[100];//待接收数据缓冲区
static uint8_t send_nclink_buf[100];//待接收数据缓冲区
uint8_t send_check_back=0;
void Reserved_Serialport_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//发送N个字节长度的数据
{
  while(ui32Count--)
  {
    UARTCharPut(UART6_BASE, *pui8Buffer++);
  }
}

void NCLink_Send_IMU_Feedback(float q1	,float q2,float q3  ,float q0,
															float gx,float gy,float gz,
															float ax,float ay,float az,
															float px,float py,float pz,
															float vx,float vy,float vz)
{
  uint8_t sum=0,_cnt=0,i=0;
  send_nclink_buf[_cnt++]=NCLink_Head[0];
  send_nclink_buf[_cnt++]=NCLink_Head[1];
  send_nclink_buf[_cnt++]=0xE0;
  send_nclink_buf[_cnt++]=0;
		
	Float2Byte(&q1,send_nclink_buf,_cnt);//4
	_cnt+=4;
	Float2Byte(&q2,send_nclink_buf,_cnt);//8
	_cnt+=4;
	Float2Byte(&q3,send_nclink_buf,_cnt);//12
	_cnt+=4;
	Float2Byte(&q0,send_nclink_buf,_cnt);//16
	_cnt+=4;
	Float2Byte(&gx,send_nclink_buf,_cnt);//20
	_cnt+=4;
	Float2Byte(&gy,send_nclink_buf,_cnt);//24
	_cnt+=4;
	Float2Byte(&gz,send_nclink_buf,_cnt);//28
	_cnt+=4;
	Float2Byte(&ax,send_nclink_buf,_cnt);//32
	_cnt+=4;
	Float2Byte(&ay,send_nclink_buf,_cnt);//36
	_cnt+=4;
	Float2Byte(&az,send_nclink_buf,_cnt);//40
	_cnt+=4;
	
	Float2Byte(&px,send_nclink_buf,_cnt);//44
	_cnt+=4;
	Float2Byte(&py,send_nclink_buf,_cnt);//48
	_cnt+=4;
	Float2Byte(&pz,send_nclink_buf,_cnt);//52
	_cnt+=4;
	Float2Byte(&vx,send_nclink_buf,_cnt);//56
	_cnt+=4;
	Float2Byte(&vy,send_nclink_buf,_cnt);//60
	_cnt+=4;
	Float2Byte(&vz,send_nclink_buf,_cnt);//64
	_cnt+=4;
	
  send_nclink_buf[3] = _cnt-4;
  for(i=0;i<_cnt;i++) sum ^= send_nclink_buf[i]; 
  send_nclink_buf[_cnt++]=sum;
	
	send_nclink_buf[_cnt++]=NCLink_End[0];
	send_nclink_buf[_cnt++]=NCLink_End[1];
  Reserved_Serialport_Send(send_nclink_buf,_cnt);
}

void NCLink_Send_Quaternion_Feedback(float q1	,float q2,float q3  ,float q0,
																		 float gx,float gy,float gz,
																		 float ax,float ay,float az)
{
  uint8_t sum=0,_cnt=0,i=0;
  send_nclink_buf[_cnt++]=NCLink_Head[0];
  send_nclink_buf[_cnt++]=NCLink_Head[1];
  send_nclink_buf[_cnt++]=0xE1;
  send_nclink_buf[_cnt++]=0;
		
	Float2Byte(&q1,send_nclink_buf,_cnt);//4
	_cnt+=4;
	Float2Byte(&q2,send_nclink_buf,_cnt);//8
	_cnt+=4;
	Float2Byte(&q3,send_nclink_buf,_cnt);//12
	_cnt+=4;
	Float2Byte(&q0,send_nclink_buf,_cnt);//16
	_cnt+=4;
	Float2Byte(&gx,send_nclink_buf,_cnt);//20
	_cnt+=4;
	Float2Byte(&gy,send_nclink_buf,_cnt);//24
	_cnt+=4;
	Float2Byte(&gz,send_nclink_buf,_cnt);//28
	_cnt+=4;
	Float2Byte(&ax,send_nclink_buf,_cnt);//32
	_cnt+=4;
	Float2Byte(&ay,send_nclink_buf,_cnt);//36
	_cnt+=4;
	Float2Byte(&az,send_nclink_buf,_cnt);//40
	_cnt+=4;
	
  send_nclink_buf[3] = _cnt-4;
  for(i=0;i<_cnt;i++) sum ^= send_nclink_buf[i]; 
  send_nclink_buf[_cnt++]=sum;
	
	send_nclink_buf[_cnt++]=NCLink_End[0];
	send_nclink_buf[_cnt++]=NCLink_End[1];
  Reserved_Serialport_Send(send_nclink_buf,_cnt);
}




static void NCLink_Send_Feedback(uint8_t response)//地面站应答校验
{
  uint8_t sum = 0,i=0;
  send_nclink_buf[0]=NCLink_Head[0];
  send_nclink_buf[1]=NCLink_Head[1];
  send_nclink_buf[2]=NCLINK_SEND_CHECK;
  send_nclink_buf[3]=1;
  send_nclink_buf[4]=response;
  for(i=0;i<5;i++) sum ^= send_nclink_buf[i];
  send_nclink_buf[5]=sum;
	send_nclink_buf[6]=NCLink_End[0];
	send_nclink_buf[7]=NCLink_End[1];
  Reserved_Serialport_Send(send_nclink_buf,8);
}

void NCLink_Send_IMU_Feedback_PC(void)//1.54ms
{
	static uint16_t _cnt=0;
	_cnt++;
	if(_cnt>=10)//50ms————20hz
	{
		_cnt=0;
		//温度稳定且陀螺仪零偏校准完毕才发送飞机IMU数据给板载计算机
		//避免激光雷达转动影响陀螺仪标定过程
		if(Gyro_Safety_Calibration_Flag==1
			&&(Optical_Type_Present==3||Optical_Type_Present==4))//设置的定位类型为RPLIDAR/T265/3DLIDAR_UNI
		{			
			NCLink_Send_IMU_Feedback(WP_AHRS.q[1],WP_AHRS.q[2],WP_AHRS.q[3],WP_AHRS.q[0],
															 WP_AHRS.Pitch_Gyro_Rad,WP_AHRS.Roll_Gyro_Rad,WP_AHRS.Yaw_Gyro_Rad,
															 WP_AHRS.Accel_X_MPSS,WP_AHRS.Accel_Y_MPSS,WP_AHRS.Accel_Z_MPSS,
															 VIO_SINS.Position[_EAST]/100.0f,VIO_SINS.Position[_NORTH]/100.0f,NamelessQuad.Position[_UP]/100.0f,
															 VIO_SINS.Speed[_EAST]/100.0f   ,VIO_SINS.Speed[_NORTH]/100.0f   ,NamelessQuad.Speed[_UP]/100.0f);
		}
	}
	
	switch(send_check_back)
	{
		case 1:
		{
			NCLink_Send_Feedback(NCLINK_SEND_DIS);
			send_check_back=0;
		}
		break;
		case 2:
		{
			NCLink_Send_Feedback(NCLINK_SEND_NAV_CTRL);
			send_check_back=0;
		}
		break;	
		case 3:
		{
			NCLink_Send_Feedback(NCLINK_SEND_NAV_CTRL_FINISH);
			send_check_back=0;		
		}
		break;
		case 4:
		{
			NCLink_Send_Feedback(NCLINK_SEND_SLAM_SYSTEM_RESET);//复位slam
			send_check_back=0;
			//蜂鸣器提示复位成功
      buzzer_setup(1000,0.5f,2);			
		}
		break;
		case 5:
		{
			NCLink_Send_Feedback(NCLINK_SEND_SLAM_STOP_MOTOR);
			send_check_back=0;		
		}
		break;
		case 6:
		{
			NCLink_Send_Feedback(NCLINK_SEND_SLAM_START_MOTOR);
			send_check_back=0;		
		}
		break;
	}
}

uint32_t nclink_receive_fault_cnt=0;
void NCLink_Data_Prase_Prepare_Lite(uint8_t data)//地面站数据解析
{
  static uint8_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==NCLink_Head[1])//判断帧头1
  {
    state=1;
    nclink_buf[0]=data;
  }
  else if(state==1&&data==NCLink_Head[0])//判断帧头2
  {
    state=2;
    nclink_buf[1]=data;
  }
  else if(state==2&&data<0XF1)//功能字节
  {
    state=3;
    nclink_buf[2]=data;
  }
  else if(state==3&&data<100)//有效数据长度
  {
    state = 4;
    nclink_buf[3]=data;
    data_len = data;
    data_cnt = 0;
  }
  else if(state==4&&data_len>0)//数据接收
  {
    data_len--;
    nclink_buf[4+data_cnt++]=data;
    if(data_len==0)  state = 5;
  }
  else if(state==5)//异或校验
  {
    state = 6;
    nclink_buf[4+data_cnt++]=data;
  }
	else if(state==6&&data==NCLink_End[0])//帧尾0
	{
			state = 7;
			nclink_buf[4+data_cnt++]=data;
	}
	else if(state==7&&data==NCLink_End[1])//帧尾1
	{
			state = 0;
			nclink_buf[4+data_cnt]=data;
		  NCLink_Data_Prase_Process_Lite(nclink_buf,data_cnt+5);//数据解析
	}
  else 
	{
		state = 0;
		nclink_receive_fault_cnt++;
	}
}




systime nclink_t;
void NCLink_Data_Prase_Process_Lite(uint8_t *data_buf,uint8_t num)//飞控数据解析进程
{
  uint8_t sum = 0;
  for(uint8_t i=0;i<(num-3);i++)  sum ^= *(data_buf+i);
  if(!(sum==*(data_buf+num-3)))    																					return;//判断sum	
	if(!(*(data_buf)==NCLink_Head[1]&&*(data_buf+1)==NCLink_Head[0]))         return;//判断帧头
	if(!(*(data_buf+num-2)==NCLink_End[0]&&*(data_buf+num-1)==NCLink_End[1])) return;//帧尾校验  

	if(*(data_buf+2)==0X0A)                             //地面站控制移动数据
  {		
    ngs_sdk.move_mode=*(data_buf+4),
		ngs_sdk.mode_order=*(data_buf+5);
    ngs_sdk.move_distance=(uint16_t)(*(data_buf+6)<<8)|*(data_buf+7);;
    ngs_sdk.update_flag=true;
		
		ngs_sdk.move_flag.sdk_front_flag=false;
		ngs_sdk.move_flag.sdk_behind_flag=false;
		ngs_sdk.move_flag.sdk_left_flag=false;
		ngs_sdk.move_flag.sdk_right_flag=false;
		ngs_sdk.move_flag.sdk_up_flag=false;
		ngs_sdk.move_flag.sdk_down_flag=false;
		
		if(*(data_buf+4)==SDK_FRONT)
		{					
			ngs_sdk.move_flag.sdk_front_flag=true;
			ngs_sdk.f_distance=ngs_sdk.move_distance;
		}
		else if(*(data_buf+4)==SDK_BEHIND) 
		{					
			ngs_sdk.move_flag.sdk_behind_flag=true;
			ngs_sdk.f_distance=-ngs_sdk.move_distance;
		}
		else if(*(data_buf+4)==SDK_LEFT)  
		{			
			ngs_sdk.move_flag.sdk_left_flag=true;
			ngs_sdk.f_distance=ngs_sdk.move_distance;
		}
		else if(*(data_buf+4)==SDK_RIGHT)
		{					
			ngs_sdk.move_flag.sdk_right_flag=true;
			ngs_sdk.f_distance=-ngs_sdk.move_distance;
		}
		else if(*(data_buf+4)==SDK_UP)
		{  				
			ngs_sdk.move_flag.sdk_up_flag=true;
			ngs_sdk.f_distance=ngs_sdk.move_distance;
		}
		else if(*(data_buf+4)==SDK_DOWN) 
		{					
			ngs_sdk.move_flag.sdk_down_flag=true;
			ngs_sdk.f_distance=-ngs_sdk.move_distance;
		}				
		NCLink_Send_Check_Flag[8]=1;
		Pilot_Status_Tick();	
		send_check_back=1;
  }
	else if(*(data_buf+2)==0X0C)
  {
		Guide_Flight_Lng =((int32_t)(*(data_buf+4)<<24)|(*(data_buf+5)<<16)|(*(data_buf+6)<<8)|*(data_buf+7));
		Guide_Flight_Lat =((int32_t)(*(data_buf+8)<<24)|(*(data_buf+9)<<16)|(*(data_buf+10)<<8)|*(data_buf+11));
		Guide_Flight_Cnt++;;
		Guide_Flight_Flag=1;
		Pilot_Status_Tick();
	}
	else if(*(data_buf+2)==0X0D)
	{
		Byte2Float(data_buf,4,&current_state.position_x);
		Byte2Float(data_buf,8,&current_state.position_y);
		Byte2Float(data_buf,12,&current_state.position_z);
		Byte2Float(data_buf,16,&current_state.velocity_x);
		Byte2Float(data_buf,20,&current_state.velocity_y);
		Byte2Float(data_buf,24,&current_state.velocity_z);
		current_state.position_x*=100.0f;
		current_state.position_y*=100.0f;
		current_state.position_z*=100.0f;
		current_state.velocity_x*=100.0f;
		current_state.velocity_y*=100.0f;
		current_state.velocity_z*=100.0f;
		
		
		Byte2Float(data_buf,28,&current_state.q[0]);		
		Byte2Float(data_buf,32,&current_state.q[1]);
		Byte2Float(data_buf,36,&current_state.q[2]);
		Byte2Float(data_buf,40,&current_state.q[3]);
		float _q[4];
		_q[0]=current_state.q[3]*(1.0f);
		_q[1]=current_state.q[0]*(1.0f);
		_q[2]=current_state.q[2]*(-1.0f);
		_q[3]=current_state.q[1]*(1.0f);	
		
		current_state.q[0]=_q[0];
		current_state.q[1]=_q[1];
		current_state.q[2]=_q[2];
		current_state.q[3]=_q[3];

		Byte2Float(data_buf,44,&current_state.quality);
		current_state.update_flag=*(data_buf+48);
		current_state.byte[0]=*(data_buf+49);
		current_state.byte[1]=*(data_buf+50);
		current_state.byte[2]=*(data_buf+51);
		current_state.byte[3]=*(data_buf+52);
		current_state.byte[4]=*(data_buf+53);
		current_state.byte[5]=*(data_buf+54);
		current_state.byte[6]=*(data_buf+55);
		current_state.byte[7]=*(data_buf+56);

		current_state.rec_update_flag=1;
		current_state.rec_head_update_flag=1;		
		Pilot_Status_Tick();
		Get_Systime(&nclink_t);
		current_state.slam_sensor=T265_SLAM;
	}
	else if(*(data_buf+2)==0X0E)
	{
		Byte2Float(data_buf,4,&current_state.position_x);
		Byte2Float(data_buf,8,&current_state.position_y);
		Byte2Float(data_buf,12,&current_state.position_z);
		//当为激光雷达时,下面三个数据分别表示最小距离、最小距离所在角度，信号强度
		Byte2Float(data_buf,16,&current_state.velocity_x);//m
		Byte2Float(data_buf,20,&current_state.velocity_y);//deg
		Byte2Float(data_buf,24,&current_state.velocity_z);
		
		current_state.position_x*=100.0f;
		current_state.position_y*=100.0f;
		current_state.position_z*=100.0f;
		
		
		if(current_state.velocity_y<0) current_state.velocity_y+=360;//356.384094
		
		for(uint16_t i=Laser_Min_Info_Num-1;i>0;i--)
		{
			min_dis_cm_backups[i]=min_dis_cm_backups[i-1];
			min_dis_angle_backups[i]=min_dis_angle_backups[i-1];
		}
			min_dis_cm_backups[0]		=current_state.velocity_x*100.0f;//转换成cm
			min_dis_angle_backups[0]=current_state.velocity_y;		
		
			
		Byte2Float(data_buf,28,&current_state.q[0]);		
		Byte2Float(data_buf,32,&current_state.q[1]);
		Byte2Float(data_buf,36,&current_state.q[2]);
		Byte2Float(data_buf,40,&current_state.q[3]);
		float _q[4];
		_q[0]=current_state.q[3]*(1.0f);
		_q[1]=current_state.q[0]*(1.0f);
		_q[2]=current_state.q[1]*(1.0f);
		_q[3]=current_state.q[2]*(1.0f);	
		
		current_state.q[0]=_q[0];
		current_state.q[1]=_q[1];
		current_state.q[2]=_q[2];
		current_state.q[3]=_q[3];
		
		Byte2Float(data_buf,44,&current_state.quality);
		current_state.update_flag=*(data_buf+48);
		current_state.byte[0]=*(data_buf+49);
		current_state.byte[1]=*(data_buf+50);
		current_state.byte[2]=*(data_buf+51);
		current_state.byte[3]=*(data_buf+52);
		current_state.byte[4]=*(data_buf+53);
		current_state.byte[5]=*(data_buf+54);
		current_state.byte[6]=*(data_buf+55);
		current_state.byte[7]=*(data_buf+56);

		current_state.rec_update_flag=1;
		current_state.rec_head_update_flag=1;		
		
		Pilot_Status_Tick();
	  Get_Systime(&nclink_t);
		current_state.slam_sensor=LIDAR_2D_SLAM;
	}	
	else if(*(data_buf+2)==0X10)//fastlio
	{ 
		Byte2Float(data_buf,4,&current_state.position_x);
		Byte2Float(data_buf,8,&current_state.position_y);
		Byte2Float(data_buf,12,&current_state.position_z);

		current_state.position_x*=100.0f;
		current_state.position_y*=100.0f;
		current_state.position_z*=100.0f;
					
		Byte2Float(data_buf,28,&current_state.q[0]);		
		Byte2Float(data_buf,32,&current_state.q[1]);
		Byte2Float(data_buf,36,&current_state.q[2]);
		Byte2Float(data_buf,40,&current_state.q[3]);
		float _q[4];
		_q[0]=current_state.q[3]*(1.0f);
		_q[1]=current_state.q[0]*(1.0f);
		_q[2]=current_state.q[1]*(1.0f);
		_q[3]=current_state.q[2]*(1.0f);	
		
		current_state.q[0]=_q[0];
		current_state.q[1]=_q[1];
		current_state.q[2]=_q[2];
		current_state.q[3]=_q[3];
		
		Byte2Float(data_buf,44,&current_state.quality);
		current_state.update_flag=*(data_buf+48);
		current_state.byte[0]=*(data_buf+49);
		current_state.byte[1]=*(data_buf+50);
		current_state.byte[2]=*(data_buf+51);
		current_state.byte[3]=*(data_buf+52);
		current_state.byte[4]=*(data_buf+53);
		current_state.byte[5]=*(data_buf+54);
		current_state.byte[6]=*(data_buf+55);
		current_state.byte[7]=*(data_buf+56);
		
		Pilot_Status_Tick();
	  Get_Systime(&nclink_t);		
		//判断loam数据已经正常更新
		if(current_state.position_x==0&&current_state.position_y==0)
		{
			current_state.loam_update_flag=0;
			return ;//数据非正常更新时,提前退出
		}
		else current_state.loam_update_flag=1;
		
		current_state.rec_update_flag=1;
		current_state.rec_head_update_flag=1;
		current_state.slam_sensor=LOAM;
		
		current_state.alt_update_flag=1;
	}	
	else if(*(data_buf+2)==0X0F)//导航控制指令
	{
		ngs_nav_ctrl.number=(uint16_t)(*(data_buf+4)<<8) |*(data_buf+5);
		Byte2Float(data_buf,6, &ngs_nav_ctrl.x);
		Byte2Float(data_buf,10,&ngs_nav_ctrl.y);
		Byte2Float(data_buf,14,&ngs_nav_ctrl.z);
		ngs_nav_ctrl.nav_mode=*(data_buf+18);
		ngs_nav_ctrl.frame_id=*(data_buf+19);
		ngs_nav_ctrl.cmd_vel_during_cnt=(uint32_t)(*(data_buf+20)<<24|*(data_buf+21)<<16|*(data_buf+22)<<8|*(data_buf+23));	//速度控制时的作用时间,单位MS
		if(ngs_nav_ctrl.nav_mode!=CMD_VEL_MODE)	ngs_nav_ctrl.update_flag=1;
		else 
		{
			if(ngs_nav_ctrl.frame_id==0)//数据来源于ROS
			{
				ngs_nav_ctrl.cmd_vel_x=-100*ngs_nav_ctrl.x;					 //转化成cm/s
				ngs_nav_ctrl.cmd_vel_y= 100*ngs_nav_ctrl.y;          //转化成cm/s
				ngs_nav_ctrl.cmd_vel_angular_z=57.3f*ngs_nav_ctrl.z; //转化成deg/s
			}	
			else//数据来源于无名创新地面站
			{
				ngs_nav_ctrl.cmd_vel_x=-ngs_nav_ctrl.x;			   //cm/s
				ngs_nav_ctrl.cmd_vel_y= ngs_nav_ctrl.y;        //cm/s
				ngs_nav_ctrl.cmd_vel_angular_z=ngs_nav_ctrl.z; //deg/s			
			}
			//限幅处理避免新手错误操作，输入值过大导致飞机期望过大造成的失控
			ngs_nav_ctrl.cmd_vel_x=constrain_float(ngs_nav_ctrl.cmd_vel_x,-ngs_nav_ctrl.cmd_vel_max,ngs_nav_ctrl.cmd_vel_max);
			ngs_nav_ctrl.cmd_vel_y=constrain_float(ngs_nav_ctrl.cmd_vel_y,-ngs_nav_ctrl.cmd_vel_max,ngs_nav_ctrl.cmd_vel_max);
			ngs_nav_ctrl.cmd_vel_angular_z=constrain_float(ngs_nav_ctrl.cmd_vel_angular_z,-ngs_nav_ctrl.cmd_angular_max,ngs_nav_ctrl.cmd_angular_max);
			ngs_nav_ctrl.cmd_vel_during_cnt/=5;//作用时间200*5=1000ms		
			ngs_nav_ctrl.cmd_vel_update=1;
		}	
		Pilot_Status_Tick();	
		send_check_back=2;
	}	
}





us100_data us100_front;
static uint16_t us100_cnt=0;
void us100_front_statemachine(void)
{
	if(Reserved_Uart!=FRONT_RANGE_FINDER) return;
	us100_cnt++;
	if(us100_cnt>=20)
	{
		us100_cnt=0;
		us100_front.pre_last_distance=us100_front.last_distance;//上上次高度
		us100_front.last_distance=us100_front.distance;//上次高度
		us100_front.distance=US_100_Distance(COM6_Rx_Buf.Ring_Buff[0],COM6_Rx_Buf.Ring_Buff[1]);

		us100_front.last_vel=us100_front.vel;
		us100_front.vel=(us100_front.distance-us100_front.last_distance)/0.1f;
		us100_front.acc=(us100_front.vel-us100_front.last_vel)/0.1f;		
		
		COM6_Rx_Buf.Head=1;
		COM6_Rx_Buf.Tail=0; 
		UARTCharPut(UART6_BASE,US_100_Distance_CMD);
	}
}




////////////////////////
void Uart2_Serialport_Init(void)
{
	switch(Uart2_Mode)
	{
		case 0x00:
		{
			ConfigureUART2(19200);//LC30X光流
			delay_ms(1000);				 //初始化延时
		}
		break;
		case 0x01:
		{
			ConfigureUART2(230400);//激光雷达
			delay_ms(1000);				 //初始化延时
			//lsn10_motor_ctrl(0x00);
			//send_check_back=5;
		}
		break;
		default:{ConfigureUART2(19200);delay_ms(1000);}
	}
}









//			NCLink_Send_IMU_Feedback(-WP_AHRS.q[2],WP_AHRS.q[1],WP_AHRS.q[3],WP_AHRS.q[0],
//															 -WP_AHRS.Roll_Gyro_Rad,WP_AHRS.Pitch_Gyro_Rad,WP_AHRS.Yaw_Gyro_Rad,
//															 -WP_AHRS.Accel_Y_MPSS,WP_AHRS.Accel_X_MPSS,WP_AHRS.Accel_Z_MPSS,
//															 -VIO_SINS.Position[_NORTH]/100.0f,VIO_SINS.Position[_EAST]/100.0f,NamelessQuad.Position[_UP]/100.0f,
//															 -VIO_SINS.Speed[_NORTH]/100.0f   ,VIO_SINS.Speed[_EAST]/100.0f   ,NamelessQuad.Speed[_UP]/100.0f);


//		{
//			NCLink_Send_IMU_Feedback(-WP_AHRS.q[2],WP_AHRS.q[1],WP_AHRS.q[3],WP_AHRS.q[0],
//															 -WP_AHRS.Roll_Gyro_Rad,WP_AHRS.Pitch_Gyro_Rad,WP_AHRS.Yaw_Gyro_Rad,
//															 -WP_AHRS.Accel_Y_MPSS,WP_AHRS.Accel_X_MPSS,WP_AHRS.Accel_Z_MPSS,
//															 -VIO_SINS.Position[_NORTH]/100.0f,VIO_SINS.Position[_EAST]/100.0f,NamelessQuad.Position[_UP]/100.0f,
//															 -VIO_SINS.Speed[_NORTH]/100.0f   ,VIO_SINS.Speed[_EAST]/100.0f   ,NamelessQuad.Speed[_UP]/100.0f);
//		}

//		quad_getangle(current_state.q,current_state.rpy);
//		current_state.rpy[2]=(int16_t)(current_state.rpy[2]);
//		if(current_state.rpy[2]<0.05f)   current_state.rpy[2]+=360.0f;
//		if(current_state.rpy[2]>359.95f) current_state.rpy[2]-=360.0f;


//	rpy[1]= RADTODEG(atan2f(q[0]*q[1] + q[2]*q[3], 0.5f - q[1]*q[1] - q[2]*q[2]));
//	rpy[0]= RADTODEG(asinf(-2.0f * (q[1]*q[3] - q[0]*q[2])));
//	rpy[2]= RADTODEG(atan2f(q[1]*q[2] + q[0]*q[3], 0.5f - q[2]*q[2] - q[3]*q[3]));


//			NCLink_Send_Quaternion_Feedback(WP_AHRS.q[1],WP_AHRS.q[2],WP_AHRS.q[3],WP_AHRS.q[0],
//																		  WP_AHRS.Pitch_Gyro_Rad,WP_AHRS.Roll_Gyro_Rad,WP_AHRS.Yaw_Gyro_Rad,
//															        WP_AHRS.Accel_X_MPSS,WP_AHRS.Accel_Y_MPSS,WP_AHRS.Accel_Z_MPSS);


