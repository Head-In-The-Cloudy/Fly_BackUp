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
#include "TOFSensor.h"


enum TOFSENSE_TYPE
{
	TOFSENSE_UART=0,//量程0.01-5m
	TOFSENSE_P,     //量程0.03-8m
  TOFSENSE_F,     //量程0.05-15m
  TOFSENSE_FP,    //量程0.05-25m
	TOFSENSE_NUM_MAX
};

#define GROUND_RANGE_TOFSENSE  TOFSENSE_UART //对地测距传感器类型 
#define TOFSENSE_DATA_EFFECTIVE_RATE 0.85f   //数据有效率 
uint16_t TOFSENSR_MAX_RANGE[TOFSENSE_NUM_MAX]={500,800,1500,2500}; 




#define FRAME_HEADER 					0x57
#define Frame_MARK_READ_FRAME 0x10

void Check_Front_Tofsense(void);



void NLink_Data_Send(uint8_t *buf, uint32_t cnt)  
{
	USART7_Send(buf,cnt);//用户移植时，重写此串口发送函数
}

int32_t NLINK_ParseInt24(nint24_t data)
{
  uint8_t *byte = (uint8_t *)(&data);
  return (int32_t)(byte[0] << 8 | byte[1] << 16 | byte[2] << 24) / 256;
}

uint8_t NLINK_VerifyCheckSum(const void *data, size_t data_length)
{
  const uint8_t *byte = (uint8_t *)data;
  uint8_t sum = 0;
  for (size_t i = 0; i < data_length - 1; ++i)
  {
    sum += byte[i];
  }
  return sum == byte[data_length - 1];
}


void  NLink_TOFSense_Read_Frame(uint8_t id)
{
	uint8_t buf[8];
	uint8_t check_sum=0;
	buf[0]=FRAME_HEADER;
	buf[1]=Frame_MARK_READ_FRAME;
	buf[2]=0xff;
	buf[3]=0xff;
	buf[4]=id;
	buf[5]=0xff;
	buf[6]=0xff;
	check_sum=buf[0]+buf[1]+buf[2]+buf[3]+buf[4]+buf[5]+buf[6];
	buf[7]=check_sum;//0x63;
	NLink_Data_Send(buf,8);
}

Testime tof_t[2];
void TOFSense_Data_Receive_Anl(uint8_t *data_buf,uint8_t num)
{
  uint8_t sum = 0;
	uint8_t id=0;
  for(uint8_t i=0;i<(num-1);i++)  sum+=*(data_buf+i);
  if(!(sum==*(data_buf+num-1))) 	return;//不满足和校验条件
  if(!(*(data_buf)==0x57 && *(data_buf+1)==0x00))return;//不满足帧头条件
	id=data_buf[3];
	
	tofdata[id].pre_last_distance=tofdata[id].last_distance;//上上次高度
	tofdata[id].last_distance=tofdata[id].distance;//上次高度
	tofdata[id].last_vel=tofdata[id].vel;//上次速度
	tofdata[id].last_system_time=tofdata[id].system_time;
	tofdata[id].id=data_buf[3];
	tofdata[id].system_time=data_buf[4]|(data_buf[5]<<8)|(data_buf[6]<<16)|(data_buf[7]<<24);
	tofdata[id].dis=(int32_t)(data_buf[8]<< 8|data_buf[9]<<16|data_buf[10]<<24)/256;
	tofdata[id].dis_status=data_buf[11];
	tofdata[id].signal_strength=data_buf[12]|(data_buf[13]<<8);
	tofdata[id].distance=tofdata[id].dis/10.0f;//cm
	tofdata[id].vel=(tofdata[id].distance-tofdata[id].last_distance)/0.1f; //观测速度
	tofdata[id].acc=(tofdata[id].vel-tofdata[id].last_vel)/0.1f;					  //观测加速度	

	if(id==0)
	{
		//赋值给观测高度观测量
		ground_distance_cm=tofdata[0].distance*WP_AHRS.rMat[8];
		ground_distance_div=tofdata[0].vel;
		ground_distance_acc=tofdata[0].acc;		
		WP_Sensor.tofsensor_updtate_flag=1;	
		
		if((ground_distance_cm<=TOFSENSR_MAX_RANGE[GROUND_RANGE_TOFSENSE]*TOFSENSE_DATA_EFFECTIVE_RATE 
			&&ground_distance_cm>-0.01f)//测得距离在量程内
			&&(tofdata[0].signal_strength!=0))//且信号强度不为0
			Sensor_Flag.Ground_Health=1;
		else  Sensor_Flag.Ground_Health=0;
	}		
}



void TOFSense_Prase(uint8_t data)
{
	static uint8_t tofsense_buf[20];
  static uint8_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==0x57)//判断帧头1
  {
    state=1;
    tofsense_buf[0]=data;
		Test_Period(&tof_t[0]);
  }
  else if(state==1&&data==0x00)//判断帧头2
  {
    state=2;
    tofsense_buf[1]=data;
  }
  else if(state==2&&data==0xff)//功能字节
  {
    state=3;
    tofsense_buf[2]=data;
  }
  else if(state==3&&data<=0xff)//id
  {
    state = 4;
    tofsense_buf[3]=data;
    data_len = 11;
    data_cnt = 0;
  }
  else if(state==4&&data_len>0)//数据接收
  {
    data_len--;
    tofsense_buf[4+data_cnt++]=data;
    if(data_len==0)  state = 5;
  }
  else if(state==5)//和校验
  {
    state = 0;
    tofsense_buf[4+data_cnt++]=data;
		TOFSense_Data_Receive_Anl(tofsense_buf,16);
		Test_Period(&tof_t[1]);
  }
  else state = 0;
}


ntsm_frame0_raw_t_4x4 tsm4x4;
ntsm_frame0_pixel tofsense_m[16];
void TOFSense_M_Prase(uint8_t data)
{
	static uint8_t tofsense_m_buf[112];
  static uint16_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==0x57)//判断帧头1
  {
    state=1;
    tofsense_m_buf[0]=data;
		Test_Period(&tof_t[0]);
  }
  else if(state==1&&data==0x01)//判断帧头2
  {
    state=2;
    tofsense_m_buf[1]=data;
  }
  else if(state==2&&data==0xff)//功能字节
  {
    state=3;
    tofsense_m_buf[2]=data;
  }
  else if(state==3&&data<=0xff)//id
  {
    state = 4;
    tofsense_m_buf[3]=data;
  }
  else if(state==4)//system time 0
  {
		state = 5;
    tofsense_m_buf[4]=data;
  }
  else if(state==5)//system time 1
  {
		state = 6;
    tofsense_m_buf[5]=data;
  }
  else if(state==6)//system time 2
  {
		state = 7;
    tofsense_m_buf[6]=data;
  }
  else if(state==7)//system time 3
  {
		state = 8;
    tofsense_m_buf[7]=data;
  }
  else if(state==8)//zone map
  {
    state = 9;
    tofsense_m_buf[8]=data;
		data_len=96+6;
    data_cnt = 0;
  }
  else if(state==9&&data_len>0)//数据接收
  {
    data_len--;
    tofsense_m_buf[9+data_cnt++]=data;
    if(data_len==0)  state = 10;
  }
  else if(state==10)//和校验
  {
    state = 0;
    tofsense_m_buf[9+data_cnt++]=data;
		memcpy(&tsm4x4,tofsense_m_buf,112);
		
		if(tsm4x4.pixel_count!=16) return;
		if(!NLINK_VerifyCheckSum(tofsense_m_buf,9+data_cnt)) return;
		Test_Period(&tof_t[1]);
		for(uint16_t i=0;i<16;i++)
		{
			tofsense_m[i].dis_status = tsm4x4.pixels[i].dis_status;
			tofsense_m[i].signal_strength = tsm4x4.pixels[i].signal_strength;
			tofsense_m[i].dis_mm=NLINK_ParseInt24(tsm4x4.pixels[i].dis)/1000.0f;
		}
		
		//赋值给观测高度观测量
		float tmp=0;
		uint8_t status=(tofsense_m[5].dis_status!=0xff)
									|(tofsense_m[6].dis_status!=0xff)
									|(tofsense_m[9].dis_status!=0xff)
									|(tofsense_m[10].dis_status!=0xff)
									|(tofsense_m[0].dis_status!=0xff)
									|(tofsense_m[3].dis_status!=0xff)
									|(tofsense_m[12].dis_status!=0xff)
									|(tofsense_m[15].dis_status!=0xff);
		
		if(status==0) return;//至少有一个数据未超量程
		
		//简单取最大测量值作为高度观测值
		if(tofsense_m[5].dis_status!=0xff)  tmp=fmaxf(tmp,tofsense_m[5].dis_mm);
		if(tofsense_m[6].dis_status!=0xff)  tmp=fmaxf(tmp,tofsense_m[6].dis_mm);
		if(tofsense_m[9].dis_status!=0xff)  tmp=fmaxf(tmp,tofsense_m[9].dis_mm);
		if(tofsense_m[10].dis_status!=0xff) tmp=fmaxf(tmp,tofsense_m[10].dis_mm);
    
		if(tofsense_m[0].dis_status!=0xff)  tmp=fmaxf(tmp,tofsense_m[0].dis_mm);
		if(tofsense_m[3].dis_status!=0xff)  tmp=fmaxf(tmp,tofsense_m[3].dis_mm);
		if(tofsense_m[12].dis_status!=0xff) tmp=fmaxf(tmp,tofsense_m[12].dis_mm);
		if(tofsense_m[15].dis_status!=0xff) tmp=fmaxf(tmp,tofsense_m[15].dis_mm);
		
		ground_distance_cm=0.1f*tmp*WP_AHRS.rMat[8];
		ground_distance_div=0;
		ground_distance_acc=0;		
		WP_Sensor.tofsensor_updtate_flag=1;	
		
		if((ground_distance_cm<=350&&ground_distance_cm>-0.01f)//测得距离在量程内
			&&status)//数据未超量程
			Sensor_Flag.Ground_Health=1;
		else  Sensor_Flag.Ground_Health=0;
  }
  else 
	{
		state = 0;
	}
}


TOFSensor_Data tofdata[TOFSENSE_MAX_NUMBER]; 
systime tofsensor;
void TOF_Statemachine(void)
{
	static uint16_t inquire_cnt=0;
	//inquire_cnt++;
	switch(inquire_cnt)
	{
		case 0 :
		{
			NLink_TOFSense_Read_Frame(0x00);
		}
		break;
		case 1 :
		{
			NLink_TOFSense_Read_Frame(0x01);	
		}
		break;
		case 2:
		{
			NLink_TOFSense_Read_Frame(0x02);
		}
		break;
		case 3:
		{
			NLink_TOFSense_Read_Frame(0x03);
		}
		break;
		case 4:
		{
			NLink_TOFSense_Read_Frame(0x04);
		}
		break;
		case 5:
		{
			NLink_TOFSense_Read_Frame(0x05);
		}
		break;
		case 6:
		{
			NLink_TOFSense_Read_Frame(0x06);
		}
		break;
		case 7:
		{
			NLink_TOFSense_Read_Frame(0x07);
		}
		break;
		case 8:
		{
			NLink_TOFSense_Read_Frame(0x08);
		}
		break;
		case 9:
		{
			inquire_cnt=0;
			NLink_TOFSense_Read_Frame(0x09);
		}
		break;
		default:	;
	}
}





float front_tofsense_distance=0;
uint8_t front_tofsense_distance_valid_cnt=0;
uint8_t front_tofsense_distance_valid_flag=0;
void Check_Front_Tofsense(void)
{
	float _dis[5]={0},tmp=0;
	front_tofsense_distance_valid_cnt=0;
	front_tofsense_distance_valid_flag=0;
	uint8_t i=0;
	for(uint8_t j=1;j<5;j++)
	{
		if(tofdata[j].signal_strength!=0) 
		{
			front_tofsense_distance_valid_cnt++;
			_dis[i++]=tofdata[j].distance;
			front_tofsense_distance_valid_flag=1;
		}
	}
	
	//新增一组超声波测距US100
	if(us100_front.distance>0&&us100_front.distance<200)
	{
		front_tofsense_distance_valid_cnt++;
		_dis[i++]=us100_front.distance;
		front_tofsense_distance_valid_flag=1;
	}
		
	if(front_tofsense_distance_valid_cnt==0) 			front_tofsense_distance=200;//没有扫描到塔杆，默认给200cm
	else if(front_tofsense_distance_valid_cnt==1) front_tofsense_distance=_dis[0];
	else if(front_tofsense_distance_valid_cnt==2) front_tofsense_distance=fminf(_dis[0],_dis[1]);
	else if(front_tofsense_distance_valid_cnt==3) 
	{
		tmp=fminf(_dis[0],_dis[1]);
		tmp=fminf(tmp,_dis[2]);
		front_tofsense_distance=tmp;
	}
	else if(front_tofsense_distance_valid_cnt==4)  
	{
		tmp=fminf(_dis[0],_dis[1]);
		tmp=fminf(tmp,_dis[2]);
		tmp=fminf(tmp,_dis[3]);
		front_tofsense_distance=tmp;
	}
	else if(front_tofsense_distance_valid_cnt==5)  
	{
		tmp=fminf(_dis[0],_dis[1]);
		tmp=fminf(tmp,_dis[2]);
		tmp=fminf(tmp,_dis[3]);
		tmp=fminf(tmp,_dis[4]);
		front_tofsense_distance=tmp;
	}	
	
}
	














uint8_t send_buf[18]={'\0'};
//计算检验和
u8 countsum(u8 *buf)
{
	uint8_t len = 0;
	uint8_t checksum =0;
	len = sizeof(buf)+1;
	while(len --)
	{
		checksum += *buf;
		buf++;
	}
	
	//保留最后两位
	checksum &=0xFF;
	
	return checksum;
}


//停止扫描
void stop_scan(void)
{
	send_buf[0] = 0xAA;
	send_buf[1] = 0x55;
	send_buf[2] = 0x61;
	send_buf[3] = 0x00;
	send_buf[4] = 0x60;
	
	USART7_Send(send_buf,5);
} 

//开始扫描
void start_scan(void)
{
	send_buf[0] = 0xAA;
	send_buf[1] = 0x55;
	send_buf[2] = 0x60;
	send_buf[3] = 0x00;
	send_buf[4] = 0x5F;
	
	USART7_Send(send_buf,5);
}

//设置用标准数据的格式输出
void SMD15_setstandard(void)
{
	send_buf[0] = 0xAA;
	send_buf[1] = 0x55;
	send_buf[2] = 0x67;
	send_buf[3] = 0x01;
	send_buf[4] = 0x00;
	send_buf[5] = 0x67;
	
	USART7_Send(send_buf,6);
}

//设置用pixhawk数据的格式输出-这种模式雷达直接输出测距信息到串口调试助手可以直接显示 
void SMD15_setpixhawk(void)//不使用
{
	send_buf[0] = 0xAA;
	send_buf[1] = 0x55;
	send_buf[2] = 0x67;
	send_buf[3] = 0x01;
	send_buf[4] = 0x01;
	send_buf[5] = 0x68;
	
	USART7_Send(send_buf,6);
}

//设置波特率 
//230400、460800、512000、921600、1500000 分别对应代号 0-4（默认为 460800）
void SMD15_setbaudrate(u8 i)
{
	send_buf[0] = 0xAA;
	send_buf[1] = 0x55;
	send_buf[2] = 0x66;
	send_buf[3] = 0x01;
	
	switch(i)
	{
		case 0:send_buf[4] = 0x00;break;
		case 1:send_buf[4] = 0x01;break;
		case 2:send_buf[4] = 0x02;break;
		case 3:send_buf[4] = 0x03;break;
		case 4:send_buf[4] = 0x04;break;
		default :break;
	}
	send_buf[5] = countsum(send_buf);
	USART7_Send(send_buf,6);
}

//设置输出频率 
//10hz、100hz、200hz、500hz、1000hz、1800hz 输出频率，分别对应代号 0-5（默认为100hz）
void SMD15_setScanfHZ(u8 hz)
{
	send_buf[0] = 0xAA;
	send_buf[1] = 0x55;
	send_buf[2] = 0x64;
	send_buf[3] = 0x01;
	
	switch(hz)
	{
		case 0:send_buf[4] = 0x00;break;
		case 1:send_buf[4] = 0x01;break;
		case 2:send_buf[4] = 0x02;break;
		case 3:send_buf[4] = 0x03;break;
		case 4:send_buf[4] = 0x04;break;
		case 5:send_buf[4] = 0x05;break;
		default :break;
	}
	send_buf[5] = countsum(send_buf);
	USART7_Send(send_buf,6);
}



systime sdm15_t;
void sdm15_prase(uint8_t data)
{
	static uint8_t sdm15_buf[9];
  static uint8_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==0xAA)//判断帧头1
  {
    state=1;
    sdm15_buf[0]=data;
  }
  else if(state==1&&data==0x55)//判断帧头2
  {
    state=2;
    sdm15_buf[1]=data;
    data_cnt = 0;
  }
  else if(state==2&&data==0x60)//命令类型
  {
    state=3;
    sdm15_buf[2]=data;
    data_cnt = 0;
  }
  else if(state==3&&data==0x04)//判断数据长度
  {
    state=4;
    sdm15_buf[3]=data;
    data_len = 4;
    data_cnt = 0;
  }
  else if(state==4&&data_len>0)//数据接收
  {
    data_len--;
    sdm15_buf[4+data_cnt++]=data;
    if(data_len==0)  state = 5;
  }
  else if(state==5)//和校验
  {
    state = 0;
    sdm15_buf[4+data_cnt++]=data;
		uint8_t sum=0;
		for(uint8_t i=0;i<8;i++)
		{
			sum+=sdm15_buf[i];
		}
		if(sum==sdm15_buf[8])//满足和校验
		{
			tofdata[0].pre_last_distance=tofdata[0].last_distance;//上上次高度
			tofdata[0].last_distance=tofdata[0].distance;//上次高度
			tofdata[0].last_vel=tofdata[0].vel;//上次速度
			tofdata[0].last_system_time=tofdata[0].system_time;
			tofdata[0].system_time=millis();
			/*********************************************************************************/	
			Get_Systime(&sdm15_t);
			tofdata[0].dis        		 = sdm15_buf[4] | sdm15_buf[5]<<8;
			tofdata[0].signal_strength = sdm15_buf[6];
			tofdata[0].range_precision = sdm15_buf[7];
			tofdata[0].dis_status    	= 0;
			tofdata[0].id          		= 0;

			tofdata[0].distance=tofdata[0].dis*0.1f;//cm		
			tofdata[0].update_flag=1;
			if(tofdata[0].signal_strength>=50)
			{
				if(tofdata[0].distance<1200&&tofdata[0].distance>1.0f)	tofdata[0].valid=1;
				else tofdata[0].valid=0;
			}
			else tofdata[0].valid=0;
			tofdata[0].period_ms=sdm15_t.period_int;
      /*********************************************************************************/	
			tofdata[0].vel=(tofdata[0].distance-tofdata[0].last_distance)/0.1f; //观测速度
			tofdata[0].acc=(tofdata[0].vel-tofdata[0].last_vel)/0.1f;					  //观测加速度
			
			//赋值给观测高度观测量
			ground_distance_cm=tofdata[0].distance*WP_AHRS.rMat[8];
			ground_distance_div=tofdata[0].vel;
			ground_distance_acc=tofdata[0].acc;		
			WP_Sensor.tofsensor_updtate_flag=1;	
			Sensor_Flag.Ground_Health=tofdata[0].valid;
		}
		memset(sdm15_buf,0,9);//清一下数据
  }
  else 
	{
		state = 0;
	}
}


//初始化配置SMD15
void SMD15_init(uint32_t baudrate)//要放在串口2初始化后面
{
	delay_ms(200);//等待串口初始化完成
//	//停止扫描
//	stop_scan();
//	delay_ms(1);
	SMD15_setstandard();//设置用标准数据的格式输出
//	SMD15_setScanfHZ(1);//设置扫描频率
//	delay_ms(5);
//	uint8_t baud;
//	//激光雷达初始化 230400、460800、512000、921600、1500000
//	if     (baudrate ==230400) 	baud = 0;
//	else if(baudrate ==460800) 	baud = 1;
//	else if(baudrate ==512000)	baud = 2;
//	else if(baudrate ==921600) 	baud = 3;//这是stm32最高的波特率，理论是4.5M,但可达到这个9.2MHz
//	else if(baudrate ==1500000) baud = 4;	
//	SMD15_setbaudrate(baud);//设置波特率 设置成功后，重启雷达生效 一般不设置
//	//开始扫描
	start_scan();
}






void tofsense_init(void)
{
	float tmp_ground_sensor=US100;
	ReadFlashParameterOne(GROUND_DISTANCE_DEFAULT,&tmp_ground_sensor);
  if(isnan(tmp_ground_sensor)==0)	rangefinder_current=(uint8_t)(tmp_ground_sensor);
  switch(rangefinder_current)
	{
		case US100:
			ConfigureUART7(9600);
		break;
		case TFMINI:
			ConfigureUART7(115200);
		break;	
		case TOFSENSE:
			ConfigureUART7(921600);
		break;	
		case SMD15:
			ConfigureUART7(460800);
			SMD15_init(460800);
		break;
		case TOFSENSE_M:
			ConfigureUART7(921600);
		break;
		case LIDAR3D:
		default:{ConfigureUART7(921600);}		
	}		
}



