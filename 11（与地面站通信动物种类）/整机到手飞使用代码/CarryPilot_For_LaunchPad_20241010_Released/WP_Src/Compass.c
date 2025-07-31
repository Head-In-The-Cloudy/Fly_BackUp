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
#include "Compass.h"


#define MAG_EXTERNAL_IST8310 0
#define MAG_EXTERNAL_QMC5883 1
#define MAG_EXTERNAL_TYPE    MAG_EXTERNAL_QMC5883//MAG_EXTERNAL_IST8310 

#define MAG_GPS_ENABLE 0//是否使用GPS模块上磁力计

uint8_t extern_mag_id=0xff;
void Compass_Init(void)
{										//外置磁力计IST8310、QMC5883
  I2C_GPIO_Config();
	Delay_Ms(10);
	if(MAG_EXTERNAL_TYPE==MAG_EXTERNAL_QMC5883) 			
	{
		//尝试性读取id，判断是否为hmc5883
		if(Single_ReadI2C0(QMC5883L_WR_ADDRESS,0x0D)!=0xff) return;
		if(Single_ReadI2C0(QMC5883L_WR_ADDRESS,0x06)==0xff) return;
		
		QMC5883L_Init();
		extern_mag_id=MAG_EXTERNAL_QMC5883;
		Sensor_Flag.Mag_Health=TRUE;
	}
  else		
	{
		if(Single_ReadI2C0(IST8310_SLAVE_ADDRESS,0x00)!=0x10) return;
		
		Single_WriteI2C0(IST8310_SLAVE_ADDRESS,0x41,0x24);//开启16x内部平均
	  Single_WriteI2C0(IST8310_SLAVE_ADDRESS,0x42,0xC0);//Set/Reset内部平均
		Single_WriteI2C0(IST8310_SLAVE_ADDRESS,IST8310_REG_CNTRL1,0x01);//Single Measurement Mode
		
		extern_mag_id=MAG_EXTERNAL_IST8310;
		Sensor_Flag.Mag_Health=TRUE;		
	}
	set_cutoff_frequency(40, 18,&Mag_Parameter);//传感器校准加计滤波值	
}

/***********************************************************
@函数名：Compass_Read_Data
@入口参数：vector3f *mag
@出口参数：无
功能描述：磁力计数据采集状态机
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
uint8_t Compass_Read_Data(vector3f *mag)//读取磁力计数据状态机
{
	if(extern_mag_id==0xff) return 0;//磁力计数据不健康
	
  static uint16_t compass_sampling_cnt=0;
  uint8_t buf[6];
  compass_sampling_cnt++;
  if(compass_sampling_cnt==1)
  {
		if(MAG_EXTERNAL_TYPE==MAG_EXTERNAL_IST8310)	Single_WriteI2C0(IST8310_SLAVE_ADDRESS,IST8310_REG_CNTRL1,0x01);//Single Measurement Mod		
  }
  else if(compass_sampling_cnt>=20/WP_Duty_Dt_Ms)//50ms
  {
		if(MAG_EXTERNAL_TYPE==MAG_EXTERNAL_QMC5883)
		{
			if(extern_mag_id==MAG_EXTERNAL_QMC5883)//实测为QMC5883
			{
				buf[0]=Single_ReadI2C0(QMC5883L_RD_ADDRESS,QMC5883L_DATA_READ_X_LSB);//OUT_X_L_A
				buf[1]=Single_ReadI2C0(QMC5883L_RD_ADDRESS,QMC5883L_DATA_READ_X_MSB);//OUT_X_H_A
				buf[2]=Single_ReadI2C0(QMC5883L_RD_ADDRESS,QMC5883L_DATA_READ_Y_LSB);//OUT_Y_L_A
				buf[3]=Single_ReadI2C0(QMC5883L_RD_ADDRESS,QMC5883L_DATA_READ_Y_MSB);//OUT_Y_H_A
				buf[4]=Single_ReadI2C0(QMC5883L_RD_ADDRESS,QMC5883L_DATA_READ_Z_LSB);//OUT_Z_L_A
				buf[5]=Single_ReadI2C0(QMC5883L_RD_ADDRESS,QMC5883L_DATA_READ_Z_MSB);//OUT_Z_H_A
				#if MAG_GPS_ENABLE
				mag->x= -(float)((int16_t)((buf[1]<<8)|buf[0])/QMC5883L_CONVERT_GAUSS_8G);
				mag->y=  (float)((int16_t)((buf[3]<<8)|buf[2])/QMC5883L_CONVERT_GAUSS_8G);
				mag->z= -(float)((int16_t)((buf[5]<<8)|buf[4])/QMC5883L_CONVERT_GAUSS_8G);
				#else
				mag->x= (float)((int16_t)((buf[1]<<8)|buf[0])/QMC5883L_CONVERT_GAUSS_8G);
				mag->y= (float)((int16_t)((buf[3]<<8)|buf[2])/QMC5883L_CONVERT_GAUSS_8G);
				mag->z= (float)((int16_t)((buf[5]<<8)|buf[4])/QMC5883L_CONVERT_GAUSS_8G);				
				#endif
			}
		}
		else if(MAG_EXTERNAL_TYPE==MAG_EXTERNAL_IST8310)
		{
			i2c0ReadNByte(IST8310_SLAVE_ADDRESS,0x03,buf,6);			
			mag->x= (float)( (int16_t)((buf[1]<<8)|buf[0])/330.0f);
			mag->y=-(float)( (int16_t)((buf[3]<<8)|buf[2])/330.0f);
			mag->z= (float)( (int16_t)((buf[5]<<8)|buf[4])/330.0f);			
		}		
    compass_sampling_cnt=0;
    return 1;
  }
  return 0;
}



uint16_t compass_fault_cnt=0;
void Compass_Fault_Check(void)
{ 
	if(extern_mag_id==0xff) 
	{
		Sensor_Flag.Mag_Health=FALSE;//磁力计数据不健康
		return ;
	}
	
  static uint16_t compass_gap_cnt=0;
  compass_gap_cnt++;
  if(compass_gap_cnt>=40)//每200ms检测一次，因为磁力计更新周期大于5ms
  {
    compass_gap_cnt=0;
    if(WP_Sensor.last_mag_raw.x==WP_Sensor.mag_raw.x
       &&WP_Sensor.last_mag_raw.y==WP_Sensor.mag_raw.y
         &&WP_Sensor.last_mag_raw.z==WP_Sensor.mag_raw.z)
    {
      compass_fault_cnt++;
      if(compass_fault_cnt>10)  Sensor_Flag.Mag_Health=FALSE;//磁力计数据不健康   
    }
    else
    {
      compass_fault_cnt/=2;
      if(compass_fault_cnt==0)  Sensor_Flag.Mag_Health=TRUE; 
    }
		WP_Sensor.last_mag_raw=WP_Sensor.mag_raw;
  }
}

