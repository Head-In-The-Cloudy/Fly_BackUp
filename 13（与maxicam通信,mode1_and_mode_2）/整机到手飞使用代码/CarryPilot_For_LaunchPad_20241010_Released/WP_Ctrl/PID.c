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
#include "Filter.h"
#include "PID.h"

#define NC330_PRODUCT_NUMBER  0//根据不同机型配置，选择默认PID参数
															 //0：NC280+NC330+NC360默认参数
															 //1：NC360搭载宇树3D雷达定位条件下默认参数 
															 //2：NC280+保护圈
															 
//4S锂电池，好盈乐天20A默认固件，NC360机架，定制电机KV980，精灵3大疆9045螺旋桨
//4S锂电池，好盈乐天20A默认固件，NC330机架，定制电机KV980，乾丰8045螺旋桨
//4S锂电池，BLHeli 30A电调无名小哥深度优化版本固件，NC280机架，定制电机KV1400，7040三叶螺旋桨

/*
1积分分离标志；  		2偏差限幅值；			3积分分离偏差值；
4积分值         	 	5积分限幅值；			6控制参数Kp；
7控制参数Ki；   	 	8控制参数Kd；			9控制器总输出；  
10上次控制器总输出	11总输出限幅度；
*/
const float Control_Unit[18][11]=
{
#if (NC330_PRODUCT_NUMBER==0)
	//4S锂电池，好盈乐天20A默认固件，NC360机架，定制电机KV980，9045螺旋桨
  {0 ,30  ,0  ,0 , 100,  5.00   ,0.00  ,0.00  ,0  ,0 , 500},//Pitch_Angle;偏航角度
  {0 ,600 ,0  ,0 , 300,  1.00   ,2.00  ,5.00  ,0  ,0 , 800},//Pitch_Gyro;偏航角速度
  {0 ,30  ,0  ,0 , 100,  5.00   ,0.00  ,0.00  ,0  ,0 , 500},//Roll_Angle;横滚角
  {0 ,600 ,0  ,0 , 300,  1.00   ,2.00  ,5.00  ,0  ,0 , 800},//Roll_Gyro;横滚角速度
  {0 ,45  ,0  ,0 , 150 , 5.00   ,0.00  ,0.00  ,0  ,0 , 500},//Yaw_Angle;偏航角
  {0 ,300 ,0  ,0 , 200,  2.00   ,1.00  ,2.00  ,0  ,0 , 800},//Yaw_Gyro;偏航角速度 
#endif
		
#if (NC330_PRODUCT_NUMBER==1)
	//4S锂电池，好盈乐天20A默认固件，NC360机架，定制电机KV980，9045螺旋桨
  {0 ,30  ,0  ,0 , 100,  5.00   ,0.00  ,0.00  ,0  ,0 , 500 },//Pitch_Angle;偏航角度
  {0 ,500 ,0  ,0 , 300,  0.80   ,2.00  ,5.00  ,0  ,0 , 800 },//Pitch_Gyro;偏航角速度
  {0 ,30  ,0  ,0 , 100,  5.00   ,0.00  ,0.00  ,0  ,0 , 500 },//Roll_Angle;横滚角
  {0 ,500 ,0  ,0 , 300,  0.80   ,2.00  ,5.00  ,0  ,0 , 800 },//Roll_Gyro;横滚角速度
  {0 ,45  ,0  ,0 , 150 , 5.00   ,0.00  ,0.00  ,0  ,0 , 500 },//Yaw_Angle;偏航角
  {0 ,300 ,0  ,0 , 200,  2.00   ,1.00  ,2.00  ,0  ,0 , 800 },//Yaw_Gyro;偏航角速度 
#endif
	
#if (NC330_PRODUCT_NUMBER==2)//NC280+保护圈
  {0 ,30  ,0  ,0 , 100,  5.00   ,0.00  ,0.00  ,0  ,0 , 500},//Pitch_Angle;偏航角度
  {0 ,600 ,0  ,0 , 300,  1.20   ,2.00  ,6.00  ,0  ,0 , 800},//Pitch_Gyro;偏航角速度
  {0 ,30  ,0  ,0 , 100,  5.00   ,0.00  ,0.00  ,0  ,0 , 500},//Roll_Angle;横滚角
  {0 ,600 ,0  ,0 , 300,  1.20   ,2.00  ,6.00  ,0  ,0 , 800},//Roll_Gyro;横滚角速度
  {0 ,45  ,0  ,0 , 150 , 5.00   ,0.00  ,0.00  ,0  ,0 , 500},//Yaw_Angle;偏航角
  {0 ,300 ,0  ,0 , 200,  3.00   ,1.00  ,2.50  ,0  ,0 , 800},//Yaw_Gyro;偏航角速度  
#endif
	
  //定高参数
  //高度单项比例控制，有偏差限幅、总输出即为最大攀升、下降速度400cm/s
  //Z轴速度比例+积分控制，无偏差限幅
  {0 ,200 ,0  ,0 ,100 ,  1.00     ,0.000   ,0    ,0  ,0 ,500 },//High_Position;海拔高度位置
  {0 ,600 ,0  ,0 ,500 ,  5.00     ,0.000   ,0    ,0  ,0 ,1000},//High_Speed;海拔攀升速度
	/*
	1积分分离标志；  		2偏差限幅值；			3积分分离偏差值；
	4积分值         	 	5积分限幅值；			6控制参数Kp；
	7控制参数Ki；   	 	8控制参数Kd；			9控制器总输出；  
	10上次控制器总输出	11总输出限幅度；
	*/
  /*                                       Kp        Ki        Kd            */
  /*1  2  3  4  5  6   7  8   9   10  11    12        13        14  15  16  17*/
  {0 ,180 ,0  ,0 ,8,   0.200    ,0.000    ,0    ,0    ,0 ,300},//Longitude_Position;水平经度位置
  {0 ,500 ,0  ,0 ,450, 1.800    ,0.450    ,0    ,0    ,0 ,800},//Longitude_Speed;水平经度速度
  {0 ,180 ,0  ,0 ,8,   0.200    ,0.000    ,0    ,0    ,0 ,300},//Latitude_Position;水平纬度位置
  {0 ,500 ,0  ,0 ,450, 1.800    ,0.450    ,0    ,0    ,0 ,800},//Latitude_Speed;水平纬度速度
  /*************加速度控制器****************/
  //期望最大加速度1000cm/s^2
  {0 ,1500 ,0  ,0 ,750,0.10    ,2.0000    ,0.0  ,0   ,0 ,1000},//垂直加速度控制器		0.12  1.5
		
  {0 ,100  ,0  ,0 ,3,  0.32    ,0.0000    ,0    ,0   ,0 ,150}, //预留用户1控制器
  {0 ,100  ,0  ,0 ,15, 0.45    ,0.0000    ,0.0  ,0   ,0 ,25 }, //预留用户2控制器
  /*************光流位置、速度控制器****************/
  {0 ,100   ,15  ,0 ,15,  0.30  ,0.00  ,0   ,0   ,0 ,30},//光流水平位置控制器  0.40        0.25
  {0 ,100   ,30  ,0 ,200, 3.00  ,0.02  ,0.0 ,0   ,0 ,450},//光流水平速度控制器 3.00  ,0.03 4.50  ,0.05
		
  {0 ,50    ,30  ,0 ,100, 0.50  ,0.00  ,2.5 ,0   ,0 ,300},//SDK位置控制器   2.5  150
};



lpf_param lpf_param_gyro,lpf_param_gyro_maple;
lpf_param Control_Device_Div_LPF_Parameter;
lpf_param Control_Device_Err_LPF_Parameter;
lpf_param Control_Device_SDK_Err_LPF_Parameter;
AllControler Total_Controller;//系统总控制器
void PID_Init(PID_Controler *Controler,Controler_Label Label)
{
  Controler->Err_Limit_Flag=1;//偏差限幅标志
  Controler->Integrate_Limit_Flag=1;//积分限幅标志
  Controler->Integrate_Separation_Flag=(uint8)(Control_Unit[Label][0]);//1积分分离标志
  Controler->Expect=0;//期望
  Controler->FeedBack=0;//反馈值
  Controler->Err=0;//偏差
  Controler->Last_Err=0;//上次偏差
  Controler->Err_Max=Control_Unit[Label][1];//2偏差限幅值
  Controler->Integrate_Separation_Err=Control_Unit[Label][2];//3积分分离偏差值
  Controler->Integrate=Control_Unit[Label][3];//4积分值
  Controler->Integrate_Max=Control_Unit[Label][4];//5积分限幅值
  Controler->Kp=Control_Unit[Label][5];//6控制参数Kp
  Controler->Ki=Control_Unit[Label][6];//7控制参数Ki
  Controler->Kd=Control_Unit[Label][7];//8控制参数Ki
  Controler->Control_OutPut=Control_Unit[Label][8];//9控制器总输出
  Controler->Last_Control_OutPut=Control_Unit[Label][9];//10上次控制器总输出
  Controler->Control_OutPut_Limit=Control_Unit[Label][10];//11上次控制器总输出
}



void set_d_lpf_alpha(int16_t cutoff_frequency, float time_step,float *_d_lpf_alpha)
{    
    // calculate alpha
    float rc = 1/(2*PI*cutoff_frequency);
    *_d_lpf_alpha = time_step / (time_step + rc);
}


void Total_PID_Init(void)
{
  PID_Init(&Total_Controller.Pitch_Angle_Control,Pitch_Angle_Controler);
  PID_Init(&Total_Controller.Pitch_Gyro_Control,Pitch_Gyro_Controler);
  PID_Init(&Total_Controller.Roll_Angle_Control,Roll_Angle_Controler);
  PID_Init(&Total_Controller.Roll_Gyro_Control,Roll_Gyro_Controler);
  PID_Init(&Total_Controller.Yaw_Angle_Control,Yaw_Angle_Controler);
  PID_Init(&Total_Controller.Yaw_Gyro_Control,Yaw_Gyro_Controler);
  PID_Init(&Total_Controller.Height_Position_Control,Height_Position_Controler);
  PID_Init(&Total_Controller.Height_Speed_Control,Height_Speed_Controler);
  PID_Init(&Total_Controller.Longitude_Position_Control,Longitude_Position_Controler);
  PID_Init(&Total_Controller.Longitude_Speed_Control,Longitude_Speed_Controler);
  PID_Init(&Total_Controller.Latitude_Position_Control,Latitude_Position_Controler);
  PID_Init(&Total_Controller.Latitude_Speed_Control,Latitude_Speed_Controler);  
  PID_Init(&Total_Controller.Height_Acce_Control,Height_Acce_Controler);
	
  PID_Init(&Total_Controller.Reserved_User1_Control,Reserved_User1_Controler);
  PID_Init(&Total_Controller.Reserved_User2_Control,Reserved_User2_Controler);
	
  PID_Init(&Total_Controller.Optical_Position_Control,Optical_Position_Controler);
  PID_Init(&Total_Controller.Optical_Speed_Control,Optical_Speed_Controler);
	
	
  PID_Init(&Total_Controller.SDK_Roll_Position_Control,SDK_Roll_Position_Controler);
  PID_Init(&Total_Controller.SDK_Pitch_Position_Control,SDK_Roll_Position_Controler);
  
  set_cutoff_frequency(100, 10 ,&Control_Device_SDK_Err_LPF_Parameter);//100 5
	//set_cutoff_frequency(10, 4 ,&Control_Device_SDK_Err_LPF_Parameter);
	
  set_cutoff_frequency(fc_ctrl_hz, 5 ,&Control_Device_Err_LPF_Parameter);//10  PID控制器偏差低通
  set_cutoff_frequency(fc_ctrl_hz, 20,&Control_Device_Div_LPF_Parameter); //20
  set_cutoff_frequency(fc_ctrl_hz, 15,&lpf_param_gyro);//15  30  10
	
	set_cutoff_frequency(fc_ctrl_hz, 49,&lpf_param_gyro_maple);//姿态角速度控制器低通滤波器设计
	
	set_d_lpf_alpha(40,1.0f/fc_ctrl_hz,&Total_Controller.Pitch_Gyro_Control.d_lpf_alpha);//40
	set_d_lpf_alpha(40,1.0f/fc_ctrl_hz,&Total_Controller.Roll_Gyro_Control.d_lpf_alpha);
	set_d_lpf_alpha(40,1.0f/fc_ctrl_hz,&Total_Controller.Yaw_Gyro_Control.d_lpf_alpha);
}

float PID_Control(PID_Controler *Controler,float period_second)
{
  float controller_dt=0;

	Get_Systime(&Controler->Systime_t);
	controller_dt=Controler->Systime_t.period/1000.0000f;
	if(controller_dt>1.05f*period_second||controller_dt<0.95f*period_second||isnan(controller_dt)!=0)   controller_dt=period_second;
	if(controller_dt<0.0001f) return 0;
	
  /*******偏差计算*********************/
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差
  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
    if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
    if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
  /*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
      Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  /*******积分限幅*********************/
  if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
  {
    if(Controler->Integrate>=Controler->Integrate_Max)   Controler->Integrate=Controler->Integrate_Max;
    if(Controler->Integrate<=-Controler->Integrate_Max)  Controler->Integrate=-Controler->Integrate_Max ;
  }
  /*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut=Controler->Kp*Controler->Err//比例
													 +Controler->Integrate//积分
													 +Controler->Kd*(Controler->Err-Controler->Last_Err);//微分
  /*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
  /*******返回总输出*********************/
  return Controler->Control_OutPut;
}

float PID_Control_Yaw(PID_Controler *Controler,float period_second)
{
  float controller_dt=0;
	Get_Systime(&Controler->Systime_t);
	controller_dt=Controler->Systime_t.period/1000.0000f;
	if(controller_dt>1.05f*period_second||controller_dt<0.95f*period_second||isnan(controller_dt)!=0)   controller_dt=period_second;
	if(controller_dt<0.0001f) return 0;
	
  /*******偏差计算*********************/
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差
	
	
  /***********************偏航角偏差超过+-180处理*****************************/
	if(Controler->Err<-180)  Controler->Err=Controler->Err+360;
	if(Controler->Err>180)  Controler->Err=Controler->Err-360;
	
  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
    if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
    if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
  /*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
      Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  /*******积分限幅*********************/
  if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
  {
    if(Controler->Integrate>=Controler->Integrate_Max)
      Controler->Integrate=Controler->Integrate_Max;
    if(Controler->Integrate<=-Controler->Integrate_Max)
      Controler->Integrate=-Controler->Integrate_Max ;
  }
  /*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut= Controler->Kp*Controler->Err//比例
														+Controler->Integrate//积分
														+Controler->Kd*(Controler->Err-Controler->Last_Err);//微分
  /*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
  /*******返回总输出*********************/
  return Controler->Control_OutPut;
}



float Control_Device_LPF(float curr_input,lpf_buf *Buffer,lpf_param *Parameter)
{
	if(Buffer->Output_Butter[0]==0&&
		 Buffer->Output_Butter[1]==0&&
		 Buffer->Output_Butter[2]==0&&
		 Buffer->Input_Butter[0]==0&&
		 Buffer->Input_Butter[1]==0&&
		 Buffer->Input_Butter[2]==0)
	{
		Buffer->Output_Butter[0]=curr_input;
		Buffer->Output_Butter[1]=curr_input;
		Buffer->Output_Butter[2]=curr_input;
		Buffer->Input_Butter[0]=curr_input;
		Buffer->Input_Butter[1]=curr_input;
		Buffer->Input_Butter[2]=curr_input;
		return curr_input;
	}
	
  /* 加速度计Butterworth滤波 */
  /* 获取最新x(n) */
  Buffer->Input_Butter[2]=curr_input;
  /* Butterworth滤波 */
  Buffer->Output_Butter[2]=Parameter->b[0] * Buffer->Input_Butter[2]
													+Parameter->b[1] * Buffer->Input_Butter[1]
													+Parameter->b[2] * Buffer->Input_Butter[0]
													-Parameter->a[1] * Buffer->Output_Butter[1]
													-Parameter->a[2] * Buffer->Output_Butter[0];
  /* x(n) 序列保存 */
  Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
  Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
  /* y(n) 序列保存 */
  Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
	
	for(uint16_t i=0;i<3;i++)
	{
	  if(isnan(Buffer->Output_Butter[i])==1
			||isnan(Buffer->Input_Butter[i])==1)		
			{		
				Buffer->Output_Butter[0]=curr_input;
				Buffer->Output_Butter[1]=curr_input;
				Buffer->Output_Butter[2]=curr_input;
				Buffer->Input_Butter[0]=curr_input;
				Buffer->Input_Butter[1]=curr_input;
				Buffer->Input_Butter[2]=curr_input;
				return curr_input;
			}
	}	
  return Buffer->Output_Butter[2];
}

float PID_Control_Div_LPF(PID_Controler *Controler,float period_second)
{
  int16  i=0;
  float controller_dt=0;
	Get_Systime(&Controler->Systime_t);
	controller_dt=Controler->Systime_t.period/1000.0000f;
	if(controller_dt>1.05f*period_second||controller_dt<0.95f*period_second||isnan(controller_dt)!=0)   controller_dt=period_second;
	if(controller_dt<0.0001f) return 0;
	
  /*******偏差计算*********************/
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差
  Controler->Dis_Err=Controler->Err-Controler->Last_Err;//原始微分
  
//  /******************************************/
//  //均值滤波，保证得到数据不跳变，避免期望阶跃时，微分输出异常
//	float tempa,tempb,tempc,max,min;//用于防跳变滤波  
//	tempa=Controler->Pre_Last_Dis_Err_LPF;
//  tempb=Controler->Last_Dis_Err_LPF;
//  tempc=Controler->Dis_Err;
//  max = tempa > tempb ? tempa:tempb;
//  max = max > tempc ? max:tempc;
//  min = tempa < tempb ? tempa:tempb;
//  min = min < tempc ? min:tempc;
//  if(tempa > min && tempa < max)    Controler->Dis_Err = tempa;
//  if(tempb > min  && tempb < max )  Controler->Dis_Err = tempb;
//  if(tempc > min  &&  tempc < max)  Controler->Dis_Err = tempc;
//  Controler->Pre_Last_Dis_Err_LPF = Controler->Last_Dis_Err_LPF;
//  Controler->Last_Dis_Err_LPF = Controler->Dis_Err;
//  /*****************************************/
  
  for(i=4;i>0;i--)//数字低通后微分项保存
  {
    Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
  }
  Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                                     &Controler->_lpf_buf,
                                                     &Control_Device_Div_LPF_Parameter);//巴特沃斯低通后得到的微分项,20hz
  
  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
    if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
    if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
  /*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
      Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  /*******积分限幅*********************/
  if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
  {
    if(Controler->Integrate>=Controler->Integrate_Max)
      Controler->Integrate=Controler->Integrate_Max;
    if(Controler->Integrate<=-Controler->Integrate_Max)
      Controler->Integrate=-Controler->Integrate_Max ;
  }
  /*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut=Controler->Kp*Controler->Err//比例
    +Controler->Integrate//积分
      //+Controler->Kd*Controler->Dis_Err;//微分
      +Controler->Kd*Controler->Dis_Error_History[0];//微分项来源于巴特沃斯低通滤波器
  /*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
  /*******返回总输出*********************/
  return Controler->Control_OutPut;
}


float PID_Control_Err_LPF(PID_Controler *Controler,float period_second)
{
  float controller_dt=0;
	Get_Systime(&Controler->Systime_t);
	controller_dt=Controler->Systime_t.period/1000.0000f;
	if(controller_dt>1.05f*period_second||controller_dt<0.95f*period_second||isnan(controller_dt)!=0)   controller_dt=period_second;
	if(controller_dt<0.0001f) return 0;
	
  /*******偏差计算*********************/
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差
  Controler->Dis_Err=Controler->Err-Controler->Last_Err;//原始微分
  
  Controler->Last_Err_LPF=Controler->Err_LPF;
  Controler->Err_LPF=Control_Device_LPF(Controler->Err,
                                        &Controler->_lpf_buf,
                                        &Control_Device_Err_LPF_Parameter);//巴特沃斯低通后得到的微分项,20hz
  
  Controler->Dis_Err_LPF=Controler->Err_LPF-Controler->Last_Err_LPF;//偏差经过低通后的微分量
  
  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
    if(Controler->Err_LPF>=Controler->Err_Max)   Controler->Err_LPF= Controler->Err_Max;
    if(Controler->Err_LPF<=-Controler->Err_Max)  Controler->Err_LPF=-Controler->Err_Max;
  }
  /*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err_LPF)<=Controler->Integrate_Separation_Err)
      Controler->Integrate+=Controler->Ki*Controler->Err_LPF*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err_LPF*controller_dt;
  }
  /*******积分限幅*********************/
  if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
  {
    if(Controler->Integrate>=Controler->Integrate_Max)
      Controler->Integrate=Controler->Integrate_Max;
    if(Controler->Integrate<=-Controler->Integrate_Max)
      Controler->Integrate=-Controler->Integrate_Max ;
  }
  /*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut=Controler->Kp*Controler->Err_LPF//比例
													 +Controler->Integrate//积分
													 +Controler->Kd*Controler->Dis_Err_LPF;//已对偏差低通，此处不再对微分项单独低通
  /*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
  /*******返回总输出*********************/
  return Controler->Control_OutPut;
}


float PID_Control_SDK_Err_LPF(PID_Controler *Controler,uint8_t Differential_Enable_Flag,float period_second)
{
  float controller_dt=0;
	Get_Systime(&Controler->Systime_t);
	controller_dt=Controler->Systime_t.period/1000.0000f;
	if(controller_dt>1.05f*period_second||controller_dt<0.95f*period_second||isnan(controller_dt)!=0)   controller_dt=period_second;
	if(controller_dt<0.0001f) return 0;
	
  /*******偏差计算*********************/
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差
  
	Controler->Dis_Err=Controler->Err-Controler->Last_Err;//原始微分
  
  Controler->Last_Err_LPF=Controler->Err_LPF;
  Controler->Err_LPF=Control_Device_LPF(Controler->Err,
                                        &Controler->_lpf_buf,
                                        &Control_Device_SDK_Err_LPF_Parameter);//巴特沃斯低通后得到的微分项,20hz
  if(Differential_Enable_Flag==0)
	{
		Controler->_lpf_buf.Input_Butter[0]=0;
		Controler->_lpf_buf.Input_Butter[1]=0;
		Controler->_lpf_buf.Input_Butter[2]=0;
		Controler->_lpf_buf.Output_Butter[0]=0;
		Controler->_lpf_buf.Output_Butter[1]=0;
		Controler->_lpf_buf.Output_Butter[2]=0;
		Controler->Last_Err_LPF=0;
		Controler->Err_LPF=0;
	}  
  Controler->Dis_Err_LPF=Controler->Err_LPF-Controler->Last_Err_LPF;//偏差经过低通后的微分量

	
  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
    if(Controler->Err_LPF>=Controler->Err_Max)   Controler->Err_LPF= Controler->Err_Max;
    if(Controler->Err_LPF<=-Controler->Err_Max)  Controler->Err_LPF=-Controler->Err_Max;
  }
  /*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err_LPF)<=Controler->Integrate_Separation_Err)
      Controler->Integrate+=Controler->Ki*Controler->Err_LPF*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err_LPF*controller_dt;
  }
  /*******积分限幅*********************/
  if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
  {
    if(Controler->Integrate>=Controler->Integrate_Max)
      Controler->Integrate=Controler->Integrate_Max;
    if(Controler->Integrate<=-Controler->Integrate_Max)
      Controler->Integrate=-Controler->Integrate_Max ;
  }
  /*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推

  Controler->Control_OutPut=Controler->Kp*Controler->Err_LPF//比例
														+Controler->Integrate//积分	
															+Controler->Kd*Controler->Dis_Err_LPF;//已对偏差低通，此处不再对微分项单独低通
  /*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
  /*******返回总输出*********************/
  return Controler->Control_OutPut;
}

void PID_LPF_Reset(PID_Controler *Controler,Controler_Label Label)
{
  Controler->_lpf_buf.Input_Butter[0]=0;
  Controler->_lpf_buf.Input_Butter[1]=0;
  Controler->_lpf_buf.Input_Butter[2]=0;
  Controler->_lpf_buf.Output_Butter[0]=0;
  Controler->_lpf_buf.Output_Butter[1]=0;
  Controler->_lpf_buf.Output_Butter[2]=0;
  Controler->Last_Err_LPF=0;
  Controler->Err_LPF=0;
}


float Differential_Forward_PID_Control_Div_LPF(PID_Controler *Controler,float period_second)//微分先行PID控制器
{
  int16  i=0;
  float tempa,tempb,tempc,max,min;//用于防跳变滤波
  float controller_dt=0;
	Get_Systime(&Controler->Systime_t);
	controller_dt=Controler->Systime_t.period/1000.0000f;
	if(controller_dt>1.05f*period_second||controller_dt<0.95f*period_second||isnan(controller_dt)!=0)   controller_dt=period_second;
	if(controller_dt<0.0001f) return 0;
	
  /*******偏差计算*********************/
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差
  Controler->Dis_Err=-(Controler->FeedBack-Controler->Last_FeedBack);//只对反馈信号微分
  Controler->Last_FeedBack=Controler->FeedBack;//记录上次反馈值
  /******************************************/
  //均值滤波，保证得到数据不跳变，避免期望阶跃时，微分输出异常
  tempa=Controler->Pre_Last_Dis_Err_LPF;
  tempb=Controler->Last_Dis_Err_LPF;
  tempc=Controler->Dis_Err;
  max = tempa > tempb ? tempa:tempb;
  max = max > tempc ? max:tempc;
  min = tempa < tempb ? tempa:tempb;
  min = min < tempc ? min:tempc;
  if(tempa > min && tempa < max)    Controler->Dis_Err = tempa;
  if(tempb > min  && tempb < max )  Controler->Dis_Err = tempb;
  if(tempc > min  &&  tempc < max)  Controler->Dis_Err = tempc;
  Controler->Pre_Last_Dis_Err_LPF = Controler->Last_Dis_Err_LPF;
  Controler->Last_Dis_Err_LPF = Controler->Dis_Err;
  /*****************************************/  
  for(i=4;i>0;i--)//数字低通后微分项保存
  {
    Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
  }
  Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                                     &Controler->_lpf_buf,
                                                     &Control_Device_Div_LPF_Parameter);//巴特沃斯低通后得到的微分项,20hz 
  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
    if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
    if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
  /*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
      Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  /*******积分限幅*********************/
  if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
  {
    if(Controler->Integrate>=Controler->Integrate_Max)
      Controler->Integrate=Controler->Integrate_Max;
    if(Controler->Integrate<=-Controler->Integrate_Max)
      Controler->Integrate=-Controler->Integrate_Max ;
  }
  /*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut=Controler->Kp*Controler->Err//比例
    +Controler->Integrate//积分
      //+Controler->Kd*Controler->Dis_Err;//微分
      +Controler->Kd*Controler->Dis_Error_History[0];//微分项来源于巴特沃斯低通滤波器
  /*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
  /*******返回总输出*********************/
  return Controler->Control_OutPut;
}

float PID_Control_Div_LPF_For_Gyro(PID_Controler *Controler,float period_second)
{
  int16  i=0; 
  float controller_dt=0;
	Get_Systime(&Controler->Systime_t);
	controller_dt=Controler->Systime_t.period/1000.0000f;	
	if(controller_dt>1.05f*period_second||controller_dt<0.95f*period_second||isnan(controller_dt)!=0)   controller_dt=period_second;
	if(controller_dt<0.0001f) return 0;
	
  /*******偏差计算*********************/
  Controler->Pre_Last_Err=Controler->Last_Err;//上上次偏差
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect-Controler->FeedBack;//期望减去反馈得到偏差
	Controler->Dis_Err=(Controler->Err-Controler->Last_Err);
	Controler->Dis_Err=constrain_float(Controler->Dis_Err,-25.0f,25.0f);
 
  for(i=4;i>0;i--)//数字低通后微分项保存
  {
    Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
  }
  Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                                     &Controler->_lpf_buf,
                                                     &lpf_param_gyro);//巴特沃斯低通后得到的微分项,30hz
  Controler->Dis_Error_History[0]=constrain_float(Controler->Dis_Error_History[0],-500,500);//微分项限幅
  //Controler->Adaptable_Kd=Controler->Kd*(1+ABS(Controler->Dis_Error_History[0])/500.0f);//自适应微分参数
  Controler->Adaptable_Kd=Controler->Kd;
	
	
  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
    if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
    if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
  /*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
      Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  /*******积分限幅*********************/
  if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
  {
    if(Controler->Integrate>=Controler->Integrate_Max)
      Controler->Integrate=Controler->Integrate_Max;
    if(Controler->Integrate<=-Controler->Integrate_Max)
      Controler->Integrate=-Controler->Integrate_Max ;
  }
  /*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut=Controler->Kp*Controler->Err//比例
    +Controler->Integrate//积分
      //+Controler->Kd*Controler->Dis_Err;//微分
      //+Controler->Kd*Controler->Dis_Error_History[0];//微分项来源于巴特沃斯低通滤波器
      +Controler->Adaptable_Kd*Controler->Dis_Error_History[0];//微分项来源于巴特沃斯低通滤波器
  /*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
  /*******返回总输出*********************/
  return Controler->Control_OutPut;
}


float pid_ctrl_rpy_gyro_maple(PID_Controler *ctrl,float period_second,diff_mode _diff_mode,lpf_mode _lpf_mode)
{
  uint16_t  i=0;
  float _dt=0;
  Get_Systime(&ctrl->Systime_t);
  _dt=ctrl->Systime_t.period/1000.0f;
	
	if(_dt>1.05f*period_second||_dt<0.95f*period_second||isnan(_dt)!=0)   _dt=period_second;
	if(_dt<0.0001f) return 0;
	
  /*******偏差计算*********************/
	ctrl->Pre_Last_Err=ctrl->Last_Err;//上上次偏差
  ctrl->Last_Err=ctrl->Err;//保存上次偏差
  ctrl->Err=ctrl->Expect-ctrl->FeedBack;//期望减去反馈得到偏差
	/*******微分计算*********************/
	ctrl->Expect_Div   = ctrl->Expect     - ctrl->Last_Expect;   //期望的微分
	ctrl->Last_Expect	 = ctrl->Expect;												   //记录上次期望
	ctrl->Feedback_Div  = ctrl->FeedBack   - ctrl->Last_FeedBack;//反馈的微分
	ctrl->Last_FeedBack = ctrl->FeedBack;												 //记录上次反馈值
	ctrl->Combine_Div  = ctrl->Expect_Div - ctrl->Feedback_Div;  //组合微分
	
	
	if(_diff_mode==interval_diff)//区间微分——对期望和反馈单独微分后作差，其中期望微分的周期可根据角度外环给定来
	{
		ctrl->Dis_Err=ctrl->Combine_Div;
	}
	else if(_diff_mode==incomplete_diff)//微分先行——只对反馈信号微分
	{
		ctrl->Dis_Err=-ctrl->Feedback_Div;
	}
  else ctrl->Dis_Err=ctrl->Err-ctrl->Last_Err;//原始微分，也可以用间隔了一次采样的微分
	
	ctrl->Dis_Err=constrain_float(ctrl->Dis_Err,-50.0f,50.0f);//原始微分项限幅	
	
  for(i=4;i>0;i--)//数字低通后微分项保存
  {
    ctrl->Dis_Error_History[i]=ctrl->Dis_Error_History[i-1];
  }
  ctrl->Dis_Error_History[0]=Control_Device_LPF(ctrl->Dis_Err,
																							 &ctrl->_lpf_buf,
																				       &lpf_param_gyro_maple);//巴特沃斯低通后得到的微分项	
	
	//对微分项进行,一阶低通滤波
	ctrl->derivative = ctrl->Dis_Err;
	ctrl->derivative = ctrl->last_derivative + ctrl->d_lpf_alpha * (ctrl->derivative - ctrl->last_derivative);
	ctrl->last_derivative = ctrl->derivative;

	//选择微分信号滤波方式
	switch(_lpf_mode)
	{
		case first_order_lpf://一阶低通
		{
			ctrl->Dis_Err_LPF=ctrl->derivative;
		}
		break;
		case second_order_lpf://二阶低通	
		{
			ctrl->Dis_Err_LPF=ctrl->Dis_Error_History[0];
		}
		break;
		case noneed_lpf://不低通	
		{
			ctrl->Dis_Err_LPF=ctrl->Dis_Err;
		}
		break;     
		default:ctrl->Dis_Err_LPF=ctrl->Dis_Err;
	}

	if(ctrl->Err_Limit_Flag==1)	ctrl->Err=constrain_float(ctrl->Err,-ctrl->Err_Max,ctrl->Err_Max);//偏差限幅度标志位
  /*******积分计算*********************/
  if(ctrl->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(FastAbs(ctrl->Err)<=ctrl->Integrate_Separation_Err) 
			ctrl->Integrate+=ctrl->Ki*ctrl->Err*_dt;
  }
  else	ctrl->Integrate+=ctrl->Ki*ctrl->Err*_dt;
  /*******积分限幅*********************/
  if(ctrl->Integrate_Limit_Flag==1)	
		ctrl->Integrate=constrain_float(ctrl->Integrate,-ctrl->Integrate_Max,ctrl->Integrate_Max);//积分总输出限制幅度标志
	
	/*******总输出计算*********************/
	ctrl->Last_Control_OutPut=ctrl->Control_OutPut;//输出值递推
	ctrl->Control_OutPut=ctrl->Kp*ctrl->Err+ctrl->Integrate;//比例+积分
	ctrl->Control_OutPut+=ctrl->Kd*constrain_float(ctrl->Dis_Err_LPF,-25.0f,25.0f);//叠加微分控制		

  /*******总输出限幅*********************/
	ctrl->Control_OutPut=constrain_float(ctrl->Control_OutPut,-ctrl->Control_OutPut_Limit,ctrl->Control_OutPut_Limit);
  /*******返回总输出*********************/
  return ctrl->Control_OutPut;
}


void  PID_Integrate_Reset(PID_Controler *Controler)  {Controler->Integrate=0.0f;}


void East_North_Ctrl_Reset(void)
{
	PID_Integrate_Reset(&Total_Controller.Latitude_Speed_Control);//清空水平速度控制器积分项
	PID_Integrate_Reset(&Total_Controller.Latitude_Position_Control);//清空水平位置控制器积分项
	PID_Integrate_Reset(&Total_Controller.Longitude_Speed_Control);//清空水平速度控制器积分项
	PID_Integrate_Reset(&Total_Controller.Longitude_Position_Control);//清空水平位置控制器积分项
}



void Take_Off_Reset(void)
{
  PID_Integrate_Reset(&Total_Controller.Roll_Gyro_Control);//起飞前屏蔽积分
  PID_Integrate_Reset(&Total_Controller.Pitch_Gyro_Control);
  PID_Integrate_Reset(&Total_Controller.Yaw_Gyro_Control);
  PID_Integrate_Reset(&Total_Controller.Pitch_Angle_Control);
  PID_Integrate_Reset(&Total_Controller.Roll_Angle_Control);
  PID_Integrate_Reset(&Total_Controller.Yaw_Angle_Control);
  
  PID_Integrate_Reset(&Total_Controller.Longitude_Speed_Control);//位置控制速度环积分项
  PID_Integrate_Reset(&Total_Controller.Latitude_Speed_Control);
}

void Throttle_Control_Reset(void)
{
  PID_Integrate_Reset(&Total_Controller.Height_Acce_Control);
  PID_Integrate_Reset(&Total_Controller.Height_Speed_Control);
  PID_Integrate_Reset(&Total_Controller.Height_Position_Control);
	
	PID_LPF_Reset(&Total_Controller.Height_Acce_Control,Height_Acce_Controler);
  Total_Controller.Height_Acce_Control.Expect=0;//4期望
  Total_Controller.Height_Acce_Control.FeedBack=0;//5反馈值
  Total_Controller.Height_Acce_Control.Err=0;//6偏差
  Total_Controller.Height_Acce_Control.Last_Err=0;//7上次偏差
  Total_Controller.Height_Acce_Control.Integrate=0;//10积分值
  Total_Controller.Height_Acce_Control.Control_OutPut=0;//15控制器总输出
  Total_Controller.Height_Acce_Control.Last_Control_OutPut=0;//16上次控制器总输出
}

#define PID_USE_NUM  14
Vector3f_pid pid_params[PID_USE_NUM]={0};
uint8_t Sort_PID_Flag=0;
/***************************************************
函数名: void Save_Or_Reset_PID_Parameter(void)
说明:	PID参数内部Flash读、写操作
入口:	无
出口:	无
备注:	配合地面站使用
注释者：无名小哥
****************************************************/
void Save_Or_Reset_PID_Parameter()
{
  if(Sort_PID_Flag==1)//将地面站设置PID参数写入Flash
  {
		Sort_PID_Flag=0;
    pid_params[0].p=Total_Controller.Pitch_Gyro_Control.Kp;
    pid_params[0].i=Total_Controller.Pitch_Gyro_Control.Ki;
    pid_params[0].d=Total_Controller.Pitch_Gyro_Control.Kd;
    
    pid_params[1].p=Total_Controller.Roll_Gyro_Control.Kp;
    pid_params[1].i=Total_Controller.Roll_Gyro_Control.Ki;
    pid_params[1].d=Total_Controller.Roll_Gyro_Control.Kd;
    
    pid_params[2].p=Total_Controller.Yaw_Gyro_Control.Kp;
    pid_params[2].i=Total_Controller.Yaw_Gyro_Control.Ki;
    pid_params[2].d=Total_Controller.Yaw_Gyro_Control.Kd;
    
    pid_params[3].p=Total_Controller.Pitch_Angle_Control.Kp;
    pid_params[3].i=Total_Controller.Pitch_Angle_Control.Ki;
    pid_params[3].d=Total_Controller.Pitch_Angle_Control.Kd;
    
    pid_params[4].p=Total_Controller.Roll_Angle_Control.Kp;
    pid_params[4].i=Total_Controller.Roll_Angle_Control.Ki;
    pid_params[4].d=Total_Controller.Roll_Angle_Control.Kd;
    
    pid_params[5].p=Total_Controller.Yaw_Angle_Control.Kp;
    pid_params[5].i=Total_Controller.Yaw_Angle_Control.Ki;
    pid_params[5].d=Total_Controller.Yaw_Angle_Control.Kd;
    
    pid_params[6].p=Total_Controller.Height_Position_Control.Kp;
    pid_params[6].i=Total_Controller.Height_Position_Control.Ki;
    pid_params[6].d=Total_Controller.Height_Position_Control.Kd;
    
    pid_params[7].p=Total_Controller.Height_Speed_Control.Kp;
    pid_params[7].i=Total_Controller.Height_Speed_Control.Ki;
    pid_params[7].d=Total_Controller.Height_Speed_Control.Kd;
    
		pid_params[8].p=Total_Controller.Height_Acce_Control.Kp;
    pid_params[8].i=Total_Controller.Height_Acce_Control.Ki;
    pid_params[8].d=Total_Controller.Height_Acce_Control.Kd;
    
    pid_params[9].p=Total_Controller.Latitude_Speed_Control.Kp;
    pid_params[9].i=Total_Controller.Latitude_Speed_Control.Ki;
    pid_params[9].d=Total_Controller.Latitude_Speed_Control.Kd;
    
    pid_params[10].p=Total_Controller.Latitude_Position_Control.Kp;
    pid_params[10].i=Total_Controller.Latitude_Position_Control.Ki;
    pid_params[10].d=Total_Controller.Latitude_Position_Control.Kd;
       
    pid_params[11].p=Total_Controller.Optical_Position_Control.Kp;
    pid_params[11].i=Total_Controller.Optical_Position_Control.Ki;
    pid_params[11].d=Total_Controller.Optical_Position_Control.Kd;
    
    pid_params[12].p=Total_Controller.Optical_Speed_Control.Kp;
    pid_params[12].i=Total_Controller.Optical_Speed_Control.Ki;
    pid_params[12].d=Total_Controller.Optical_Speed_Control.Kd;
    
    pid_params[13].p=Total_Controller.SDK_Roll_Position_Control.Kp;
    pid_params[13].i=Total_Controller.SDK_Roll_Position_Control.Ki;
    pid_params[13].d=Total_Controller.SDK_Roll_Position_Control.Kd;
    Save_PID_Parameter();
  }
  else if(Sort_PID_Flag==2)//将复位PID参数，并写入Flash
  {
		Sort_PID_Flag=0;
    Total_PID_Init();//将PID参数重置为参数Control_Unit表里面参数
    pid_params[0].p=Total_Controller.Pitch_Gyro_Control.Kp;
    pid_params[0].i=Total_Controller.Pitch_Gyro_Control.Ki;
    pid_params[0].d=Total_Controller.Pitch_Gyro_Control.Kd;
    
    pid_params[1].p=Total_Controller.Roll_Gyro_Control.Kp;
    pid_params[1].i=Total_Controller.Roll_Gyro_Control.Ki;
    pid_params[1].d=Total_Controller.Roll_Gyro_Control.Kd;
    
    pid_params[2].p=Total_Controller.Yaw_Gyro_Control.Kp;
    pid_params[2].i=Total_Controller.Yaw_Gyro_Control.Ki;
    pid_params[2].d=Total_Controller.Yaw_Gyro_Control.Kd;
    
    pid_params[3].p=Total_Controller.Pitch_Angle_Control.Kp;
    pid_params[3].i=Total_Controller.Pitch_Angle_Control.Ki;
    pid_params[3].d=Total_Controller.Pitch_Angle_Control.Kd;
    
    pid_params[4].p=Total_Controller.Roll_Angle_Control.Kp;
    pid_params[4].i=Total_Controller.Roll_Angle_Control.Ki;
    pid_params[4].d=Total_Controller.Roll_Angle_Control.Kd;
    
    pid_params[5].p=Total_Controller.Yaw_Angle_Control.Kp;
    pid_params[5].i=Total_Controller.Yaw_Angle_Control.Ki;
    pid_params[5].d=Total_Controller.Yaw_Angle_Control.Kd;
    
    pid_params[6].p=Total_Controller.Height_Position_Control.Kp;
    pid_params[6].i=Total_Controller.Height_Position_Control.Ki;
    pid_params[6].d=Total_Controller.Height_Position_Control.Kd;
    
    pid_params[7].p=Total_Controller.Height_Speed_Control.Kp;
    pid_params[7].i=Total_Controller.Height_Speed_Control.Ki;
    pid_params[7].d=Total_Controller.Height_Speed_Control.Kd;
   
	  pid_params[8].p=Total_Controller.Height_Acce_Control.Kp;
    pid_params[8].i=Total_Controller.Height_Acce_Control.Ki;
    pid_params[8].d=Total_Controller.Height_Acce_Control.Kd;  
		
    pid_params[9].p=Total_Controller.Latitude_Speed_Control.Kp;
    pid_params[9].i=Total_Controller.Latitude_Speed_Control.Ki;
    pid_params[9].d=Total_Controller.Latitude_Speed_Control.Kd;
    
    pid_params[10].p=Total_Controller.Latitude_Position_Control.Kp;
    pid_params[10].i=Total_Controller.Latitude_Position_Control.Ki;
    pid_params[10].d=Total_Controller.Latitude_Position_Control.Kd;
   
    pid_params[11].p=Total_Controller.Optical_Position_Control.Kp;
    pid_params[11].i=Total_Controller.Optical_Position_Control.Ki;
    pid_params[11].d=Total_Controller.Optical_Position_Control.Kd;
    
    pid_params[12].p=Total_Controller.Optical_Speed_Control.Kp;
    pid_params[12].i=Total_Controller.Optical_Speed_Control.Ki;
    pid_params[12].d=Total_Controller.Optical_Speed_Control.Kd;
    
    pid_params[13].p=Total_Controller.SDK_Roll_Position_Control.Kp;
    pid_params[13].i=Total_Controller.SDK_Roll_Position_Control.Ki;
    pid_params[13].d=Total_Controller.SDK_Roll_Position_Control.Kd;
    Save_PID_Parameter();
    
    NCLink_Send_Ask_Flag[0]=1;//恢复默认参数后，将更新的数据发送置地面站
    NCLink_Send_Ask_Flag[1]=1;
    NCLink_Send_Ask_Flag[2]=1;
    NCLink_Send_Ask_Flag[3]=1;
    NCLink_Send_Ask_Flag[4]=1;
    NCLink_Send_Ask_Flag[5]=1;
  }
  else if(Sort_PID_Flag==3)//将复位PID参数，并写入Flash
  {
    Sort_PID_Flag=0;
    Total_PID_Init();//将PID参数重置为参数Control_Unit表里面参数
    pid_params[0].p=Total_Controller.Pitch_Gyro_Control.Kp;
    pid_params[0].i=Total_Controller.Pitch_Gyro_Control.Ki;
    pid_params[0].d=Total_Controller.Pitch_Gyro_Control.Kd;
    
    pid_params[1].p=Total_Controller.Roll_Gyro_Control.Kp;
    pid_params[1].i=Total_Controller.Roll_Gyro_Control.Ki;
    pid_params[1].d=Total_Controller.Roll_Gyro_Control.Kd;
    
    pid_params[2].p=Total_Controller.Yaw_Gyro_Control.Kp;
    pid_params[2].i=Total_Controller.Yaw_Gyro_Control.Ki;
    pid_params[2].d=Total_Controller.Yaw_Gyro_Control.Kd;
    
    pid_params[3].p=Total_Controller.Pitch_Angle_Control.Kp;
    pid_params[3].i=Total_Controller.Pitch_Angle_Control.Ki;
    pid_params[3].d=Total_Controller.Pitch_Angle_Control.Kd;
    
    pid_params[4].p=Total_Controller.Roll_Angle_Control.Kp;
    pid_params[4].i=Total_Controller.Roll_Angle_Control.Ki;
    pid_params[4].d=Total_Controller.Roll_Angle_Control.Kd;
    
    pid_params[5].p=Total_Controller.Yaw_Angle_Control.Kp;
    pid_params[5].i=Total_Controller.Yaw_Angle_Control.Ki;
    pid_params[5].d=Total_Controller.Yaw_Angle_Control.Kd;
    
    pid_params[6].p=Total_Controller.Height_Position_Control.Kp;
    pid_params[6].i=Total_Controller.Height_Position_Control.Ki;
    pid_params[6].d=Total_Controller.Height_Position_Control.Kd;
    
    pid_params[7].p=Total_Controller.Height_Speed_Control.Kp;
    pid_params[7].i=Total_Controller.Height_Speed_Control.Ki;
    pid_params[7].d=Total_Controller.Height_Speed_Control.Kd;
    
		pid_params[8].p=Total_Controller.Height_Acce_Control.Kp;
    pid_params[8].i=Total_Controller.Height_Acce_Control.Ki;
    pid_params[8].d=Total_Controller.Height_Acce_Control.Kd;    
    
		pid_params[9].p=Total_Controller.Latitude_Speed_Control.Kp;
    pid_params[9].i=Total_Controller.Latitude_Speed_Control.Ki;
    pid_params[9].d=Total_Controller.Latitude_Speed_Control.Kd;
    
    pid_params[10].p=Total_Controller.Latitude_Position_Control.Kp;
    pid_params[10].i=Total_Controller.Latitude_Position_Control.Ki;
    pid_params[10].d=Total_Controller.Latitude_Position_Control.Kd;
        
    pid_params[11].p=Total_Controller.Optical_Position_Control.Kp;
    pid_params[11].i=Total_Controller.Optical_Position_Control.Ki;
    pid_params[11].d=Total_Controller.Optical_Position_Control.Kd;
    
    pid_params[12].p=Total_Controller.Optical_Speed_Control.Kp;
    pid_params[12].i=Total_Controller.Optical_Speed_Control.Ki;
    pid_params[12].d=Total_Controller.Optical_Speed_Control.Kd;
    
    pid_params[13].p=Total_Controller.SDK_Roll_Position_Control.Kp;
    pid_params[13].i=Total_Controller.SDK_Roll_Position_Control.Ki;
    pid_params[13].d=Total_Controller.SDK_Roll_Position_Control.Kd;
	
    Save_PID_Parameter();
		
    NCLink_Send_Ask_Flag[0]=1;//恢复默认参数后，将更新的数据发送置地面站
    NCLink_Send_Ask_Flag[1]=1;
    NCLink_Send_Ask_Flag[2]=1;
    NCLink_Send_Ask_Flag[3]=1;
    NCLink_Send_Ask_Flag[4]=1;
    NCLink_Send_Ask_Flag[5]=1;
  }
}



uint8_t pid_params_valid[PID_USE_NUM];
void PID_Paramter_Init_With_Flash()
{
  for(uint8_t i=0;i<PID_USE_NUM;i++)
  {
    ReadFlashParameterThree(PID1_PARAMETER_KP+3*i,&pid_params[i].p,&pid_params[i].i,&pid_params[i].d);
    if(isnan(pid_params[i].p)==0&&isnan(pid_params[i].i)==0&&isnan(pid_params[i].d)==0)
      pid_params_valid[i]=0x01;//eeprom数据有效
  }
	
  if(pid_params_valid[0]!=0x00 &&pid_params_valid[1] !=0x00 &&pid_params_valid[2] !=0x00
   &&pid_params_valid[3]!=0x00 &&pid_params_valid[4] !=0x00 &&pid_params_valid[5] !=0x00
   &&pid_params_valid[6]!=0x00 &&pid_params_valid[7] !=0x00 &&pid_params_valid[8] !=0x00
   &&pid_params_valid[9]!=0x00 &&pid_params_valid[10]!=0x00 &&pid_params_valid[11]!=0x00
	 &&pid_params_valid[12]!=0x00&&pid_params_valid[13]!=0x00)//Flash内数据正常，更新PID参数值
  {
		Total_PID_Init();
    Total_Controller.Pitch_Gyro_Control.Kp=pid_params[0].p;
    Total_Controller.Pitch_Gyro_Control.Ki=pid_params[0].i;
    Total_Controller.Pitch_Gyro_Control.Kd=pid_params[0].d;
    
    Total_Controller.Roll_Gyro_Control.Kp=pid_params[1].p;
    Total_Controller.Roll_Gyro_Control.Ki=pid_params[1].i;
    Total_Controller.Roll_Gyro_Control.Kd=pid_params[1].d;
    
    Total_Controller.Yaw_Gyro_Control.Kp=pid_params[2].p;
    Total_Controller.Yaw_Gyro_Control.Ki=pid_params[2].i;
    Total_Controller.Yaw_Gyro_Control.Kd=pid_params[2].d;
    
    Total_Controller.Pitch_Angle_Control.Kp=pid_params[3].p;
    Total_Controller.Pitch_Angle_Control.Ki=pid_params[3].i;
    Total_Controller.Pitch_Angle_Control.Kd=pid_params[3].d;
    
    Total_Controller.Roll_Angle_Control.Kp=pid_params[4].p;
    Total_Controller.Roll_Angle_Control.Ki=pid_params[4].i;
    Total_Controller.Roll_Angle_Control.Kd=pid_params[4].d;
    
    Total_Controller.Yaw_Angle_Control.Kp=pid_params[5].p;
    Total_Controller.Yaw_Angle_Control.Ki=pid_params[5].i;
    Total_Controller.Yaw_Angle_Control.Kd=pid_params[5].d;
    
    Total_Controller.Height_Position_Control.Kp=pid_params[6].p;
    Total_Controller.Height_Position_Control.Ki=pid_params[6].i;
    Total_Controller.Height_Position_Control.Kd=pid_params[6].d;
    
    Total_Controller.Height_Speed_Control.Kp=pid_params[7].p;
    Total_Controller.Height_Speed_Control.Ki=pid_params[7].i;
    Total_Controller.Height_Speed_Control.Kd=pid_params[7].d;
				
    Total_Controller.Height_Acce_Control.Kp=pid_params[8].p;
    Total_Controller.Height_Acce_Control.Ki=pid_params[8].i;
    Total_Controller.Height_Acce_Control.Kd=pid_params[8].d;		
    
    Total_Controller.Latitude_Speed_Control.Kp=pid_params[9].p;
    Total_Controller.Latitude_Speed_Control.Ki=pid_params[9].i;
    Total_Controller.Latitude_Speed_Control.Kd=pid_params[9].d;
    
    Total_Controller.Latitude_Position_Control.Kp=pid_params[10].p;
    Total_Controller.Latitude_Position_Control.Ki=pid_params[10].i;
    Total_Controller.Latitude_Position_Control.Kd=pid_params[10].d;
    /***********************位置控制：位置、速度参数共用一组PID参数**********************************************************/
    Total_Controller.Longitude_Speed_Control.Kp=pid_params[9].p;
    Total_Controller.Longitude_Speed_Control.Ki=pid_params[9].i;
    Total_Controller.Longitude_Speed_Control.Kd=pid_params[9].d;
    
    Total_Controller.Longitude_Position_Control.Kp=pid_params[10].p;
    Total_Controller.Longitude_Position_Control.Ki=pid_params[10].i;
    Total_Controller.Longitude_Position_Control.Kd=pid_params[10].d;
			   
    Total_Controller.Optical_Position_Control.Kp=pid_params[11].p;
    Total_Controller.Optical_Position_Control.Ki=pid_params[11].i;
    Total_Controller.Optical_Position_Control.Kd=pid_params[11].d;
    
    Total_Controller.Optical_Speed_Control.Kp=pid_params[12].p;
    Total_Controller.Optical_Speed_Control.Ki=pid_params[12].i;
    Total_Controller.Optical_Speed_Control.Kd=pid_params[12].d;
    
    Total_Controller.SDK_Roll_Position_Control.Kp=pid_params[13].p;
    Total_Controller.SDK_Roll_Position_Control.Ki=pid_params[13].i;
    Total_Controller.SDK_Roll_Position_Control.Kd=pid_params[13].d;
    
    Total_Controller.SDK_Pitch_Position_Control.Kp=pid_params[13].p;
    Total_Controller.SDK_Pitch_Position_Control.Ki=pid_params[13].i;
    Total_Controller.SDK_Pitch_Position_Control.Kd=pid_params[13].d;
  }
	else//首次下载程序后运行时，会将默认pid参数写入eeprom中
	{
		Sort_PID_Flag=3;
	}
}

void Save_PID_Parameter(void)
{
  for(uint8_t i=0;i<PID_USE_NUM;i++)
  {
    WriteFlashParameter_Three(PID1_PARAMETER_KP+i*3,pid_params[i].p,pid_params[i].i,pid_params[i].d);
  }
}







//	/******************************************/
//  //均值滤波，保证得到数据不跳变，避免期望阶跃时，微分输出异常
//	float tempa,tempb,tempc,max,min;//用于防跳变滤波  
//	tempa=Controler->Pre_Last_Dis_Err_LPF;
//  tempb=Controler->Last_Dis_Err_LPF;
//  tempc=Controler->Dis_Err;
//  max = tempa > tempb ? tempa:tempb;
//  max = max > tempc ? max:tempc;
//  min = tempa < tempb ? tempa:tempb;
//  min = min < tempc ? min:tempc;
//  if(tempa > min && tempa < max)    Controler->Dis_Err = tempa;
//  if(tempb > min  && tempb < max )  Controler->Dis_Err = tempb;
//  if(tempc > min  &&  tempc < max)  Controler->Dis_Err = tempc;
//  Controler->Pre_Last_Dis_Err_LPF = Controler->Last_Dis_Err_LPF;
//  Controler->Last_Dis_Err_LPF = Controler->Dis_Err;
//  /*****************************************/








