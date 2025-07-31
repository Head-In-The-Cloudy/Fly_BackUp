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
#include "Calibration.h"
#include "CalibrationRoutines.h"


/**********************************************************************************************
椭球方程:[beta3(x-beta0)]^2+[beta4(y-beta1)]^2+[beta4(z-beta2)]^2=1
误差函数f(x)=[beta3(x-beta0)]^2+[beta4(y-beta1)]^2+[beta4(z-beta2)]^2-1
待求参数:D=[beta0,beta1,beta2,beta3,beta4,beta5]
记D的增量为delta即detla_D，可以理解成每次迭代的步长
求雅可比矩阵J(x)=[j[0],j[1],j[2],j[3],j[4],j[5]]'
j[0]=-2beta3*beta3*(Xk-beta0)
j[1]=-2beta4*beta4*(Yk-beta1)
j[2]=-2beta5*beta5*(Zk-beta2)
j[3]= 2beta3*(Xk-beta0)*(Xk-beta0)
j[4]= 2beta4*(Yk-beta1)*(Yk-beta1)
j[5]= 2beta5*(Zk-beta2)*(Zk-beta2)
根据最小二乘中高斯牛顿法求解线性问题
记海森矩阵H(x)=J*J';g(x)=-J*f(x)，求取最小二乘优化问题：H*D=g
步骤如下：
1、给定初值D(0)=[0,0,0,1,1,1]'
2、根据k次的输入值(Xi,Yi,Zi),求取当前的雅可比矩阵j(k)和误差f(k)
3、求解增量方程:H*D=g,计算得到当前的delta(k)
4、若delta(k)足够小，则停止迭代
5、否则令D(k+1)=D(k)+delta(k)，继续执行第2步迭代

//特别注意
1、注意在APM里面代码实现中误差函数f_apm(x)=-f(x)=1-[beta3(x-beta0)]^2+[beta4(y-beta1)]^2+[beta4(z-beta2)]^2
2、故在计算雅可比矩阵J时J_APM(x)=-J(x)
3、计算H(x)时，H_APM=J_APM(x)*J_APM(x)'=[-J(x)]*[-J(x)]'=J(x)*J(x)',故二者的H矩阵是一样的
4、APM代码在计算g时，g_APM=J_apm(x)*f_apm(x),根据高斯牛顿迭代处理线性问题解法为g_APM=-J_APM(x)*f_APM(x)
5、APM代码实现上，计算残差矩阵时无负号，故第5步迭代时取D(k+1)=D(k)-delta(k)

相关参考链接如下：
https://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
https://chionophilous.wordpress.com/2012/09/08/implementing-the-gauss-newton-algorithm-for-sphere-fitting-2-of-3/
https://chionophilous.wordpress.com/2011/08/26/accelerometer-calibration-iii-improving-accuracy-with-least-squares-and-the-gauss-newton-method/
https://en.wikipedia.org/wiki/Gauss%E2%80%93Newton_algorithm
https://github.com/rolfeschmidt/muCSense/blob/master/BestSphereGaussNewtonCalibrator.cpp
https://zhuanlan.zhihu.com/p/113946848
https://www.cnblogs.com/guanglun/p/12313193.html
https://blog.csdn.net/qq_31073871/article/details/87008583
https://zhuanlan.zhihu.com/p/634190807?utm_id=0
https://www.zhihu.com/tardis/bd/art/555298443
https://blog.csdn.net/jiangxing11/article/details/127251071
**********************************************************************************************/


/***************加速度计6面矫正，参考APM代码，配合遥控器进行现场矫正**************************/
/***********************************************************
@函数名：Calibrate_Reset_Matrices
@入口参数：float dS[6], float JS[6][6]
@出口参数：无
@功能描述：矩阵数据复位
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
static void Calibrate_Reset_Matrices(float dS[6], float JS[6][6])
{
  int16_t j,k;
  for( j=0; j<6; j++ )
  {
    dS[j] = 0.0f;
    for( k=0; k<6; k++ )
    {
      JS[j][k] = 0.0f;
    }
  }
}


/***********************************************************
@函数名：gaussElimination_six_order
@入口参数：float mat_Y[6], float mat_A[6][6], float x[6]
@出口参数：无
@功能描述：求解矩阵方程JS*x = dS，第一步把矩阵化上三角阵，
将JS所在的列下方的全部消为0，然后回代得到线性方程的解
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
#define N 6
static void gaussElimination_six_order(float mat_Y[N],float mat_A[N][N],float x[N]) 
{
	double mat[N][N + 1];
	for(uint16_t i=0;i<N;i++)
	{
		for(uint16_t j=0;j<N+1;j++)	
		{
			if(j!=N)  mat[i][j]=mat_A[i][j];
		  else mat[i][j]=mat_Y[i];
		}
	}
	
	for (int i = 0; i < N; i++) {
			// Making the diagonal element non-zero
			if (mat[i][i] == 0) {
					for (int k = i + 1; k < N; k++) {
							if (mat[k][i] != 0) {
									for (int j = 0; j <= N; j++) {
											double temp = mat[i][j];
											mat[i][j] = mat[k][j];
											mat[k][j] = temp;
									}
									break;
							}
					}
			}

			// Making the elements below the diagonal zero
			for (int k = i + 1; k < N; k++) {
					double factor = mat[k][i] / mat[i][i];
					for (int j = 0; j <= N; j++) {
							mat[k][j] -= factor * mat[i][j];
					}
			}
	}
	// mat是高斯消元后的矩阵
	for (int i = N - 1; i >= 0; i--) {
			x[i] = mat[i][N];
			for (int j = i + 1; j < N; j++) {
					x[i] -= mat[i][j] * x[j];
			}
			x[i] /= mat[i][i];
	}		
}

static void Calibrate_Update_Matrices(float dS[6],
																			float JS[6][6],
																			float beta[6],
																			float data[3])
{
  int16_t j, k;
  float dx, b;
  float residual = 1.0;
  float jacobian[6];
	//根据当前输入,计算的本次的雅可比矩阵J(x),即误差函数f(x)对待求状态量beta求偏导
  for(j=0;j<3;j++)
  {
    b = beta[3+j];
    dx = (float)data[j] - beta[j];
		//计算残差f(x)=[beta3(x-beta0)]^2+[beta4(y-beta1)]^2+[beta4(z-beta2)]^2-1这里相当于是计算-f(x)
    residual -= b*b*dx*dx;
    jacobian[j] = 2.0f*b*b*dx;
    jacobian[3+j] = -2.0f*b*dx*dx;
  }
  
  for(j=0;j<6;j++)
  {
		//雅可比矩阵和残差的乘积即g(x)
    dS[j]+=jacobian[j]*residual;
		//雅可比矩阵和雅可比矩阵转置乘积即矩阵H(x)
    for(k=0;k<6;k++)
    {
      JS[j][k]+=jacobian[j]*jacobian[k];
    }
  }
}



uint8 Calibrate_accel(Acce_Unit accel_sample[6],
                      Acce_Unit *accel_offsets,
                      Acce_Unit *accel_scale)
{
  int16_t i;
  int16_t num_iterations = 0;
  float eps = 0.000000001;
  float change = 100.0;
  float data[3]={0};
  float beta[6]={0};
  float delta[6]={0};
  float ds[6]={0};
  float JS[6][6]={0};
  bool success = TRUE;
  //第一步给定迭代初始值D(0)
  beta[0] = beta[1] = beta[2] = 0;
  beta[3] = beta[4] = beta[5] = 1.0f/GRAVITY_MSS;
  while( num_iterations < 20 && change > eps )//步长和迭代次数限制 
	{
    num_iterations++;
		//复位g=ds和H=JS
    Calibrate_Reset_Matrices(ds, JS);
		//第二步根据k次的输入值(Xi,Yi,Zi),求取当前的雅可比矩阵j(k)和误差f(k)
    for( i=0; i<6; i++ ) {
      data[0] = accel_sample[i].x;
      data[1] = accel_sample[i].y;
      data[2] = accel_sample[i].z;
      Calibrate_Update_Matrices(ds, JS, beta, data);
    }
		//第三步求解增量方程:H*D=g,计算得到当前的delta(k)
    gaussElimination_six_order(ds, JS, delta);
		//第四步若delta(k)足够小，则停止迭代，此处用的是delta的平方作为步长判据
    change =delta[0]*delta[0] +
						delta[1]*delta[1] +
						delta[2]*delta[2] +
						delta[3]*delta[3] / (beta[3]*beta[3]) +
						delta[4]*delta[4] / (beta[4]*beta[4]) +
						delta[5]*delta[5] / (beta[5]*beta[5]);
		//第五步令D(k+1)=D(k)+delta(k)，继续执行第2步迭代
    for( i=0; i<6; i++ ) {
      beta[i] -= delta[i];
    }
  }
  // copy results out
  accel_scale->x = beta[3] * GRAVITY_MSS;
  accel_scale->y = beta[4] * GRAVITY_MSS;
  accel_scale->z = beta[5] * GRAVITY_MSS;
  accel_offsets->x = beta[0] * accel_scale->x;
  accel_offsets->y = beta[1] * accel_scale->y;
  accel_offsets->z = beta[2] * accel_scale->z;
  
  // sanity check scale
  if(fabsf(accel_scale->x-1.0f) > 0.2f
     || fabsf(accel_scale->y-1.0f) > 0.2f
       || fabsf(accel_scale->z-1.0f) > 0.2f )
  {
    success = FALSE;
  }
  // sanity check offsets (3.0 is roughly 3/10th of a G, 5.0 is roughly half a G)
  if(fabsf(accel_offsets->x) > 5.0f
     || fabsf(accel_offsets->y) > 5.0f
       || fabsf(accel_offsets->z) > 5.0f )
  {
    success = FALSE;
  }
  // return success or failure
  return success;
}



Acce_Unit new_offset={
  0,0,0,
};
Acce_Unit new_scales={
  1.0,1.0,1.0,
};

Acce_Unit Accel_Offset_Read={
  0,0,0,
};
Acce_Unit Accel_Scale_Read={
  0,0,0,
};


Acce_Unit Accel_Hor_Read={
  0,0,0,
};
uint8_t cal_finished_flag=0;



uint8_t flight_direction=6;
uint8_t Accel_Calibration_Flag=0;//加速度计校准模式
uint8_t Accel_Calibration_Finished[6]={0,0,0,0,0,0};//对应面校准完成标志位
uint8_t Accel_Calibration_All_Finished=0;//6面校准全部校准完成标志位
uint16_t Accel_Calibration_Makesure_Cnt=0;
uint16_t Accel_flight_direction_cnt=0;
/***********************************************************
@函数名：Accel_Calibration_Check
@入口参数：无
@出口参数：无
@功能描述：加速度计标定函数遥控器动作位检测
数据
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void Accel_Calibration_Check(void)
{
  uint16_t  i=0;
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control>=Pit_Rol_Max*Scale_Pecent_Max)
  {
    Accel_Calibration_Makesure_Cnt++;
  }

	if(((Throttle_Control<=(1000+Buttom_Safe_Deadband)
	 &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
		 &&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max
			 &&Pitch_Control>=Pit_Rol_Max*Scale_Pecent_Max
				 &&Accel_Calibration_Makesure_Cnt>=200*3)||cal_flag==0x02)//持续三秒
					 &&Controler_State==Lock_Controler)//必须为上锁状态才可以进入校准模式
		
  {
    Bling_Mode=1;
    Accel_Calibration_Flag=1;//加速度校准模式
    cal_finished_flag=0;	
		Bling_Set(&rgb_red,2000,1000,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_1,1);

    flight_direction=6;
    Accel_Calibration_All_Finished=0;//全部校准完成标志位清零
    Accel_Calibration_Makesure_Cnt=0;
    for(i=0;i<6;i++)
    {
      Accel_Calibration_Finished[i]=0;//对应面标志位清零
      acce_sample[i].x=0; //清空对应面的加速度计量
      acce_sample[i].y=0; //清空对应面的加速度计量
      acce_sample[i].z=0; //清空对应面的加速度计量
    }
    Page_Number=10;//OLED加速度计矫正页面
    Reset_Mag_Calibartion(1);
    Reset_RC_Calibartion(1);
    Forced_Lock_Makesure_Cnt=0;
		cal_flag=0x00;
  }
  
  if(Accel_Calibration_Flag==1)
  {
    if((Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control<=-Yaw_Max*Scale_Pecent_Max&&Roll_Control==0&&Pitch_Control==0)
			||cal_step==0x01)
    {
      Accel_flight_direction_cnt++;
      if((Accel_flight_direction_cnt>=20)//100ms
				||cal_step==0x01)
      {
        flight_direction=0;
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
      cal_step=0x00;
    }
    else if((Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control==0&&Roll_Control>=Pit_Rol_Max*Scale_Pecent_Max&&Pitch_Control==0)
			||cal_step==0x02)
    {
      Accel_flight_direction_cnt++;
      if((Accel_flight_direction_cnt>=20)//100ms
				||cal_step==0x02)
      {
        flight_direction=1;
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
			cal_step=0x00;
    }
    else if((Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control==0&&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max&&Pitch_Control==0)
			||cal_step==0x03)
    {
      Accel_flight_direction_cnt++;
      if((Accel_flight_direction_cnt>=20)//100ms
				||cal_step==0x03)
      {
        flight_direction=2;
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
			cal_step=0x00;
    }
    else if((Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control==0&&Roll_Control==0&&Pitch_Control>=Pit_Rol_Max*Scale_Pecent_Max)
			||cal_step==0x04)
    {
      Accel_flight_direction_cnt++;
      if((Accel_flight_direction_cnt>=20)//100ms
				||cal_step==0x04)
      {
        flight_direction=3;
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
			cal_step=0x00;
    }
    else if((Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control==0&&Roll_Control==0&&Pitch_Control<=-Pit_Rol_Max*Scale_Pecent_Max)
			||cal_step==0x05)
    {
      Accel_flight_direction_cnt++;
      if((Accel_flight_direction_cnt>=20)//100ms
				||cal_step==0x05)
      {
        flight_direction=4;
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
			cal_step=0x00;
    }
    else if((Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control>Yaw_Max*Scale_Pecent_Max&&Roll_Control==0&&Pitch_Control==0)
			||cal_step==0x06)
    {
      Accel_flight_direction_cnt++;
      if((Accel_flight_direction_cnt>=20)//100ms
				||cal_step==0x06)
      {
        flight_direction=5;
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
			cal_step=0x00;
    }
    else
    {
      Accel_flight_direction_cnt/=2;
    }
    
    if(Accel_flight_direction_cnt>=200)  Accel_flight_direction_cnt=0;
    
  }
  
}

Acce_Unit acce_sample[6]={0};//三行6列，保存6面待矫正数据
uint8_t Flash_Buf[12]={0};
/***********************************************************
@函数名：Accel_Calibartion
@入口参数：无
@出口参数：无
@功能描述：加速度标定、利用遥控器直接进入
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
uint8_t Accel_Calibartion(void)
{
  uint16 i,j=0;
  float acce_sample_sum[3]={0,0,0};//加速度和数据
  /*第一面飞控平放，Z轴正向朝着正上方，Z axis is about 1g,X、Y is about 0g*/
  /*第二面飞控平放，X轴正向朝着正上方，X axis is about 1g,Y、Z is about 0g*/
  /*第三面飞控平放，X轴正向朝着正下方，X axis is about -1g,Y、Z is about 0g*/
  /*第四面飞控平放，Y轴正向朝着正下方，Y axis is about -1g,X、Z is about 0g*/
  /*第五面飞控平放，Y轴正向朝着正上方，Y axis is about 1g,X、Z is about 0g*/
  /*第六面飞控平放，Z轴正向朝着正下方，Z axis is about -1g,X、Y is about 0g*/
  if(flight_direction<=5)//检测到对应面数据
  {
    uint16_t num_samples=0;
    while(num_samples<1000)//采样200次
    {
      if(Gyro_Length<=20.0f
         &&WP_Sensor.imu_updtate_flag==1)//通过陀螺仪模长来确保机体静止
      {
        for(j=0;j<3;j++){
          acce_sample_sum[j]+=WP_Sensor.acce_filter[j]*RAW_TO_G;//加速度计转化为1g量程下
        }
        //delay_ms(4);//间隔10ms，1s内数据取平均
        num_samples++;
        WP_Sensor.imu_updtate_flag=0;
      }
      Accel_Calibration_Finished[flight_direction]=1;//对应面校准完成标志位置1
    }
    acce_sample[flight_direction].x=acce_sample_sum[0]/num_samples; //保存对应面的加速度计量
    acce_sample[flight_direction].y=acce_sample_sum[1]/num_samples; //保存对应面的加速度计量
    acce_sample[flight_direction].z=acce_sample_sum[2]/num_samples; //保存对应面的加速度计量
    flight_direction=6;//单面矫正完毕
  }
  
  if((Accel_Calibration_Finished[0]
      &Accel_Calibration_Finished[1]
        &Accel_Calibration_Finished[2]
          &Accel_Calibration_Finished[3]
            &Accel_Calibration_Finished[4]
              &Accel_Calibration_Finished[5])
     &&Accel_Calibration_All_Finished==0)//6面全部校准完毕
  {
    Accel_Calibration_All_Finished=1;//加速度计6面校准完成标志
    Accel_Calibration_Flag=0;//加速度计校准结束，释放遥感操作
    cal_finished_flag=Calibrate_accel(acce_sample,
                             &new_offset,
                             &new_scales);//将所得6面数据
    for(i=0;i<6;i++)
    {
      Accel_Calibration_Finished[i]=0;//对应面标志位清零
    }
    if(cal_finished_flag==TRUE)//加速度计校准成功
    {
      WriteFlashParameter_Three(ACCEL_X_OFFSET1,
                                new_offset.x,
                                new_offset.y,
                                new_offset.z);
      WriteFlashParameter_Three(ACCEL_X_SCALE1,
                                new_scales.x,
                                new_scales.y,
                                new_scales.z);
      
      Parameter_Init();//读取写入参数
      Bling_Mode=0;//恢复正常指示模式
			Bling_Set(&rgb_red,5000,2000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_1,0);
			NCLink_Send_Check_Flag[9]=0x01;//校准完毕后发送校准数据
    }
    else//加速度计校准失败
    {
      Bling_Mode=0;//恢复正常指示模式
			Bling_Set(&rgb_red,5000,2000,0.1,0,GPIO_PORTF_BASE,GPIO_PIN_1,0);
      Page_Number=0;//OLED恢复首页
    }
    return TRUE;
  }
  return FALSE;
}


/***********************************************************
@函数名：Reset_Accel_Calibartion
@入口参数：uint8_t Type
@出口参数：无
@功能描述：加速度标定清空数据与强制复位
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void Reset_Accel_Calibartion(uint8_t Type)
{
  uint16 i=0;
  for(i=0;i<6;i++)
  {
    Accel_Calibration_Finished[i]=0;//对应面标志位清零
    acce_sample[i].x=0; //清空对应面的加速度计量
    acce_sample[i].y=0; //清空对应面的加速度计量
    acce_sample[i].z=0; //清空对应面的加速度计量
  }
  Accel_Calibration_All_Finished=0;//全部校准完成标志位清零
  if(Type==1)  Accel_Calibration_Flag=0;
}


typedef struct
{
  uint8_t accel_off;
  uint8_t accel_scale;
  uint8_t mag;
}Parameter_Flag;

Parameter_Flag Parameter_Read_Flag;
float Accel_Simple_Mode=1;//默认加速度计简单校准模式
/***********************************************************
@函数名：Parameter_Init
@入口参数：无
@出口参数：无
@功能描述：传感器校准参数初始化
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
bool Parameter_Init(void)//加速度计、磁力计校准参数获取
{
  bool success=TRUE;
  /************加速度计零偏与标度值*******/
  ReadFlashParameterThree(ACCEL_X_OFFSET1,
                          &Accel_Offset_Read.x,
                          &Accel_Offset_Read.y,
                          &Accel_Offset_Read.z);
  
  ReadFlashParameterThree(ACCEL_X_SCALE1,
                          &Accel_Scale_Read.x,
                          &Accel_Scale_Read.y,
                          &Accel_Scale_Read.z);
  /************磁力计零偏****************/
  ReadFlashParameterThree(MAG_X_OFFSET,
                          &Mag_Offset_Read.x,
                          &Mag_Offset_Read.y,
                          &Mag_Offset_Read.z);
	
	ReadFlashParameterThree(HOR_CAL_ACCEL_X,
													&Accel_Hor_Read.x,
													&Accel_Hor_Read.y,
													&Accel_Hor_Read.z);	
  // sanity check scale
  if(ABS(Accel_Scale_Read.x-1.0f)>0.5f
     || ABS(Accel_Scale_Read.y-1.0f)>0.5f
       || ABS(Accel_Scale_Read.z-1.0f)>0.5f)
  {
    success = FALSE;
  }
  // sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
  if(ABS(Accel_Offset_Read.x) > 5.0f
     || ABS(Accel_Offset_Read.y) > 5.0f
       || ABS(Accel_Offset_Read.z) > 5.0f)
  {
    success = FALSE;
  }
  
  
  if(isnan(Accel_Offset_Read.x)==0
     &&isnan(Accel_Offset_Read.y)==0
       &&isnan(Accel_Offset_Read.z)==0
         &&isnan(Accel_Scale_Read.x)==0
           &&isnan(Accel_Scale_Read.y)==0
             &&isnan(Accel_Scale_Read.z)==0)//Flash内数据正常，更新加速度校正值
  {    
    B[0]=Accel_Offset_Read.x;
    B[1]=Accel_Offset_Read.y;
    B[2]=Accel_Offset_Read.z;
    K[0]=Accel_Scale_Read.x;
    K[1]=Accel_Scale_Read.y;
    K[2]=Accel_Scale_Read.z;	
  }
	else if(success==TRUE)
	{
    B[0]=0;
    B[1]=0;
    B[2]=0;
    K[0]=1;
    K[1]=1;
    K[2]=1;
	}

	float _accel_simple_mode=0;	
	ReadFlashParameterOne(ACCEL_SIMPLE_MODE,&_accel_simple_mode);
	if(isnan(_accel_simple_mode)==0)   Accel_Simple_Mode=_accel_simple_mode;
	vector3f _accel_hor;	
	ReadFlashParameterThree(HOR_CAL_ACCEL_X,&_accel_hor.x,&_accel_hor.y,&_accel_hor.z);
	if(isnan(_accel_hor.x)==0
		 &&isnan(_accel_hor.y)==0
			 &&isnan(_accel_hor.z)==0)
	{
	  Accel_Hor_Read.x=_accel_hor.x;
		Accel_Hor_Read.y=_accel_hor.y;
		Accel_Hor_Read.z=_accel_hor.z;
	}
	
	if(Accel_Simple_Mode==1)//加速度计校准——简单模式
	{
		 if(isnan(Accel_Hor_Read.x)==0
			&&isnan(Accel_Hor_Read.y)==0
			&&isnan(Accel_Hor_Read.z)==0)//Flash内数据正常
		{
			B[0]=Accel_Hor_Read.x;
			B[1]=Accel_Hor_Read.y;
			B[2]=Accel_Hor_Read.z;
			K[0]=1;
			K[1]=1;
			K[2]=1;
			Pitch_Offset=0;
			Roll_Offset=0;			
		}
		else
		{
			B[0]=0;
			B[1]=0;
			B[2]=0;
			K[0]=1;
			K[1]=1;
			K[2]=1;
		}
	}	
	
	
  /**********磁力计中心偏执获取************/
  if(isnan(Mag_Offset_Read.x)==0
     &&isnan(Mag_Offset_Read.y)==0
       &&isnan(Mag_Offset_Read.z==0))
  {
    mag_offset.x=(int16_t)(Mag_Offset_Read.x);
    mag_offset.y=(int16_t)(Mag_Offset_Read.y);
    mag_offset.z=(int16_t)(Mag_Offset_Read.z);
  }
  else
  {
    mag_offset.x=0;
    mag_offset.y=0;
    mag_offset.z=0;    
  }
  return success;
}
/************加速度计6面矫正结束***********************/


/***********磁力计中心矫正，取单轴最大、最小值平均******/
uint8_t Mag_Calibration_Flag=0,Mag_Calibration_All_Finished;
uint8_t Mag_Calibration_Finished[3]={0};
uint16_t Mag_Calibration_Makesure_Cnt=0;
uint8_t  Mag_Calibration_Mode=3;
uint16_t Mag_Calibration_Cnt=0;
float Yaw_Correct=0;
/*********************************************/
const int16_t Mag_360_define[36]={
  0,10,20,30,40,50,60,70,80,90,
  100,110,120,130,140,150,160,170,180,190,
  200,210,220,230,240,250,260,270,280,290,
  300,310,320,330,340,350
};//磁力计矫正遍历角度，确保数据采集充分
uint8_t Last_Mag_360_Flag[3][36]={0};
uint8_t Mag_360_Flag[3][36]={0};
uint16_t Mag_Is_Okay_Flag[3];
Calibration Mag;
Mag_Unit DataMag;
Mag_Unit Mag_Offset_Read={
  0,0,0,
};
/***********************************************************
@函数名：Mag_Calibration_Check
@入口参数：无
@出口参数：无
@功能描述：磁力计标定函数遥控器动作位检测数据
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void Mag_Calibration_Check(void)
{
  uint16_t  i=0,j=0;
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control>=Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control>=Pit_Rol_Max*Scale_Pecent_Max)
    Mag_Calibration_Makesure_Cnt++;
  
  if(((Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control>=Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control>=Pit_Rol_Max*Scale_Pecent_Max
           &&Mag_Calibration_Makesure_Cnt>200*5)||cal_flag==0x03)//持续5S
             &&Controler_State==Lock_Controler)//必须为上锁状态才可以进入校准模式
    //进入磁力计校准模式
  {
    Bling_Mode=2;
    Mag_Calibration_Flag=1;//磁力计校准模式
    Mag_Calibration_Mode=3;
		Bling_Set(&rgb_red,2000,2000,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_1,1);		
    Mag_Calibration_Makesure_Cnt=0;
    Mag_Calibration_All_Finished=0;//全部校准完成标志位清零
    for(i=0;i<3;i++)
    {
      Mag_Calibration_Finished[i]=0;//对应面标志位清零
      for(j=0;j<36;j++) {Mag_360_Flag[i][j]=0;}
    }
    Page_Number=11;
    Reset_Accel_Calibartion(1);
    Reset_RC_Calibartion(1);
    Forced_Lock_Makesure_Cnt=0;
		cal_flag=0x00;
  }
  
  if(Mag_Calibration_Flag==1)
  {
    if((Throttle_Control<=(1000+Buttom_Safe_Deadband)
       &&Yaw_Control<=-Yaw_Max*Scale_Pecent_Max
         &&Roll_Control==0
           &&Pitch_Control==0) //第一面矫正
					 ||cal_step==0x01) 
    {
      Mag_Calibration_Cnt++;
      if(Mag_Calibration_Cnt>=20||cal_step==0x01)
      {
        Mag_Calibration_Mode=0;
        Mag_Is_Okay_Flag[0]=0;//单面数据采集完成标志位置0
        Mag_Is_Okay_Flag[1]=0;//单面数据采集完成标志位置0
        Mag_Is_Okay_Flag[2]=0;//单面数据采集完成标志位置0
        for(i=0;i<36;i++) Mag_360_Flag[0][i]=0;//清空采集角遍历数据点
        for(i=0;i<36;i++) Mag_360_Flag[1][i]=0;//清空采集角遍历数据点
        for(i=0;i<36;i++) Mag_360_Flag[2][i]=0;//清空采集角遍历数据点
        LS_Init(&Mag_LS);//清空拟合中间变量
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
      cal_step=0x00;
    }
    else if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
            &&Yaw_Control>Yaw_Max*Scale_Pecent_Max
              &&Roll_Control==0
                &&Pitch_Control==0) //第二面矫正
    {
      Mag_Calibration_Cnt++;
      if(Mag_Calibration_Cnt>=20)
      {
        Mag_Calibration_Mode=1;
        Mag_Is_Okay_Flag[0]=0;//单面数据采集完成标志位置0
        Mag_Is_Okay_Flag[1]=0;//单面数据采集完成标志位置0
        Mag_Is_Okay_Flag[2]=0;//单面数据采集完成标志位置0
        for(i=0;i<36;i++) Mag_360_Flag[0][i]=0;//清空采集角遍历数据点
        for(i=0;i<36;i++) Mag_360_Flag[1][i]=0;//清空采集角遍历数据点
        for(i=0;i<36;i++) Mag_360_Flag[2][i]=0;//清空采集角遍历数据点
        LS_Init(&Mag_LS);//清空拟合中间变量
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
    }
    else
    {
      Mag_Calibration_Cnt/=2;
    }
    if(Mag_Calibration_Cnt>=200)  Mag_Calibration_Cnt=200;
    
  }
  
}

/***********************************************************
@函数名：Reset_Mag_Calibartion
@入口参数：uint8_t Type
@出口参数：无
@功能描述：磁力计标定清空数据与强制复位
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void Reset_Mag_Calibartion(uint8_t Type)
{
  uint16 i=0;
  for(i=0;i<36;i++)
  {
    Mag_360_Flag[0][i]=0;//清空采集角点
    Mag_360_Flag[1][i]=0;//清空采集角点
    Mag_360_Flag[2][i]=0;//清空采集角点
  }
  Mag_Is_Okay_Flag[0]=0;
  Mag_Is_Okay_Flag[1]=0;
  Mag_Is_Okay_Flag[2]=0;
  Mag_Calibration_Mode=3;
  if(Type==1)  Mag_Calibration_Flag=0;
}


uint8_t Check_Plane_Sampling_Okay(uint8_t plane_number)
{
  uint8_t finished_flag=0;
  if(Mag_360_Flag[plane_number][0]&Mag_360_Flag[plane_number][1]&Mag_360_Flag[plane_number][2]
     &Mag_360_Flag[plane_number][3]&Mag_360_Flag[plane_number][4]&Mag_360_Flag[plane_number][5]
       &Mag_360_Flag[plane_number][6]&Mag_360_Flag[plane_number][7]&Mag_360_Flag[plane_number][8]
         &Mag_360_Flag[plane_number][9]&Mag_360_Flag[plane_number][10]&Mag_360_Flag[plane_number][11]
           &Mag_360_Flag[plane_number][12]&Mag_360_Flag[plane_number][13]&Mag_360_Flag[plane_number][14]
             &Mag_360_Flag[plane_number][15]&Mag_360_Flag[plane_number][16]&Mag_360_Flag[plane_number][17]
               &Mag_360_Flag[plane_number][18]&Mag_360_Flag[plane_number][19]&Mag_360_Flag[plane_number][20]
                 &Mag_360_Flag[plane_number][21]&Mag_360_Flag[plane_number][22]&Mag_360_Flag[plane_number][23]
                   &Mag_360_Flag[plane_number][24]&Mag_360_Flag[plane_number][25]&Mag_360_Flag[plane_number][26]
                     &Mag_360_Flag[plane_number][27]&Mag_360_Flag[plane_number][28]&Mag_360_Flag[plane_number][29]
                       &Mag_360_Flag[plane_number][30]&Mag_360_Flag[plane_number][31]&Mag_360_Flag[plane_number][32]
                         &Mag_360_Flag[plane_number][33]&Mag_360_Flag[plane_number][34]&Mag_360_Flag[plane_number][35])
    finished_flag=1;
  return finished_flag;
}

/***********************************************************
@函数名：Mag_Calibartion
@入口参数：陀螺仪积分角度值、三轴磁力计原始值
@出口参数：无
@功能描述：磁力计中心标定、利用遥控器直接进入
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
uint8_t Mag_Calibartion(Vector3f *magdata,Vector3f_Body Circle_Angle_Calibartion)
{
  uint16 i=0;
  for(i=0;i<36;i++)
  {
    Last_Mag_360_Flag[0][i]=Mag_360_Flag[0][i];
    Last_Mag_360_Flag[1][i]=Mag_360_Flag[1][i];
    Last_Mag_360_Flag[2][i]=Mag_360_Flag[2][i];
  }
  /********第一面Z轴正向朝着正上方，
  开始绕竖直轴旋转，Z axis is about 1g,X、Y is about 0g*/
  /********第二面Y轴正向朝着正上方，
  开始绕竖直轴旋转，Y axis is about 1g,X、Z is about 0g*/
  if(Mag_Calibration_Mode<3)//检测到对应面数据
  {
    for(i=0;i<36;i++)
    {
      if(ABS(Circle_Angle_Calibartion.Yaw-Mag_360_define[i])<=5.0f
         &&Mag_Calibration_Mode==0
           &&WP_Sensor.acce_filter[2]>=GRAVITY_RAW/2)//Z轴基本竖直
      {
        Mag_360_Flag[0][i]=1;
      }
      
      if(ABS(Circle_Angle_Calibartion.Rol-Mag_360_define[i])<=5.0f
         &&Mag_Calibration_Mode==1
           &&WP_Sensor.acce_filter[1]>=GRAVITY_RAW/2)//Y轴基本竖直
      {
        Mag_360_Flag[1][i]=1;
      }
      
      if(ABS(Circle_Angle_Calibartion.Pit-Mag_360_define[i])<=5.0f
         &&Mag_Calibration_Mode==2
           &&WP_Sensor.acce_filter[0]>=GRAVITY_RAW/2)//X轴基本竖直
      {
        Mag_360_Flag[2][i]=1;
      }
    }
    if(magdata->x >= Mag.x_max)   Mag.x_max = (int16_t)(magdata->x);
    if(magdata->x <  Mag.x_min)   Mag.x_min = (int16_t)(magdata->x);
    if(magdata->y >= Mag.y_max)   Mag.y_max = (int16_t)(magdata->y);
    if(magdata->y <  Mag.y_min)   Mag.y_min = (int16_t)(magdata->y);
    if(magdata->z >= Mag.z_max)   Mag.z_max = (int16_t)(magdata->z);
    if(magdata->z <  Mag.z_min)   Mag.z_min = (int16_t)(magdata->z);
  }
  if(Check_Plane_Sampling_Okay(0))
  {
    Mag_Is_Okay_Flag[0]=1;//单面数据采集完成标志位置1
    for(i=0;i<36;i++)  Mag_360_Flag[0][i]=0;//清空采集角点
    if(Mag_Is_Okay_Flag[1]==0)//另外一面数据未完成
      Mag_Calibration_Mode=1;//自动进入下一面数据采集模式
    else Mag_Calibration_Mode=3;//
  }
  
  if(Check_Plane_Sampling_Okay(1))
  {
    Mag_Is_Okay_Flag[1]=1;//单面数据采集完成标志位置1
    for(i=0;i<36;i++) Mag_360_Flag[1][i]=0;//清空采集角点
    if(Mag_Is_Okay_Flag[2]==0)//另外一面数据未完成
      Mag_Calibration_Mode=2;//自动进入下一面数据采集模式
    else Mag_Calibration_Mode=3;
  }
  
  if(Check_Plane_Sampling_Okay(2))
  {
    Mag_Is_Okay_Flag[2]=1;//单面数据采集完成标志位置1
    for(i=0;i<36;i++) Mag_360_Flag[2][i]=0;//清空采集角点
    if(Mag_Is_Okay_Flag[0]==0)//另外一面数据未完成
      Mag_Calibration_Mode=0;//自动进入下一面数据采集模式
    else Mag_Calibration_Mode=3;
  }
  
  
  
  if(Mag_Calibration_Mode==0)  Yaw_Correct=Circle_Angle_Calibartion.Yaw;
  else if(Mag_Calibration_Mode==1)  Yaw_Correct=Circle_Angle_Calibartion.Rol;
  else if(Mag_Calibration_Mode==2)  Yaw_Correct=Circle_Angle_Calibartion.Pit;
  else Yaw_Correct=0;
  
  
  
  if(Mag_Is_Okay_Flag[0]==1
     &&Mag_Is_Okay_Flag[1]==1
       &&Mag_Is_Okay_Flag[2]==1)//三面数据全部采集完毕，计算磁力计零点
  {
    Mag.x_offset=(Mag.x_min+Mag.x_max)/2.0;
    Mag.y_offset=(Mag.y_min+Mag.y_max)/2.0;
    Mag.z_offset=(Mag.z_min+Mag.z_max)/2.0;
    Mag_Offset_Read.x=Mag.x_offset;
    Mag_Offset_Read.y=Mag.y_offset;
    Mag_Offset_Read.z=Mag.z_offset;
    Mag_Is_Okay_Flag[0]=0;
    Mag_Is_Okay_Flag[1]=0;
    Mag_Is_Okay_Flag[2]=0;
    Mag_Calibration_Flag=0;//磁力计校准结束，释放遥感操作
    Bling_Mode=0;//恢复正常指示模式
    
    
		Bling_Set(&rgb_red,4000,2000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_1,0);  
    //写入加速度零点偏执与磁力计中心偏执 
    WriteFlashParameter_Three(MAG_X_OFFSET,
                              Mag.x_offset,
                              Mag.y_offset,
                              Mag.z_offset);
    NCLink_Send_Check_Flag[9]=0x01;//校准完毕后发送校准数据
    return TRUE;
  }
  return FALSE;
}


void Mag_LS_Init()
{
  LS_Init(&Mag_LS);
}
float mag_a,mag_b,mag_c,mag_r;
/***********************************************************
@函数名：Mag_Calibartion_LS
@入口参数：陀螺仪积分角度值、三轴磁力计原始值
@出口参数：无
@功能描述：磁力计最小二乘法拟合球面、利用遥控器直接进入
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
uint8_t Mag_Calibartion_LS(Vector3f *magdata,Vector3f_Body Circle_Angle_Calibartion)
{
  uint16 i=0;
  for(i=0;i<36;i++)
  {
    Last_Mag_360_Flag[0][i]=Mag_360_Flag[0][i];
    Last_Mag_360_Flag[1][i]=Mag_360_Flag[1][i];
    Last_Mag_360_Flag[2][i]=Mag_360_Flag[2][i];
  }
  
  /********第一面Z轴正向朝着正上方，
  开始绕竖直轴旋转，Z axis is about 1g,X、Y is about 0g*/
  /********第二面Y轴正向朝着正上方，
  开始绕竖直轴旋转，Y axis is about 1g,X、Z is about 0g*/
  if(Mag_Calibration_Mode<3)//检测到对应面数据
  {
    for(i=0;i<36;i++)
    {
      if(ABS(Circle_Angle_Calibartion.Yaw-Mag_360_define[i])<=5.f
         &&Mag_Calibration_Mode==0
           &&WP_Sensor.accel_raw.z>=GRAVITY_RAW/2)//Z轴基本竖直
      {
        Mag_360_Flag[0][i]=1;
      }
      
      if(ABS(Circle_Angle_Calibartion.Rol-Mag_360_define[i])<=5.0f
         &&Mag_Calibration_Mode==1
           &&WP_Sensor.accel_raw.y>=GRAVITY_RAW/2)//Y轴基本竖直
      {
        Mag_360_Flag[1][i]=1;
      }
      
      if(ABS(Circle_Angle_Calibartion.Pit-Mag_360_define[i])<=5.0f
         &&Mag_Calibration_Mode==2
           &&WP_Sensor.accel_raw.x>=GRAVITY_RAW/2)//X轴基本竖直
      {
        Mag_360_Flag[2][i]=1;
      }
    }
    
    for(i=0;i<36;i++)
    {
      if((Last_Mag_360_Flag[0][i]==0&&Mag_360_Flag[0][i]==1)
         ||(Last_Mag_360_Flag[1][i]==0&&Mag_360_Flag[1][i]==1)
           ||(Last_Mag_360_Flag[2][i]==0&&Mag_360_Flag[2][i]==1))
      {
        LS_Accumulate(&Mag_LS, magdata->x,magdata->y,magdata->z);
        //LS_Calculate(&Mag_LS,36*3,0.0f,&mag_a, &mag_b, &mag_c,&mag_r);
      }
    }
  }
  
  if(Check_Plane_Sampling_Okay(0))
  {
    Mag_Is_Okay_Flag[0]=1;//单面数据采集完成标志位置1
    for(i=0;i<36;i++)  Mag_360_Flag[0][i]=0;//清空采集角点
    if(Mag_Is_Okay_Flag[1]==0)//另外一面数据未完成
      Mag_Calibration_Mode=1;//自动进入下一面数据采集模式
    else Mag_Calibration_Mode=3;//
  }
  
  if(Check_Plane_Sampling_Okay(1))
  {
    Mag_Is_Okay_Flag[1]=1;//单面数据采集完成标志位置1
    for(i=0;i<36;i++) Mag_360_Flag[1][i]=0;//清空采集角点
    if(Mag_Is_Okay_Flag[2]==0)//另外一面数据未完成
      Mag_Calibration_Mode=2;//自动进入下一面数据采集模式
    else Mag_Calibration_Mode=3;
  }
  
  if(Check_Plane_Sampling_Okay(2))
  {
    Mag_Is_Okay_Flag[2]=1;//单面数据采集完成标志位置1
    for(i=0;i<36;i++) Mag_360_Flag[2][i]=0;//清空采集角点
    if(Mag_Is_Okay_Flag[0]==0)//另外一面数据未完成
      Mag_Calibration_Mode=0;//自动进入下一面数据采集模式
    else Mag_Calibration_Mode=3;
  }
  
  if(Mag_Calibration_Mode==0)  			Yaw_Correct=Circle_Angle_Calibartion.Yaw;
  else if(Mag_Calibration_Mode==1)  Yaw_Correct=Circle_Angle_Calibartion.Rol;
  else if(Mag_Calibration_Mode==2)  Yaw_Correct=Circle_Angle_Calibartion.Pit;
  else Yaw_Correct=0;
  
  if(Mag_Is_Okay_Flag[0]==1
     &&Mag_Is_Okay_Flag[1]==1
       &&Mag_Is_Okay_Flag[2]==1)//三面数据全部采集完毕，计算磁力计零点
  {
    LS_Calculate(&Mag_LS,36*3,0.0f,&mag_a, &mag_b, &mag_c,&mag_r);
    Mag_Offset_Read.x=mag_a;
    Mag_Offset_Read.y=mag_b;
    Mag_Offset_Read.z=mag_c;
    Mag_Is_Okay_Flag[0]=0;
    Mag_Is_Okay_Flag[1]=0;
    Mag_Is_Okay_Flag[2]=0;
    Mag_Calibration_Flag=0;//磁力计校准结束，释放遥感操作
    Bling_Mode=0;//恢复正常指示模式
    
		
		Bling_Set(&rgb_red,4000,2000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_1,0);		
    WriteFlashParameter_Three(MAG_X_OFFSET,
                              Mag_Offset_Read.x,
                              Mag_Offset_Read.y,
                              Mag_Offset_Read.z);
		NCLink_Send_Check_Flag[9]=0x01;//校准完毕后发送校准数据
    return TRUE;
  }
  return FALSE;
}



#define  RC_TOP_DEFAULT       2000
#define  RC_BUTTOM_DEFAULT    1000
#define  RC_MIDDLE_DEFAULT    1500
#define  RC_DEADBAND_DEFAULT  100
#define  RC_DEADBAND_PERCENT   		0.1f		//中位死区占实际行程的百分比
#define  RC_THR_DEADBAND_PERCENT   0.2f		//油门中位死区占实际行程的百分比
#define  RC_RESET_DEFAULT  1500

uint8_t RC_Read_Flag[8];
/***********************************************************
@函数名：RC_Calibration_Init
@入口参数：无
@出口参数：无
@功能描述：遥控器行程标定初始化
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void RC_Calibration_Init()
{
	float _rc_deadband_percent=0;
  uint16_t i=0;
  float max_read[8]={0},min_read[8]={0};
  for(i=0;i<8;i++)
  {
    ReadFlashParameterTwo(RC_CH1_MAX+2*i,&max_read[i],&min_read[i]);
    if(isnan(max_read[i])==0&&isnan(min_read[i])==0)  	
      RC_Read_Flag[i]=0x01;
  }
  
  if(RC_Read_Flag[0]!=0x00
     &&RC_Read_Flag[1]!=0x00
       &&RC_Read_Flag[2]!=0x00
         &&RC_Read_Flag[3]!=0x00
           &&RC_Read_Flag[4]!=0x00
             &&RC_Read_Flag[5]!=0x00
               &&RC_Read_Flag[6]!=0x00
                 &&RC_Read_Flag[7]!=0x00)//flash中存在数据
  {
    for(i=0;i<8;i++)
    {
			if(i==RC_THR_CHANNEL) _rc_deadband_percent=RC_THR_DEADBAND_PERCENT;
			else _rc_deadband_percent=RC_DEADBAND_PERCENT;
			
			RC_Data.cal[i].max=max_read[i];
			RC_Data.cal[i].min=min_read[i];
			RC_Data.cal[i].middle=(float)((max_read[i]+min_read[i])/2.0f);

			RC_Data.cal[i].deadband=(float)((max_read[i]-min_read[i])*_rc_deadband_percent/1.0f);
			RC_Data.cal[i].reverse_flag=false;
			RC_Data.cal[i].scale=(RC_Data.cal[i].max-RC_Data.cal[i].min-RC_Data.cal[i].deadband)*0.5f;	
    }
  }
  else//flash中不存在数据
  {
    for(i=0;i<8;i++)
    {		
			if(i==RC_THR_CHANNEL) _rc_deadband_percent=RC_THR_DEADBAND_PERCENT;
			else _rc_deadband_percent=RC_DEADBAND_PERCENT;
			
      RC_Data.cal[i].max=RC_TOP_DEFAULT;
      RC_Data.cal[i].min=RC_BUTTOM_DEFAULT;
      RC_Data.cal[i].middle=RC_MIDDLE_DEFAULT;
			
			RC_Data.cal[i].deadband=(float)((RC_Data.cal[i].max-RC_Data.cal[i].min)*_rc_deadband_percent/1.0f);;
			RC_Data.cal[i].reverse_flag=false;
			RC_Data.cal[i].scale=(RC_Data.cal[i].max-RC_Data.cal[i].min-RC_Data.cal[i].deadband)*0.5f;
    }
  }
}
/***********************************************************
@函数名：RC_Calibration_RESET
@入口参数：无
@出口参数：无
@功能描述：遥控器行程标定复位
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void RC_Calibration_RESET()
{
  uint16_t i=0;
  for(i=0;i<8;i++)
  {
    RC_Data.cal[i].max=RC_RESET_DEFAULT;
    RC_Data.cal[i].min=RC_RESET_DEFAULT;
  }
}


uint8_t RC_Calibration_Trigger_Flag=0;
/***********************************************************
@函数名：RC_Calibration_Trigger
@入口参数：无
@出口参数：无
@功能描述：遥控器行程标定触发
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void RC_Calibration_Trigger(void)
{
  if(QuadKey2==0)
  {
    delay_ms(500);
    if(QuadKey2==0)
    {
      while(QuadKey2==0);
      RC_Calibration_RESET();//复位遥控器行程值，等待校准完毕
      RC_Calibration_Trigger_Flag=1;
      Page_Number=14;
      Key_Right_Release=1;
    }
  }
  else
  {
    RC_Calibration_Init();//直接从flash里面（或者DEFAULT值）获取遥控器行程输出
    RC_Calibration_Trigger_Flag=0;
  }
}

/***********************************************************
@函数名：RC_Calibration_Check
@入口参数：uint16 *rc_date
@出口参数：无
@功能描述：遥控器行程校准检测，涉及内部Flash读、写操作
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
bool RC_Calibration_Check(uint16 *rc_date)
{
  uint16_t i=0;
  bool success_flag=FALSE;
	
  if(cal_flag==0x05)
	{
		RC_Calibration_RESET();//复位遥控器行程值，等待校准完毕
		RC_Calibration_Trigger_Flag=1;
		Page_Number=14;
		Key_Right_Release=1;
		cal_flag=0x00;
	}
	
  if(RC_Calibration_Trigger_Flag==0) return success_flag;
  for(i=0;i<8;i++)
  {
    if(rc_date[i] >= RC_Data.cal[i].max)   RC_Data.cal[i].max = rc_date[i];//最大行程值
    if(rc_date[i] <  RC_Data.cal[i].min)   RC_Data.cal[i].min = rc_date[i];//最小行程值
    RC_Data.cal[i].middle=(float)((RC_Data.cal[i].max+RC_Data.cal[i].min)/2.0f);//行程中位
    RC_Data.cal[i].deadband=(float)((RC_Data.cal[i].max-RC_Data.cal[i].min)*RC_DEADBAND_PERCENT/1.0f);//设置满量程的百分之RC_DEADBAND_PERCENT为中位死区
  }
  
	if(cal_flag==0x06)
	{
		RC_Calibration_Trigger_Flag=0;
		Key_Right_Release=0;
		success_flag=TRUE;
		for(i=0;i<8;i=i+1)
		{
			WriteFlashParameter_Two(RC_CH1_MAX+2*i,
															RC_Data.cal[i].max,
															RC_Data.cal[i].min);
		}
	  cal_flag=0x00;
	}
	
	
  if(QuadKey2==0)//遥控器标定完成后，通过按键来结束标定过程
  {
    delay_ms(2000);
    if(QuadKey2==0)
    {
      while(QuadKey2==0);
      RC_Calibration_Trigger_Flag=0;
      Key_Right_Release=0;
      success_flag=TRUE;
      for(i=0;i<8;i=i+1)
      {
        WriteFlashParameter_Two(RC_CH1_MAX+2*i,
                                RC_Data.cal[i].max,
                                RC_Data.cal[i].min);
      }
    }
  }
  return success_flag;
}

/***********************************************************
@函数名：Reset_RC_Calibartion
@入口参数：uint8_t Type
@出口参数：无
@功能描述：遥控器行程校准强制复位
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void Reset_RC_Calibartion(uint8_t Type)
{
  if(Type==1)  
  {
    RC_Calibration_Trigger_Flag=0;
    Key_Right_Release=0;
  }
}
uint16_t ESC_Calibration_Makesure_Cnt=0;
float ESC_Calibration_Flag=0;
void ESC_Calibration_Check(void)
{
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control>=Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control<=-Pit_Rol_Max*Scale_Pecent_Max)
    ESC_Calibration_Makesure_Cnt++;
  //else ESC_Calibration_Makesure_Cnt/=2;
  
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control>=Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control<=-Pit_Rol_Max*Scale_Pecent_Max
           &&ESC_Calibration_Makesure_Cnt>200*5//持续5S
             &&Controler_State==Lock_Controler)//必须为上锁状态才可以进入校准模式
    //进入ESC校准模式
  {
    ESC_Calibration_Flag=1;
    ESC_Calibration_Makesure_Cnt=0;
    Forced_Lock_Makesure_Cnt=0;
    WriteFlashParameter(ESC_CALIBRATION_FLAG,
                        ESC_Calibration_Flag);
    Bling_Set(&rgb_red,5000,500,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_1,0);	
    Page_Number=15;
  }
}

#define Thr_Chl_Num  2
/***********************************************************
@函数名：ESC_Calibration
@入口参数：无
@出口参数：无
@功能描述：电调行程校准
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void ESC_Calibration(void)
{
  PWM_Output((uint16_t)(Receiver_PPM_Databuf[Thr_Chl_Num]),
						 (uint16_t)(Receiver_PPM_Databuf[Thr_Chl_Num]),
						 (uint16_t)(Receiver_PPM_Databuf[Thr_Chl_Num]),
						 (uint16_t)(Receiver_PPM_Databuf[Thr_Chl_Num]));
}

/***********************************************************
@函数名：Check_Calibration_Flag
@入口参数：无
@出口参数：无
@功能描述：当前校准模式检测
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
uint8_t Check_Calibration_Flag(void)
{
  uint8_t cal_flag=0x00; 
  if(Key_Right_Release==1)      cal_flag|=0x01;//遥控器校准
  if(Accel_Calibration_Flag==1) cal_flag|=0x02;//加速度计校准
  if(Mag_Calibration_Flag==1)   cal_flag|=0x04;//磁力计校准
  return cal_flag;
}

float Pitch_Offset=0,Roll_Offset=0;
float Hor_Accel_Offset[3]={0};
uint16_t Horizontal_Calibration_Makesure_Cnt=0;
/***********************************************************
@函数名：Horizontal_Calibration_Check
@入口参数：无
@出口参数：无
@功能描述：机架水平校准
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void Horizontal_Calibration_Check(void)
{
  float acce_sample_sum[3]={0,0,0};//加速度和数据
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control>=Yaw_Max*Scale_Pecent_Max&&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max&&Pitch_Control<=-Pit_Rol_Max*Scale_Pecent_Max)
  {
    Horizontal_Calibration_Makesure_Cnt++;
  }
  if(((Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control<=-Pit_Rol_Max*Scale_Pecent_Max
           &&Horizontal_Calibration_Makesure_Cnt>=50*3)||cal_flag==0x04)//持续3秒
             &&Controler_State==Lock_Controler)//必须为上锁状态才可以进入校准模式
  {
		delay_ms(2000);
    Pitch_Offset=WP_AHRS.Pitch;
    Roll_Offset=WP_AHRS.Roll;
    uint16_t num_samples=0;
    while(num_samples<200)//采样200次
    {
      if(Gyro_Length<=20.0f)//通过陀螺仪模长来确保机体静止
      {
        for(int16_t j=0;j<3;j++){
          acce_sample_sum[j]+=WP_Sensor.acce_filter[j];//加速度计转化为1g量程下
        }
        delay_ms(5);//间隔10ms，1s内数据取平均
        num_samples++;
      }
    }
    Hor_Accel_Offset[0]=(acce_sample_sum[0]/num_samples)*RAW_TO_G; //保存对应面的加速度计量
    Hor_Accel_Offset[1]=(acce_sample_sum[1]/num_samples)*RAW_TO_G; //保存对应面的加速度计量
    Hor_Accel_Offset[2]=(acce_sample_sum[2]/num_samples-GRAVITY_RAW)*RAW_TO_G;//保存对应面的加速度计量
	    
    Horizontal_Calibration_Makesure_Cnt=0;
		WriteFlashParameter(HOR_CAL_ACCEL_X,Hor_Accel_Offset[0]);
		WriteFlashParameter(HOR_CAL_ACCEL_Y,Hor_Accel_Offset[1]);
		WriteFlashParameter(HOR_CAL_ACCEL_Z,Hor_Accel_Offset[2]);
		
		
    WriteFlashParameter(PITCH_OFFSET1,Pitch_Offset);
    WriteFlashParameter(ROLL_OFFSET1,Roll_Offset);
			
		Parameter_Init();	
		
    Bling_Set(&rgb_red,500,100,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_1,0);
	  NCLink_Send_Check_Flag[9]=0x01;
		cal_flag=0x00;
  }
}

/***********************************************************
@函数名：Horizontal_Calibration_Init
@入口参数：无
@出口参数：无
@功能描述：机架水平校准初始化
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void Horizontal_Calibration_Init(void)
{
  float pitch_offset_temp=0,roll_offset_temp=0;	
  ReadFlashParameterOne(PITCH_OFFSET1,&pitch_offset_temp);
  ReadFlashParameterOne(ROLL_OFFSET1,&roll_offset_temp);
  
  if(isnan(pitch_offset_temp)==0)   Pitch_Offset=pitch_offset_temp;
  if(isnan(roll_offset_temp)==0)    Roll_Offset=roll_offset_temp;
}


uint16_t Headless_Mode_Makesure_Cnt=0;
float Headless_Mode_Yaw=0.0f;
/***********************************************************
@函数名：Headless_Mode_Calibration_Check
@入口参数：无
@出口参数：无
@功能描述：无头模式初始偏航角获取
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void Headless_Mode_Calibration_Check(void)
{
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control==0)
  {
    Headless_Mode_Makesure_Cnt++;
  }
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control==0
           &&Headless_Mode_Makesure_Cnt>=200*5//持续三秒
             &&Controler_State==Lock_Controler)//必须为上锁状态才可以进入校准模式
  {    
    Headless_Mode_Yaw=WP_AHRS.Yaw;
    Horizontal_Calibration_Makesure_Cnt=0;
    WriteFlashParameter(HEADLESS_MODE_YAW,Headless_Mode_Yaw);
    Bling_Set(&rgb_red,500,100,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_1,0);
  }
}


uint8_t Gyro_Safety_Calibration_Flag=0;
uint16_t Gyro_Calibration_Makesure_Cnt=0;
void Gyro_Calibration_Check(vector3f *gyro)
{	
	if(Controler_State==Unlock_Controler||Page_Number!=0) return;//上锁状态下+显示屏处于第一页才允许校准
	
	if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
	 &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
	 &&Roll_Control==0
	 &&Pitch_Control<=-Scale_Pecent_Max*Pit_Rol_Max)
  {
    Gyro_Calibration_Makesure_Cnt++;
  }
	
	if(((Throttle_Control<=(1000+Buttom_Safe_Deadband)
	 &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
	 &&Roll_Control==0
	 &&Pitch_Control<=-Scale_Pecent_Max*Pit_Rol_Max           
	 &&Gyro_Calibration_Makesure_Cnt>=200*3)||cal_flag==0x01)//持续3秒
   &&Controler_State==Lock_Controler)//必须为上锁状态才可以进入校准模式
  {
		Gyro_Calibration_Makesure_Cnt=0;
		Gyro_Safety_Calibration_Flag=0;
		cal_flag=0x00;
  }	

	
	if(Gyro_Safety_Calibration_Flag==1)  return;
	
	
	
	static vector3f offset;
	static uint16_t offset_cnt=0;
  static vector3f last_gyro;
	const float scale=GYRO_CALIBRATION_COFF;
   if(ABS(gyro->x-last_gyro.x)*scale<=10.0f
		   &&ABS(gyro->y-last_gyro.y)*scale<=10.0f
	      &&ABS(gyro->z-last_gyro.z)*scale<=10.0f
	       &&Temperature_Stable_Flag==1)
	{
	  offset.x+=gyro->x;
		offset.y+=gyro->y;
		offset.z+=gyro->z;
		offset_cnt++;
	}
  else
	{
		offset.x=0;
		offset.y=0;
		offset.z=0;
		offset_cnt=0;
	}
  last_gyro.x=gyro->x;
	last_gyro.y=gyro->y;
	last_gyro.z=gyro->z;
	
	if(offset_cnt>=400)//持续满足2s
	{
		gyro_offset.x =(offset.x/offset_cnt);//得到标定偏移
		gyro_offset.y =(offset.y/offset_cnt);
		gyro_offset.z =(offset.z/offset_cnt);
		Euler_Angle_Init_Again();
		WriteFlashParameter_Three(GYRO_X_OFFSET,
															gyro_offset.x,
															gyro_offset.y,
															gyro_offset.z);		
		Bling_Set(&rgb_red,2000,100,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_1,0);
		
		
		offset.x=0;
		offset.y=0;
		offset.z=0;
		offset_cnt=0;
		
		NCLink_Send_Check_Flag[9]=0x01;
		Gyro_Safety_Calibration_Flag=1;
		
		buzzer_setup(500,0.25f,3);
	}
}


//校准、参数检测总函数
void Calibration_All_Check(void)
{
	if(Controler_State==Unlock_Controler) return;//上锁状态下才允许校准
	Accel_Calibration_Check();			  //加速度校准检测
  Mag_Calibration_Check();				  //磁力计校准检测
  ESC_Calibration_Check();				  //电调行程校准检测
}




void Reserved_Params_Init(void)
{
  float _param_value[RESERVED_PARAM_NUM];
	for(uint16_t i=0;i<RESERVED_PARAM_NUM;i++)
	{
		ReadFlashParameterOne(RESERVED_PARAM+i,&_param_value[i]);
		if(isnan(_param_value[i])==0) param_value[i]=_param_value[i];
		else param_value[i]=0;
	}
}




uint16_t Target_Height=0,Safe_Height=0,Safe_Vbat=0,Max_Height=0,Max_Radius=0,Max_Upvel=0,Max_Downvel=0,Max_Horvel=0,Reserved_Uart=0,Near_Ground_Height=0;
uint16_t Uart2_Mode=0,Avoid_Obstacle=0;
void Other_Parameter_Init(void)
{
	float targeheight,safeheight,safevbat,maxheight,maxradius,maxupvel,maxdownvel,maxhorvel,reserveduart,neargroundheight;
	float uart2mode,avoidobstacle;
	ReadFlashParameterOne(TARGET_HEIGHT,&targeheight);
	ReadFlashParameterOne(SAFE_HEIGHT,&safeheight);
	ReadFlashParameterOne(SAFE_VBAT,&safevbat);
	ReadFlashParameterOne(MAX_HEIGHT,&maxheight);
	ReadFlashParameterOne(MAX_RADIUS,&maxradius);
	ReadFlashParameterOne(MAX_UPVEL,&maxupvel);
	ReadFlashParameterOne(MAX_DOWNVEL,&maxdownvel);
	ReadFlashParameterOne(MAX_HORVEL,&maxhorvel);
 	ReadFlashParameterOne(RESERVED_UART_FUNCTION,&reserveduart);
	ReadFlashParameterOne(NEAR_GROUND_HEIGHT,&neargroundheight);
 	ReadFlashParameterOne(UART2_FUNCTION,&uart2mode);
	ReadFlashParameterOne(AVOID_OBSTACLE,&avoidobstacle);	


	
	if(isnan(targeheight)==0)   Target_Height=targeheight;
	else Target_Height=Auto_Launch_Target;
	
	if(isnan(safeheight)==0)    Safe_Height=safeheight;
	else Safe_Height=Nav_Safety_Height;
	
	if(isnan(safevbat)==0)      Safe_Vbat=safevbat;
	else Safe_Vbat=Flight_Safe_Vbat;//
	
	if(isnan(maxheight)==0)     Max_Height=maxheight;
	else Max_Height=Flight_Max_Height;//
	
	if(isnan(maxradius)==0)     Max_Radius=maxradius;
	else Max_Radius=Flight_Max_Radius;//
	
	if(isnan(maxupvel)==0)      Max_Upvel=maxupvel;
	else Max_Upvel=Climb_Up_Speed_Max;
	
	if(isnan(maxdownvel)==0)    Max_Downvel=maxdownvel;
	else Max_Downvel=Climb_Down_Speed_Max;
	
	if(isnan(maxhorvel)==0)     Max_Horvel=maxhorvel;
	else Max_Horvel=Nav_Speed_Max;

	if(isnan(reserveduart)==0)    Reserved_Uart=reserveduart;
	else Reserved_Uart=RESERVED_UART_DEFAULT;
	
	if(isnan(neargroundheight)==0)     Near_Ground_Height=neargroundheight;
	else Near_Ground_Height=Nav_Near_Ground_Height_Default;

	if(isnan(uart2mode)==0)    Uart2_Mode=uart2mode;
	else Uart2_Mode=UART2_DEFAULT;
	
	if(isnan(avoidobstacle)==0)     Avoid_Obstacle=avoidobstacle;
	else Avoid_Obstacle=AVOID_OBSTACLE_DEFAULT;
	

	Reserved_Params_Init();
}

void Other_Parameter_Default(void)
{
	Target_Height=Auto_Launch_Target;
	Safe_Height=Nav_Safety_Height;
	Safe_Vbat=Flight_Safe_Vbat;
	Max_Height=Flight_Max_Height;
	Max_Radius=Flight_Max_Radius;
	Max_Upvel=Climb_Up_Speed_Max;
	Max_Downvel=Climb_Down_Speed_Max;
	Max_Horvel=Nav_Speed_Max;
	Reserved_Uart=RESERVED_UART_DEFAULT;
	Near_Ground_Height=Nav_Near_Ground_Height_Default;
	Uart2_Mode=UART2_DEFAULT;
	Avoid_Obstacle=AVOID_OBSTACLE_DEFAULT;
	
	WriteFlashParameter(TARGET_HEIGHT,Target_Height);
	WriteFlashParameter(SAFE_HEIGHT,Safe_Height);
	WriteFlashParameter(SAFE_VBAT,Safe_Vbat);
	WriteFlashParameter(MAX_HEIGHT,Max_Height);
	WriteFlashParameter(MAX_RADIUS,Max_Radius);
	WriteFlashParameter(MAX_UPVEL,Max_Upvel);
	WriteFlashParameter(MAX_DOWNVEL,Max_Downvel);
	WriteFlashParameter(MAX_HORVEL,Max_Horvel);
	WriteFlashParameter(RESERVED_UART_FUNCTION,Reserved_Uart);
	WriteFlashParameter(NEAR_GROUND_HEIGHT,Near_Ground_Height);
	WriteFlashParameter(UART2_FUNCTION,Uart2_Mode);
	WriteFlashParameter(AVOID_OBSTACLE,Avoid_Obstacle);
}


