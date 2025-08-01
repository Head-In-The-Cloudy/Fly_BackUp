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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

lpf_param loam_ft={
	0,
// {						 1,   -1.808964975513,   0.9099299881777},
// {0.9549649940889,  -1.808964975513,   0.9549649940889}};//200-9-12
// {						 1,   -1.927971114804,   0.9539525559078},
// {0.9769762779539,   -1.927971114804,   0.9769762779539}//400-9-12
	 {					 1,      -1.977103407037,   0.9813258904927},
	 {0.9906629452463,   -1.977103407037,   0.9906629452463}//1000-9-12
};
	 

//-----Butterworth变量-----//
lpf_param Butter_5HZ_Parameter_RC;
lpf_buf Butterworth_Buffer_Baro,Butterworth_Buffer_Baro_Acc;

/****************************************
Butterworth低通滤波器参数初始化：http://blog.csdn.net/u011992534/article/details/73743955
***************************************/
/***********************************************************
@函数名：Butterworth_Parameter_Init
@入口参数：无
@出口参数：无
功能描述：巴特沃斯低通滤波器初始化
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void Butterworth_Parameter_Init(void)
{
	set_cutoff_frequency(50, 10 ,&Butter_5HZ_Parameter_RC);
	sensor_filter_init();
}

/*************************************************
函数名:	float LPButterworth(float curr_input,lpf_buf *Buffer,lpf_param *Parameter)
说明:	加速度计低通滤波器
入口:	float curr_input 当前输入加速度计,滤波器参数，滤波器缓存
出口:	无
备注:	2阶Butterworth低通滤波器
*************************************************/
float LPButterworth(float curr_input,lpf_buf *Buffer,lpf_param *Parameter)
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





// discrete low pass filter, cuts out the
// high frequency noise that can drive the controller crazy
//derivative = _last_derivative + _d_lpf_alpha * (derivative - _last_derivative);
float set_lpf_alpha(int16_t cutoff_frequency, float time_step)
{
  // calculate alpha
  float lpf_alpha;
  float rc = 1/(2*PI*cutoff_frequency);
  lpf_alpha = time_step / (time_step + rc);
  return lpf_alpha;
}



//https://blog.csdn.net/sszhouplus/article/details/43113505
//https://blog.csdn.net/shengzhadon/article/details/46784509
//https://blog.csdn.net/shengzhadon/article/details/46791903
//https://blog.csdn.net/shengzhadon/article/details/46803401
/***********************************************************
@函数名：set_cutoff_frequency
@入口参数：float sample_frequent, float cutoff_frequent,
lpf_param *LPF
@出口参数：无
功能描述：巴特沃斯低通滤波器初始化
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
void set_cutoff_frequency(float sample_frequent, float cutoff_frequent,lpf_param *LPF)
{
  float fr = sample_frequent / cutoff_frequent;
  float ohm = tanf(M_PI_F / fr);
  float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;
  if (cutoff_frequent <= 0.0f) {
    // no filtering
    return;
  }
  LPF->b[0] = ohm * ohm / c;
  LPF->b[1] = 2.0f * LPF->b[0];
  LPF->b[2] = LPF->b[0];
  LPF->a[0]=1.0f;
  LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
  LPF->a[2] = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
}




//以下代码为无名创新首创高阶滤波器参数计算代码，保留版权所有，不得做任何商业用途
//以下代码为无名创新首创高阶滤波器参数计算代码，保留版权所有，不得做任何商业用途
//以下代码为无名创新首创高阶滤波器参数计算代码，保留版权所有，不得做任何商业用途
#define SYMBOL_ADD  0
#define SYMBOL_SUB  1
/*======================================================================
* 函数名：  pascalTriangle
* 函数功能：计算杨辉三角的第N行的值（数组），该系列值为(x+1)^N的系数，
*         加改进(x-1)^N的系数，最低次数在第一个
*
* 变量名称：
*          N      - 杨辉三角第N行，N=0,1,...,N
*          symbol - 运算符号，0——(x+1)^N，1——(x-1)^N
*          vector - 返回数组，杨辉三角的第N行的值
*
* 返回值：  void
*=====================================================================*/
void pascalTriangle(int N,int symbol,int *vector)
{
  vector[0] = 1;
  if(N == 0)
  {
    return;
  }
  else if (N == 1)
  {
    if(symbol == SYMBOL_ADD)
    {
      vector[1] = 1;
    }
    else
    {
      vector[0] = -1; //如果是减号，则第二项系数是-1
      vector[1] = 1;
    }
    return;
  }
  int length = N + 1; //数组长度
  //int *temp;//[N];   //定义中间变量

  int temp[20];
	//int *temp=(int *)malloc(N*sizeof(int));
	//memset(temp,0,sizeof(int));
	
  temp[0] = 1;
  temp[1] = 1;
  
  for(int i = 2; i <= N; i++)
  {
    vector[i] = 1;
    for(int j = 1; j < i; j++)
    {
      vector[j] = temp[j - 1] + temp[j]; //x[m][n] = x[m-1][n-1] + x[m-1][n]
    }
    if(i == N) //最后一次不需要给中间变量赋值
    {
      if(symbol == SYMBOL_SUB) //运算符为减号
      {
        for(int k = 0; k < length; k++)
        {
          vector[k] = vector[k] * pow(-1, length - 1 - k);
        }
      }
      return;
    }
    for(int j = 1; j <= i; j++)
    {
      temp[j] = vector[j];
    }
  }
	free(temp);
}

/*======================================================================
 * 函数名：  coefficientEquation（整数）和coefficientEquation2（浮点数）
 * 函数功能：计算多项式相乘的系数，最低次数在第一个
 *
 * 变量名称：
 *          originalCoef - 原来的系数数组，计算后的系数也存储在该数组内
 *          N            - 原来数组中数据的长度，多项式最高次为N-1
 *          nextCoef     - 与原数组相乘的数组的系数（两项）
 *
 * 返回值：  void
 *=====================================================================*/
float tempCoef[20];
void coefficientEquation(int *originalCoef,int N,int *nextCoef,int nextN)
{    
		//tempCoef=(float *)malloc(sizeof(float)*(N+nextN-1)); //[N + nextN - 1];    //中间变量
    for(int i = 0; i < N + nextN - 1; i++)
    {
        tempCoef[i] = originalCoef[i]; //中间变量初始化
        originalCoef[i] = 0;
    }
    
    for(int j = 0; j < nextN; j++)
    {
        for(int i = j; i < N + nextN - 1; i++)
        {
            originalCoef[i] += tempCoef[i-j] * nextCoef[j];
        }
    }
		//free(tempCoef);
}

void coefficientEquation2(float *originalCoef,int N,float *nextCoef,int nextN)
{
    float tempCoef[N + nextN - 1];    //中间变量
    for(int i = 0; i < N + nextN - 1; i++)
    {
        tempCoef[i] = originalCoef[i]; //中间变量初始化
        originalCoef[i] = 0;
    }
    
    for(int j = 0; j < nextN; j++)
    {
        for(int i = j; i < N + nextN - 1; i++)
        {
            originalCoef[i] += tempCoef[i-j] * nextCoef[j];
        }
    }
}


/***********************************************************
@函数名：GildeAverageValueFilter_MAG
@入口参数：float NewValue,float *Data
@出口参数：无
功能描述：滑动窗口滤波
@作者：无名小哥
@日期：2024/10/01
*************************************************************/
float GildeAverageValueFilter_MAG(float NewValue,float *Data)
{
  float sum;
  unsigned char i;
  Data[0]=NewValue;
  for(i=N2-1;i>0;i--)
  {
    sum+=Data[i];
    Data[i]=Data[i-1];
  }
  sum=sum/N2;
  return(sum);
}


//#define M_PI  PI


//定义巴特沃斯滤波器pb系数列表（b0,b1,...,bn）
static float g_butterPb[10][10] = {{1.0,0,0,0,0,0,0,0,0,0},
    {1.0, 1.4142136, 0,0,0,0,0,0,0,0},
    {1.0, 2.0, 2.0, 0,0,0,0,0,0,0},
    {1.0, 2.6131259, 3.4142136, 2.6131259, 0,0,0,0,0,0},
    {1.0, 3.236068, 5.236068, 5.236068, 3.236068, 0,0,0,0,0},
    {1.0, 3.8637033, 7.4641016, 9.1416202, 7.4641016, 3.8637033, 0,0,0,0},
    {1.0, 4.4939592, 10.0978347, 14.5917939, 14.5917939, 10.0978347, 4.4939592, 0,0,0},
    {1.0, 5.1258309, 13.1370712, 21.8461510, 25.6883559, 21.8461510, 13.1370712, 5.1258309, 0,0},
    {1.0, 5.7587705, 16.5817187, 31.1634375, 41.9863857, 41.9863857, 31.1634375, 16.5817187, 5.7587705, 0},
    {1.0, 6.3924532, 20.4317291, 42.8020611, 64.8823963, 74.2334292, 64.8823963, 42.8020611, 20.4317291, 6.3924532}};
/*======================================================================
 * 方法名：  filterIIRButterLowpass
 * 方法功能：设计巴特沃斯样本低通示波器
 *
 * 变量名称：
 *          fpass - 通带截止频率（模拟频率）
 *          fstop - 阻带截止频率（模拟频率）
 *          rp    - 通带最大衰减（dB）
 *          rs    - 阻带最小衰减（dB）
 *          Fs    - 采样频率
 *
 * 返回值：  返回巴特沃斯低通滤波器的阶数N和截止频率Ws结构体
 *=====================================================================*/
ButterFilterStruct filterIIRButterLowpass( float passF_alpha,
	                                         float passF_beta,
																					 float stopF_alpha,
																					 float stopF_beta,
																					 float rp,
																					 float rs,
																					 float fs,
																					 int filterType)
{
    ButterFilterStruct nAndFc;      //返回滤波器的阶数N和截止频率fc
    nAndFc.filterType = filterType; //滤波器类型
    float nOfN = 0.0;
    float passW = 0.0, stopW = 0.0, wa, wc; //wa = stopW/passW或其导数
    float passF1 = 0.0, passF2 = 0.0, stopF1 = 0.0, stopF2 = 0.0, w0 = 0.0;//w0 - 中心频率
    float passW1 = 0.0, passW2 = 0.0, stopW1 = 0.0, stopW2 = 0.0, fc = 0.0;
    
    rs = fabs(rs);
    rp = fabs(rp);
    passF1 = passF_alpha;
    stopF1 = stopF_alpha;
    
    //根据滤波器类型，选择不同的预畸变换式
    switch (filterType) {
        case FILTER_IIR_BUTTER_LOW:
            if(passF1 >= stopF1)
            {
                nAndFc.isFOK = false;
                //NSLog(@"错误！应满足：passF < stopF");
                return nAndFc;
            }
            
            nAndFc.isFOK = true;
            passW = tan(passF1 * M_PI / fs);    //数字低通，频率预畸，W = tan(w/2)
            stopW = tan(stopF1 * M_PI / fs);
            wa = fabs(stopW/passW);
            break;
        case FILTER_IIR_BUTTER_HIGH:
            if(passF1 <= stopF1)
            {
                nAndFc.isFOK = false;
                //NSLog(@"错误！应满足：passF > stopF");
                return nAndFc;
            }
            
            nAndFc.isFOK = true;
            passW = 1/tan(passF1 * M_PI / fs); //数字高通，频率预畸，W = cot(w/2)
            stopW = 1/tan(stopF1 * M_PI / fs);
            wa = fabs(stopW/passW);
            break;
            
        case FILTER_IIR_BUTTER_PASS:
            passF2 = passF_beta;
            stopF2 = stopF_beta;
            if(!(stopF1 < passF1 && passF1 < passF2 && passF2 < stopF2))
            {
                nAndFc.isFOK = false;
                //NSLog(@"错误！应满足：stopF[1] < passF[1] < passF[2] < stopF[2]");
                return nAndFc;
            }
            
            nAndFc.isFOK = true;
            //转换为数字频率（不进行预畸）
            passW1 = 2 * M_PI * passF1 / fs;
            passW2 = 2 * M_PI * passF2 / fs;
            stopW1 = 2 * M_PI * stopF1 / fs;
            stopW2 = 2 * M_PI * stopF2 / fs;
            
            nAndFc.cosW0 = cos((passW1 + passW2)/2)/cos((passW1 - passW2)/2); //保存cos(w0)
            w0 = acos(nAndFc.cosW0);//求带通滤波器的中心频率
            
            passW1 = (cos(w0)-cos(passW1))/sin(passW1);  //通带截止频率
            passW2 = (cos(w0)-cos(passW2))/sin(passW2);
            
            stopW1 = (cos(w0)-cos(stopW1))/sin(stopW1);
            stopW2 = (cos(w0)-cos(stopW2))/sin(stopW2);
            
            passW = MAX(passW1, passW2);                    //通带截止频率
            stopW = MIN(stopW1, stopW2);                    //阻带截止频率
            wa = fabs(stopW/passW);
            
            break;
            
        case FILTER_IIR_BUTTER_STOP:
            passF2 = passF_beta;
            stopF2 = stopF_beta;
            if(!(passF1 < stopF1 && stopF1 < stopF2 && stopF2 < passF2))
            {
                nAndFc.isFOK = false;
                //NSLog(@"错误！应满足：passF[1] < stopF[1] < stopF[2] < passF[2]");
                return nAndFc;
            }
            
            nAndFc.isFOK = true;
            //转换为数字频率（不进行预畸）
            passW1 = 2 * M_PI * passF1 / fs;
            passW2 = 2 * M_PI * passF2 / fs;
            stopW1 = 2 * M_PI * stopF1 / fs;
            stopW2 = 2 * M_PI * stopF2 / fs;
            
            nAndFc.cosW0 = cos((stopW1 + stopW2)/2)/cos((stopW1 - stopW2)/2); //保存cos(w0)
            w0 = acos(nAndFc.cosW0);//求带通滤波器的中心频率
            
            passW1 = sin(passW1)/(cos(passW1)-nAndFc.cosW0);  //通带截止频率
            passW2 = sin(passW2)/(cos(passW2)-nAndFc.cosW0);
            
            stopW1 = sin(stopW1)/(cos(stopW1)-nAndFc.cosW0);
            stopW2 = sin(stopW2)/(cos(stopW2)-nAndFc.cosW0);
            
            passW = MAX(passW1, passW2);                    //通带截止频率
            stopW = MIN(stopW1, stopW2);                    //阻带截止频率
            
            wa = fabs(stopW/passW);
            
            break;
            
        default:
            break;
    }
    nAndFc.fs = fs; //采样频率
    
		nAndFc.N =ceil((double)(0.5f * log10((pow(10.0f, 0.1f*rs)-1.0f)/(pow(10.0f, 0.1f*rp)-1.0f))/log10(wa))); //计算N
    
    nOfN = (float)nAndFc.N;   //将N转化为float型
    
    //根据滤波器类型，选择不同的预畸变换式
    switch (filterType) {
        case FILTER_IIR_BUTTER_LOW:
            wc = stopW / pow((pow(10.0f, 0.1f*rs) - 1.0f), 1.0f/(2*nOfN));
            nAndFc.fc = fs/M_PI*atan(wc);                         //计算截止频率(3dB)Hz
            
            nAndFc.length = nAndFc.N + 1; //系数数组长度
            
            break;
            
        case FILTER_IIR_BUTTER_HIGH:
            wc = stopW / pow((pow(10.0f, 0.1f*rs) - 1.0f), 1.0f/(2*nOfN));
            //wc = passW / pow((pow(10, 0.1*rp) - 1), 1/(2*nOfN));
            
            nAndFc.fc = fs/M_PI*atan(1/wc); //计算截止频率(3dB)Hz
            
            nAndFc.length = nAndFc.N + 1; //系数数组长度
            
            break;
            
        case FILTER_IIR_BUTTER_PASS:
            wc = stopW1 / pow((pow(10.0f, 0.1f*rs) - 1.0f), 1.0f/(2*nOfN));
            fc =asin((2*cos(w0)*wc + sqrt(pow(2*cos(w0)*wc, 2)-4*(wc*wc+1)*(cos(w0)*cos(w0)-1)))/(2*wc*wc+2));
            
//            wc = passW1 / pow((pow(10, 0.1*rp) - 1), 1/(2*nOfN));
//            fc =asin((2*cos(w0)*wc + sqrt(pow(2*cos(w0)*wc, 2)-4*(wc*wc+1)*(cos(w0)*cos(w0)-1)))/(2*wc*wc+2));
            
            nAndFc.fc = fs / (2*M_PI) * fc;
            
            nAndFc.length = 2 * nAndFc.N + 1; //系数数组长度
            
            break;
        
        case FILTER_IIR_BUTTER_STOP:
            wc = -1.0f/(stopW1 / pow((pow(10.0f, 0.1f*rs) - 1.0f), 1.0f/(2*nOfN)));
            fc =asin((2*cos(w0)*wc + sqrt(pow(2*cos(w0)*wc, 2)-4*(wc*wc+1)*(cos(w0)*cos(w0)-1)))/(2*wc*wc+2));
            
            nAndFc.fc = fs / (2*M_PI) * fc;
            
            nAndFc.length = 2 * nAndFc.N + 1; //系数数组长度
            break;
        default:
            break;
    }
    return nAndFc;
}



/*======================================================================
 * 方法名：  butterSbValue
 * 方法功能：计算巴特沃斯滤波器分母多项式H(s)的系数Sb，注意：分子为Wc^N
 * 说明：   Sb[k] = Wc^(N-k) * Pb，其中Pb是归一化的分母多项式的根，可查表得到
 *         系数由低次向高次排列
 *
 * 变量名称：
 *          butterValue   - 存放滤波器参数（阶数和截止频率）的结构体变量
 *          returnSb      - 计算结果
 *
 * 返回值：  void
 *=====================================================================*/
void butterSbValue(ButterFilterStruct *butterValue)
{
    int length = butterValue->N;        //滤波器阶数
    float Wc = 0.0;                   //滤波器的截止频率 
    //选择预畸方法
    switch (butterValue->filterType) {
        case FILTER_IIR_BUTTER_LOW:
            Wc = fabs(tan(butterValue->fc * M_PI / butterValue->fs));
            break;       
        case FILTER_IIR_BUTTER_HIGH:
            Wc = fabs(1/tan(butterValue->fc * M_PI / butterValue->fs));
            break;
        case FILTER_IIR_BUTTER_PASS:
            Wc = 2 * M_PI * butterValue->fc / butterValue->fs;
            Wc = fabs((butterValue->cosW0 - cos(Wc))/sin(Wc));
            break;
        case FILTER_IIR_BUTTER_STOP:
            Wc = 2 * M_PI * butterValue->fc / butterValue->fs;
            Wc = fabs(sin(Wc)/(cos(Wc) - butterValue->cosW0));      
            break;
        default:
            break;
    }  
    for(int i = 0; i < length; i++)
    {
        butterValue->sbvalue[i] = g_butterPb[length - 1][i] * pow(Wc, length-i); //计算系数
    }  
    butterValue->sbvalue[length] = 1.0; //最高次幂的系数为1
}




/*======================================================================
 * 方法名：  butterLowOrHigh
 * 方法功能：计算巴特沃斯低通（高通）滤波器系统方法的系数，包括分子和分母系数
 *
 * 变量名称：
 *          butterValue   - 存放滤波器参数（阶数和截止频率）的结构体变量
 *          sb            - 传入的模拟滤波器的系数，即H(s)的分母系数
 *          numerator     - 计算后的分子系数数组
 *          denominator   - 计算后的分母系数数组
 *
 * 返回值：  void
 *=====================================================================*/
void butterLowOrHigh(ButterFilterStruct *butterValue)
{
    int length = butterValue->N;    //滤波器阶数   
    int tempCoef1[20];//[length + 1];     //定义系数数组，用于存放1 - z^(-1)、1 + z^(-1)每项次幂（0-N）系数，最低次在第一个
    int tempCoef2[20];//[length + 1];
	  //tempCoef1=(int *)malloc(sizeof(int)*(length + 1));
	  //tempCoef2=(int *)malloc(sizeof(int)*(length + 1));
		//free(tempCoef1);
		///free(tempCoef2);
    int otherN;                    //1+z^(-1)的次数
    float Fsx2 = 1;//butterValue.fs * 2; //计算2/T  
    for(int i = 0; i<= length; i++)
    {
        butterValue->num[i] = 0.0;   //初始化numerator和denominator
        butterValue->den[i] = 0.0;
    }    
    for(int i = 0; i <= length; i++)
    {
        for(int j = 0; j<= length; j++)
        {
            tempCoef1[j] = 0;     //tempCoef1和tempCoef2进行初始化
            tempCoef2[j] = 0;
        }        
        otherN = length - i;
        if(butterValue->filterType == FILTER_IIR_BUTTER_LOW)
        {
					   pascalTriangle(i,SYMBOL_SUB,tempCoef1);//利用杨辉三角计算1 - z^(-1)幂的系数
             pascalTriangle(otherN,SYMBOL_ADD,tempCoef2);////利用杨辉三角计算1 + z^(-1)幂的系数
        }
        else
        {
						 pascalTriangle(i,SYMBOL_ADD,tempCoef1); //利用杨辉三角计算1 + z^(-1)幂的系数
             pascalTriangle(otherN,SYMBOL_SUB,tempCoef2);//利用杨辉三角计算1 - z^(-1)幂的系数
        }        
        coefficientEquation(tempCoef1,i+1,tempCoef2,otherN+1); //两个多项式相乘，求其系数 	
        for(int j = 0; j <= length; j++)
        {
            butterValue->den[j] += pow(Fsx2, i) * (float)tempCoef1[length - j] * butterValue->sbvalue[i];
        }      
        //分子系数
        if(i == 0)
        {
            for(int j = 0; j <= length; j++)
            {
                butterValue->num[j] = butterValue->sbvalue[0] * tempCoef2[length - j];
            }
        }
    } 
    //系数归一化，分母的常数项为1
    for(int i = length; i >= 0; i--)
    {
        butterValue->num[i] = butterValue->num[i] / butterValue->den[0];
        butterValue->den[i] = butterValue->den[i] / butterValue->den[0];
    }
		//free(tempCoef1);
		//free(tempCoef2);
}


/*======================================================================
 * 方法名：  butterPassOrStop
 * 方法功能：计算巴特沃斯带通（带阻）滤波器系统方法的系数，包括分子和分母系数
 *
 * 变量名称：
 *          butterValue   - 存放滤波器参数（阶数和截止频率）的结构体变量
 *          sb            - 传入的模拟滤波器的系数，即H(s)的分母系数
 *          numerator     - 计算后的分子系数数组
 *          denominator   - 计算后的分母系数数组
 *
 * 返回值：  void
 *=====================================================================*/
 
void butterPassOrStop(ButterFilterStruct butterValue,float *sb,float *numerator,float *denominator)
{
    int length = butterValue.length;      //滤波器系数长度   
    int tempCoef1[length];                //定义系数数组，用于存放1 - z^(-2)、1 - 2*cos(w0)*z^(-1) + z^(-2)每项次幂（0-N）系数，最低次在第一个
    float tempCoef2[length];
    float tempCoef3[length], tempCoef[3];
    int otherN;                           //1+z^(-1)的次数（pass）,1 - 2*cos(w0)*z^(-1) + z^(-2)的次数（stop）   
    float Fsx2 = 1;//butterValue.fs * 2;  //计算2/T = 1    
    for(int i = 0; i < length; i++)
    {
        numerator[i] = 0.0;   //初始化numerator和denominator
        denominator[i] = 0.0;
        tempCoef1[i] = 0;     //tempCoef1和tempCoef2进行初始化
        tempCoef2[i] = 0.0;
        tempCoef3[i] = 0.0;
    }    
    tempCoef[0] = 1.0;
    tempCoef[1] = -2.0 * butterValue.cosW0;
    tempCoef[2] = 1.0;    
    //----------计算分子系数-----------
    if(butterValue.filterType == FILTER_IIR_BUTTER_PASS) //带通滤波器
    {
			  pascalTriangle(butterValue.N,SYMBOL_SUB,tempCoef1);//利用杨辉三角计算1 - z^(-1)幂的系数       
        for(int i = 0; i < length; i++)  //变为1 - z^(-2)幂的系数，填充奇次幂0
        {
            int temp = i%2;  //判断i奇偶
            if(!temp)        //偶次幂不为0
                numerator[i] = sb[0] * tempCoef1[butterValue.N - i/2];
            else
                numerator[i] = 0.0;
        }
    }
    else //带阻滤波器
    {
        tempCoef3[0] = 1.0;                       //1 - 2*cos(w0)*z^(-1) + z^(-2)的系数1,-2cos(w0),1
        tempCoef3[1] = -2.0 * butterValue.cosW0;
        tempCoef3[2] = 1.0;
        
        for(int j = 1; j < butterValue.N; j++)
        {
					   coefficientEquation2(tempCoef3,j*2+1,tempCoef,3);
        }
        for(int i = 0; i < length; i++)
        {
            numerator[i] = sb[0] * tempCoef3[length - i - 1];
        }
    }
    
    //----------计算分母系数,计算每一加数的系数----------
    for(int i = 0; i <= butterValue.N; i++)
    {
        if(butterValue.filterType == FILTER_IIR_BUTTER_PASS)
        {
            otherN = butterValue.N - i;
        }
        else
        {
            otherN = i;
        }
        
        for(int j = 0; j < length; j++)
        {
            tempCoef1[j] = 0;     //tempCoef1、tempCoef2和tempCoef3进行初始化
            tempCoef2[j] = 0.0;
            tempCoef3[j] = 0.0;
        }
        tempCoef3[0] = 1.0;
        if(butterValue.N - otherN > 0) //当第0次相乘时，第一项为1，其余为0
        {
            tempCoef3[1] = -2.0 * butterValue.cosW0;
            tempCoef3[2] = 1.0;
        }
 
        pascalTriangle(otherN,SYMBOL_SUB,tempCoef1);//利用杨辉三角计算1 - z^(-1)幂的系数
        for(int j = 0; j < otherN*2+1; j++)  //变为1 - z^(-2)幂的系数，填充奇次幂0
        {
            int temp = j%2;  //判断i奇偶
            if(!temp)        //偶次幂不为0
            {
                tempCoef2[j] = (float)tempCoef1[j/2];
                tempCoef1[j/2] = 0;
            }
            else
                tempCoef2[j] = 0.0;
        }
        
        //利用多项式相乘法，计算1 - 2*cos(w0)*z^(-1) + z^(-2)幂的系数,j表示第几次相乘
        for(int j = 1; j < butterValue.N - otherN; j++)
        {
					  coefficientEquation2(tempCoef3,j*2+1,tempCoef,3);
        }
        
        coefficientEquation2(tempCoef3,(butterValue.N - otherN)*2+1,tempCoef2,2*otherN+1); //两个多项式相乘，求其系数
        for(int j = 0; j < length; j++)
        {
            denominator[j] += pow(Fsx2, i) * tempCoef3[length - j - 1] * sb[i];
        }
    }
    
    //系数归一化，分母的常数项为1
    for(int i = length - 1; i >= 0; i--)
    {
        numerator[i] = numerator[i] / denominator[0];
        denominator[i] = denominator[i] / denominator[0];
    }
}



 
 
/*======================================================================
 * 方法名：  filter
 * 方法功能：根据数字滤波器系统方法（系数），对原始信号进行滤波
 *
 * 变量名称：
 *          butterValue   - 存放滤波器参数（阶数和截止频率）的结构体变量
 *          numerator     - 系统方法，分子系数数组
 *          denominator   - 系统方法，分母系数数组
 *          xVector       - 输入的原始信号（数组）
 *          length        - 原始信号的长度，也是滤波后信号的长度
 *          yVector       - 滤波后的信号（数组）
 *
 * 返回值：  设计是否成功，true-成功，false-失败
 *=====================================================================*/
 
bool filter(ButterFilterStruct butterValue,float *numerator,float *denominator,float *xVector,int length,float *yVector)
{
    bool isFilterOK = false;  
    if(!butterValue.isFOK)
    {
        //NSLog(@"系统方法错误！");
        isFilterOK = false;
        return isFilterOK;
    }
    if(butterValue.N > 10)
    {
        //NSLog(@"失败！滤波器的阶数不能大于10。");
        isFilterOK = false;
        return isFilterOK;
    }
    int N = butterValue.length; //系数数组的长度 
    //返回值初始化
    for(int i = 0; i < length; i++)
    {
        yVector[i] = 0.0; //后面循环中用到y递归算法，需要提前初始化
    }   
    //第一层循环，计算length个y的输出值
    for(int i = 0; i < length; i++)
    {
        if(i == 0)
        {
            yVector[i] = numerator[i]*xVector[i];
        }
        else
        {
            yVector[i] = numerator[0]*xVector[i];
            //第二层循环，计算每个y的每一项
            for(int j = 1; j <= i && j < N; j++)
            {
                yVector[i] += numerator[j]*xVector[i-j] - denominator[j]*yVector[i-j];
            }
        }
        yVector[i] /= denominator[0];
    } 
    isFilterOK = true;
    return isFilterOK;
}

/*======================================================================
 * 方法名：  filter
 * 方法功能：根据数字滤波器系统方法（系数），对原始信号进行滤波
 *
 * 变量名称：
 *          butterValue   - 存放滤波器参数（阶数和截止频率）的结构体变量
 *          numerator     - 系统方法，分子系数数组
 *          denominator   - 系统方法，分母系数数组
 *          xVector       - 输入的原始信号（数组）
 *          length        - 原始信号的长度，也是滤波后信号的长度
 *          yVector       - 滤波后的信号（数组）
 *
 * 返回值：  设计是否成功，true-成功，false-失败
 *=====================================================================*/
 
bool IIR_High_Order_Filter(ButterFilterStruct *butterValue,float data)
{
    bool isFilterOK = false;  
    if(!butterValue->isFOK)
    {
        //NSLog(@"系统方法错误！");
        isFilterOK = false;
        return isFilterOK;
    }
    if(butterValue->N > 10)
    {
        //NSLog(@"失败！滤波器的阶数不能大于10。");
        isFilterOK = false;
        return isFilterOK;
    }
    int N = butterValue->length; //系数数组的长度 
    butterValue->input[N-1]=data;
		
    //第一层循环，计算length个y的输出值
    butterValue->output[N-1]=0;
		 for(int i = 0; i < N; i++)
    {
      butterValue->output[N-1]+=butterValue->num[i]*butterValue->input[N-1-i];
    }
     for(int i = 1; i < N; i++)
    {
      butterValue->output[N-1]-=butterValue->den[i]*butterValue->output[N-1-i];
    }
		for(int i = 0; i < N; i++)
		{
		  butterValue->output[i]=butterValue->output[i+1];
			butterValue->input[i] =butterValue->input[i+1];
		}
		
		for(int i = 0; i <=N; i++)
		{
			if(isnan(butterValue->output[i])==1)
			{
				for(int j = 0; j<=N; j++)
				{
					butterValue->output[j]=data;
				}
			}
	 }		
    isFilterOK = true;
    return isFilterOK;
}
float iir_high_order_filter_both_export(ButterFilterStruct *butterValue,float data)
{
	float output=data;
	if(IIR_High_Order_Filter(butterValue,data)) output=butterValue->output[butterValue->N-1];
	return output;
}


void iir_high_order_filter_init(ButterFilterStruct *butterValue,
															  float passF_alpha,
															  float passF_beta,
															  float stopF_alpha,
															  float stopF_beta,
															  float rp,
															  float rs,
															  float fs,
															  int filterType)
{
	*butterValue=filterIIRButterLowpass(passF_alpha,passF_beta,stopF_alpha,stopF_beta,rp,rs,fs,filterType);
	butterSbValue(butterValue);
	butterLowOrHigh(butterValue);
}

