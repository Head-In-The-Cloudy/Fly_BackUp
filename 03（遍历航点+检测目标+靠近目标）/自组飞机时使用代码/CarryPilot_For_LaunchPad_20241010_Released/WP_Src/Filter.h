#ifndef _FILTER_H_
#define _FILTER_H_

#define N2 3


#define FILTER_IIR_BUTTER_LOW  0
#define FILTER_IIR_BUTTER_HIGH 1
#define FILTER_IIR_BUTTER_PASS 2
#define FILTER_IIR_BUTTER_STOP 3


typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}lpf_buf;


typedef struct
{
 float Output_Butter[3];
}Notch_Filter_BufferData;


typedef struct
{
	float cf;
  float a[3];
  float b[3];
}lpf_param;



//巴特沃斯滤波器参数
typedef struct
{
    int N;          //巴特沃斯滤波器最小实现阶数
    int length;     //滤波器系统函数系数数组的长度
    float fc;       //巴特沃斯滤波器截止频率
    float cosW0;    //中心频率，带通带阻时用到
    float fs;       //采样频率
    int filterType; //需要设计的数字滤波器类型 
	  float pass[2];
	  float stop[2];
	  float rp ;//通带衰减，典型值2dB
	  float rs ;//阻带衰减，典型值20dB
	  float sbvalue[11];
	  float num[11];
	  float den[11];
	  float input[11];
	  float output[11];
	  bool isFOK;
}ButterFilterStruct;


void Tes(void);

extern lpf_param Butter_5HZ_Parameter_RC;


extern lpf_buf Butterworth_Buffer_Baro,Butterworth_Buffer_Baro_Acc;

extern lpf_param loam_ft;

float set_lpf_alpha(int16_t cutoff_frequency, float time_step);
void Acce_Control_Filter(void);
float LPButterworth(float curr_input,lpf_buf *Buffer,lpf_param *Parameter);


void set_cutoff_frequency(float sample_frequent, float cutoff_frequent,lpf_param *LPF);
void Butterworth_Parameter_Init(void);
void Test_Filter(void);
float BPF_Butterworth(float curr_input,lpf_buf *Buffer,lpf_param *Parameter);


void pascalTriangle(int N,int symbol,int *vector);
bool filter(ButterFilterStruct butterValue,float *numerator,float *denominator,float *xVector,int length,float *yVector);
bool IIR_High_Order_Filter(ButterFilterStruct *butterValue,float data);
void IIR_High_Order_Init(void);


void iir_high_order_filter_init(ButterFilterStruct *butterValue,
															  float passF_alpha,
															  float passF_beta,
															  float stopF_alpha,
															  float stopF_beta,
															  float rp,
															  float rs,
															  float fs,
															  int filterType);
																
float iir_high_order_filter_both_export(ButterFilterStruct *butterValue,float data);
																

float GildeAverageValueFilter_MAG(float NewValue,float *Data);
#endif


