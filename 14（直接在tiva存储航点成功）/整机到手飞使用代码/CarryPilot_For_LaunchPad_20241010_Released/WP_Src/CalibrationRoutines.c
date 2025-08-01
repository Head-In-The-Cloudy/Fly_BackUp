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
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
* calibrationRoutines.cpp
*
*  Created on: 30/set/2013
*      Author: Murtas Matteo
*/
/****************************************************************************
*
*   Copyright (C) 2012 PX4 Development Team. All rights reserved.
*   Author: Lorenz Meier <lm@inf.ethz.ch>
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
*    used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/
#include "CalibrationRoutines.h"
#include <math.h>
#include <string.h>
int sphere_fit_least_squares(const float x[], const float y[], const float z[],
                             unsigned int size, unsigned int max_iterations, float delta, float *sphere_x, float *sphere_y, float *sphere_z, float *sphere_radius)
{
  
  float x_sumplain = 0.0f;
  float x_sumsq = 0.0f;
  float x_sumcube = 0.0f;
  
  float y_sumplain = 0.0f;
  float y_sumsq = 0.0f;
  float y_sumcube = 0.0f;
  
  float z_sumplain = 0.0f;
  float z_sumsq = 0.0f;
  float z_sumcube = 0.0f;
  
  float xy_sum = 0.0f;
  float xz_sum = 0.0f;
  float yz_sum = 0.0f;
  
  float x2y_sum = 0.0f;
  float x2z_sum = 0.0f;
  float y2x_sum = 0.0f;
  float y2z_sum = 0.0f;
  float z2x_sum = 0.0f;
  float z2y_sum = 0.0f;
  
  for (unsigned int i = 0; i < size; i++) {
    
    float x2 = x[i] * x[i];
    float y2 = y[i] * y[i];
    float z2 = z[i] * z[i];
    
    x_sumplain += x[i];
    x_sumsq += x2;
    x_sumcube += x2 * x[i];
    
    y_sumplain += y[i];
    y_sumsq += y2;
    y_sumcube += y2 * y[i];
    
    z_sumplain += z[i];
    z_sumsq += z2;
    z_sumcube += z2 * z[i];
    
    xy_sum += x[i] * y[i];
    xz_sum += x[i] * z[i];
    yz_sum += y[i] * z[i];
    
    x2y_sum += x2 * y[i];
    x2z_sum += x2 * z[i];
    
    y2x_sum += y2 * x[i];
    y2z_sum += y2 * z[i];
    
    z2x_sum += z2 * x[i];
    z2y_sum += z2 * y[i];
  }
  
  //
  //Least Squares Fit a sphere A,B,C with radius squared Rsq to 3D data
  //
  //    P is a structure that has been computed with the data earlier.
  //    P.npoints is the number of elements; the length of X,Y,Z are identical.
  //    P's members are logically named.
  //
  //    X[n] is the x component of point n
  //    Y[n] is the y component of point n
  //    Z[n] is the z component of point n
  //
  //    A is the x coordiante of the sphere
  //    B is the y coordiante of the sphere
  //    C is the z coordiante of the sphere
  //    Rsq is the radius squared of the sphere.
  //
  //This method should converge; maybe 5-100 iterations or more.
  //
  float x_sum = x_sumplain / size;        //sum( X[n] )
  float x_sum2 = x_sumsq / size;    //sum( X[n]^2 )
  float x_sum3 = x_sumcube / size;    //sum( X[n]^3 )
  float y_sum = y_sumplain / size;        //sum( Y[n] )
  float y_sum2 = y_sumsq / size;    //sum( Y[n]^2 )
  float y_sum3 = y_sumcube / size;    //sum( Y[n]^3 )
  float z_sum = z_sumplain / size;        //sum( Z[n] )
  float z_sum2 = z_sumsq / size;    //sum( Z[n]^2 )
  float z_sum3 = z_sumcube / size;    //sum( Z[n]^3 )
  
  float XY = xy_sum / size;        //sum( X[n] * Y[n] )
  float XZ = xz_sum / size;        //sum( X[n] * Z[n] )
  float YZ = yz_sum / size;        //sum( Y[n] * Z[n] )
  float X2Y = x2y_sum / size;    //sum( X[n]^2 * Y[n] )
  float X2Z = x2z_sum / size;    //sum( X[n]^2 * Z[n] )
  float Y2X = y2x_sum / size;    //sum( Y[n]^2 * X[n] )
  float Y2Z = y2z_sum / size;    //sum( Y[n]^2 * Z[n] )
  float Z2X = z2x_sum / size;    //sum( Z[n]^2 * X[n] )
  float Z2Y = z2y_sum / size;    //sum( Z[n]^2 * Y[n] )
  
  //Reduction of multiplications
  float F0 = x_sum2 + y_sum2 + z_sum2;
  float F1 =  0.5f * F0;
  float F2 = -8.0f * (x_sum3 + Y2X + Z2X);
  float F3 = -8.0f * (X2Y + y_sum3 + Z2Y);
  float F4 = -8.0f * (X2Z + Y2Z + z_sum3);
  
  //Set initial conditions:
  float A = x_sum;
  float B = y_sum;
  float C = z_sum;
  
  //First iteration computation:
  float A2 = A * A;
  float B2 = B * B;
  float C2 = C * C;
  float QS = A2 + B2 + C2;
  float QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);
  
  //Set initial conditions:
  float Rsq = F0 + QB + QS;
  
  //First iteration computation:
  float Q0 = 0.5f * (QS - Rsq);
  float Q1 = F1 + Q0;
  float Q2 = 8.0f * (QS - Rsq + QB + F0);
  float aA, aB, aC, nA, nB, nC, dA, dB, dC;
  
  //Iterate N times, ignore stop condition.
  unsigned int n = 0;
  
  while (n < max_iterations) {
    n++;
    
    //Compute denominator:
    aA = Q2 + 16.0f * (A2 - 2.0f * A * x_sum + x_sum2);
    aB = Q2 + 16.0f * (B2 - 2.0f * B * y_sum + y_sum2);
    aC = Q2 + 16.0f * (C2 - 2.0f * C * z_sum + z_sum2);
    aA = (aA == 0.0f) ? 1.0f : aA;
    aB = (aB == 0.0f) ? 1.0f : aB;
    aC = (aC == 0.0f) ? 1.0f : aC;
    
    //Compute next iteration
    nA = A - ((F2 + 16.0f * (B * XY + C * XZ + x_sum * (-A2 - Q0) + A * (x_sum2 + Q1 - C * z_sum - B * y_sum))) / aA);
    nB = B - ((F3 + 16.0f * (A * XY + C * YZ + y_sum * (-B2 - Q0) + B * (y_sum2 + Q1 - A * x_sum - C * z_sum))) / aB);
    nC = C - ((F4 + 16.0f * (A * XZ + B * YZ + z_sum * (-C2 - Q0) + C * (z_sum2 + Q1 - A * x_sum - B * y_sum))) / aC);
    
    //Check for stop condition
    dA = (nA - A);
    dB = (nB - B);
    dC = (nC - C);
    
    if ((dA * dA + dB * dB + dC * dC) <= delta) { break; }
    
    //Compute next iteration's values
    A = nA;
    B = nB;
    C = nC;
    A2 = A * A;
    B2 = B * B;
    C2 = C * C;
    QS = A2 + B2 + C2;
    QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);
    Rsq = F0 + QB + QS;
    Q0 = 0.5f * (QS - Rsq);
    Q1 = F1 + Q0;
    Q2 = 8.0f * (QS - Rsq + QB + F0);
  }
  
  *sphere_x = A;
  *sphere_y = B;
  *sphere_z = C;
  *sphere_radius = sqrtf(Rsq);
  
  return 0;
}





//original source of：https://github.com/stevenjiaweixie/vrgimbal
//https://github.com/stevenjiaweixie/vrgimbal/blob/master/Firmware/VRGimbal/calibrationRoutines.cpp
//来源于开源云台项目：https://vrgimbal.wordpress.com/
Least_Squares_Intermediate_Variable Mag_LS;
void LS_Init(Least_Squares_Intermediate_Variable * pLSQ)
{
  memset(pLSQ, 0, sizeof(Least_Squares_Intermediate_Variable));
}

unsigned int LS_Accumulate(Least_Squares_Intermediate_Variable * pLSQ, float x, float y, float z)
{
  float x2 = x * x;
  float y2 = y * y;
  float z2 = z * z;
  
  pLSQ->x_sumplain += x;
  pLSQ->x_sumsq += x2;
  pLSQ->x_sumcube += x2 * x;
  
  pLSQ->y_sumplain += y;
  pLSQ->y_sumsq += y2;
  pLSQ->y_sumcube += y2 * y;
  
  pLSQ->z_sumplain += z;
  pLSQ->z_sumsq += z2;
  pLSQ->z_sumcube += z2 * z;
  
  pLSQ->xy_sum += x * y;
  pLSQ->xz_sum += x * z;
  pLSQ->yz_sum += y * z;
  
  pLSQ->x2y_sum += x2 * y;
  pLSQ->x2z_sum += x2 * z;
  
  pLSQ->y2x_sum += y2 * x;
  pLSQ->y2z_sum += y2 * z;
  
  pLSQ->z2x_sum += z2 * x;
  pLSQ->z2y_sum += z2 * y;
  
  pLSQ->size++;
  
  return pLSQ->size;
}


void LS_Calculate(Least_Squares_Intermediate_Variable * pLSQ,
                  unsigned int max_iterations,
                  float delta,
                  float *sphere_x, float *sphere_y, float *sphere_z,
                  float *sphere_radius)
{
  //
  //Least Squares Fit a sphere A,B,C with radius squared Rsq to 3D data
  //
  //    P is a structure that has been computed with the data earlier.
  //    P.npoints is the number of elements; the length of X,Y,Z are identical.
  //    P's members are logically named.
  //
  //    X[n] is the x component of point n
  //    Y[n] is the y component of point n
  //    Z[n] is the z component of point n
  //
  //    A is the x coordiante of the sphere
  //    B is the y coordiante of the sphere
  //    C is the z coordiante of the sphere
  //    Rsq is the radius squared of the sphere.
  //
  //This method should converge; maybe 5-100 iterations or more.
  //
  float x_sum = pLSQ->x_sumplain / pLSQ->size;        //sum( X[n] )
  float x_sum2 = pLSQ->x_sumsq / pLSQ->size;    //sum( X[n]^2 )
  float x_sum3 = pLSQ->x_sumcube / pLSQ->size;    //sum( X[n]^3 )
  float y_sum = pLSQ->y_sumplain / pLSQ->size;        //sum( Y[n] )
  float y_sum2 = pLSQ->y_sumsq / pLSQ->size;    //sum( Y[n]^2 )
  float y_sum3 = pLSQ->y_sumcube / pLSQ->size;    //sum( Y[n]^3 )
  float z_sum = pLSQ->z_sumplain / pLSQ->size;        //sum( Z[n] )
  float z_sum2 = pLSQ->z_sumsq / pLSQ->size;    //sum( Z[n]^2 )
  float z_sum3 = pLSQ->z_sumcube / pLSQ->size;    //sum( Z[n]^3 )
  
  float XY = pLSQ->xy_sum / pLSQ->size;        //sum( X[n] * Y[n] )
  float XZ = pLSQ->xz_sum / pLSQ->size;        //sum( X[n] * Z[n] )
  float YZ = pLSQ->yz_sum / pLSQ->size;        //sum( Y[n] * Z[n] )
  float X2Y = pLSQ->x2y_sum / pLSQ->size;    //sum( X[n]^2 * Y[n] )
  float X2Z = pLSQ->x2z_sum / pLSQ->size;    //sum( X[n]^2 * Z[n] )
  float Y2X = pLSQ->y2x_sum / pLSQ->size;    //sum( Y[n]^2 * X[n] )
  float Y2Z = pLSQ->y2z_sum / pLSQ->size;    //sum( Y[n]^2 * Z[n] )
  float Z2X = pLSQ->z2x_sum / pLSQ->size;    //sum( Z[n]^2 * X[n] )
  float Z2Y = pLSQ->z2y_sum / pLSQ->size;    //sum( Z[n]^2 * Y[n] )
  
  //Reduction of multiplications
  float F0 = x_sum2 + y_sum2 + z_sum2;
  float F1 =  0.5f * F0;
  float F2 = -8.0f * (x_sum3 + Y2X + Z2X);
  float F3 = -8.0f * (X2Y + y_sum3 + Z2Y);
  float F4 = -8.0f * (X2Z + Y2Z + z_sum3);
  
  //Set initial conditions:
  float A = x_sum;
  float B = y_sum;
  float C = z_sum;
  
  //First iteration computation:
  float A2 = A * A;
  float B2 = B * B;
  float C2 = C * C;
  float QS = A2 + B2 + C2;
  float QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);
  
  //Set initial conditions:
  float Rsq = F0 + QB + QS;
  
  //First iteration computation:
  float Q0 = 0.5f * (QS - Rsq);
  float Q1 = F1 + Q0;
  float Q2 = 8.0f * (QS - Rsq + QB + F0);
  float aA, aB, aC, nA, nB, nC, dA, dB, dC;
  
  //Iterate N times, ignore stop condition.
  unsigned int n = 0;
  
  while (n < max_iterations) {
    n++;
    
    //Compute denominator:
    aA = Q2 + 16.0f * (A2 - 2.0f * A * x_sum + x_sum2);
    aB = Q2 + 16.0f * (B2 - 2.0f * B * y_sum + y_sum2);
    aC = Q2 + 16.0f * (C2 - 2.0f * C * z_sum + z_sum2);
    aA = (aA == 0.0f) ? 1.0f : aA;
    aB = (aB == 0.0f) ? 1.0f : aB;
    aC = (aC == 0.0f) ? 1.0f : aC;
    
    //Compute next iteration
    nA = A - ((F2 + 16.0f * (B * XY + C * XZ + x_sum * (-A2 - Q0) + A * (x_sum2 + Q1 - C * z_sum - B * y_sum))) / aA);
    nB = B - ((F3 + 16.0f * (A * XY + C * YZ + y_sum * (-B2 - Q0) + B * (y_sum2 + Q1 - A * x_sum - C * z_sum))) / aB);
    nC = C - ((F4 + 16.0f * (A * XZ + B * YZ + z_sum * (-C2 - Q0) + C * (z_sum2 + Q1 - A * x_sum - B * y_sum))) / aC);
    
    //Check for stop condition
    dA = (nA - A);
    dB = (nB - B);
    dC = (nC - C);
    
    if ((dA * dA + dB * dB + dC * dC) <= delta) { break; }
    
    //Compute next iteration's values
    A = nA;
    B = nB;
    C = nC;
    A2 = A * A;
    B2 = B * B;
    C2 = C * C;
    QS = A2 + B2 + C2;
    QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);
    Rsq = F0 + QB + QS;
    Q0 = 0.5f * (QS - Rsq);
    Q1 = F1 + Q0;
    Q2 = 8.0f * (QS - Rsq + QB + F0);
  }
  
  *sphere_x = A;
  *sphere_y = B;
  *sphere_z = C;
  *sphere_radius = sqrt(Rsq);
}


/*****************************************************************************************************/
//void update_matrix(float dS[4],
//                   float JS[4][4],
//                   float beta[4],
//                   float data[3])
//{
//	int16_t j, k;
//	float residual = 1.0;
//	float jacobian[4];
//	//计算当前输入量的残差
//	residual=(((data[0]-beta[0])*(data[0]-beta[0])
//				    +(data[1]-beta[1])*(data[1]-beta[1])
//				    +(data[2]-beta[2])*(data[2]-beta[2]))
//				    - beta[3]*beta[3]);
//	//计算雅可比矩阵
//	jacobian[0]= -2*(data[0]-beta[0]);
//	jacobian[1]= -2*(data[1]-beta[1]);
//	jacobian[2]= -2*(data[2]-beta[2]);
//	jacobian[3]= -2* beta[3];
//  //计算H矩阵和-J*f(x)
//	for(j=0;j<4;j++)
//	{
//		dS[j]-= jacobian[j]*residual;
//		for(k=0;k<4;k++)	JS[j][k]+=jacobian[j]*jacobian[k];
//	}
//}

//#define N 4
//static void gaussElimination(float mat_Y[N],float mat_A[N][N],float x[N]) 
//{
//	double mat[N][N + 1];
//	for(uint16_t i=0;i<N;i++)
//	{
//		for(uint16_t j=0;j<N+1;j++)	
//		{
//			if(j!=N)  mat[i][j]=mat_A[i][j];
//		  else mat[i][j]=mat_Y[i];
//		}
//	}
//	
//	for (int i = 0; i < N; i++) {
//			// Making the diagonal element non-zero
//			if (mat[i][i] == 0) {
//					for (int k = i + 1; k < N; k++) {
//							if (mat[k][i] != 0) {
//									for (int j = 0; j <= N; j++) {
//											double temp = mat[i][j];
//											mat[i][j] = mat[k][j];
//											mat[k][j] = temp;
//									}
//									break;
//							}
//					}
//			}

//			// Making the elements below the diagonal zero
//			for (int k = i + 1; k < N; k++) {
//					double factor = mat[k][i] / mat[i][i];
//					for (int j = 0; j <= N; j++) {
//							mat[k][j] -= factor * mat[i][j];
//					}
//			}
//	}
//	// mat是高斯消元后的矩阵
//	for (int i = N - 1; i >= 0; i--) {
//			x[i] = mat[i][N];
//			for (int j = i + 1; j < N; j++) {
//					x[i] -= mat[i][j] * x[j];
//			}
//			x[i] /= mat[i][i];
//	}		
//}

//void sphere_fit_namelesstech(sphere_fit *p,uint16_t number,float x[],float y[],float z[])
//{
//  int16_t num_iterations = 0;
//  float eps = 0.000000001;
//  float change = 100.0;
//  float data[3]={0};
//  // reset
//  p->beta[0] = p->beta[1] = p->beta[2] = 0;
//  p->beta[3] = 1.0f;
//	while( num_iterations < 100 && change > eps )
//	{
//		num_iterations++;
//		int16_t i,j,k;
//		for( j=0; j<N; j++ )
//		{
//			p->ds[j] = 0.0f;
//			for( k=0; k<N; k++ )	p->js[j][k] = 0.0f;
//		}
//		
//		for(uint16_t i=0;i<number;i++)
//		{
//      data[0]=x[i];
//			data[1]=y[i];
//			data[2]=z[i];
//			update_matrix(p->ds,p->js,p->beta,data);
//		}
//		gaussElimination(p->ds, p->js, p->delta);
//    change=p->delta[0]*p->delta[0]+p->delta[1]*p->delta[1]+p->delta[2]*p->delta[2]+p->delta[3]*p->delta[3];
//    for(i=0;i<4;i++) 
//		{
//      p->beta[i]+=p->delta[i];
//    }
//	}
//	
//	p->a=p->beta[0];
//	p->b=p->beta[1];
//	p->c=p->beta[2];
//	p->r=p->beta[3];
//}

//spf_data mag_sample;
//sphere_fit spf;
//if(last_mag_corner_flag[0][i]==0&&mag_corner_flag[0][i]==1)
//{
//	mag_sample.x[0][i]=magdata->x;
//	mag_sample.y[0][i]=magdata->y;
//	mag_sample.z[0][i]=magdata->z;		
//	mag_sample.axis[0][i]='z';
//	LS_Accumulate(&Mag_LS, magdata->x,magdata->y,magdata->z);
//}
//if(last_mag_corner_flag[1][i]==0&&mag_corner_flag[1][i]==1)
//{
//	mag_sample.x[1][i]=magdata->x;
//	mag_sample.y[1][i]=magdata->y;
//	mag_sample.z[1][i]=magdata->z;			
//	mag_sample.axis[1][i]='y';
//	LS_Accumulate(&Mag_LS, magdata->x,magdata->y,magdata->z);
//}			
//if(last_mag_corner_flag[2][i]==0&&mag_corner_flag[2][i]==1)
//{
//	mag_sample.x[2][i]=magdata->x;
//	mag_sample.y[2][i]=magdata->y;
//	mag_sample.z[2][i]=magdata->z;			
//	mag_sample.axis[2][i]='x';
//	LS_Accumulate(&Mag_LS, magdata->x,magdata->y,magdata->z);
//}			

//sphere_fit_namelesstech(&spf,12*3,mag_sample.x[0],mag_sample.y[0],mag_sample.z[0]);



