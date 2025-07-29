/*
The MIT License (MIT)

Copyright (c) 2015-? suhetao

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "Quaternion.h"
#include "WP_Math.h"


static float Fast_Abs(float x){
	union { unsigned int i; float f;} y;
	y.f = x;
	y.i = y.i & 0x7FFFFFFF;
	return (float)y.f;
}

void Quaternion_Add(float *r, float *a, float *b)
{
	r[0] = a[0] + b[0];
	r[1] = a[1] + b[1];
	r[2] = a[2] + b[2];
	r[3] = a[3] + b[3];
}

void Quaternion_Sub(float *r, float *a, float *b)
{
	r[0] = a[0] - b[0];
	r[1] = a[1] - b[1];
	r[2] = a[2] - b[2];
	r[3] = a[3] - b[3];
}

void Quaternion_Multiply(float *r, float *a, float *b)
{
	r[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
	r[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
	r[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
	r[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

void Quaternion_Conjugate(float *r, float *a)
{
	r[0] = a[0];
	r[1] = -a[1];
	r[2] = -a[2];
	r[3] = -a[3];
}

void Quaternion_Scalar(float *r, float *q, float scalar)
{
	r[0] = q[0] * scalar;
	r[1] = q[1] * scalar;
	r[2] = q[2] * scalar;
	r[3] = q[3] * scalar;
}


void Quaternion_Normalize(float *q)
{
	float norm = FastSqrtI(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= norm;
	q[1] *= norm;
	q[2] *= norm;
	q[3] *= norm;
}

void Quaternion_FromEuler(float *q, float *rpy)
{
	float sPhi2, cPhi2; // sin(phi/2) and cos(phi/2)
	float sThe2, cThe2; // sin(theta/2) and cos(theta/2)
	float sPsi2, cPsi2; // sin(psi/2) and cos(psi/2)
	// calculate sines and cosines
	
	FastSinCos(0.5f * rpy[0], &sPhi2, &cPhi2);//roll
	FastSinCos(0.5f * rpy[1], &sThe2, &cThe2);//pitch
	FastSinCos(0.5f * rpy[2], &sPsi2, &cPsi2);//yaw
	
	// compute the quaternion elements
	q[0] = cPhi2 * cThe2 * cPsi2 + sPhi2 * sThe2 * sPsi2 ;
	q[1] = sPhi2 * cThe2 * cPsi2 - cPhi2 * sThe2 * sPsi2 ;
	q[2] = cPhi2 * sThe2 * cPsi2 + sPhi2 * cThe2 * sPsi2 ;
	q[3] = cPhi2 * cThe2 * sPsi2 - sPhi2 * sThe2 * cPsi2 ;
	
	
}

void quaternion_from_euler(float *q, float *rpy)
{
    double c1, c2, c3, s1, s2, s3;    
	  c1 = FastCos(rpy[0] / 2);//roll
    c2 = FastCos(rpy[1] / 2);//pitch
    c3 = FastCos(rpy[2] / 2);//yaw
    s1 = FastSin(rpy[0] / 2);
    s2 = FastSin(rpy[1] / 2);
    s3 = FastSin(rpy[2] / 2);

    q[0] = c1 * c2 * c3 + s1 * s2 * s3;
    q[1] = s1 * c2 * c3 - c1 * s2 * s3;
    q[2] = c1 * s2 * c3 + s1 * c2 * s3;    
    q[3] = c1 * c2 * s3 - s1 * s2 * c3;
}

void quaternion_from_dcm(float *R, float *Q)
{
  Q[0]=0.5f*FastSqrt(1+R[0]+R[4]+R[8]);
	Q[1]=(R[7]-R[5])/(4*Q[0]);
	Q[2]=(R[2]-R[6])/(4*Q[0]);
	Q[3]=(R[3]-R[1])/(4*Q[0]);
	Quaternion_Normalize(Q);
}


float q_max=0;
unsigned char index=0;
void quaternion_from_dcm_accurate(float *R, float *Q)
{
	float u[4];
	unsigned char i=0;
	u[0]=1+R[0]+R[4]+R[8];
	u[1]=1+R[0]-R[4]-R[8];
	u[2]=1-R[0]+R[4]-R[8];
	u[3]=1-R[0]-R[4]+R[8];
	q_max=u[0];
	for(i=1;i<4;i++)
	{
	  if(q_max<u[i]) 
		{
			q_max=u[i];
			index=i;
		}			
	}
	switch(index)
	{
	  case 0:
		{
			Q[0]=u[0];
			Q[1]=R[7]-R[5];
			Q[2]=R[2]-R[6];
			Q[3]=R[3]-R[1];
		}
		break;
	  case 1:
		{
			Q[0]=R[7]-R[5];
			Q[1]=u[1];
			Q[2]=R[1]+R[3];
			Q[3]=R[2]+R[6];	
		}
		break;	
	  case 2:
		{
			Q[0]=R[2]-R[6];
			Q[1]=R[1]+R[3];
			Q[2]=u[2];
			Q[3]=R[5]+R[7];		
		}
		break;	
	  case 3:
		{
			Q[0]=R[3]-R[1];
			Q[1]=R[2]+R[6];
			Q[2]=R[5]+R[7];
			Q[3]=u[3];		
		}
		break;	
	}
	Quaternion_Normalize(Q);
}





void Quaternion_ToEuler(float *q, float* rpy)
{
	float R[3][3];
	//Z-Y-X
	R[0][0] = 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f;
	R[0][1] = 2.0f * (q[1] * q[2] + q[0] * q[3]);
	R[0][2] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
	//R[1][0] = 2.0f * (q[1] * q[2] - q[0] * q[3]);
	//R[1][1] = 2.0f * (q[0] * q[0] + q[2] * q[2]) - 1.0f;
	R[1][2] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
	//R[2][0] = 2.0f * (q[1] * q[3] + q[0] * q[2]);
	//R[2][1] = 2.0f * (q[2] * q[3] - q[0] * q[1]);
	R[2][2] = 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f;

	//roll
	rpy[0] = FastAtan2(R[1][2], R[2][2]);
	if (rpy[0] == PI)
		rpy[0] = -PI_2;
	//pitch
	if (R[0][2] >= 1.0f)
		rpy[1] = -PI_2;
	else if (R[0][2] <= -1.0f)
		rpy[1] = PI_2;
	else
		rpy[1] = FastAsin(-R[0][2]);
	//yaw
	rpy[2] = FastAtan2(R[0][1], R[0][0]);
	if (rpy[2] < 0.0f){
		rpy[2] += _2_PI;
	}
	if (rpy[2] > _2_PI){
		rpy[2] = 0.0f;
	}
	rpy[0] = RADTODEG(rpy[0]);
	rpy[1] = RADTODEG(rpy[1]);
	rpy[2] = RADTODEG(rpy[2]);
}

void Quaternion_FromRotationMatrix(float *R, float *Q)
{
#if 0
	// calculate the trace of the matrix
	float trace = R[0] + R[4] + R[8];
	float s;
	if(trace > 0)
	{
		s = 0.5f * FastSqrtI(trace + 1.0f);
		Q[0] = 0.25f / s;
		Q[1] = (R[7] - R[5]) * s;
		Q[2] = (R[2] - R[6]) * s;
		Q[3] = (R[3] - R[1]) * s;
	}
	else
	{
		if(R[0] > R[4] && R[0] > R[8] )
		{
			s = 0.5f * FastSqrtI(1.0f + R[0] - R[4] - R[8]);
			Q[0] = (R[7] - R[5]) * s;
			Q[1] = 0.25f / s;
			Q[2] = (R[1] + R[3]) * s;
			Q[3] = (R[2] + R[6]) * s;
		}
		else if(R[4] > R[8]) 
		{
			s = 0.5f * FastSqrtI(1.0f + R[4] - R[0] - R[8]);
			Q[0] = (R[2] - R[6]) * s;
			Q[1] = (R[1] + R[3]) * s;
			Q[2] = 0.25f / s;
			Q[3] = (R[5] + R[7]) * s;
		}
		else
		{
			s = 0.5f * FastSqrtI(1.0f + R[8] - R[0] - R[4]);
			Q[0] = (R[3] - R[1]) * s;
			Q[1] = (R[2] + R[6]) * s;
			Q[2] = (R[5] + R[7]) * s;
			Q[3] = 0.25f / s;
		};
	}
	Quaternion_Normalize(Q);
#else
	// get the instantaneous orientation quaternion
	float fq0sq; // q0^2
	float recip4q0; // 1/4q0
	float fmag; // quaternion magnitude
#define SMALLQ0 0.01F // limit where rounding errors may appear
	// get q0^2 and q0
	fq0sq = 0.25f * (1.0f + R[0] + R[4] + R[8]);
	Q[0] = (float)FastSqrt(Fast_Abs(fq0sq));
	// normal case when q0 is not small meaning rotation angle not near 180 deg
	if (Q[0] > SMALLQ0)
	{
		// calculate q1 to q3
		recip4q0 = 0.25F / Q[0];
		Q[1] = recip4q0 * (R[5] - R[7]);
		Q[2] = recip4q0 * (R[6] - R[2]);
		Q[3] = recip4q0 * (R[1] - R[3]);
	}
	else
	{
		// special case of near 180 deg corresponds to nearly symmetric matrix
		// which is not numerically well conditioned for division by small q0
		// instead get absolute values of q1 to q3 from leading diagonal
		Q[1] = FastSqrt(Fast_Abs(0.5f * (1.0f + R[0]) - fq0sq));
		Q[2] = FastSqrt(Fast_Abs(0.5f * (1.0f + R[4]) - fq0sq));
		Q[3] = FastSqrt(Fast_Abs(0.5f * (1.0f + R[8]) - fq0sq));
		// first assume q1 is positive and ensure q2 and q3 are consistent with q1
		if ((R[1] + R[3]) < 0.0f)
		{
			// q1*q2 < 0 so q2 is negative
			Q[2] = -Q[2];
			if ((R[5] + R[7]) > 0.0f)
			{
				// q1*q2 < 0 and q2*q3 > 0 so q3 is also both negative
				Q[3] = -Q[3];
			}
		}
		else if ((R[1] + R[3]) > 0.0f)
		{
			if ((R[5] + R[7]) < 0.0f)
			{
				// q1*q2 > 0 and q2*q3 < 0 so q3 is negative
				Q[3] = -Q[3];
			}
		}
		// negate the vector components if q1 should be negative
		if ((R[5] - R[7]) < 0.0f)
		{
			Q[1] = -Q[1];
			Q[2] = -Q[2];
			Q[3] = -Q[3];
		}
	}
	// finally re-normalize
	fmag = FastSqrtI(Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]);
	Q[0] *= fmag;
	Q[1] *= fmag;
	Q[2] *= fmag;
	Q[3] *= fmag;
#endif
}

void Quaternion_RungeKutta4(float *q, float *w, float dt, int normalize)
{
	float half = 0.5f;
	float two = 2.0f;
	float qw[4], k2[4], k3[4], k4[4];
	float tmpq[4], tmpk[4];

	//qw = q * w * half;
	Quaternion_Multiply(qw, q, w);
	Quaternion_Scalar(qw, qw, half);
	//k2 = (q + qw * dt * half) * w * half;
	Quaternion_Scalar(tmpk, qw, dt * half);
	Quaternion_Add(tmpk, q, tmpk);
	Quaternion_Multiply(k2, tmpk, w);
	Quaternion_Scalar(k2, k2, half);
	//k3 = (q + k2 * dt * half) * w * half;
	Quaternion_Scalar(tmpk, k2, dt * half);
	Quaternion_Add(tmpk, q, tmpk);
	Quaternion_Multiply(k3, tmpk, w);
	Quaternion_Scalar(k3, k3, half);
	//k4 = (q + k3 * dt) * w * half;
	Quaternion_Scalar(tmpk, k3, dt);
	Quaternion_Add(tmpk, q, tmpk);
	Quaternion_Multiply(k4, tmpk, w);
	Quaternion_Scalar(k4, k4, half);
	//q += (qw + k2 * two + k3 * two + k4) * (dt / 6);
	Quaternion_Scalar(tmpk, k2, two);
	Quaternion_Add(tmpq, qw, tmpk);
	Quaternion_Scalar(tmpk, k3, two);
	Quaternion_Add(tmpq, tmpq, tmpk);
	Quaternion_Add(tmpq, tmpq, k4);
	Quaternion_Scalar(tmpq, tmpq, dt / 6.0f);
	Quaternion_Add(q, q, tmpq);

	if (normalize){
		Quaternion_Normalize(q);
	}
}

void Quaternion_From6AxisData(float* q, float *accel, float *mag)
{
	// local variables
	float norma, normx, normy;
	//float normm;
	//3x3 rotation matrix
	float R[9];
	// place the un-normalized gravity and geomagnetic vectors into
	// the rotation matrix z and x axes
	R[2] = accel[0]; R[5] = accel[1]; R[8] = accel[2];
	R[0] = mag[0]; R[3] = mag[1]; R[6] = mag[2];
	// set y vector to vector product of z and x vectors
	R[1] = R[5] * R[6] - R[8] * R[3];
	R[4] = R[8] * R[0] - R[2] * R[6];
	R[7] = R[2] * R[3] - R[5] * R[0];
	// set x vector to vector product of y and z vectors
	R[0] = R[4] * R[8] - R[7] * R[5];
	R[3] = R[7] * R[2] - R[1] * R[8];
	R[6] = R[1] * R[5] - R[4] * R[2];
	// calculate the vector moduli invert
	norma = FastSqrtI(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	//normm = FastSqrtI(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
	normx = FastSqrtI(R[0] * R[0] + R[3] * R[3] + R[6] * R[6]);
	normy = FastSqrtI(R[1] * R[1] + R[4] * R[4] + R[7] * R[7]);
	// normalize the rotation matrix
	// normalize x axis
	R[0] *= normx; R[3] *= normx; R[6] *= normx;
	// normalize y axis
	R[1] *= normy; R[4] *= normy; R[7] *= normy;
	// normalize z axis
	R[2] *= norma; R[5] *= norma; R[8] *= norma;
	
	Quaternion_FromRotationMatrix(R, q);
}
