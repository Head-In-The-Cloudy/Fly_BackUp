//只修改MV部分
/*
_DX = 0.5f * alt * _TX / _P1;
_DY = 0.5f * alt * _TY / _P2;
0.5f为经验因子，需要具体实验进行标定与调整
*/
#define  Pixel_Size_MV    0.00056f
                        //6um=0.000006m=0.0006cm
                       //320---0.0012
                       //160---0.0024 原
                       //80 ---0.0048
#define  Focal_Length_MV  0.43f //原0.42

#define OV7725_Sensor_Width_MM    		3.984f//3984um
#define OV7725_Sensor_Height_MM   		2.952f//2952um
#define Pixel_Image_Width_MV    		640//320,160
#define Pixel_Image_Height_MV   		480//240,120
#define Pixel_Image_Focal_MM_MV 		4.3f
//#define Pixel_Image_View_Angle_X_MV  (56.72/2)//deg(50.75/2)
//#define Pixel_Image_View_Angle_Y_MV  (44.07/2)//deg(38.72/2)
#define Pixel_Image_View_Angle_X_MV   (91.3f / 2)   // 45.65°
#define Pixel_Image_View_Angle_Y_MV   (46.1f / 2)   // 23.05°
/*************************************************************************/
#define  Pixel_Size_CV    0.00056f//cm
													//2592:1944——0.00014cm
													//640:480——0.00056cm
#define  Focal_Length_CV  0.43f  //原焦距3.6mm 现焦距4.3mm

#define OV5647_Sensor_Width_MM    		3.674f//3674um
#define OV5647_Sensor_Height_MM   		2.738f//2738.4um#
#define Pixel_Image_Width_CV    			640//640
#define Pixel_Image_Height_CV   			480//480
#define Pixel_Image_Focal_MM_CV 			4.3f
/*
#define Pixel_Image_View_Angle_X_CV  (53.5/2)//约为66deg广角
#define Pixel_Image_View_Angle_Y_CV  (41.4/2)
*/

#define Pixel_Image_View_Angle_X_CV   (91.3f / 2)   // 45.65°
#define Pixel_Image_View_Angle_Y_CV   (46.1f / 2)   // 23.05°


#define AprilTag_Side_Length  13.6f//cm13.6

float _Pixel_Image_View_Angle_X,_Pixel_Image_View_Angle_Y;
void Get_Camera_Wide_Angle(float view_angle)
{
	float fh=5.0f/FastTan(0.5f*view_angle*DEG2RAD);
  _Pixel_Image_View_Angle_X=2*RAD2DEG*fast_atan(4/fh);
  _Pixel_Image_View_Angle_Y=2*RAD2DEG*fast_atan(3/fh);
}
	
float _P1=0,_P2=0;
uint16_t _CX=0,_CY=0;
float _TX=0,_TY=0;
float _DX=0,_DY=0;
void Sensor_Parameter_Sort(uint16_t tx,uint16_t ty,float pitch,float roll,float alt)
{
//	Get_Camera_Wide_Angle(68);//根据摄像头广角，计算得到X、Y轴视角
	float theta_x_max=0,theta_y_max=0;
	theta_x_max=Pixel_Image_View_Angle_X_MV; // 45.65°
	theta_y_max=Pixel_Image_View_Angle_Y_MV; // 23.05°
    _P1=0.5f*Pixel_Image_Width_MV/FastTan(theta_x_max*DEG2RAD);	//角度转弧度 
	_P2=0.5f*Pixel_Image_Height_MV/FastTan(theta_y_max*DEG2RAD);
	
	_CX=Pixel_Image_Width_MV/2; //320
	_CY=Pixel_Image_Height_MV/2;//240
	
	float tmp_x=0,tmp_y=0;
	tmp_x=fast_atan((_CX-tx)/_P1);//实际x方向上的角度
	tmp_y=fast_atan((_CY-ty)/_P2);//实际y方向上的角度
	
	_TX= FastTan(tmp_x+roll*DEG2RAD) *_P1;
	_TY= FastTan(tmp_y+pitch*DEG2RAD)*_P2;
	
//	_DX=alt*_TX/_P1;
//	_DY=alt*_TY/_P2;
	_DX=0.5f*alt*_TX/_P1;
	_DY=0.5f*alt*_TY/_P2;

	Opv_Top_View_Target.sdk_target.x=_DX;
    Opv_Top_View_Target.sdk_target.y=_DY;
}
