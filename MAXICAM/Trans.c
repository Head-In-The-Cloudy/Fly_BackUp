//下面为ChatGPT解释流程
/*
Pixel_Size_MV 和 Pixel_Size_CV：这些是针对两种不同的摄像头模型（OV7725 和 OV5647）的每个像素对应的实际尺寸。它们用于计算每个像素代表的实际物理距离。

Focal_Length_MV 和 Focal_Length_CV：分别表示摄像头的焦距。

OV7725_Sensor_Width_MM 和 OV7725_Sensor_Height_MM：这些是传感器的物理尺寸，单位是毫米。

Pixel_Image_Width_MV 和 Pixel_Image_Height_MV：表示摄像头的图像分辨率（在像素级别）。

Pixel_Image_View_Angle_X_MV 和 Pixel_Image_View_Angle_Y_MV：这两个参数给出了摄像头的视角宽度和高度。即，在一个特定的角度内，摄像头能看到的区域宽度和高度。

AprilTag_Side_Length：表示AprilTag（视觉标记）边长，用于计算物体位置。
*/

/*
Get_Camera_Wide_Angle 函数根据视角计算，得到相应的水平视角和垂直视角的范围。
这是根据视角和焦距来决定的，目的是为了从焦距和视角推算出真实的视场范围。
*/

/*
. Sensor_Parameter_Sort函数

_P1 和 _P2 分别表示在 X 和 Y 轴上的像素对应的实际物理距离（通过视角和焦距计算得到的）。

_CX 和 _CY 分别是图像中心的像素坐标。

通过传入的 (tx, ty)（目标在图像中的像素位置），
以及传感器的 pitch、roll 和 alt（飞行器的姿态角度和高度），可以计算目标的实际位置
*/

/*
tmp_x 和 tmp_y 分别代表目标的像素坐标与图像中心之间的角度差。

tx 和 ty 通过 fast_atan 计算得到的角度变化，将目标从像素坐标系转化为真实世界的角度。

roll 和 pitch（飞行器的俯仰角度和滚转角度）会影响目标的实际位置。

通过 FastTan（快速计算正切函数）以及计算的角度和视场宽度/高度 (_P1 和 _P2)
最终得到目标的实际物理坐标 _DX 和 _DY。
*/

//只能修改MV部分
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
