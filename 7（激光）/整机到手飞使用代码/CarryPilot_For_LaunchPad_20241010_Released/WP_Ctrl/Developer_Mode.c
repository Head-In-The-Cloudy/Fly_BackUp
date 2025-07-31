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
#include "Developer_Mode.h"

uint8_t SDK1_Mode_Setup = 0x00;			  // SDK任务控制计数器，同时也可以用做底部视觉工作模式配置
uint8_t Forward_Vision_Mode_Setup = 0x00; // 前向视觉传感器的工作模式
int16_t task_select_cnt = 1;			  // 任务选择计数器,在完成自动起飞后,会在当前case的基础上
										  // 自加或者自减task_select_cnt,从而实现自动起飞后,执行不同的飞行任务
// 特别提示，任务1~9中并没有自动起飞的程序，需要自己手动起飞后，再去切sdk通道执行，
// 也可以在任务10中自动起飞函数执行完毕后,修改SDK1_Mode_Setup的值后再执行1~9
void Auto_Flight_Ctrl(uint8_t *mode)
{
	static uint16_t openmv_work_mode = 0;
	switch (*mode)
	{
	case 0: // 用户事先指定的SDK开发者模式:起飞+前进+降落
	{
		NCQ_SDK_Run(); // 水内含平控制+高度控制
		Flight.yaw_ctrl_mode = ROTATE;
		Flight.yaw_outer_control_output = RC_Data.rc_rpyt[RC_YAW];
	}
	break;
	case 1: // 俯视OPENMV视觉追踪色块
	{
		Color_Block_Control_Pilot(); // 俯视OPENMV视觉水平追踪
		Flight.yaw_ctrl_mode = ROTATE;
		Flight.yaw_outer_control_output = RC_Data.rc_rpyt[RC_YAW];
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL, NUL, NUL); // 高度控制
	}
	break;
	case 2: // 俯视OPENMV视觉追踪AprilTag，控制逻辑与追踪色块一致
	{
		Top_APrilTag_Control_Pilot(); // 俯视OPENMV视觉水平追踪
		Flight.yaw_ctrl_mode = ROTATE;
		Flight.yaw_outer_control_output = RC_Data.rc_rpyt[RC_YAW];
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL, NUL, NUL); // 高度控制
	}
	break;
	case 3: // 俯视OPENMV循迹控制，默认黑线，阈值可在openmv程序中调整
	{
		Self_Track_Control_Pilot();								// 循迹控制内含有水平控制+偏航控制
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL, NUL, NUL); // 高度控制
	}
	break;
	case 4: // 前视OPENMV视觉追踪
	{
		Front_AprilTag_Control_Pilot(); // 前视OPENMV视觉追踪
		Flight.yaw_ctrl_mode = ROTATE;
		Flight.yaw_outer_control_output = RC_Data.rc_rpyt[RC_YAW];
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL, NUL, NUL); // 高度控制
	}
	break;
	case 5: // 自动绕杆,需要自己加装测距/超声波
	{
		Front_Surround_Pole_Control_Pilot();
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL, NUL, NUL); // 高度控制
	}
	break;
	case 6:
	{
		flight_subtask_1(); // 顺时针转动90度，完成后降落
	}
	break;
	case 7:
	{
		flight_subtask_3(); // 以10deg/s的角速度顺时针转动10000ms，完成后降落
	}
	break;
	case 8:
	{
		flight_subtask_5(); // 机体坐标系下相对位移,正方形轨迹
	}
	break;
	case 9:
	{
		flight_subtask_7(); // 导航坐标系下，基于初始点的绝对坐标位移,正方形轨迹
	}
	break;
	case 10:
	{
		flight_subtask_8(); // 航点飞行轨迹圆，半径、航点数量可设置
	}
	break;
	case 11: // 自动起飞到某一高度
	{
		if (Auto_Takeoff(Target_Height) == 1) // WORK_HEIGHT_CM
		{
			*mode += 14; // 到达目标高度后，切换到下一SDK任务
						 //*mode+=4;//到达目标高度后，切换2022年7月省赛第一部分任务
						 //*mode+=5;//到达目标高度后，切换2022年7月省赛第二部分任务
						 //*mode+=6;//到达目标高度后，切换2022年7月省赛第三部分任务
		}
	}
	break;
	case 12: // 2021年电赛植保无人机赛题
	{
		if (openmv_work_mode == 0) // 只配置一次
		{
			openmv_work_mode = 0x07;
			SDK_DT_Send_Check(openmv_work_mode, UART3_SDK); // 起飞完毕之后，将底部OPENMV设置成检测农作物模式
		}
		// 2021年电子设计竞赛G题植保无人机
		Agriculture_UAV_Closeloop(); // 基础部分
									 // Agriculture_UAV_Innovation();//发挥部分
	}
	break;
	case 13: // ROS上位机控制无人机
	{
		ros_flight_support();
	}
	break;
	case 14: // 导航控制控制函数，可以用于无名创新地面站控制
	{
		basic_auto_flight_support();
	}
	break;
	case 15: // 2021年电赛送货无人机赛题——基础
	{
		if (openmv_work_mode == 0) // 只配置一次
		{
			openmv_work_mode = 0x10;
			SDK_DT_Send_Check(openmv_work_mode, UART3_SDK); // 起飞完毕之后，将底部OPENMV设置成色块、形状检测模式
		}
		// 2022年月电子设计竞赛B题送货无人机——第1部分
		Deliver_UAV_Basic();
	}
	break;
	case 16: // 2021年电赛送货无人机赛题——发挥
	{
		if (openmv_work_mode == 0) // 只配置一次
		{
			openmv_work_mode = 0x10;
			SDK_DT_Send_Check(openmv_work_mode, UART3_SDK); // 起飞完毕之后，将底部OPENMV设置成色块、形状检测模式
		}
		// 2022年月电子设计竞赛B题送货无人机——第2部分
		Deliver_UAV_Innovation();
	}
	break;
	case 17: // 2021年电赛送货无人机赛题——创新
	{
		// 2022年月电子设计竞赛B题送货无人机——第3部分
		Deliver_UAV_Hulahoop();
	}
	break;
	case 18: // 自动起飞到某一高度
	{
		if (Auto_Takeoff(Target_Height) == 1)
		{
			*mode += 1; // 到达目标高度后，切换到下一SDK任务
		}
	}
	break;
	case 19: // 用户自定义航点飞行-无需二次编程，就可以实现3维空间内的若干航点遍历飞行
	{
		// 用户通过地面站自定义或者按键手动输入三维的航点位置，无人机依次遍历各个航点，当前最多支持28个航点，可以自由扩展
		Navigation_User_Setpoint();
	}
	break;
	case 20: // 自动起飞到180cm高度，完成后根据task_select_cnt值来决策执行基础任务还是发挥任务
	{
		if (Auto_Takeoff(180) == 1) // 期望高度180cm
		{
			*mode += task_select_cnt; // task_select_cnt设置为1:到达目标高度后，切换到基础部分任务
									  // task_select_cnt设置为2:到达目标高度后，切换到发挥部分任务
		}
	}
	break;
	case 21: // 2023年TI国赛G题——基础部分
	{
		Air_Ground_Extinguish_Fire_System_Basic(); // 无人机航点遍历后返航降落
	}
	break;
	case 22: // 2023年TI国赛G题——发挥部分
	{
		Air_Ground_Extinguish_Fire_System_Innovation(); // 无人机航点遍历后返航降落+灭火动作+检测火源信息后发送给小车
	}
	break;
	case 23: // 2023年TI国赛G题目——单独调试小车运行状态时用：无需实际飞行,直接发送火源坐标信息给小车
	{
		// 进入此sdk后就会产生火情信息，实现飞机飞行途中检测到火源信息后
		// 再发生给小车一样的效果，方便用户对小车灭火功能单独进行调试
		fire_flag = 1;	   // 火源更新标志位
		fire_x = 65 + 35;  // 去掉初始偏置后火源绝对坐标X
		fire_y = 110 + 35; // 去掉初始偏置后火源绝对坐标Y
	}
	break;
	case 24:
	{
		flight_subtask_code_read();
		// flight_subtask_test();
		// 用户预留任务，编写后注意加上break跳出
	}
	break;
	case 25:
	{
		Traverse_Search_Track_Land();
		// Traverse_Search_Track_Land_Test();
		// 用户预留任务，编写后注意加上break跳出
	}
	break;
	case 26:
	{
		Traverse_Search_3D_Track();
		// 用户预留任务，编写后注意加上break跳出
	}
	break;
	case 27:
	{
		// 用户预留任务，编写后注意加上break跳出
	}
	break;
	case 28:
	{
		// 用户预留任务，编写后注意加上break跳出
	}
	case 30:
	{
		// 用户预留任务，编写后注意加上break跳出
	}
	case 31: // 前面预留case不满足情况下执行此情形
	{
		Flight.roll_outer_control_output = RC_Data.rc_rpyt[RC_ROLL];
		Flight.pitch_outer_control_output = RC_Data.rc_rpyt[RC_PITCH];
		Flight.yaw_ctrl_mode = ROTATE;
		Flight.yaw_outer_control_output = RC_Data.rc_rpyt[RC_YAW];
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL, NUL, NUL); // 高度控制
	}
	break;
	case 32: // SDK模式中原地降落至地面怠速后停桨,用于任务执行完成后降落
	{
		OpticalFlow_Control(0);
		Flight.yaw_ctrl_mode = ROTATE;
		Flight.yaw_outer_control_output = RC_Data.rc_rpyt[RC_YAW];
		Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL, NUL, -50); // 高度控制
	}
	break;
	default:
	{
		Flight.roll_outer_control_output = RC_Data.rc_rpyt[RC_ROLL];
		Flight.pitch_outer_control_output = RC_Data.rc_rpyt[RC_PITCH];
		Flight.yaw_ctrl_mode = ROTATE;
		Flight.yaw_outer_control_output = RC_Data.rc_rpyt[RC_YAW];
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL, NUL, NUL); // 高度控制
	}
	}
}
