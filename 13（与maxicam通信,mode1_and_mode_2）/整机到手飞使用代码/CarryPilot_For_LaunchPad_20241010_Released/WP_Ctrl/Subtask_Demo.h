#ifndef _SUBTASK_DEMO_
#define _SUBTASK_DEMO_

#define FLIGHT_SUBTASK_NUM 50

#define WAY_POINT_NUM 5 // 本次任务中的航点总个数  不包含自动起飞前后

#define WORK_HEIGHT_CM 125 // 150

void flight_subtask_reset(void);

void flight_subtask_1(void);
void flight_subtask_2(void);
void flight_subtask_3(void);
void flight_subtask_4(void);
void flight_subtask_5(void);
void flight_subtask_6(void);
void flight_subtask_7(void);
void flight_subtask_8(void);

void flight_subtask_code_read(void);
void flight_subtask_test(void);
void go_to_point(vector3f target);
void Traverse_Search_Track_Land(void);
void Traverse_Search_Track_Land_Test(void);
void Traverse_Search_3D_Track(void);
void Traverse_Search_Animal_Find(void);


uint8_t Auto_Takeoff(float target);
void basic_auto_flight_support(void);
void ros_flight_support(void);

void Agriculture_UAV_Basic(void);
void Agriculture_UAV_Closeloop(void);
void Agriculture_UAV_Innovation(void);

void Deliver_UAV_Basic(void);
void Deliver_UAV_Innovation(void);
void Deliver_UAV_Hulahoop(void);

void Air_Ground_Extinguish_Fire_System_Basic(void);
void Air_Ground_Extinguish_Fire_System_Innovation(void);

void Navigation_User_Setpoint(void);

void substask_params_init(void);

extern uint16_t flight_subtask_cnt[FLIGHT_SUBTASK_NUM];
extern uint32_t execute_time_ms[FLIGHT_SUBTASK_NUM];
extern uint32_t flight_global_cnt[FLIGHT_SUBTASK_NUM];

extern Vector3f base_position;

extern uint16_t barcode_id;
extern uint8_t barcode_flag;

extern float min_dis_cm, min_dis_angle, target_yaw_err;

#define Laser_Min_Info_Num 20
extern float min_dis_cm_backups[Laser_Min_Info_Num], min_dis_angle_backups[Laser_Min_Info_Num];

extern int16_t fire_x, fire_y; // 火源位置cm
extern uint8_t fire_flag;	   // 火源标志

extern uint16_t release_pwm_us; // 释放灭火包舵机pwm
extern uint16_t pinch_pwm_us;	// 夹住灭火包舵机pwm,需要在初始化的时候就给舵机赋此pwm值

#endif

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
*               修改日期:2023/06/20
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
