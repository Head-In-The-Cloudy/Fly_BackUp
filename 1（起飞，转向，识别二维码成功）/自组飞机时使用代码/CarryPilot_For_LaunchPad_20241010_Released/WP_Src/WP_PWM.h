#ifndef __WP_PWM_H__
#define __WP_PWM_H__

//飞控电调所接PWM来源
#define FLIGHT_ESC_PWM  1//0:使用EPWM、1:使用排针PWM
/*
飞控扩展板标号   单片机引脚号    单片机PWM资源通道
			P1						  PC4               M0PWM6
			P2						  PC5               M0PWM7
			P3						  PB7               M0PWM1
			P4						  PB6               M0PWM0
			P5						  PE5               M0PWM5
			P6						  PE4               M0PWM4
			P7						  PB4               M0PWM2
			P8						  PB5               M0PWM3
*/


/*
0:使用EPWM时
飞控扩展板的EPWM接口标号为P5、P6、P7、P8分别接电机1、2、3、4所在的电调信号线
电调信号序号				飞控扩展板标号   单片机引脚号    单片机PWM资源通道
		M1										P5						  PE5               M0PWM5
		M2										P6						  PE4               M0PWM4
		M3										P7						  PB4               M0PWM2
		M4										P8						  PB5               M0PWM3
————————————————————————————————————————————————————————————————————————————
<两个通道预留PWM分别为排针PWM接口的P3、P4>
<排针PWM接口的P1、P2被用作串口资源的UART4_RXD、UART_TXD>
————————————————————————————————————————————————————————————————————————————
1:使用排针PWM时
飞控扩展板的排针PWM标号为P1、P2、P3、P4分别接电机1、2、3、4所在的电调信号线
电调信号序号				飞控扩展板标号   单片机引脚号    单片机PWM资源通道
		M1										P1						  PC4               M0PWM6
		M2										P2						  PC5               M0PWM7
		M3										P3						  PB7               M0PWM1
		M4										P4						  PB6               M0PWM0
————————————————————————————————————————————————————————————————————————————
<两个通道预留PWM分别为EPWM接口的P7、P8>
<EPWM接口接口的P5、P6被用作串口资源的UART5_TXD、UART5_RXD>
————————————————————————————————————————————————————————————————————————————
*/


void PWM_Output(uint16_t width1,uint16_t width2,uint16_t width3,uint16_t width4);
void Reserved_PWM1_Output(uint16_t us);
void Reserved_PWM2_Output(uint16_t us);
void PWM_Init(void);

#endif




