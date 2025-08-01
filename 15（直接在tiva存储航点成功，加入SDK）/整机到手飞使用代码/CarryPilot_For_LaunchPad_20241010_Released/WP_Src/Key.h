#ifndef __KEY_H__
#define __KEY_H__

typedef enum {
	UP=0,
	DOWN,
//
	UP_3D,
	DN_3D,
	LT_3D,
	RT_3D,
	ME_3D,
	BUTTON_NUM
}BUTTON;

typedef enum
{
	NO_PRESS=0,
	SHORT_PRESS,
  LONG_PRESS,
  IN_PRESS,	
}press_state;

typedef struct
{
	press_state press;
  uint8_t value;
	uint8_t last_value;	
	uint_fast8_t  port;
	uint_fast16_t pin;
	uint32_t press_time;
  uint32_t release_time; 
  uint32_t in_time; 
	uint32_t in_press_cnt;
	uint32_t state_lock_time;
}button_state;

typedef struct
{
  button_state state[BUTTON_NUM];
}rc_buttton;

#define LONG_PRESS_LIMIT  1000//1000ms
#define IN_PRESS_LIMIT  	250//250ms



#define QuadKey1  GPIOPinRead(GPIO_PORTF_BASE , GPIO_PIN_4)
#define QuadKey2  GPIOPinRead(GPIO_PORTF_BASE , GPIO_PIN_0)

void Key_Init(void);
bool Key_Scan(uint8_t release);
void QuadShow(uint8_t show_enable);
void Read_Button_State_All(void);


extern rc_buttton _button;
extern uint8_t Key_Right_Release;
extern int16_t Page_Number;




#endif





