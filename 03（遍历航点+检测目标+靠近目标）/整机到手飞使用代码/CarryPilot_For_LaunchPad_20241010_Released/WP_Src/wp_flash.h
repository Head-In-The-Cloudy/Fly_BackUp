#ifndef __WP_FLASH_H__
#define __WP_FLASH_H__

void EEPROM_Init(void);


#define PARAMETER_TABLE_STARTADDR   0x0000 
#define FLIGHT_PARAMETER_TABLE_NUM  200
typedef struct
{
   float Parameter_Table[FLIGHT_PARAMETER_TABLE_NUM];
}FLIGHT_PARAMETER;


typedef enum
{
   PITCH_OFFSET1=0,
   ROLL_OFFSET1=1,
   ACCEL_X_OFFSET1=2,
   ACCEL_Y_OFFSET1=3,
   ACCEL_Z_OFFSET1=4,
   ACCEL_X_SCALE1=5,
   ACCEL_Y_SCALE1=6,
   ACCEL_Z_SCALE1=7,
   MAG_X_OFFSET=8,
   MAG_Y_OFFSET=9,
   MAG_Z_OFFSET=10,
   RC_CH1_MAX=11,
   RC_CH1_MIN=12,
   RC_CH2_MAX=13,
   RC_CH2_MIN=14,
   RC_CH3_MAX=15,
   RC_CH3_MIN=16,
   RC_CH4_MAX=17,
   RC_CH4_MIN=18,
   RC_CH5_MAX=19,
   RC_CH5_MIN=20,
   RC_CH6_MAX=21,
   RC_CH6_MIN=22,
   RC_CH7_MAX=23,
   RC_CH7_MIN=24,
   RC_CH8_MAX=25,
   RC_CH8_MIN=26,
	 PID1_PARAMETER_KP=27,//pitch_gyro
   PID1_PARAMETER_KI=28,
   PID1_PARAMETER_KD=29,
   PID2_PARAMETER_KP=30,//roll_gyro
   PID2_PARAMETER_KI=31,
   PID2_PARAMETER_KD=32,
   PID3_PARAMETER_KP=33,//yaw_gyro
   PID3_PARAMETER_KI=34,
   PID3_PARAMETER_KD=35,
   PID4_PARAMETER_KP=36,//pitch_angle
   PID4_PARAMETER_KI=37,
   PID4_PARAMETER_KD=38,
   PID5_PARAMETER_KP=39,//roll_angle
   PID5_PARAMETER_KI=40,
   PID5_PARAMETER_KD=41,
   PID6_PARAMETER_KP=42,//yaw_angle
   PID6_PARAMETER_KI=43,
   PID6_PARAMETER_KD=44,
   PID7_PARAMETER_KP=45,//height_position
   PID7_PARAMETER_KI=46,
   PID7_PARAMETER_KD=47,
   PID8_PARAMETER_KP=48,//height_speed
   PID8_PARAMETER_KI=49,
   PID8_PARAMETER_KD=50,
   PID9_PARAMETER_KP=51,//height_accel
   PID9_PARAMETER_KI=52,
   PID9_PARAMETER_KD=53,
   PID10_PARAMETER_KP=54,//latitude_speed
   PID10_PARAMETER_KI=55,
   PID10_PARAMETER_KD=56,
   PID11_PARAMETER_KP=57,//latitude_position
   PID11_PARAMETER_KI=58,
   PID11_PARAMETER_KD=59,
   PID12_PARAMETER_KP=60,//optical_position
   PID12_PARAMETER_KI=61,
   PID12_PARAMETER_KD=62,
   PID13_PARAMETER_KP=63,//optical_speed
   PID13_PARAMETER_KI=64,
   PID13_PARAMETER_KD=65,
   PID14_PARAMETER_KP=66,//sdk_position
   PID14_PARAMETER_KI=67,
   PID14_PARAMETER_KD=68,
   PID15_PARAMETER_KP=69,
   PID15_PARAMETER_KI=70,
   PID15_PARAMETER_KD=71,
   PID16_PARAMETER_KP=72,
   PID16_PARAMETER_KI=73,
   PID16_PARAMETER_KD=74,   
   PID17_PARAMETER_KP=75,
   PID17_PARAMETER_KI=76,
   PID17_PARAMETER_KD=77,
   PID18_PARAMETER_KP=78,
   PID18_PARAMETER_KI=79,
   PID18_PARAMETER_KD=80,   
   PID19_PARAMETER_KP=81,
   PID19_PARAMETER_KI=82,
   PID19_PARAMETER_KD=83,
   ESC_CALIBRATION_FLAG=84,
   HEADLESS_MODE_YAW=85,
   HOR_CAL_ACCEL_X=86,
   HOR_CAL_ACCEL_Y=87,
   HOR_CAL_ACCEL_Z=88,
	 SDK1_MODE_DEFAULT=89,
	 GROUND_DISTANCE_DEFAULT=90,
	 GYRO_X_OFFSET=91,
	 GYRO_Y_OFFSET=92,
	 GYRO_Z_OFFSET=93,
	 PITCH_ROLL_FEEDFORWARD_KP=94,
	 PITCH_ROLL_FEEDFORWARD_KD=95,
	 TARGET_HEIGHT=96,
	 SAFE_HEIGHT=97,
	 SAFE_VBAT=98,
	 MAX_HEIGHT=99,
	 MAX_RADIUS=100,
	 MAX_UPVEL=101,
	 MAX_DOWNVEL=102,
	 MAX_HORVEL=103,
	 RESERVED_UART_FUNCTION=104,
	 NEAR_GROUND_HEIGHT=105,
	 OPTICAL_TYPE=106,
	 FORWARD_VISION_MODE_DEFAULT=107,
	 ACCEL_SIMPLE_MODE=108,
	 UART2_FUNCTION=109,
	 AVOID_OBSTACLE=110,
	 ADC_KEY1_ENABLE=111,
	 ADC_KEY2_ENABLE=112,
	 WIRELESS_ADC_KEY_ENABLE=113,
	 RES_SERVO_PINCH_PWM_US=114,
	 RES_SERVO_RELEASE_PWM_US=115,
	 TASK_SELECT_AFTER_TAKEOFF=116,
	 IMU_SENSE_TYPE=117,
   PITCH_OFFSET2=118,
   ROLL_OFFSET2=119,
   ACCEL_X_OFFSET2=120,
   ACCEL_Y_OFFSET2=121,
   ACCEL_Z_OFFSET2=122,
   ACCEL_X_SCALE2=123,
   ACCEL_Y_SCALE2=124,
   ACCEL_Z_SCALE2=125,
	 GYRO_X_OFFSET2=126,
	 GYRO_Y_OFFSET2=127,
	 GYRO_Z_OFFSET2=128,	 
   HOR_CAL_ACCEL_X2=129,
   HOR_CAL_ACCEL_Y2=130,
   HOR_CAL_ACCEL_Z2=131,
	 GYRO_LPF_CF=132,
	 ACCEL_LPF_CF=133,
	 INS_LPF_CF=134,
	 FB_LPF_CF=135,
	 DRONE_PLAYER_LEVEL=136,	 
	 ZERO_HEIGHT_CM_OFFSET=137,
	 DISTANCE_TOF_3DLIDAR=138,
	 
	 
	 RESERVED_PARAM=170,
}FLIGHT_PARAMETER_TABLE;

//extern FLIGHT_PARAMETER Flight_Params;

uint32_t STMFLASH_ReadWord(uint32_t faddr);		  	//读出字  
void STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite);		//从指定地址开始写入指定长度的数据
void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead);   		//从指定地址开始读出指定长度的数据

void ReadFlashParameterALL(FLIGHT_PARAMETER *WriteData);
void ReadFlashParameterOne(uint16_t Label,float *ReadData);
void ReadFlashParameterTwo(uint16_t Label,float *ReadData1,float *ReadData2);
void ReadFlashParameterThree(uint16_t Label,float *ReadData1,float *ReadData2,float *ReadData3);
   
   
void WriteFlashParameter(uint16_t Label,float WriteData);
void WriteFlashParameter_Two(uint16_t Label,
                         float WriteData1,
                         float WriteData2);
void WriteFlashParameter_Three(uint16_t Label,
                         float WriteData1,
                         float WriteData2,
                         float WriteData3);
void ReadFlashParameterTwo(uint16_t Label,float *ReadData1,float *ReadData2);

#endif


												 
												 
/*
												 
//FLASH起始地址
#define WP_FLASH_BASE PARAMETER_TABLE_STARTADDR 	//STM32 FLASH的起始地址

FLIGHT_PARAMETER Flight_Params;



void EEPROM_Init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0); // EEPROM activate
  EEPROMInit(); // EEPROM start
}

void STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite)	
{ 
  EEPROMProgram(pBuffer,WriteAddr,NumToWrite);
}

void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)   	
{
  EEPROMRead(pBuffer,ReadAddr,NumToRead);
}


void ReadFlashParameterALL(FLIGHT_PARAMETER *WriteData)
{
  EEPROMRead((uint32_t *)(&WriteData->Parameter_Table),WP_FLASH_BASE,FLIGHT_PARAMETER_TABLE_NUM*4);
}


void ReadFlashParameterOne(uint16_t Label,float *ReadData)
{
  EEPROMRead((uint32_t *)(ReadData),WP_FLASH_BASE+4*Label,4);
}

void ReadFlashParameterTwo(uint16_t Label,float *ReadData1,float *ReadData2)
{
  EEPROMRead((uint32_t *)(ReadData1),WP_FLASH_BASE+4*Label,4);;
  EEPROMRead((uint32_t *)(ReadData2),WP_FLASH_BASE+4*Label+4,4);
}

void ReadFlashParameterThree(uint16_t Label,float *ReadData1,float *ReadData2,float *ReadData3)
{
  EEPROMRead((uint32_t *)(ReadData1),WP_FLASH_BASE+4*Label,4);;
  EEPROMRead((uint32_t *)(ReadData2),WP_FLASH_BASE+4*Label+4,4);
  EEPROMRead((uint32_t *)(ReadData3),WP_FLASH_BASE+4*Label+8,4);
}

void WriteFlashParameter(uint16_t Label,
                         float WriteData,
                         FLIGHT_PARAMETER *Table)
{
  ReadFlashParameterALL(Table);//先把片区内的所有数据都都出来值
  Table->Parameter_Table[Label]=WriteData;//将需要更改的字段赋新
  EEPROMProgram((uint32_t *)(&Table->Parameter_Table[Label]),WP_FLASH_BASE+4*Label,4);
}

void WriteFlashParameter_Two(uint16_t Label,
                             float WriteData1,
                             float WriteData2,
                             FLIGHT_PARAMETER *Table)
{
  ReadFlashParameterALL(Table);//先把片区内的所有数据都都出来
  Table->Parameter_Table[Label]=WriteData1;//将需要更改的字段赋新值
  Table->Parameter_Table[Label+1]=WriteData2;//将需要更改的字段赋新值
  EEPROMProgram((uint32_t *)(&Table->Parameter_Table[Label]),WP_FLASH_BASE+4*Label,8);
}

void WriteFlashParameter_Three(uint16_t Label,
                               float WriteData1,
                               float WriteData2,
                               float WriteData3,
                               FLIGHT_PARAMETER *Table)
{
  ReadFlashParameterALL(Table);//先把片区内的所有数据都都出来
  Table->Parameter_Table[Label]=WriteData1;//将需要更改的字段赋新值
  Table->Parameter_Table[Label+1]=WriteData2;//将需要更改的字段赋新值
  Table->Parameter_Table[Label+2]=WriteData3;//将需要更改的字段赋新值
  EEPROMProgram((uint32_t *)(&Table->Parameter_Table[Label]),WP_FLASH_BASE+4*Label,12);
}

*/





