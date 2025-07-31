#ifndef _SPEAKER_H_
#define _SPEAKER_H_

#include "main.h"
#include "stm32f4xx_hal_uart.h"
#define  FORBIDDEN_RIGION_MAX_NUM 10 // 最多可以承载的 禁飞点的数量



typedef  struct
{
    uint8_t A;
    uint8_t B;
}vector2f;


typedef struct
{
		uint8_t animal_type;
		uint8_t animal_num;

}Animal;


typedef  struct
{
		uint8_t animal_type_num;
		Animal animal_all[5];
    vector2f Find_Animal_Rigion;//一个二维坐标用来存储 发现动物的位置
		uint8_t Find_Aniaml_Flag;
}GroundStation_To_F4_Target;



typedef  struct
{
    uint8_t Fobidden_Rigion_Num;
    vector2f Fobidden_Rigion[FORBIDDEN_RIGION_MAX_NUM];
    uint8_t Get_Forbidden_Rigion;  //初始化之后是要 赋值 为0 比较保险
}GroundStation_To_TIVA_Target;

void Data_Receive_Unpack_From_TIVA(uint8_t *data_buf,uint8_t num,GroundStation_To_F4_Target* target);
void SDK_Data_Receive_Prepare_From_TIVA(uint8_t data);

void GroundStation_Send_To_TIVA(GroundStation_To_TIVA_Target* target);


#endif
