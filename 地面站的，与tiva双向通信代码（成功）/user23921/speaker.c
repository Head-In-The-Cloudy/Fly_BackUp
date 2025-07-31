#include "speaker.h"






GroundStation_To_F4_Target To_F4_Target; //�������� �ɿش���������



GroundStation_To_TIVA_Target To_TIVA_Target;   //������� Ҫ���͸��ɿص�������




//UART2 ������  ��TIVA  ͨ��
uint8_t _uart2_byte_buf;
uint8_t _uart2_state=0;
uint8_t _uart2_data_buf[255]={0};
uint8_t _uart2_data_len=0;  //������Ž������ݵ�ʱ��� ���ݳ����ֶ�
uint8_t _uart2_data_cnt=0;  //�������� 

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;



unsigned char sdk_data_to_send[20];   //������� ���ݵĻ�����


//����վ���ݽ���
void SDK_Data_Receive_Prepare_From_TIVA(uint8_t data)
{
	
  if(_uart2_state==0&&data==0xFF)//֡ͷ1
  {
    _uart2_state=1;
    _uart2_data_buf[0]=data;
  }
  else if(_uart2_state==1&&data==0xFC)//֡ͷ2
  {
    _uart2_state=2;
    _uart2_data_buf[1]=data;
  }
  else if(_uart2_state==2&&data<0XFF)//�����ֽ�
  {
    _uart2_state=3;
    _uart2_data_buf[2]=data;
  }
  else if(_uart2_state==3&&data<50)//���ݳ���
  {
    _uart2_state = 4;
    _uart2_data_buf[3]=data;
    _uart2_data_len = data;
    _uart2_data_cnt = 0;
  }
  else if(_uart2_state==4&&_uart2_data_len>0)//�ж������ݳ��ȣ��ʹ���ٸ�
  {
    _uart2_data_len--;
   _uart2_data_buf[4+_uart2_data_cnt++]=data;
    if(_uart2_data_len==0)  _uart2_state = 5;
  }
  else if( _uart2_state==5)//����������У���
  {
     _uart2_state = 0;
    _uart2_data_buf[4+_uart2_data_cnt]=data;
		Data_Receive_Unpack_From_TIVA(_uart2_data_buf,_uart2_data_cnt+5,&To_F4_Target);
  }
  else 
	{
		_uart2_state = 0;
	}
	
	
}
	
	//��TIVA�������� ������Ϣ ��� To_F4_Target�����ݵ���   
//�ڴ������ݽ����������ȷ ֮�� ����λ һ����־λ��Find_Animal_HERE=1
//�����������л� ���������To_F4_Target->Find_Animal_HERE ������ѯ  �����ֱ�����λ  �ͷ��͸�TJC  ��֪������Ϣ

void Data_Receive_Unpack_From_TIVA(uint8_t *data_buf,uint8_t num,GroundStation_To_F4_Target* target)
{
    uint8_t sum = 0;

    // ֡ͷ�ж�
    if(!(data_buf[0]==0xFF && data_buf[1]==0xFC)) return;

    // У���
    for(uint8_t i=0;i<(num-1);i++)  sum+=data_buf[i];
    if(sum != data_buf[num-1]) return;

    // �������������Ͷ���2 * animal_type_num��
    target->animal_type_num = (data_buf[3] -2)/ 2;

    // ����������ʼ�� data_buf[4]
    for(int i=0;i<target->animal_type_num;i++)
    {
        target->animal_all[i].animal_type = data_buf[4 + 2*i];
        target->animal_all[i].animal_num  = data_buf[4 + 2*i + 1];
    }

    // A �� B �����ڶ������ݺ���
    target->Find_Animal_Rigion.A = data_buf[4 + 2*target->animal_type_num];
    target->Find_Animal_Rigion.B = data_buf[4 + 2*target->animal_type_num + 1];

    // ��־λ
    target->Find_Aniaml_Flag = 1;
}










//�� ��������Ϣ  ��֪ ����TIVA
//void GroundStation_Send_To_TIVA(GroundStation_To_TIVA_Target* target)
//{
//  sdk_data_to_send[0]=0xFF;
//  sdk_data_to_send[1]=0xFC;
//  sdk_data_to_send[2]=0xA0;
//  sdk_data_to_send[3]=0x02*target->Fobidden_Rigion_Num;
//	//sdk_data_to_send[4]=target->Fobidden_Rigion_Num;    // ���͸� ����TIVA�� �ǲ�����������
//	for(int tem=0;tem<target->Fobidden_Rigion_Num;tem++)
//	{
//			sdk_data_to_send[4+2*tem]=target->Fobidden_Rigion[tem].A;
//			sdk_data_to_send[4+2*tem+1]=target->Fobidden_Rigion[tem].B;
//		
//	}
//  uint8_t sum = 0;
//  for(uint8_t i=0;i<(4+2*target->Fobidden_Rigion_Num);i++) sum += sdk_data_to_send[i];
//  sdk_data_to_send[4+2*target->Fobidden_Rigion_Num]=sum;  //У���
//  //for(int i=0;i<5+2*target->Fobidden_Rigion_Num;i++)
//			HAL_UART_Transmit(&huart2,sdk_data_to_send,5+2*target->Fobidden_Rigion_Num,HAL_MAX_DELAY);	
//}



void GroundStation_Send_To_TIVA(GroundStation_To_TIVA_Target* target) {
    // ����֡ͷ������
    sdk_data_to_send[0] = 0xFF;
    sdk_data_to_send[1] = 0xFC;
    sdk_data_to_send[2] = 0xA0;
    sdk_data_to_send[3] = 0x02 * target->Fobidden_Rigion_Num;

    // ����ֹ��������
    for (int tem = 0; tem < target->Fobidden_Rigion_Num; tem++) {
        sdk_data_to_send[4 + 2 * tem] = target->Fobidden_Rigion[tem].A;
        sdk_data_to_send[4 + 2 * tem + 1] = target->Fobidden_Rigion[tem].B;
    }
		uint8_t sum=0;
    // ����У���
    for (uint8_t i = 0; i < (4 + 2 * target->Fobidden_Rigion_Num); i++)
        sum += sdk_data_to_send[i];
    
    sdk_data_to_send[4 + 2 * target->Fobidden_Rigion_Num] = sum;  // У���

    // ��������
    HAL_UART_Transmit(&huart2, sdk_data_to_send, 5 + 2 * target->Fobidden_Rigion_Num, HAL_MAX_DELAY);
}











