#include "TJC.h"
#include "string.h"



//UART2 ������  ��TIVA  ͨ��
uint8_t _uart1_byte_buf;
uint8_t _uart1_state=0;
uint8_t _uart1_data_buf[255]={0};
uint8_t _uart1_data_len=0;  //������Ž������ݵ�ʱ��� ���ݳ����ֶ�
uint8_t _uart1_data_cnt=0;  //�������� 

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;







void Send_To_TJC(char* str)
{
	for(int i=0;i<strlen(str);i++)
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)&str[i], 1, 0xffff);
	}
}

