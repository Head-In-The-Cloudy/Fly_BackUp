#include "nano.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include "speaker.h"


extern  uint8_t _uart2_byte_buf;  
extern  uint8_t _uart1_byte_buf;  

uint8_t Point_cnt=0;
uint8_t get_all_point =0;

vector2f FB_Point[10]={0};


uint8_t uart1_sta=0;
uint8_t uart1_data[255];
uint8_t uart1_cnt=0;   //ÿ�������ֶγ��� ���ǰ�3���ֽ�   


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)   //�жϻص�����
{
	
	
	
	if(huart->Instance == USART2)  //TIVA  ����
	{
		SDK_Data_Receive_Prepare_From_TIVA(_uart2_byte_buf);//  �ѵ��ֽڻ������͵� ֡���ݻ����� ���з�֡
		HAL_UART_Receive_IT(&huart2, &_uart2_byte_buf, 1);	   //�ݹ���� ѭ������		
	}
	
	else if(huart->Instance == USART1)   //������
	{
		   switch(uart1_sta)
			 {
			case 0:
				if(_uart1_byte_buf==0xff) //���յ�֡ͷ
				{			
					uart1_sta++;
				}
				break;
			case 1:
				if(_uart1_byte_buf==0xfe)
				{
					
			
					//��������
					if(uart1_data[0]==0xaa)
					{
								Point_cnt=0;
					}
					else if(uart1_data[0]==0xbb)
					{
							FB_Point[Point_cnt].A=uart1_data[1];
							FB_Point[Point_cnt].B=uart1_data[2];
							Point_cnt++;
						
					}
					else if(uart1_data[0]==0xcc)
					{
							get_all_point=1;
					}
					//��մ˴γɹ���������ݺͼ����Ĳ���	
					_uart1_byte_buf=0;   
					memset(uart1_data,0x00,sizeof(uart1_data));
					uart1_cnt=0;
					uart1_sta=0;
					break;			 

				}
				else 
				{
					uart1_data[uart1_cnt]=_uart1_byte_buf;
					uart1_cnt++;
					
					if(uart1_cnt>=254)   //�������
					{
						_uart1_byte_buf=0;   
						memset(uart1_data,0x00,sizeof(uart1_data));
						uart1_cnt=0;
						uart1_sta=0;
						break;
					}					
				}
				break;
			case 2:
				break;
			default:
				break;
			

			 
			 }
			
		HAL_UART_Receive_IT(&huart1 ,&_uart1_byte_buf, 1);	   //�ݹ���� ѭ������		
			
	
	
	

	}

}

