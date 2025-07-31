#include "speaker.h"






GroundStation_To_F4_Target To_F4_Target; //用来接收 飞控传来的数据



GroundStation_To_TIVA_Target To_TIVA_Target;   //用来存放 要发送给飞控的数数据




//UART2 缓冲区  和TIVA  通信
uint8_t _uart2_byte_buf;
uint8_t _uart2_state=0;
uint8_t _uart2_data_buf[255]={0};
uint8_t _uart2_data_len=0;  //用来存放解析数据的时候的 数据长度字段
uint8_t _uart2_data_cnt=0;  //用来计数 

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;



unsigned char sdk_data_to_send[20];   //打包发送 数据的缓冲区


//地面站数据解析
void SDK_Data_Receive_Prepare_From_TIVA(uint8_t data)
{
	
  if(_uart2_state==0&&data==0xFF)//帧头1
  {
    _uart2_state=1;
    _uart2_data_buf[0]=data;
  }
  else if(_uart2_state==1&&data==0xFC)//帧头2
  {
    _uart2_state=2;
    _uart2_data_buf[1]=data;
  }
  else if(_uart2_state==2&&data<0XFF)//功能字节
  {
    _uart2_state=3;
    _uart2_data_buf[2]=data;
  }
  else if(_uart2_state==3&&data<50)//数据长度
  {
    _uart2_state = 4;
    _uart2_data_buf[3]=data;
    _uart2_data_len = data;
    _uart2_data_cnt = 0;
  }
  else if(_uart2_state==4&&_uart2_data_len>0)//有多少数据长度，就存多少个
  {
    _uart2_data_len--;
   _uart2_data_buf[4+_uart2_data_cnt++]=data;
    if(_uart2_data_len==0)  _uart2_state = 5;
  }
  else if( _uart2_state==5)//最后接收数据校验和
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
	
	//把TIVA传过来的 禁区信息 存进 To_F4_Target的内容当中   
//在串口数据解析完毕且正确 之后 会置位 一个标志位，Find_Animal_HERE=1
//在主函数当中会 对这个变量To_F4_Target->Find_Animal_HERE 进行轮询  当发现变量置位  就发送给TJC  告知更新信息

void Data_Receive_Unpack_From_TIVA(uint8_t *data_buf,uint8_t num,GroundStation_To_F4_Target* target)
{
    uint8_t sum = 0;

    // 帧头判断
    if(!(data_buf[0]==0xFF && data_buf[1]==0xFC)) return;

    // 校验和
    for(uint8_t i=0;i<(num-1);i++)  sum+=data_buf[i];
    if(sum != data_buf[num-1]) return;

    // 解析数量（发送端是2 * animal_type_num）
    target->animal_type_num = (data_buf[3] -2)/ 2;

    // 动物数据起始于 data_buf[4]
    for(int i=0;i<target->animal_type_num;i++)
    {
        target->animal_all[i].animal_type = data_buf[4 + 2*i];
        target->animal_all[i].animal_num  = data_buf[4 + 2*i + 1];
    }

    // A 和 B 紧接在动物数据后面
    target->Find_Animal_Rigion.A = data_buf[4 + 2*target->animal_type_num];
    target->Find_Animal_Rigion.B = data_buf[4 + 2*target->animal_type_num + 1];

    // 标志位
    target->Find_Aniaml_Flag = 1;
}










//把 禁飞区信息  告知 主控TIVA
//void GroundStation_Send_To_TIVA(GroundStation_To_TIVA_Target* target)
//{
//  sdk_data_to_send[0]=0xFF;
//  sdk_data_to_send[1]=0xFC;
//  sdk_data_to_send[2]=0xA0;
//  sdk_data_to_send[3]=0x02*target->Fobidden_Rigion_Num;
//	//sdk_data_to_send[4]=target->Fobidden_Rigion_Num;    // 发送给 主控TIVA的 是不定长的数据
//	for(int tem=0;tem<target->Fobidden_Rigion_Num;tem++)
//	{
//			sdk_data_to_send[4+2*tem]=target->Fobidden_Rigion[tem].A;
//			sdk_data_to_send[4+2*tem+1]=target->Fobidden_Rigion[tem].B;
//		
//	}
//  uint8_t sum = 0;
//  for(uint8_t i=0;i<(4+2*target->Fobidden_Rigion_Num);i++) sum += sdk_data_to_send[i];
//  sdk_data_to_send[4+2*target->Fobidden_Rigion_Num]=sum;  //校验和
//  //for(int i=0;i<5+2*target->Fobidden_Rigion_Num;i++)
//			HAL_UART_Transmit(&huart2,sdk_data_to_send,5+2*target->Fobidden_Rigion_Num,HAL_MAX_DELAY);	
//}



void GroundStation_Send_To_TIVA(GroundStation_To_TIVA_Target* target) {
    // 设置帧头和命令
    sdk_data_to_send[0] = 0xFF;
    sdk_data_to_send[1] = 0xFC;
    sdk_data_to_send[2] = 0xA0;
    sdk_data_to_send[3] = 0x02 * target->Fobidden_Rigion_Num;

    // 填充禁止区域数据
    for (int tem = 0; tem < target->Fobidden_Rigion_Num; tem++) {
        sdk_data_to_send[4 + 2 * tem] = target->Fobidden_Rigion[tem].A;
        sdk_data_to_send[4 + 2 * tem + 1] = target->Fobidden_Rigion[tem].B;
    }
		uint8_t sum=0;
    // 计算校验和
    for (uint8_t i = 0; i < (4 + 2 * target->Fobidden_Rigion_Num); i++)
        sum += sdk_data_to_send[i];
    
    sdk_data_to_send[4 + 2 * target->Fobidden_Rigion_Num] = sum;  // 校验和

    // 发送数据
    HAL_UART_Transmit(&huart2, sdk_data_to_send, 5 + 2 * target->Fobidden_Rigion_Num, HAL_MAX_DELAY);
}











