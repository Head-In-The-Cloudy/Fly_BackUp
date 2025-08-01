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
#include "Soft_I2C.h"
#include "myiic.h"

//#define USE_SOFT_I2C

#define SYSCTL_PERIPH_GPIO_I2C SYSCTL_PERIPH_GPIOB
#define GPIO_STRENGTH_I2C GPIO_STRENGTH_4MA
#define GPIO_I2C   		GPIO_PORTB_BASE 
#define SCL_PIN   		GPIO_PIN_2
#define SDA_PIN   		GPIO_PIN_3
#define I2C_READ_SDA  GPIOPinRead(GPIO_I2C,SDA_PIN)			//SDA 
#define I2C_SDA_H  		GPIOPinWrite(GPIO_I2C,SDA_PIN,SDA_PIN)//SDA 
#define I2C_SDA_L 		GPIOPinWrite(GPIO_I2C,SDA_PIN,0)			//SDA
#define I2C_SCL_H  		GPIOPinWrite(GPIO_I2C,SCL_PIN,SCL_PIN)//SCL
#define I2C_SCL_L  		GPIOPinWrite(GPIO_I2C,SCL_PIN,0)			//SCL


void I2C_GPIO_Config(void)
{
#if defined(USE_SOFT_I2C)	
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIO_I2C);
	
	HWREG(GPIO_I2C + GPIO_O_LOCK) = GPIO_LOCK_KEY;//解锁
	HWREG(GPIO_I2C + GPIO_O_CR) |= 0x000000FF;//确认
	HWREG(GPIO_I2C + GPIO_O_LOCK) = 0;//重新锁定
	
  GPIOPinTypeGPIOOutput(GPIO_I2C, SCL_PIN);
  GPIOPinTypeGPIOOutput(GPIO_I2C, SDA_PIN);
	GPIOPadConfigSet(GPIO_I2C,SCL_PIN,GPIO_STRENGTH_I2C,GPIO_PIN_TYPE_STD);
	GPIOPadConfigSet(GPIO_I2C,SDA_PIN,GPIO_STRENGTH_I2C,GPIO_PIN_TYPE_STD);
	
  Delay_Ms(10);
  GPIOPinWrite(GPIO_I2C, SCL_PIN, SCL_PIN);//设置高
  GPIOPinWrite(GPIO_I2C, SDA_PIN, SDA_PIN);//设置高
#else
  Init_I2C0();
#endif	
}

#if defined(USE_SOFT_I2C)
void I2C_SDA_OUT(void)
{
	GPIOPinTypeGPIOOutput(GPIO_I2C, SDA_PIN);
	GPIOPadConfigSet(GPIO_I2C,SDA_PIN,GPIO_STRENGTH_I2C,GPIO_PIN_TYPE_STD);///GPIO_PIN_TYPE_STD);
}

void I2C_SDA_IN(void)
{
  GPIOPinTypeGPIOInput(GPIO_I2C, SDA_PIN);
	GPIOPadConfigSet(GPIO_I2C,SDA_PIN,GPIO_STRENGTH_I2C,GPIO_PIN_TYPE_STD_WPU);
}

#define SDA_OUT I2C_SDA_OUT()
#define SDA_IN  I2C_SDA_IN()
#define sdaRead I2C_READ_SDA  
#define SDA_H   I2C_SDA_H  		 
#define SDA_L   I2C_SDA_L
#define SCL_H   I2C_SCL_H  		
#define SCL_L   I2C_SCL_L  		

uint16_t  hu=5;
static void i2cDelay()
{
//  volatile int i = 100;//7
//  while (i)
//  i--;
	//delay_us(hu);
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
		__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
		__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
		__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
}
 
// SCL高电平期间，SDA出现下降沿为起始信号
static bool i2cStart()
{
    SDA_OUT;
    SDA_H;
	  SCL_H;   
    i2cDelay();
    if (!sdaRead)  // 如果SDA为低电平，则总线忙，退出
        return false;
    SDA_L;
    if (sdaRead)  // 如果SDA为高电平，则总线忙，退出
        return false;
    SDA_L;
    return true;
}
 
// SCL高电平期间，SDA出现上升沿为停止信号
static void i2cStop(void)
{
    SDA_OUT;
    SDA_L;
	  SCL_L;  
    i2cDelay();  // STOP:when CLK is high DATA from low to high 
    //SCL_H;
    SDA_H;  
    i2cDelay();
}
 
static void i2cAck(void)
{
    SCL_L;
	  SDA_OUT; 
    SDA_L;
    i2cDelay();   
    SCL_H;
    i2cDelay();
    SCL_L;
}
 
static void i2cNoAck(void)
{
    SCL_L;
	  SDA_OUT;
    SDA_H;
    i2cDelay();
    SCL_H;
    i2cDelay();
    SCL_L;
}
 
// SCL高电平期间，SDA电平被从设备拉低表示应答
static bool i2cWaitAck(void)
{
    uint16_t errTimes = 0;
    SDA_H;
	  i2cDelay();
	  SDA_IN;
    SCL_H;
    i2cDelay();
    while (sdaRead) {
        if (errTimes++ > 200) //20
				{
            SCL_L;
            return false;
        }           
        i2cDelay();
    }
    SCL_L;
    return true;
}
 
// 发送数据，数据从高位到低位传输  
static void i2cSendByte(uint8_t byte)  
{
    uint8_t i = 8;
 
    SDA_OUT;
    while (i--) {      
        SCL_L;  // 时钟信号为低电平期间，允许数据线电平变化
        i2cDelay();
        if (byte & 0x80)
            SDA_H;
        else
            SDA_L; 
        byte <<= 1; 
        i2cDelay();
        SCL_H;
        i2cDelay();
    }
    SCL_L;
}
 
static uint8_t i2cReceiveByte()  
{
    uint8_t i = 8;
    uint8_t byte = 0;
    SDA_IN;
    SDA_H;
    while (i--) {
        byte <<= 1;
        SCL_H;
        i2cDelay();
        if (sdaRead) {
            byte |= 0x01;
        }
        SCL_L;
        i2cDelay();
    }
    SCL_L;
    return byte; 
}
 
 

 
/**
 * 通过I2C总线写一字节数据
 * @param[in] dev:设备I2C地址
 * @param[in] reg:寄存器地址
 * @param[in] data:要写入的数据
 */
bool i2cWriteOneByte(uint8_t dev, uint8_t reg, uint8_t data)
{
    if (!i2cStart())        
        return false;
    i2cSendByte(dev << 1);  // 从机地址由高7位+读写位构成   
    if (!i2cWaitAck()) {     
        i2cStop();
        return false;
    }
    i2cSendByte(reg);       
    i2cWaitAck();
    i2cSendByte(data);     
    i2cWaitAck();
    return true;
}
 

/**
 *  
 * @param[in] dev:设备I2C地址
 * @param[in] reg:寄存器地址
 * @param[in] len:字节数 
 * @param[in] data:待写入的数据 
 */
uint8_t i2cReadOneBytes(uint8_t dev, uint8_t reg)
{
	unsigned char REG_data;
  i2cStart();
  i2cSendByte(dev<<1);
	i2cWaitAck();
  i2cSendByte(reg);
	i2cWaitAck();
  i2cStart();
  i2cSendByte((dev << 1) | 0x01);
  i2cWaitAck();
	REG_data=i2cReceiveByte();
  i2cStop();
  return REG_data;	
} 


/**
 *  
 * @param[in] dev:设备I2C地址
 * @param[in] reg:寄存器地址
 * @param[in] len:字节数 
 * @param[in] data:待写入的数据 
 */
bool i2cWriteBytes(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data)
{
    uint8_t i;
 
    if (!i2cStart())        
        return false;
    i2cSendByte(dev << 1);          
    if (!i2cWaitAck()) {     
        i2cStop();
        return false;
    }
    i2cSendByte(dev);   
    i2cWaitAck();
    for (i = 0; i < len; i++) {
        i2cSendByte(data[i]);
        if (!i2cWaitAck()) {
            i2cStop();
            return false;
        }
    }
    i2cStop();
    return true;
}
 
 
/**
 * 从I2C设备中读取数据
 * @param[in] dev:设备I2C地址
 * @param[in] reg:寄存器地址
 * @param[in] len:数据字节数
 * @param[out] data:读出的数据
 */
bool i2cReadBytes(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data)
{
    if (!i2cStart())        
        return false;
    i2cSendByte(dev << 1);      
    if (!i2cWaitAck()) {     
        i2cStop();
        return false;
    }
    i2cSendByte(reg);     
    i2cWaitAck();
    i2cStart();           
    i2cSendByte((dev << 1) | 0x01);  // 器件地址+读命令    
    i2cWaitAck();
    while (len) {
        *data = i2cReceiveByte();
        if (len == 1)
            i2cNoAck();  // 最后一个字节不应答
        else
            i2cAck();
        data++;
        len--;
    }
    i2cStop();
    return true;
}
#endif


void Single_WriteI2C0(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)
{
#if defined(USE_SOFT_I2C)
  i2cWriteOneByte(SlaveAddress,REG_Address,REG_data);
#else
  i2c0Write(SlaveAddress,REG_Address,REG_data);
#endif
}

//**************************************
unsigned char Single_ReadI2C0(unsigned char SlaveAddress,unsigned char REG_Address)
{
#if defined(USE_SOFT_I2C)
  return i2cReadOneBytes(SlaveAddress,REG_Address);
#else
  return i2c0Read(SlaveAddress,REG_Address);
#endif
}


void i2c0ReadNByte(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length)
{
#if defined(USE_SOFT_I2C)
  i2cReadBytes(addr, regAddr,length,data);
#else
  i2c0ReadData(addr,regAddr, data,length);
#endif
}

