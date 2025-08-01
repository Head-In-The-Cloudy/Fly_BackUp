#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#include <stdbool.h>


void I2C_GPIO_Config(void);
void Single_WriteI2C0(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);
unsigned char Single_ReadI2C0(unsigned char SlaveAddress,unsigned char REG_Address);
void i2c0ReadNByte(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length);
bool i2cWriteBytes(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data);
bool i2cReadBytes(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data);
uint8_t i2cReadOneBytes(uint8_t dev, uint8_t reg);


#endif

