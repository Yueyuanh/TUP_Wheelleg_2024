#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#include "struct_typedef.h"
#include "i2c.h"

#define I2C_ACK 1
#define I2C_NO_ACK  0

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
void BspI2CMasterTransmit(I2C_HandleTypeDef *I2C, uint16_t I2C_address, uint8_t *data, uint16_t len, uint32_t Timeout);	
void BspI2CReset(I2C_HandleTypeDef *I2C);
bool_t BspI2CCheckAck(I2C_HandleTypeDef *hi2c, uint16_t I2C_address);
#endif
	
#ifdef __cplusplus
}
#endif

#endif
