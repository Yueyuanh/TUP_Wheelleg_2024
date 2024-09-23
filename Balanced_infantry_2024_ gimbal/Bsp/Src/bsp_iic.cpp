/**
 ******************************************************************************
 * @file    bsp_ciic.cpp
 * @author  Xushuang
 * @version V1.0.0 基本完成
 * @date    2023/10/1
 * @brief		iic函数库
 *					
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "bsp_iic.h"

void BspI2CMasterTransmit(I2C_HandleTypeDef *I2C, uint16_t I2C_address, uint8_t *data, uint16_t len, uint32_t Timeout)
{
	HAL_I2C_Master_Transmit(I2C, I2C_address, data, len, Timeout);
}


void BspI2CReset(I2C_HandleTypeDef *I2C)
{
    SET_BIT(I2C->Instance->CR1, I2C_CR1_SWRST);
    CLEAR_BIT(I2C->Instance->CR1, I2C_CR1_SWRST);
    if (HAL_I2C_Init(I2C) != HAL_OK)
    {
        Error_Handler();
    }
}

bool_t BspI2CCheckAck(I2C_HandleTypeDef *hi2c, uint16_t I2C_address)
{
    if((hi2c->Instance->CR2 & I2C_CR2_DMAEN) && ((hi2c->hdmatx != NULL && hi2c->hdmatx->Instance->NDTR != 0) || (hi2c->hdmarx != NULL && hi2c->hdmarx->Instance->NDTR != 0))){
			return I2C_ACK;
    }else{
      uint16_t timeout = 0;

      timeout = 0;
      while(hi2c->Instance->SR2 & 0x02){
				timeout ++;
        if(timeout > 100){
					SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
          return I2C_NO_ACK;
        }
      }

      CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_POS);

      SET_BIT(hi2c->Instance->CR1, I2C_CR1_START);

      timeout = 0;
      while(!(hi2c->Instance->SR1 & 0x01)){
				timeout ++;
        if(timeout > 100){
					SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
          return I2C_NO_ACK;
        }
      }

      hi2c->Instance->DR = I2C_7BIT_ADD_WRITE(I2C_address);

      timeout = 0;
      while(!(hi2c->Instance->SR1 & 0x02)){
				timeout ++;
        if(timeout > 500){
					SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
          return I2C_NO_ACK;
        }
      }

      do{
				__IO uint32_t tmpreg = 0x00U;
        tmpreg = hi2c->Instance->SR1;
        tmpreg = hi2c->Instance->SR2;
        UNUSED(tmpreg);
      } while(0);

      timeout = 0;
      while(!(hi2c->Instance->SR1 & 0x80)){
        timeout ++;
        if(timeout > 500){
          SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
          return I2C_NO_ACK;
        }
      }

      SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);

      return I2C_ACK;
     }
}
