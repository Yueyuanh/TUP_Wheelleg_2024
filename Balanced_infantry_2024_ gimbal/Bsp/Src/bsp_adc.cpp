/**
 ******************************************************************************
 * @file    bsp_adc.cpp
 * @author  Xushuang
 * @version V1.0.0 基本完成
 * @date    2023/9/29
 * @brief		ADC函数库
 *					
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "bsp_adc.h"
#include "adc.h"

volatile fp32 voltage_vrefint_proportion = 8.0586080586080586080586080586081e-4f;
static uint16_t AdcxGetChxValue(ADC_HandleTypeDef *ADCx, uint32_t ch)
{
    static ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ch;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;//ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(ADCx, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(ADCx);

    HAL_ADC_PollForConversion(ADCx, 10);
    return (uint16_t)HAL_ADC_GetValue(ADCx);

}

void InitVrefintReciprocal(void)
{
    uint8_t i = 0;
    uint32_t total_adc = 0;
    for(i = 0; i < 200; i++)
    {
        total_adc += AdcxGetChxValue(&hadc1, ADC_CHANNEL_VREFINT);
    }

    voltage_vrefint_proportion = 200 * 1.2f / total_adc;

}

fp32 GetTemprate(void)
{
    uint16_t adcx = 0;
    fp32 temperate;

    adcx = AdcxGetChxValue(&hadc1, ADC_CHANNEL_TEMPSENSOR);
    temperate = (fp32)adcx * voltage_vrefint_proportion;
    temperate = (temperate - 0.76f) * 400.0f + 25.0f;

    return temperate;
}

fp32 GetBatteryVoltage(void)
{
    fp32 voltage;
    uint16_t adcx = 0;

    adcx = AdcxGetChxValue(&hadc3, ADC_CHANNEL_8);
    voltage =  (fp32)adcx * voltage_vrefint_proportion * 10.090909090909090909090909090909f;

    return voltage;
}

uint8_t GetHardwareVersion(void)
{
    uint8_t hardware_version;
    hardware_version = HAL_GPIO_ReadPin(HW0_GPIO_Port, HW0_Pin)
                                | (HAL_GPIO_ReadPin(HW1_GPIO_Port, HW1_Pin)<<1)
                                | (HAL_GPIO_ReadPin(HW2_GPIO_Port, HW2_Pin)<<2);


    return hardware_version;
}

