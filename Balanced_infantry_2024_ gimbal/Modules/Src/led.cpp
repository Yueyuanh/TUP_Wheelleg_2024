/**
 ******************************************************************************
 * @file    led.cpp
 * @author  Xushuang
 * @version V1.0.0 Xushuang �������  2023/9/27
 * 					V2.0.0 Xushuang ���Ӹ�����˸��ʽ��װ 2023/10/20
 *					V2.1.0 Xushuang �޸���˸������DWT��ʱ�������ã��� 2023/10/31
 *					V2.2.0 Xushuang ����FlowLEDUpdate���� 2023/11/3
 *					V2.3.0 Xushuang �������ȼ��ж� 2023/11/11
 * @date    2023/11/11
 * @brief		�˴�ΪLED�Ƶĺ�����,��ʱ��˸�͵��
 ******************************************************************************
 * @attention
 *	���ȼ��ж�����Led_State_e��Ƕ�룬��һ��Ϊ���ȼ���ߣ�����µ�Ϊ���ȼ����
 *	����չ������˸����ʱ��led.h��Led_State_e�������˸�������ƣ�����Ligthten������
 *����µ�case����˸���谴�����ȼ�˳������
 *	���������ȼ�����ɰ��������Ų������ֵ����⡣
 *	���ȼ�����˳�򣺲��� > ��ʾ����״̬ > ��ʾ���ڽ���ĳ�ֲ��� > ��ͬģʽ
 ******************************************************************************
 */
#include "led.h"
#include "bsp_pwm.h"
#include "bsp_dwt.h"

/**
  * @brief          LED����
  * @param[in]      aRGB: RGB����
  * @retval         NULL
  */
void aRGBLedShow(uint32_t aRGB)
{
    static uint8_t alpha;
    static uint16_t red,green,blue;

    alpha = (aRGB & 0xFF000000) >> 24;
    red = ((aRGB & 0x00FF0000) >> 16) * alpha;
    green = ((aRGB & 0x0000FF00) >> 8) * alpha;
    blue = ((aRGB & 0x000000FF) >> 0) * alpha;

    TIMSetPWM(&htim5, TIM_CHANNEL_1, blue);
    TIMSetPWM(&htim5, TIM_CHANNEL_2, green);
    TIMSetPWM(&htim5, TIM_CHANNEL_3, red);
}

/**
  * @brief          LED��˸��ͨ��������
  * @param[in]      aRGB: RGB����
  * @param[in]      count_out: ������˸����(��λ����)
  * @retval         NULL
  */
void BlinkLEDByCount(uint32_t aRGB,fp32 count_out)
{
	static fp32 count;
	if(count++ <= count_out/2){
		aRGBLedShow(aRGB);
	}else{
		aRGBLedShow((0x00FFFFFF && aRGB));
		if(count >= count_out)
			count = 0;
	}
}

/**
  * @brief          LED��˸
  * @param[in]      aRGB: RGB����
  * @param[in]      period: ������˸����(��λ��ms)
  * @retval         NULL
  */
void BlinkLED(uint32_t aRGB,fp32 period)
{
	static float record_time = DWT_GetTimeline_ms();
	float now_time = DWT_GetTimeline_ms();;
	
	if(period == 0){
		aRGBLedShow(aRGB);
		return;
	}
	
	if(now_time - record_time < period*0.5f){
		aRGBLedShow(aRGB);
	}else if(now_time - record_time < period){
		aRGBLedShow((0x00FFFFFF && aRGB));
	}else{
		record_time = DWT_GetTimeline_ms();
	}
}

/**
  * @brief          ��ˮ����˸����
  * @param[in]      *flow_aRGB: ָ��������ɫ��ӦaRGBֵ������
	* @param[in]      per_blink_period: ����ÿһ�׶Σ���ͬ��ɫ����˸����(��λ��ms)
  * @param[in]      change_period: ����ÿ�θı���˸��ɫ��ʱ��(��λ��ms)
	* @param[in]      numBlinks: ���õ�λdelay��˸����
  * @retval         NULL
  */
uint8_t FlowLEDUpdate(uint32_t *flow_aRGB,fp32 per_blink_period,fp32 change_period,uint8_t flow_num)
{
	static fp32 last_time = DWT_GetTimeline_ms();
	static uint8_t cnt;

	if(CheckTimePeriod(&last_time,change_period))
		cnt = (cnt + 1) % flow_num;
		
	BlinkLED(flow_aRGB[cnt],per_blink_period);
	return cnt;
}

/**
  * @brief          LEDָ��������˸
  * @param[in]      aRGB: RGB����
  * @param[in]      period: ������˸����(��λ��ms)
	* @param[in]      numBlinks: ���õ�λdelay��˸����
  * @retval         NULL
  */
void BlinkLEDWithDelay(uint32_t aRGB,fp32 period,uint8_t numBlinks)
{
	static float last_time = DWT_GetTimeline_ms();
	static uint8_t cnt = 0;
	float now_time = DWT_GetTimeline_ms();
	
	if(period <= 0 || numBlinks == 0)
		return;
	
	if(cnt < numBlinks){
		if(now_time - last_time < 150){
			aRGBLedShow(aRGB);
		}else if(now_time - last_time < 300){
			aRGBLedShow((0x00FFFFFF && aRGB));
		}else{
			cnt++;
		  last_time = DWT_GetTimeline_ms();
		}
	}else{
		if(now_time - last_time > period){
			cnt = 0;
			last_time = DWT_GetTimeline_ms();
		}
	}
	
}

/**
  * @brief          �����ȼ��ߵ��ж�������˸����
  * @param[in]      input_type: ��˸����
  * @retval         Led_State_e���ȽϺ����˸����
  */
Led_State_e SetLEDWorkType(Led_State_e input_type)
{
	static Led_State_e last_type = GREEN_ON;// = OFF_LIGHT;
	static float time =  DWT_GetTimeline_ms();
	float now_time = DWT_GetTimeline_ms();
	
	//�����ȼ��������� ���� �����ȼ�����˸������ʧ
	if((last_type > input_type) || (last_type < input_type && (now_time - time) > 500)){
		last_type = input_type;
		time = DWT_GetTimeline_ms();
	}
	return last_type;
}

/**
  * @brief          ����ͬ��˸���͵���LED�����������ȼ�˳���ŷţ�
  * @param[in]      input_type: ��˸����
  * @retval         NULL
  */
void Ligthten(Led_State_e input_type)
{
	static uint32_t three_flow_aRGB[]={0xFFFF0000,0xFFFFFF00,0xFF00FF00};

	switch(input_type){
		//������ȼ�
		case OFF_LIGHT:
			aRGBLedShow(0x00000000);
			break;
		
		case THREE_FLOW:	//��ɫ��
			FlowLEDUpdate(three_flow_aRGB,500,500,3);
			break;
		
		case RED_RAPID_FLASH:	//��ɫ����
			BlinkLED(0xFFFF0000,100);
			break;
		
		case YELLOW_RAPID_FLASH: //��ɫ����
			BlinkLED(0xFFFFFF00,100);
			break;
		
		case RED_TWO_BLINK: //��ɫ˫��
			BlinkLEDWithDelay(0xFFFF0000,1000,2);
			break;
		
		case YELLOW_TWO_BLINK: //��ɫ˫��
			BlinkLEDWithDelay(0xFFFFFF00,1000,2);
			break;
		
		case BLUE_TWO_BLINK: //��ɫ˫��
			BlinkLEDWithDelay(0xFF0000FF,1000,2);
			break;
		
		case RED_SLOW: //��ɫ����
			BlinkLED(0xFFFF0000,1000);
			break;
		
		case YELLOW_SLOW: //��ɫ����
			BlinkLED(0xFFFFFF00,1000);
			break;
		
		case BLUE_SLOW: //��ɫ����
			BlinkLED(0xFF0000FF,1000);
			break;
		
		case PURPLE_SLOW: //��ɫ����
			BlinkLED(0xFFFF00FF,1000);
			break;
		
		case CYAN_SLOW: //��ɫ����
			BlinkLED(0xFF00FFFF,1000);
			break;
		
		case GREEN_SLOW: //��ɫ����
			BlinkLED(0xFF00FF00,1000);
			break;
		//������ȼ�
		case GREEN_ON: //��ɫ����
			aRGBLedShow(0xFF00FF00);
			break;
		default:
			break;
	}
}
