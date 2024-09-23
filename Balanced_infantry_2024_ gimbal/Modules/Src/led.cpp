/**
 ******************************************************************************
 * @file    led.cpp
 * @author  Xushuang
 * @version V1.0.0 Xushuang 基本完成  2023/9/27
 * 					V2.0.0 Xushuang 增加各类闪烁形式封装 2023/10/20
 *					V2.1.0 Xushuang 修改闪烁函数（DWT延时函数勿用！） 2023/10/31
 *					V2.2.0 Xushuang 增加FlowLEDUpdate函数 2023/11/3
 *					V2.3.0 Xushuang 增加优先级判断 2023/11/11
 * @date    2023/11/11
 * @brief		此处为LED灯的函数库,定时闪烁和点灯
 ******************************************************************************
 * @attention
 *	优先级判断已在Led_State_e中嵌入，第一个为优先级最高，最底下的为优先级最低
 *	需扩展增添闪烁类型时在led.h的Led_State_e中添加闪烁类型名称，并在Ligthten函数中
 *添加新的case和闪烁。需按照优先级顺序排序。
 *	按以下优先级排序可帮助快速排查程序出现的问题。
 *	优先级排序顺序：不亮 > 显示错误状态 > 表示正在进行某种操作 > 不同模式
 ******************************************************************************
 */
#include "led.h"
#include "bsp_pwm.h"
#include "bsp_dwt.h"

/**
  * @brief          LED点亮
  * @param[in]      aRGB: RGB参数
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
  * @brief          LED闪烁（通过计数）
  * @param[in]      aRGB: RGB参数
  * @param[in]      count_out: 设置闪烁周期(单位：次)
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
  * @brief          LED闪烁
  * @param[in]      aRGB: RGB参数
  * @param[in]      period: 设置闪烁周期(单位：ms)
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
  * @brief          流水灯闪烁更新
  * @param[in]      *flow_aRGB: 指向所需颜色对应aRGB值的数组
	* @param[in]      per_blink_period: 设置每一阶段（相同颜色）闪烁周期(单位：ms)
  * @param[in]      change_period: 设置每次改变闪烁颜色的时间(单位：ms)
	* @param[in]      numBlinks: 设置单位delay闪烁次数
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
  * @brief          LED指定次数闪烁
  * @param[in]      aRGB: RGB参数
  * @param[in]      period: 设置闪烁周期(单位：ms)
	* @param[in]      numBlinks: 设置单位delay闪烁次数
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
  * @brief          按优先级高低判断最终闪烁类型
  * @param[in]      input_type: 闪烁类型
  * @retval         Led_State_e：比较后的闪烁类型
  */
Led_State_e SetLEDWorkType(Led_State_e input_type)
{
	static Led_State_e last_type = GREEN_ON;// = OFF_LIGHT;
	static float time =  DWT_GetTimeline_ms();
	float now_time = DWT_GetTimeline_ms();
	
	//按优先级正常运行 或者 低优先级的闪烁类型消失
	if((last_type > input_type) || (last_type < input_type && (now_time - time) > 500)){
		last_type = input_type;
		time = DWT_GetTimeline_ms();
	}
	return last_type;
}

/**
  * @brief          按不同闪烁类型点亮LED（尽量按优先级顺序排放）
  * @param[in]      input_type: 闪烁类型
  * @retval         NULL
  */
void Ligthten(Led_State_e input_type)
{
	static uint32_t three_flow_aRGB[]={0xFFFF0000,0xFFFFFF00,0xFF00FF00};

	switch(input_type){
		//最高优先级
		case OFF_LIGHT:
			aRGBLedShow(0x00000000);
			break;
		
		case THREE_FLOW:	//三色闪
			FlowLEDUpdate(three_flow_aRGB,500,500,3);
			break;
		
		case RED_RAPID_FLASH:	//红色快闪
			BlinkLED(0xFFFF0000,100);
			break;
		
		case YELLOW_RAPID_FLASH: //黄色快闪
			BlinkLED(0xFFFFFF00,100);
			break;
		
		case RED_TWO_BLINK: //红色双闪
			BlinkLEDWithDelay(0xFFFF0000,1000,2);
			break;
		
		case YELLOW_TWO_BLINK: //黄色双闪
			BlinkLEDWithDelay(0xFFFFFF00,1000,2);
			break;
		
		case BLUE_TWO_BLINK: //蓝色双闪
			BlinkLEDWithDelay(0xFF0000FF,1000,2);
			break;
		
		case RED_SLOW: //红色慢闪
			BlinkLED(0xFFFF0000,1000);
			break;
		
		case YELLOW_SLOW: //黄色慢闪
			BlinkLED(0xFFFFFF00,1000);
			break;
		
		case BLUE_SLOW: //蓝色慢闪
			BlinkLED(0xFF0000FF,1000);
			break;
		
		case PURPLE_SLOW: //紫色慢闪
			BlinkLED(0xFFFF00FF,1000);
			break;
		
		case CYAN_SLOW: //青色慢闪
			BlinkLED(0xFF00FFFF,1000);
			break;
		
		case GREEN_SLOW: //绿色慢闪
			BlinkLED(0xFF00FF00,1000);
			break;
		//最低优先级
		case GREEN_ON: //绿色常亮
			aRGBLedShow(0xFF00FF00);
			break;
		default:
			break;
	}
}
