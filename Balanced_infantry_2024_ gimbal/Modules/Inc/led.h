#ifndef LED_H
#define LED_H

#include "struct_typedef.h"
#include "board_def.h"

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
/*按照优先级顺序排放，优先将表示错误状态的指示灯排在高优先级
	优先级从高到低：0->∞
	快闪：表示紧急或错误情况
	慢闪：表示待命或代码状态
	闪烁：进行某种操作
	常亮：正常工作状态
*/
enum Led_State_e
{
	OFF_LIGHT = 0, //不点亮
	THREE_FLOW,		 //红绿黄三色闪烁初始化
	RED_RAPID_FLASH,  	//红灯快闪
	YELLOW_RAPID_FLASH, //黄灯快闪
	RED_TWO_BLINK,    //红灯双闪
	YELLOW_TWO_BLINK,	//黄灯双闪
	BLUE_TWO_BLINK, 	//蓝灯双闪
	RED_SLOW,			//红灯慢闪
	YELLOW_SLOW,	//黄灯慢闪
	BLUE_SLOW,    //蓝灯慢闪
	PURPLE_SLOW,//紫灯慢闪
	CYAN_SLOW, //青色慢闪
	GREEN_SLOW,		//绿灯慢闪
	GREEN_ON, //绿灯常亮
};

enum Led_Colour_e
{
	LED_RED, LED_GREEN, LED_BLUE, 
	LED_PURPLE, LED_CYAN, LED_RED_YELLOW,
};

void aRGBLedShow(uint32_t aRGB);	
void BlinkLEDByCount(uint32_t aRGB,fp32 countout);
void BlinkLED(uint32_t aRGB,fp32 period);
void BlinkLEDWithDelay(uint32_t aRGB,fp32 period,uint8_t numBlinks);
void Ligthten(Led_State_e input_type);
void SetLedState(Led_State_e input_state,uint16_t priority);
Led_State_e SetLEDWorkType(Led_State_e input_type);
#endif

#ifdef __cplusplus
}
#endif

#endif
