#ifndef LED_H
#define LED_H

#include "struct_typedef.h"
#include "board_def.h"

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
/*�������ȼ�˳���ŷţ����Ƚ���ʾ����״̬��ָʾ�����ڸ����ȼ�
	���ȼ��Ӹߵ��ͣ�0->��
	��������ʾ������������
	��������ʾ���������״̬
	��˸������ĳ�ֲ���
	��������������״̬
*/
enum Led_State_e
{
	OFF_LIGHT = 0, //������
	THREE_FLOW,		 //���̻���ɫ��˸��ʼ��
	RED_RAPID_FLASH,  	//��ƿ���
	YELLOW_RAPID_FLASH, //�Ƶƿ���
	RED_TWO_BLINK,    //���˫��
	YELLOW_TWO_BLINK,	//�Ƶ�˫��
	BLUE_TWO_BLINK, 	//����˫��
	RED_SLOW,			//�������
	YELLOW_SLOW,	//�Ƶ�����
	BLUE_SLOW,    //��������
	PURPLE_SLOW,//�ϵ�����
	CYAN_SLOW, //��ɫ����
	GREEN_SLOW,		//�̵�����
	GREEN_ON, //�̵Ƴ���
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
