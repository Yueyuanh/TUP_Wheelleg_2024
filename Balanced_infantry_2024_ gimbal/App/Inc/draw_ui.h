#ifndef __DRAW_UI_H
#define __DRAW_UI_H

#include "ui_drawing_protocol.h"
#include "message_center.h"

#define X_CIRCLE_O 250
#define Y_CIRCLE_O 540
#define Y_Circle_error 40

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
class UI_t
{
	private:

		typedef struct{
			bool spin_flag;
			bool auto_flag;
			bool firc_flag;
		}Ui_Info_t;

	public:
		Ui_Info_t   ui_info;
		float       pitch_angle;
		float       yaw_angle;
		float       dump_energy;
	


	public:

		//�ַ���ʼ������
		static void PitchCharInit();
		static void SuperCapCharInit();
		static void FirCharInit();
		static void AutoCharInit();
		static void SpinCharInit();
		static void SideCharInit();
		static void SterCharInit();


		//���ֳ�ʼ������
		static void FloatGroup1Init(fp32 first_data,fp32 second_data);
		//���ָ��º���
		static void FloatGroup1Update(fp32 first_data,fp32 second_data);
		//ͼ�γ�ʼ������
		static void GraphicGroup1Init();
		static void GraphicGroup2Init();
		//ͼ�θ��º���
		static void GraphicGroup1Update();
		static void GraphicGroup2Update();

};
void UiInit(void);
void UiInfoSend(void);	
	
#endif
void UiTask(void const *pvParameters);
#ifdef __cplusplus
}
#endif

#endif
