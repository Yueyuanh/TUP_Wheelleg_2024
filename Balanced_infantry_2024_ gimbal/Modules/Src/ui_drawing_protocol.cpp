/**
  ****************************(C) COPYRIGHT 2023 TUP****************************
  * @file       ui_drawing_protocol.cpp/h
  * @brief      Judge system data reading, C board through the serial port one for data transmission,
  *							A board through the serial port three.For data transmission, the specific protocol 
	*							reference serial port the appendix.
  *							This file is forbidden to be modified.
  *             ����ϵͳ���ݶ�ȡ��C��ͨ������һ�������ݴ��ݣ�A��ͨ��������
	*							�������ݴ��ݣ�����Э��ο�����Э�鸽¼
	*							�������ļ���ֹ�޸�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Aug-25-2023     Xushuang        1. done
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 TUP****************************
*/
#include "ui_drawing_protocol.h"
#include "string.h"

/*********************�����ַ�********************/
char empty_line[30] = {"                             "};
void Char_Graphic(ext_client_string_t* graphic,//����Ҫ����ȥ�������е����ݶ�����
									const char* name,
									uint32_t operate_tpye,
									
									uint32_t layer,
									uint32_t color,
									uint32_t size,
									uint32_t length,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,					
									
									const char *character)//�ⲿ���������
{
	graphic_data_struct_t *data_struct = &graphic->grapic_data_struct;
	for(char i=0;i<3;i++)
		data_struct->graphic_name[i] = name[i];	//�ַ�����
	data_struct->operate_tpye = operate_tpye; //ͼ�����
	data_struct->graphic_tpye = CHAR;         //Char��
	data_struct->layer = layer;//���ڵ����
	data_struct->color = color;//���ǰ�ɫ
	data_struct->start_angle = size;
	data_struct->end_angle = length;	
	data_struct->width = width;
	data_struct->start_x = start_x;
	data_struct->start_y = start_y;	
	
	data_struct->radius = 0;
	data_struct->end_x = 0;
	data_struct->end_y = 0;
	
	memcpy(graphic->data,empty_line,19);
  memcpy(graphic->data,character,length);
}

//************************************��������*******************************/
void Figure_Graphic(graphic_data_struct_t* graphic,//����Ҫ����ȥ����������ݶ�����
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//����ʲôͼ��
									uint32_t layer,
									uint32_t color,
									uint32_t start_angle,
									uint32_t end_angle,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									uint32_t radius,
									uint32_t end_x,
									uint32_t end_y)							
{
	for(char i=0;i<3;i++)
		graphic->graphic_name[i] = name[i];	//�ַ�����
	graphic->operate_tpye = operate_tpye; //ͼ�����
	graphic->graphic_tpye = graphic_tpye;         //Char��
	graphic->layer        = layer;//���ڵ�һ��
	graphic->color        = color;//��ɫ
	graphic->start_angle  = start_angle;
	graphic->end_angle    = end_angle;	
	graphic->width        = width;
	graphic->start_x      = start_x;
	graphic->start_y      = start_y;	
	graphic->radius = radius;
	graphic->end_x  = end_x;
	graphic->end_y  = end_y;
}

/******************���Ƹ�����*************************/

void Float_Graphic(Float_data_struct_t* graphic,//����Ҫ����ȥ����������ݶ�����
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//����ʲôͼ��
									uint32_t layer,
									uint32_t color,
									uint32_t size,
									uint32_t decimal,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									float number)							
{
	for(char i=0;i<3;i++)
		graphic->graphic_name[i] = name[i];	//�ַ�����
	graphic->operate_tpye = operate_tpye; //ͼ�����
	graphic->graphic_tpye = graphic_tpye;  
	graphic->layer        = layer;//
	graphic->color        = color;//��ɫ
	graphic->start_angle  = size;
	graphic->end_angle    = decimal;//С����Чλ	
	graphic->width        = width;
	graphic->start_x      = start_x;
	graphic->start_y      = start_y;	
	int32_t IntData = number * 1000;
	graphic->radius          = (IntData & 0x000003ff) >>  0;
	graphic->end_x           = (IntData & 0x001ffc00) >> 10;
	graphic->end_y           = (IntData & 0xffe00000) >> 21;
}
/************************����������****************************/
void Int_Graphic(Int_data_struct_t* graphic,//����Ҫ����ȥ����������ݶ�����
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//����ʲôͼ��
									uint32_t layer,
									uint32_t color,
									uint32_t size,
									uint32_t zero,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									int number)							
{
	for(char i=0;i<3;i++)
		graphic->graphic_name[i] = name[i];	//�ַ�����
	graphic->operate_tpye = operate_tpye; //ͼ�����
	graphic->graphic_tpye = graphic_tpye;        
	graphic->layer        = layer;//���ڵ�һ��
	graphic->color        = color;//��ɫ
	graphic->start_angle  = size;
	graphic->end_angle    = zero;	
	graphic->width        = width;
	graphic->start_x      = start_x;
	graphic->start_y      = start_y;	
	graphic->number       = number;
}

//NEW
void AssignFigureGraphic(
									uint8_t array_num,
									ext_graphic_seven_data_t * graphic,//����Ҫ����ȥ����������ݶ�����
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//����ʲôͼ��
									uint32_t layer,
									uint32_t color,
									uint32_t start_angle,
									uint32_t end_angle,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									uint32_t radius,
									uint32_t end_x,
									uint32_t end_y)							
{
	for(char i=0;i<3;i++)
		graphic->clientData[array_num].graphic_name[i] = name[i];	//�ַ�����
	graphic->clientData[array_num].operate_tpye = operate_tpye; //ͼ�����
	graphic->clientData[array_num].graphic_tpye = graphic_tpye;         //Char��
	graphic->clientData[array_num].layer        = layer;//���ڵ�һ��
	graphic->clientData[array_num].color        = color;//��ɫ
	graphic->clientData[array_num].start_angle  = start_angle;
	graphic->clientData[array_num].end_angle    = end_angle;	
	graphic->clientData[array_num].width        = width;
	graphic->clientData[array_num].start_x      = start_x;
	graphic->clientData[array_num].start_y      = start_y;	
	graphic->clientData[array_num].radius = radius;
	graphic->clientData[array_num].end_x  = end_x;
	graphic->clientData[array_num].end_y  = end_y;
}

void AssignFloatGraphic(
									uint8_t array_num,
									ext_float_two_data_t * graphic,//����Ҫ����ȥ����������ݶ�����
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//����ʲôͼ��
									uint32_t layer,
									uint32_t color,
									uint32_t size,
									uint32_t decimal,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									float number)							
{
	for(char i=0;i<3;i++)
		graphic->clientData[array_num].graphic_name[i] = name[i];	//�ַ�����
	graphic->clientData[array_num].operate_tpye = operate_tpye; //ͼ�����
	graphic->clientData[array_num].graphic_tpye = graphic_tpye;         //Char��
	graphic->clientData[array_num].layer        = layer;//���ڵ�һ��
	graphic->clientData[array_num].color        = color;//��ɫ
	graphic->clientData[array_num].start_angle  = size;
	graphic->clientData[array_num].end_angle    = decimal;//С����Чλ	
	graphic->clientData[array_num].width        = width;
	graphic->clientData[array_num].start_x      = start_x;
	graphic->clientData[array_num].start_y      = start_y;	
	int32_t IntData = number * 1000;
	graphic->clientData[array_num].radius = (IntData & 0x000003ff) >>  0;
	graphic->clientData[array_num].end_x  = (IntData & 0x001ffc00) >> 10;
	graphic->clientData[array_num].end_y  = (IntData & 0x001ffc00) >> 21;
}

void AssignIntGraphic(
									uint8_t array_num,
									ext_int_two_data_t * graphic,//����Ҫ����ȥ����������ݶ�����
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//����ʲôͼ��
									uint32_t layer,
									uint32_t color,
									uint32_t size,
									uint32_t zero,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									int number)							
{
	for(char i=0;i<3;i++)
		graphic->clientData[array_num].graphic_name[i] = name[i];	//�ַ�����
	graphic->clientData[array_num].operate_tpye = operate_tpye; //ͼ�����
	graphic->clientData[array_num].graphic_tpye = graphic_tpye;         //Char��
	graphic->clientData[array_num].layer        = layer;//���ڵ�һ��
	graphic->clientData[array_num].color        = color;//��ɫ
	graphic->clientData[array_num].start_angle  = size;
	graphic->clientData[array_num].end_angle    = zero;//С����Чλ	
	graphic->clientData[array_num].width        = width;
	graphic->clientData[array_num].start_x      = start_x;
	graphic->clientData[array_num].start_y      = start_y;	
	graphic->clientData[array_num].number = number;
}
