#ifndef __UI_DRAWING_PROTOCOL
#define __UI_DRAWING_PROTOCOL

#include "referee_data.h"

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
//ͼ������
typedef enum
{
	LINE      = 0,//ֱ��
	RECTANGLE = 1,//����
	CIRCLE    = 2,//��Բ
	OVAL      = 3,//��Բ
	ARC       = 4,//Բ��
	FLOAT     = 5,//������
	INT       = 6,//������
	CHAR      = 7 //�ַ�
}Graphic_Type;

//ͼ�β���
typedef enum
{
	NONE   = 0,/*�ղ���*/
	ADD    = 1,/*����ͼ��*/
	MODIFY = 2,/*�޸�ͼ��*/
	DELETE = 3,/*ɾ��ͼ��*/
}Graphic_Operate;//graphic_data_struct_t��uint32_t operate_tpye

/*ͼ������*/
//bit 6-9ͼ���� ���Ϊ9����С0
//bit 10-13��ɫ
//ͼ����ɫ
typedef enum
{
	RED_BLUE  = 0,//������ɫ	
	YELLOW    = 1,
	GREEN     = 2,
	ORANGE    = 3,
	FUCHSIA   = 4,	/*�Ϻ�ɫ*/
	PINK      = 5,
	CYAN_BLUE = 6,	/*��ɫ*/
	BLACK     = 7,
	WHITE     = 8
}Graphic_Color;

//����ID��data_cmd_id
enum
{
	//0x200-0x02ff 	�����Զ������� ��ʽ  INTERACT_ID_XXXX
	INTERACT_ID_delete_graphic 			= 0x0100,	/*�ͻ���ɾ��ͼ��*/
	INTERACT_ID_draw_one_graphic 		= 0x0101,	/*�ͻ��˻���һ��ͼ��*/
	INTERACT_ID_draw_two_graphic 		= 0x0102,	/*�ͻ��˻���2��ͼ��*/
	INTERACT_ID_draw_five_graphic 	= 0x0103,	/*�ͻ��˻���5��ͼ��*/
	INTERACT_ID_draw_seven_graphic 	= 0x0104,	/*�ͻ��˻���7��ͼ��*/
	INTERACT_ID_draw_char_graphic 	= 0x0110,	/*�ͻ��˻����ַ�ͼ��*/
	INTERACT_ID_bigbome_num					= 0x02ff
};

//ɾ������
typedef enum
{
	NONE_delete    = 0,
	GRAPHIC_delete = 1,
	ALL_delete     = 2
}delete_Graphic_Operate;
//�ͻ���ɾ��ͼ�� �����˼�ͨ�ţ� 0x0301
typedef __packed struct
{
uint8_t operate_type;
uint8_t layer;
} ext_client_custom_graphic_delete_t;
	
//ͼ������
typedef __packed struct
{
uint8_t graphic_name[3]; 		 //ͼ����
uint32_t operate_tpye:3;
uint32_t graphic_tpye:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
uint32_t radius:10;
uint32_t end_x:11;
uint32_t end_y:11;
} graphic_data_struct_t;

//������
typedef __packed struct
{                          
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; 
	uint32_t layer:4;        
	uint32_t color:4;        
	uint32_t start_angle:9;  
	uint32_t end_angle:9;    
	uint32_t width:10;       
	uint32_t start_x:11;    
	uint32_t start_y:11;     
  int number;       
} Int_data_struct_t;

//������
typedef __packed struct
{                          
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; 
	uint32_t layer:4;        
	uint32_t color:4;        
	uint32_t start_angle:9;  
	uint32_t end_angle:9;    
	uint32_t width:10;       
	uint32_t start_x:11;    
	uint32_t start_y:11;     
  uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;            
} Float_data_struct_t;

//�ͻ��˻���һ��ͼ�� �����˼�ͨ�ţ�0x0301
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;

// �ͻ��˻��ƶ���ͼ�� �����˼�ͨ�ţ� 0x0301
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

//�ͻ��˻������ͼ�� �����˼�ͨ�ţ� 0x0301
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

//�ͻ��˻����߸�ͼ�� �����˼�ͨ�ţ� 0x0301
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

//���ַ���
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} ext_client_string_t;

//�ͻ��˵�һ��ͼ�����
typedef __packed struct
{
	xFrameHeader   				 			             txFrameHeader;//֡ͷ(5���ֽ�)
	uint16_t		 				 	                 	 CmdID;//������(2���ֽ�)
	ext_student_interactive_header_data_t    dataFrameHeader;//���ݶ�ͷ�ṹ(6���ֽ�)
	ext_client_custom_graphic_single_t  		 customGraphicData;//���ݶ�(15���ֽ�)
	uint16_t		 						                 FrameTail;//֡β(2���ֽ�)
}ext_SendClientGraph_t;

//�ͻ���ɾ��ͼ��
typedef __packed struct
{
	xFrameHeader   				 			             txFrameHeader;//֡ͷ(5���ֽ�)
	uint16_t		 				 	                 	 CmdID;//������(2���ֽ�)
	ext_student_interactive_header_data_t    dataFrameHeader;//���ݶ�ͷ�ṹ(6���ֽ�)
	ext_client_custom_graphic_delete_t  		 customGraphicDelete;//���ݶ�(2���ֽ�)
	uint16_t		 						                 FrameTail;//֡β(2���ֽ�)
}ext_SendClientGraphDelete_t;

//�̶����ݶγ������ݰ�
typedef __packed struct
{
	xFrameHeader txFrameHeader;			//֡ͷ
	uint16_t  CmdID;										//������
	ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
	ext_client_string_t clientData;//���ݶ�
	uint16_t	FrameTail;								//֡β
}ext_charstring_data_t;

//���͵�float������
typedef __packed struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	Float_data_struct_t clientData;		
	uint16_t	FrameTail;								
}ext_float_one_data_t;

//��������float������
typedef __packed struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	Float_data_struct_t clientData[2];		
	uint16_t	FrameTail;								
}ext_float_two_data_t;

//��������int��ͼ��
typedef __packed struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	Int_data_struct_t clientData[2];		
	uint16_t	FrameTail;								
}ext_int_two_data_t;

//���͵���ͼ��ͼ��
typedef __packed struct
{
	xFrameHeader txFrameHeader;			//֡ͷ
	uint16_t  CmdID;										//������
	ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
	graphic_data_struct_t clientData;		//���ݶ�
	uint16_t	FrameTail;								//֡β
}ext_graphic_one_data_t;

//��������ͼ��ͼ��
typedef __packed struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	graphic_data_struct_t clientData[2];		
	uint16_t	FrameTail;								
}ext_graphic_two_data_t;

//�������ͼ��ͼ��
typedef __packed struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	graphic_data_struct_t clientData[5];		
	uint16_t	FrameTail;								
}ext_graphic_five_data_t;

//�����߸�ͼ��ͼ��
typedef __packed struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	graphic_data_struct_t clientData[7];		
	uint16_t	FrameTail;								
}ext_graphic_seven_data_t;

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
									
									const char *character);

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
									uint32_t end_y);


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
									float number);

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
									int number);	

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
									uint32_t end_y);

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
									float number);

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
									int number);									
#endif

#ifdef __cplusplus
}
#endif

#endif
