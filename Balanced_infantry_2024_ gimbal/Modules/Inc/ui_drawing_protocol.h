#ifndef __UI_DRAWING_PROTOCOL
#define __UI_DRAWING_PROTOCOL

#include "referee_data.h"

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
//图形类型
typedef enum
{
	LINE      = 0,//直线
	RECTANGLE = 1,//矩形
	CIRCLE    = 2,//正圆
	OVAL      = 3,//椭圆
	ARC       = 4,//圆弧
	FLOAT     = 5,//浮点数
	INT       = 6,//整型数
	CHAR      = 7 //字符
}Graphic_Type;

//图形操作
typedef enum
{
	NONE   = 0,/*空操作*/
	ADD    = 1,/*增加图层*/
	MODIFY = 2,/*修改图层*/
	DELETE = 3,/*删除图层*/
}Graphic_Operate;//graphic_data_struct_t：uint32_t operate_tpye

/*图层类型*/
//bit 6-9图层数 最大为9，最小0
//bit 10-13颜色
//图层颜色
typedef enum
{
	RED_BLUE  = 0,//红蓝主色	
	YELLOW    = 1,
	GREEN     = 2,
	ORANGE    = 3,
	FUCHSIA   = 4,	/*紫红色*/
	PINK      = 5,
	CYAN_BLUE = 6,	/*青色*/
	BLACK     = 7,
	WHITE     = 8
}Graphic_Color;

//绘制ID：data_cmd_id
enum
{
	//0x200-0x02ff 	队伍自定义命令 格式  INTERACT_ID_XXXX
	INTERACT_ID_delete_graphic 			= 0x0100,	/*客户端删除图形*/
	INTERACT_ID_draw_one_graphic 		= 0x0101,	/*客户端绘制一个图形*/
	INTERACT_ID_draw_two_graphic 		= 0x0102,	/*客户端绘制2个图形*/
	INTERACT_ID_draw_five_graphic 	= 0x0103,	/*客户端绘制5个图形*/
	INTERACT_ID_draw_seven_graphic 	= 0x0104,	/*客户端绘制7个图形*/
	INTERACT_ID_draw_char_graphic 	= 0x0110,	/*客户端绘制字符图形*/
	INTERACT_ID_bigbome_num					= 0x02ff
};

//删除类型
typedef enum
{
	NONE_delete    = 0,
	GRAPHIC_delete = 1,
	ALL_delete     = 2
}delete_Graphic_Operate;
//客户端删除图形 机器人间通信： 0x0301
typedef __packed struct
{
uint8_t operate_type;
uint8_t layer;
} ext_client_custom_graphic_delete_t;
	
//图形数据
typedef __packed struct
{
uint8_t graphic_name[3]; 		 //图形名
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

//整型数
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

//浮点数
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

//客户端绘制一个图形 机器人间通信：0x0301
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;

// 客户端绘制二个图形 机器人间通信： 0x0301
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

//客户端绘制五个图形 机器人间通信： 0x0301
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

//客户端绘制七个图形 机器人间通信： 0x0301
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

//绘字符串
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} ext_client_string_t;

//客户端第一个图形添加
typedef __packed struct
{
	xFrameHeader   				 			             txFrameHeader;//帧头(5个字节)
	uint16_t		 				 	                 	 CmdID;//命令码(2个字节)
	ext_student_interactive_header_data_t    dataFrameHeader;//数据段头结构(6个字节)
	ext_client_custom_graphic_single_t  		 customGraphicData;//数据段(15个字节)
	uint16_t		 						                 FrameTail;//帧尾(2个字节)
}ext_SendClientGraph_t;

//客户端删除图形
typedef __packed struct
{
	xFrameHeader   				 			             txFrameHeader;//帧头(5个字节)
	uint16_t		 				 	                 	 CmdID;//命令码(2个字节)
	ext_student_interactive_header_data_t    dataFrameHeader;//数据段头结构(6个字节)
	ext_client_custom_graphic_delete_t  		 customGraphicDelete;//数据段(2个字节)
	uint16_t		 						                 FrameTail;//帧尾(2个字节)
}ext_SendClientGraphDelete_t;

//固定数据段长度数据包
typedef __packed struct
{
	xFrameHeader txFrameHeader;			//帧头
	uint16_t  CmdID;										//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	ext_client_string_t clientData;//数据段
	uint16_t	FrameTail;								//帧尾
}ext_charstring_data_t;

//发送单float型数据
typedef __packed struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	Float_data_struct_t clientData;		
	uint16_t	FrameTail;								
}ext_float_one_data_t;

//发送两个float型数据
typedef __packed struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	Float_data_struct_t clientData[2];		
	uint16_t	FrameTail;								
}ext_float_two_data_t;

//发送两个int型图层
typedef __packed struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	Int_data_struct_t clientData[2];		
	uint16_t	FrameTail;								
}ext_int_two_data_t;

//发送单个图形图层
typedef __packed struct
{
	xFrameHeader txFrameHeader;			//帧头
	uint16_t  CmdID;										//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	graphic_data_struct_t clientData;		//数据段
	uint16_t	FrameTail;								//帧尾
}ext_graphic_one_data_t;

//发送两个图形图层
typedef __packed struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	graphic_data_struct_t clientData[2];		
	uint16_t	FrameTail;								
}ext_graphic_two_data_t;

//发送五个图形图层
typedef __packed struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	graphic_data_struct_t clientData[5];		
	uint16_t	FrameTail;								
}ext_graphic_five_data_t;

//发送七个图形图层
typedef __packed struct
{
	xFrameHeader txFrameHeader;			
	uint16_t  CmdID;										
	ext_student_interactive_header_data_t   dataFrameHeader;
	graphic_data_struct_t clientData[7];		
	uint16_t	FrameTail;								
}ext_graphic_seven_data_t;

void Char_Graphic(ext_client_string_t* graphic,//最终要发出去的数组中的数据段内容
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

void Figure_Graphic(graphic_data_struct_t* graphic,//最终要发出去的数组的数据段内容
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//绘制什么图像
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


void Float_Graphic(Float_data_struct_t* graphic,//最终要发出去的数组的数据段内容
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//绘制什么图像
									uint32_t layer,
									uint32_t color,
									uint32_t size,
									uint32_t decimal,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									float number);

void Int_Graphic(Int_data_struct_t* graphic,//最终要发出去的数组的数据段内容
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//绘制什么图像
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
									ext_graphic_seven_data_t * graphic,//最终要发出去的数组的数据段内容
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//绘制什么图像
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
									ext_float_two_data_t * graphic,//最终要发出去的数组的数据段内容
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//绘制什么图像
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
									ext_int_two_data_t * graphic,//最终要发出去的数组的数据段内容
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//绘制什么图像
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
