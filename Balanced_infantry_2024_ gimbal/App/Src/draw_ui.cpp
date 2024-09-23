/**
 ******************************************************************************
 * @file    draw_ui.cpp
 * @author  Xushuang
 * @version V1.0.0 Xushuang �������
 *			
 * @date    2023/9/22
 * @brief		�˴�ΪUI������
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "draw_ui.h"
#include "cmsis_os.h"
#include "string.h"
#include "super_cap.h"
#define UI_UART huart6
//UI��Ϣ
UI_Pub_Msg_t ui_msg;
UI_t ui;
//UI��ʼ��
void UiInit()
{
	CenterPointer()->PointerInit(&ui_msg,UIPUB);
}

//����
void CharLayerSend(ext_charstring_data_t *tx_char,void (*CharCallback)());
void GraphicLayerSend(ext_graphic_seven_data_t *tx_graphic,void (*GraphicCallback)());
void GraphicLayerSend(ext_graphic_seven_data_t *tx_graphic,void (*GraphicCallback)(fp32[7]),fp32 send_data[7]);
void FloatLayerSend(ext_float_two_data_t *tx_float,void (*FloatCallback)(fp32,fp32),fp32 send_data1,fp32 send_data2);
void IntLayerSend(ext_int_two_data_t *tx_int,void (*IntCallback)(int,int),int send_data1,int send_data2);
//test
void GraphicGroup1LayerSend(ext_graphic_seven_data_t *tx_graphic);
void FloatGroup1LayerSend(ext_float_two_data_t *tx_float,fp32 send_data1,fp32 send_data2);
ext_charstring_data_t    tx_client_char;      //�ַ�����
ext_float_two_data_t     tx_group1_float;     //����������
ext_graphic_seven_data_t tx_group1_graphic;
ext_graphic_seven_data_t tx_group2_graphic;

uint8_t CliendTxBuffer[200];

//������Ϣ
void SetGraphicGroup1()
{
//	//����
//	Figure_Graphic(&tx_group1_graphic.clientData[0],"Tar",ADD,CIRCLE,2,GREEN,0,0,2,(x),(y),radius,0,0);
//	Figure_Graphic(&tx_group1_graphic.clientData[1],"Pre",ADD,CIRCLE,3,YELLOW,0,0,1,(pre_x),(pre_y),pre_radius,0,0);
	
	
}
void UiTask(void const *pvParameters)
{
	uint16_t ui_heart_cnt = 0;
	uint16_t init_step = 0;
	while(1)
	{
		vTaskDelay(2);
		ui_heart_cnt++;

		//��ʼ��
		if(init_step == 0)
		{
			if(ui_heart_cnt <= 20)//Pitch���ַ���ʼ��
			{
				CharLayerSend(&tx_client_char,&UI_t::PitchCharInit);
				continue;
			}
			if(ui_heart_cnt <= 40)//���������ַ���ʼ��
			{
				CharLayerSend(&tx_client_char,&UI_t::SuperCapCharInit);
				continue;
			}
			if(ui_heart_cnt <= 60)//��������ַ���ʼ��
			{
				CharLayerSend(&tx_client_char,&UI_t::FirCharInit);
				continue;
			}
			if(ui_heart_cnt <= 80)//�����ַ���ʼ��
			{
				CharLayerSend(&tx_client_char,&UI_t::AutoCharInit);
				continue;
			}
			if(ui_heart_cnt <= 100)//С�����ַ���ʼ��
			{
				CharLayerSend(&tx_client_char,&UI_t::SpinCharInit);
				continue;
			}

			if(ui_heart_cnt <= 120)//С�����ַ���ʼ��
			{
				CharLayerSend(&tx_client_char,&UI_t::SideCharInit);
				continue;
			}

		  if(ui_heart_cnt <= 140)//�����ַ���ʼ��
			{
				CharLayerSend(&tx_client_char,&UI_t::SterCharInit);
				continue;
			}


			if(ui_heart_cnt <= 160)//ת��
			{
			     FloatLayerSend(&tx_group1_float,&UI_t::FloatGroup1Init,RevolverPointer()->firc_speed_set,0);
				 continue;
			}

			if(ui_heart_cnt <=180)
			{
				GraphicLayerSend(&tx_group1_graphic,&UI_t::GraphicGroup1Init);  //ԲȦ��ʼ��
				continue;
			}
			if(ui_heart_cnt <=200)
			{
//				Client_graphic_aim_line_init();  //ԲȦ��ʼ��
				continue;
			}
			init_step = 1;
		}
		
		if(ui_heart_cnt % 9 == 0)//
		{
			FloatLayerSend(&tx_group1_float,&UI_t::FloatGroup1Update,RevolverPointer()->firc_speed_set,0);
			continue;
		}
		

		if(ui_heart_cnt % 7 == 0)//״̬����
		{
			GraphicLayerSend(&tx_group1_graphic,&UI_t::GraphicGroup1Update);  //ԲȦ����
			continue;
		}
		
		


	}
}
/********�ַ����º���*************/
void UI_t::PitchCharInit()
{
	char pitch_line[10]= {"FIRE:"};//pitch��Ƕ�,float
	Char_Graphic(&tx_client_char.clientData,"PC",ADD,0,ORANGE,20,strlen(pitch_line),2,(1920-200-200),(750),pitch_line);
}

void UI_t::SuperCapCharInit()
{
	char super_line[10]= {"VCAP:"};//��������ʣ����,float
	Char_Graphic(&tx_client_char.clientData,"Cp",ADD,0,ORANGE,20,strlen(super_line),2,(1920-200-200),(590),super_line);
}

void UI_t::FirCharInit()
{
	char fir_line[10]= {"FIRC:"};//�������״̬
	Char_Graphic(&tx_client_char.clientData,"Fir",ADD,0,ORANGE,20,strlen(fir_line),2,100,840,fir_line);
}

void UI_t::AutoCharInit()
{
	char auto_line[10]= {"AUTO:"};//����״̬
	Char_Graphic(&tx_client_char.clientData,"Auto",ADD,0,ORANGE,20,strlen(auto_line),2,100,770,auto_line);
}

void UI_t::SpinCharInit()
{
	char spin_line[10]= {"SPIN:"};//С����״̬
	Char_Graphic(&tx_client_char.clientData,"Spin",ADD,0,ORANGE,20,strlen(spin_line),2,100,700,spin_line);
}

void UI_t::SideCharInit()
{
	char side_line[10]= {"SIDE:"};//����״̬
	Char_Graphic(&tx_client_char.clientData,"Side",ADD,0,ORANGE,20,strlen(side_line),2,100,630,side_line);
}

void UI_t::SterCharInit()
{
	char ster_line[10]= {"STER:"};//���ո�
	Char_Graphic(&tx_client_char.clientData,"Ster",ADD,0,ORANGE,20,strlen(ster_line),2,100,560,ster_line);
}

/********���ֳ�ʼ������*************/
void UI_t::FloatGroup1Init(fp32 first_data,fp32 second_data)
{
	Float_Graphic(&tx_group1_float.clientData[0],"Fi",ADD,FLOAT,4,CYAN_BLUE,30,1,3,((1920-350)),(700),(float)(first_data));
	Float_Graphic(&tx_group1_float.clientData[1],"Sc" ,ADD,FLOAT,4,CYAN_BLUE,30,1,3, (1920-350),  540,( float)(second_data));
}

/********���ָ��º���*************/
void UI_t::FloatGroup1Update(fp32 first_data,fp32 second_data)
{
	Float_Graphic(&tx_group1_float.clientData[0],"Fi",MODIFY,FLOAT,4,CYAN_BLUE,30,1,3,((1920-350)),(700),(float)(first_data));
	Float_Graphic(&tx_group1_float.clientData[1],"Sc", MODIFY,FLOAT,4,CYAN_BLUE,30,1,3, (1920-350),  540, (float)(second_data));
}

/********ͼ�γ�ʼ������*************/
void UI_t::GraphicGroup1Init()
{
	//״̬Ȧ��ʼ��
	//������
	Figure_Graphic(&tx_group1_graphic.clientData[3],"LCW",ADD ,LINE ,2,ORANGE,0,0,2, 700,8,0,850,281);
	Figure_Graphic(&tx_group1_graphic.clientData[4],"RCW",ADD ,LINE ,2,ORANGE,0,0,2, 1250,11,0,1135,283);

	//Ħ����
	Figure_Graphic(&tx_group1_graphic.clientData[2],"Fi", ADD,CIRCLE,2,RED_BLUE ,0,0,4,X_CIRCLE_O,840,20,0,0);

  //����
  Figure_Graphic(&tx_group1_graphic.clientData[1],"Au", ADD,CIRCLE,2,RED_BLUE ,0,0,4,X_CIRCLE_O,770,20,0,0);

   //С����
	Figure_Graphic(&tx_group1_graphic.clientData[0],"Sp", ADD,CIRCLE,2,RED_BLUE ,0,0,4,X_CIRCLE_O,700,20,0,0);
 
	//����
	Figure_Graphic(&tx_group1_graphic.clientData[5],"Sd", ADD,CIRCLE,2,RED_BLUE ,0,0,4,X_CIRCLE_O,630,20,0,0);

	//����
	Figure_Graphic(&tx_group1_graphic.clientData[6],"St", ADD,CIRCLE,2,RED_BLUE ,0,0,4,X_CIRCLE_O,560,20,0,0);


//	//���飨�������֣�
//	Figure_Graphic(&tx_group1_graphic.clientData[5],"Tar",ADD,CIRCLE,2,GREEN,0,0,2,0,0,0,0,0);
//	Figure_Graphic(&tx_group1_graphic.clientData[6],"Pri",ADD,CIRCLE,3,YELLOW,0,0,1,0,0,0,0,0);
	//��addһ��ͼ��
//	Figure_Graphic(&tx_group1_graphic.clientData[5],"FL",ADD,LINE,4,GREEN,0,0,3, 960+(int)200*arm_sin_f32((angle)*2*PI/360.0f),540+(int)200*arm_cos_f32((angle)*2*PI/360.0f),0,960+(int)270*arm_sin_f32((angle)*2*PI/360.0f),540+(int)270*arm_cos_f32((angle)*2*PI/360.0f));
}

void UI_t::GraphicGroup2Init()
{
//	tx_group2_graphic
}
/********ͼ�θ��º���*************/
void UI_t::GraphicGroup1Update()
{

	if(SysPointer()->key_flag.fir_flag==2)//��
	{
		Figure_Graphic(&tx_group1_graphic.clientData[2],"Fi",MODIFY,CIRCLE,2,GREEN,0,0,4,X_CIRCLE_O,840,20,0,0);
	}
	else if(SysPointer()->key_flag.fir_flag==1)//��
	{
		Figure_Graphic(&tx_group1_graphic.clientData[2],"Fi",MODIFY,CIRCLE,2,RED_BLUE,0,0,4,X_CIRCLE_O,840,20,0,0);
	}



	if(SysPointer()->key_flag.auto_aim_flag)
	{
		Figure_Graphic(&tx_group1_graphic.clientData[1],"Au",MODIFY,CIRCLE,2,GREEN,0,0,4,X_CIRCLE_O,770,20,0,0);
	}
	else
	{
		Figure_Graphic(&tx_group1_graphic.clientData[1],"Au",MODIFY,CIRCLE,2,RED_BLUE,0,0,4,X_CIRCLE_O,770,20,0,0);
	}


	if(SysPointer()->key_flag.spin_flag)
	{
		Figure_Graphic(&tx_group1_graphic.clientData[0],"Sp",MODIFY,CIRCLE,2,GREEN,0,0,4,X_CIRCLE_O,700,20,0,0);
	}
	else
	{
		Figure_Graphic(&tx_group1_graphic.clientData[0],"Sp",MODIFY,CIRCLE,2,RED_BLUE,0,0,4,X_CIRCLE_O,700,20,0,0);
	}


	if(SysPointer()->key_flag.side_flag)
	{
		Figure_Graphic(&tx_group1_graphic.clientData[5],"Sd",MODIFY,CIRCLE,2,GREEN,0,0,4,X_CIRCLE_O,630,20,0,0);
	}
	else
	{
		Figure_Graphic(&tx_group1_graphic.clientData[5],"Sd",MODIFY,CIRCLE,2,RED_BLUE,0,0,4,X_CIRCLE_O,630,20,0,0);
	}

if(SysPointer()->key_flag.ster_flag)
	{
		Figure_Graphic(&tx_group1_graphic.clientData[6],"St",MODIFY,CIRCLE,2,GREEN,0,0,4,X_CIRCLE_O,560,20,0,0);
	}
	else
	{
		Figure_Graphic(&tx_group1_graphic.clientData[6],"St",MODIFY,CIRCLE,2,RED_BLUE,0,0,4,X_CIRCLE_O,560,20,0,0);
	}








//	//������
//	Figure_Graphic(&tx_group1_graphic.clientData[3],"LCW",MODIFY,LINE,2,ORANGE,0,0,2,625,8,0,775,281);
//	Figure_Graphic(&tx_group1_graphic.clientData[4],"RCW",MODIFY,LINE,2,ORANGE,0,0,2,1190,11,0,1045,283);
	//���飨�������֣���������Ӿ���ȡ���ݺ�Ĵ�����communicate�ļ��У�
//	Figure_Graphic(&tx_group1_graphic.clientData[5],"Tar",ADD,CIRCLE,2,GREEN,0,0,2,(x),(y),radius,0,0);
//	Figure_Graphic(&tx_group1_graphic.clientData[6],"Pri",ADD,CIRCLE,3,YELLOW,0,0,1,(pre_x),(pre_y),pre_radius,0,0);
	//��addһ��ͼ��
//	Figure_Graphic(&tx_group1_graphic.clientData[5],"FL",MODIFY,LINE,4,GREEN,0,0,3, 960+(int)200*arm_sin_f32((angle)*2*PI/360.0f),540+(int)200*arm_cos_f32((angle)*2*PI/360.0f),0,960+(int)270*arm_sin_f32((angle)*2*PI/360.0f),540+(int)270*arm_cos_f32((angle)*2*PI/360.0f));

}

void UI_t::GraphicGroup2Update()
{
	
}

//�ַ�ͼ�㷢��
void CharLayerSend(ext_charstring_data_t *tx_char,void (*CharCallback)())//�ⲿ���������)
{
	tx_char->txFrameHeader.SOF = JUDGE_FRAME_HEADER;
	tx_char->txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_string_t);
	tx_char->txFrameHeader.Seq = 0;//�����
	memcpy(CliendTxBuffer,&tx_char->txFrameHeader,sizeof(xFrameHeader));
	append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��
	
	//������
	tx_char->CmdID = ID_robot_interactive_header_data;
		
	//���ݶ�ͷ�ṹ
	tx_char->dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
	tx_char->dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
	tx_char->dataFrameHeader.receiver_ID = REF.self_client;
		
	CharCallback();
	memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_char->CmdID, LEN_CMD_ID+tx_char->txFrameHeader.DataLength);//���������볤��2
		
	//֡β
	append_CRC16_check_sum(CliendTxBuffer,sizeof(ext_charstring_data_t));
		
	USARTSend(&UI_UART,CliendTxBuffer,sizeof(ext_charstring_data_t),USART_TRANSFER_BLOCKING);
}
//test
void GraphicGroup1LayerSend(ext_graphic_seven_data_t *tx_graphic)
{
	tx_graphic->txFrameHeader.SOF = JUDGE_FRAME_HEADER;
	tx_graphic->txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*7;
	tx_graphic->txFrameHeader.Seq = 0;//�����
	memcpy(CliendTxBuffer,&tx_graphic->txFrameHeader,sizeof(xFrameHeader));
	append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��
	
	//������
	tx_graphic->CmdID = ID_robot_interactive_header_data;
		
	//���ݶ�ͷ�ṹ
	tx_graphic->dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
	tx_graphic->dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
	tx_graphic->dataFrameHeader.receiver_ID = REF.self_client;
		
//	GraphicCallback(send_data);
	ui.GraphicGroup1Update();
	memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_graphic->CmdID, LEN_CMD_ID+tx_graphic->txFrameHeader.DataLength);//���������볤��2
		
	//֡β
	append_CRC16_check_sum(CliendTxBuffer,sizeof(ext_graphic_seven_data_t));
	USARTSend(&UI_UART,CliendTxBuffer,sizeof(ext_graphic_seven_data_t),USART_TRANSFER_BLOCKING);
}
//test
void FloatGroup1LayerSend(ext_float_two_data_t *tx_float,fp32 send_data1,fp32 send_data2)
{
	tx_float->txFrameHeader.SOF = JUDGE_FRAME_HEADER;
	tx_float->txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*2;
	tx_float->txFrameHeader.Seq = 0;//�����
	memcpy(CliendTxBuffer,&tx_float->txFrameHeader,sizeof(xFrameHeader));
	append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��
	
	//������
	tx_float->CmdID = ID_robot_interactive_header_data;
		
	//���ݶ�ͷ�ṹ
	tx_float->dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
	tx_float->dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
	tx_float->dataFrameHeader.receiver_ID = REF.self_client;
		
	ui.FloatGroup1Update(send_data1,send_data2);
	memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_float->CmdID, LEN_CMD_ID+tx_float->txFrameHeader.DataLength);//���������볤��2
		
	//֡β
	append_CRC16_check_sum(CliendTxBuffer,sizeof(ext_float_two_data_t));
	USARTSend(&UI_UART,CliendTxBuffer,sizeof(ext_float_two_data_t),USART_TRANSFER_BLOCKING);
}

void GraphicLayerSend(ext_graphic_seven_data_t *tx_graphic,void (*GraphicCallback)())
{
	tx_graphic->txFrameHeader.SOF = JUDGE_FRAME_HEADER;
	tx_graphic->txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*7;
	tx_graphic->txFrameHeader.Seq = 0;//�����
	memcpy(CliendTxBuffer,&tx_graphic->txFrameHeader,sizeof(xFrameHeader));
	append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��
	
	//������
	tx_graphic->CmdID = ID_robot_interactive_header_data;
		
	//���ݶ�ͷ�ṹ
	tx_graphic->dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
	tx_graphic->dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
	tx_graphic->dataFrameHeader.receiver_ID = REF.self_client;
		
	GraphicCallback();
	memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_graphic->CmdID, LEN_CMD_ID+tx_graphic->txFrameHeader.DataLength);//���������볤��2
		
	//֡β
	append_CRC16_check_sum(CliendTxBuffer,sizeof(ext_graphic_seven_data_t));
	USARTSend(&UI_UART,CliendTxBuffer,sizeof(ext_graphic_seven_data_t),USART_TRANSFER_BLOCKING);
}

void GraphicLayerSend(ext_graphic_seven_data_t *tx_graphic,void (*GraphicCallback)(fp32[7]),fp32 send_data[7])
{
	tx_graphic->txFrameHeader.SOF = JUDGE_FRAME_HEADER;
	tx_graphic->txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*7;
	tx_graphic->txFrameHeader.Seq = 0;//�����
	memcpy(CliendTxBuffer,&tx_graphic->txFrameHeader,sizeof(xFrameHeader));
	append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��
	
	//������
	tx_graphic->CmdID = ID_robot_interactive_header_data;
		
	//���ݶ�ͷ�ṹ
	tx_graphic->dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
	tx_graphic->dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
	tx_graphic->dataFrameHeader.receiver_ID = REF.self_client;
		
	GraphicCallback(send_data);
	memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_graphic->CmdID, LEN_CMD_ID+tx_graphic->txFrameHeader.DataLength);//���������볤��2
		
	//֡β
	append_CRC16_check_sum(CliendTxBuffer,sizeof(ext_graphic_seven_data_t));
	USARTSend(&UI_UART,CliendTxBuffer,sizeof(ext_graphic_seven_data_t),USART_TRANSFER_BLOCKING);
}

void FloatLayerSend(ext_float_two_data_t *tx_float,void (*FloatCallback)(fp32,fp32),fp32 send_data1,fp32 send_data2)
{
	tx_float->txFrameHeader.SOF = JUDGE_FRAME_HEADER;
	tx_float->txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*2;
	tx_float->txFrameHeader.Seq = 0;//�����
	memcpy(CliendTxBuffer,&tx_float->txFrameHeader,sizeof(xFrameHeader));
	append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��
	
	//������
	tx_float->CmdID = ID_robot_interactive_header_data;
		
	//���ݶ�ͷ�ṹ
	tx_float->dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
	tx_float->dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
	tx_float->dataFrameHeader.receiver_ID = REF.self_client;
		
	FloatCallback(send_data1,send_data2);
	memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_float->CmdID, LEN_CMD_ID+tx_float->txFrameHeader.DataLength);//���������볤��2
		
	//֡β
	append_CRC16_check_sum(CliendTxBuffer,sizeof(ext_float_two_data_t));
	USARTSend(&UI_UART,CliendTxBuffer,sizeof(ext_float_two_data_t),USART_TRANSFER_BLOCKING);
}

void IntLayerSend(ext_int_two_data_t *tx_int,void (*IntCallback)(int,int),int send_data1,int send_data2)
{
	tx_int->txFrameHeader.SOF = JUDGE_FRAME_HEADER;
	tx_int->txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*2;
	tx_int->txFrameHeader.Seq = 0;//�����
	memcpy(CliendTxBuffer,&tx_int->txFrameHeader,sizeof(xFrameHeader));
	append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��
	
	//������
	tx_int->CmdID = ID_robot_interactive_header_data;
		
	//���ݶ�ͷ�ṹ
	tx_int->dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
	tx_int->dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
	tx_int->dataFrameHeader.receiver_ID = REF.self_client;
		
	IntCallback(send_data1,send_data2);
	memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_int->CmdID, LEN_CMD_ID+tx_int->txFrameHeader.DataLength);//���������볤��2
		
	//֡β
	append_CRC16_check_sum(CliendTxBuffer,sizeof(ext_int_two_data_t));
	USARTSend(&UI_UART,CliendTxBuffer,sizeof(ext_int_two_data_t),USART_TRANSFER_BLOCKING);
}
