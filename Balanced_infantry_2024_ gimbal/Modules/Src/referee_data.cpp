/**
  ****************************(C) COPYRIGHT 2023 TUP****************************
  * @file       referee_data.cpp/h
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
#include "referee_data.h"
#include "string.h"
#include "cmsis_os.h"
/*****************����ʵ������**********************/
Usart_Instance_t referee_usart; //����ϵͳ����ʵ��

//���ڽ��ջص��������Խ�������
void DecodeRefereeData()
{
	Judge_Read_Data(referee_usart.rx_buff);  //��ȡ��Ϣ
}

//���ڳ�ʼ��
void RefereeInit()
{
	USARTInit(&referee_usart,&huart6,USART_RX_DMA,JUDGE_BUFFER_LEN,DecodeRefereeData); //��ʼ�����ô��ڽ���
	UsartRegister(&referee_usart);  //�Ǽ�ע��
}

/*****************ϵͳ���ݶ���**********************/
ext_game_status_t                   Game_status;
ext_game_result_t                   Game_result;
ext_game_robot_HP_t                 Robot_HP;
ext_dart_status_t                   Dart_status;
ext_ICRA_buff_debuff_zone_status_t  ICRA_buff_debuff_zone_status;
ext_event_data_t                    Event_data;
ext_supply_projectile_action_t      Supply_projectile_action;
ext_referee_warning_t               Referee_warning;
ext_dart_remaining_time_t           Dart_remaining_time;
ext_game_robot_status_t             Robot_status;
ext_power_heat_data_t               Power_heat_data;
ext_game_robot_pos_t                robot_position;
ext_buff_t                          buff;
aerial_robot_energy_t         			Aerial_robot_energy;
ext_robot_hurt_t                    Robot_hurt;
ext_shoot_data_t                    Shoot_data;
ext_bullet_remaining_t              Bullet_remaining;
ext_rfid_status_t                   Rfid_status;
ext_dart_client_cmd_t               Dart_client_cmd;


ext_robot_command_t					Minimap_robot_command;	
ext_client_map_command_t			Client_map_command_t;
ext_robot_command_t_change			Transfer_image_robot_command;


xFrameHeader              FrameHeader;		//����֡ͷ��Ϣ
//δ��
//ext_SendClientData_t      ShowData;			//�ͻ�����Ϣ
//ext_CommunatianData_t     CommuData;		//����ͨ����Ϣ
//ext_SendClientGraph_t     ClientGraph;  //ͼ��ͨ��
//ext_SendClientGraphDelete_t ClientGraphDelete; //ͼ��ɾ��


/****************************************************/
bool Judge_Data_TF = FALSE;//���������Ƿ����,������������
uint8_t Judge_Self_ID;//��ǰ�����˵�ID
uint16_t Judge_SelfClient_ID;//�����߻����˶�Ӧ�Ŀͻ���ID

/**************����ϵͳ���ݸ���****************/
uint16_t ShootNum_17mm;//ͳ�Ʒ�����,0x0003����һ������Ϊ������һ��
uint16_t ShootNum_42mm;//ͳ�Ʒ�����,0x0003����һ������Ϊ������һ��
bool Hurt_Data_Update = FALSE;//װ�װ��˺������Ƿ����,ÿ��һ���˺���TRUE,Ȼ��������FALSE,������������

#define BLUE  0
#define RED   1

Referee_info_t 	REF; // ����ϵͳ��Ϣ

/**
  * @brief  ��ȡ��������,loop��ѭ�����ô˺�������ȡ����
  * @param  ��������
  * @retval �Ƿ�������ж�������
  * @attention  �ڴ��ж�֡ͷ��CRCУ��,������д������
  */
bool Judge_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE;//������ȷ����־,ÿ�ε��ö�ȡ����ϵͳ���ݺ�������Ĭ��Ϊ����
	
	uint16_t judge_length;//ͳ��һ֡���ݳ��� 
	
	int CmdID = 0;//�������������
	
	/***------------------*****/
	//�����ݰ��������κδ���
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	
	//д��֡ͷ����,�����ж��Ƿ�ʼ�洢��������
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
	memcpy(&REF.FrameHeader,ReadFromUsart,LEN_HEADER);   //����֡ͷ����
	
		//�ж�֡ͷ�����Ƿ�Ϊ0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		//֡ͷCRC8У��
		if (verify_CRC8_check_sum( ReadFromUsart, LEN_HEADER ) == TRUE)
		{
			//ͳ��һ֡���ݳ���,����CR16У��
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			
			//֡βCRC16У��
			if(verify_CRC16_check_sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//��У�������˵�����ݿ���
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				//��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)
			
				switch(CmdID)
				{
					case ID_game_status:         //0x0001
						memcpy(&Game_status, (ReadFromUsart + DATA), LEN_game_status);
						memcpy(&REF.GameState, (ReadFromUsart + DATA), LEN_game_status);
					break;
					
					case ID_game_result:          //0x0002
						memcpy(&Game_result, (ReadFromUsart + DATA), LEN_game_result);		
						memcpy(&REF.GameResult, (ReadFromUsart + DATA), LEN_game_result);					
					break;
					
					case ID_robot_HP:             //0x0003
						memcpy(&Robot_HP, (ReadFromUsart + DATA), LEN_robot_HP);
						memcpy(&REF.GameRobotHP, (ReadFromUsart + DATA), LEN_robot_HP);
					break;
					
					case ID_dart_status:          //0x0004
						memcpy(&Dart_status, (ReadFromUsart + DATA), LEN_dart_status);
						memcpy(&REF.GameRobotmissile, (ReadFromUsart + DATA), LEN_dart_status);
					break;
					
					case ID_ICRA_buff_debuff_zone_status:          //0x0005
						memcpy(&ICRA_buff_debuff_zone_status, (ReadFromUsart + DATA), LEN_ICRA_buff_debuff_zone_status);
						memcpy(&REF.Game_ICRA_buff, (ReadFromUsart + DATA), LEN_ICRA_buff_debuff_zone_status);
					break;
					
					case ID_event_data:   //0x0101
						memcpy(&Event_data, (ReadFromUsart + DATA), LEN_event_data);
						memcpy(&REF.EventData, (ReadFromUsart + DATA), LEN_event_data);
					break;
					
					case ID_supply_projectile_action:      //0x0102
						memcpy(&Supply_projectile_action, (ReadFromUsart + DATA), LEN_supply_projectile_action);
						memcpy(&REF.SupplyProjectileAction, (ReadFromUsart + DATA), LEN_supply_projectile_action);
					break;
					
					case ID_referee_warning:      //0x0104
						memcpy(&Referee_warning, (ReadFromUsart + DATA), LEN_referee_warning);
						memcpy(&REF.RefereeWarning, (ReadFromUsart + DATA), LEN_referee_warning);
					break;
					
					case ID_dart_remaining_time:      //0x0105
						memcpy(&Dart_remaining_time, (ReadFromUsart + DATA), LEN_dart_remaining_time);
						memcpy(&REF.dart_remaining_time, (ReadFromUsart + DATA), LEN_dart_remaining_time);
					break;
					
					case ID_robot_status:      //0x0201
						memcpy(&Robot_status, (ReadFromUsart + DATA), LEN_robot_status);
						memcpy(&REF.GameRobotStat, (ReadFromUsart + DATA), LEN_robot_status);
						Determine_ID();
					break;
					
					case ID_heat_data:      //0x0202
						memcpy(&Power_heat_data, (ReadFromUsart + DATA), LEN_heat_data);
						memcpy(&REF.PowerHeatData, (ReadFromUsart + DATA), LEN_heat_data);
					break;
					
					case ID_robot_pos:      //0x0203
						memcpy(&robot_position, (ReadFromUsart + DATA), LEN_robot_pos);
						memcpy(&REF.GameRobotPos, (ReadFromUsart + DATA), LEN_robot_pos);
					break;
					
					case ID_buff:      //0x0204
						memcpy(&buff, (ReadFromUsart + DATA), LEN_buff);
						memcpy(&REF.Buff, (ReadFromUsart + DATA), LEN_buff);
					break;
					
					case ID_aerial_robot_energy:      //0x0205
						memcpy(&Aerial_robot_energy, (ReadFromUsart + DATA), LEN_aerial_robot_energy);
						memcpy(&REF.AerialRobotEnergy, (ReadFromUsart + DATA), LEN_aerial_robot_energy);
					break;
					
					case ID_robot_hurt:      //0x0206
						memcpy(&Robot_hurt, (ReadFromUsart + DATA), LEN_robot_hurt);
						memcpy(&REF.RobotHurt, (ReadFromUsart + DATA), LEN_robot_hurt);
					  if(Robot_hurt.hurt_type == 0)//��װ�װ���������˺�
						{	Hurt_Data_Update = TRUE;	}//װ������ÿ����һ�����ж�Ϊ�ܵ�һ���˺�
					break;
					
					case ID_shoot_data:      //0x0207
						memcpy(&Shoot_data, (ReadFromUsart + DATA), LEN_shoot_data);
//					    JUDGE_ShootNumCount_17mm();//������ͳ��,��������˫ǹ��,��׼
//					    JUDGE_ShootNumCount_42mm();//������ͳ��,��������˫ǹ��,��׼
					break;
					
					case ID_bullet_remaining:      //0x0208
						memcpy(&Bullet_remaining, (ReadFromUsart + DATA), LEN_bullet_remaining);
					break;
					
					case ID_rfid_status:      //0x0209
						memcpy(&Rfid_status, (ReadFromUsart + DATA), LEN_rfid_status);
					break;
					
					case ID_dart_client_cmd:      //0x020A
						memcpy(&Dart_client_cmd, (ReadFromUsart + DATA), LEN_dart_client_cmd);
					break;
					
					case ID_minimap_interactive_data:  //0x0303
						memcpy(&Minimap_robot_command, (ReadFromUsart + DATA), LEN_minimap_interactive_data);
					break;

					case ID_keybord_and_mouse_massage:  //0x0304
						memcpy(&Transfer_image_robot_command, (ReadFromUsart + DATA), LEN_keybord_and_mouse_massage);
					break;
					
					case ID_client_map_command:  //0x0305
						memcpy(&Transfer_image_robot_command, (ReadFromUsart + DATA), LEN_client_map_command);
					break;
					
				}
				
				//�׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
				if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
				{
					//���һ�����ݰ������˶�֡����,���ٴζ�ȡ
					Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
				}
			}					
		}		
	}
	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE;//����������
	}
	else		//ֻҪCRC16У�鲻ͨ����ΪFALSE
	{
		Judge_Data_TF = FALSE;//����������
	}

	return retval_tf;//����������������	

}

/**
  * @brief  �ж��Լ�������
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
bool Color;
bool is_red_or_blue(void)
{
	Judge_Self_ID = Robot_status.robot_id;//��ȡ��ǰ������ID
	
	if(Robot_status.robot_id > 10)
	{
		return BLUE;
	}
	else 
	{
		return RED;
	}
}

/**
  * @brief  �ж�����ID��ѡ��ͻ���ID
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
//void determine_ID(void)
//{
//	Color = is_red_or_blue();
//	if(Color == BLUE)
//	{
//		Judge_SelfClient_ID = 0x0110 + (Judge_Self_ID-10);//����ͻ���ID
//	}
//	else if(Color == RED)
//	{
//		Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//����ͻ���ID
//	}
//}
void Determine_ID(void)//�ж��Լ����ĸ�����
{
	if(REF.GameRobotStat.robot_id < 10)//�������˵�ID���췽
	{ 
		REF.ids.teammate_hero 		 	= 1;
		REF.ids.teammate_engineer  = 2;
		REF.ids.teammate_infantry3 = 3;
		REF.ids.teammate_infantry4 = 4;
		REF.ids.teammate_infantry5 = 5;
		REF.ids.teammate_plane		 	= 6;
		REF.ids.teammate_sentry		= 7;
		
		REF.ids.client_hero 		 	= 0x0101;
		REF.ids.client_engineer  = 0x0102;
		REF.ids.client_infantry3 = 0x0103;
		REF.ids.client_infantry4 = 0x0104;
		REF.ids.client_infantry5 = 0x0105;
		REF.ids.client_plane			= 0x0106;
		
		if     (REF.GameRobotStat.robot_id == hero_red)//����ˢ�·����ڱ����и�����ɫ
			REF.self_client = REF.ids.client_hero;
		else if(REF.GameRobotStat.robot_id == engineer_red)
			REF.self_client = REF.ids.client_engineer;
		else if(REF.GameRobotStat.robot_id == infantry3_red)
			REF.self_client = REF.ids.client_infantry3;
		else if(REF.GameRobotStat.robot_id == infantry4_red)
			REF.self_client = REF.ids.client_infantry4;
		else if(REF.GameRobotStat.robot_id == infantry5_red)
			REF.self_client = REF.ids.client_infantry5;
		else if(REF.GameRobotStat.robot_id == plane_red)
			REF.self_client = REF.ids.client_plane;
	}
	else //����
	{
		REF.ids.teammate_hero 		 	= 101;
		REF.ids.teammate_engineer  = 102;
		REF.ids.teammate_infantry3 = 103;
		REF.ids.teammate_infantry4 = 104;
		REF.ids.teammate_infantry5 = 105;
		REF.ids.teammate_plane		 	= 106;
		REF.ids.teammate_sentry		= 107;
		
		REF.ids.client_hero 		 	= 0x0165;
		REF.ids.client_engineer  = 0x0166;
		REF.ids.client_infantry3 = 0x0167;
		REF.ids.client_infantry4 = 0x0168;
		REF.ids.client_infantry5 = 0x0169;
		REF.ids.client_plane			= 0x016A;
		
		if     (REF.GameRobotStat.robot_id == hero_blue)
			REF.self_client = REF.ids.client_hero;
		else if(REF.GameRobotStat.robot_id == engineer_blue)
			REF.self_client = REF.ids.client_engineer;
		else if(REF.GameRobotStat.robot_id == infantry3_blue)
			REF.self_client = REF.ids.client_infantry3;
		else if(REF.GameRobotStat.robot_id == infantry4_blue)
			REF.self_client = REF.ids.client_infantry4;
		else if(REF.GameRobotStat.robot_id == infantry5_blue)
			REF.self_client = REF.ids.client_infantry5;
		else if(REF.GameRobotStat.robot_id == plane_blue)
			REF.self_client = REF.ids.client_plane;
		
	}

}

/********************�������ݸ����жϺ���***************************/

/**
  * @brief  �����Ƿ����
  * @param  void
  * @retval  TRUE����   FALSE������
  * @attention  �ڲ��ж�ȡ������ʵʱ�ı䷵��ֵ
*/
bool JUDGE_sGetDataState(void)
{
	return Judge_Data_TF;
}

/**
  * @brief  ��ȡ˲ʱ����
  * @param  void
  * @retval ʵʱ����ֵ
  * @attention  
*/
float JUDGE_fGetChassisPower(void)
{
	return (Power_heat_data.chassis_power);
}


float JUDGE_fGetChassisVolt(void)
{
	return (Power_heat_data.chassis_volt);
}

/**
	* @brief  ��ȡʣ�ཹ������
	* @param  void
	* @retval ʣ�໺�役������(���60)
	* @attention  
*/
uint16_t JUDGE_fGetRemainEnergy(void)
{
	return (Power_heat_data.chassis_power_buffer);
}

/**
  * @brief  ��ȡ��ǰ�ȼ�
  * @param  void
  * @retval ��ǰ�ȼ�
  * @attention  
*/
uint8_t JUDGE_ucGetRobotLevel(void)
{
    return	Robot_status.robot_level;
}

/**
  * @brief  ��ȡid1_17mmǹ������
  * @param  void
  * @retval 17mm
  * @attention  
*/
uint16_t JUDGE_usGetRemoteHeat_id1_17mm(void)
{
	return Power_heat_data.shooter_id1_17mm_cooling_heat;
}

/**
  * @brief  ��ȡid2_17mmǹ������
  * @param  void
  * @retval 17mm
  * @attention  
*/
uint16_t JUDGE_usGetRemoteHeat_id2_17mm(void)
{
	return Power_heat_data.shooter_id2_17mm_cooling_heat;
}

/**
  * @brief  ��ȡid1_42mmǹ������
  * @param  void
  * @retval 42mm
  * @attention  
*/
uint16_t JUDGE_usGetRemoteHeat_id1_42mm(void)
{
	return Power_heat_data.shooter_id1_42mm_cooling_heat;
}


/**
  * @brief  ��ȡ����
  * @param  void
  * @retval 17mm
  * @attention  ʵʱ����
*/
float JUDGE_usGetSpeedHeat(void)
{
	return Shoot_data.bullet_speed;
}

/**
  * @brief  ͳ��17mm������
  * @param  void
  * @retval void
  * @attention  
*/
portTickType shoot_time_17mm;//������ʱ����
portTickType shoot_ping_17mm;//����������շ����ӳ�

float Shoot_Speed_Now_17mm = 0;
float Shoot_Speed_Last_17mm = 0;

void JUDGE_ShootNumCount_17mm(void)
{
	//��⵽��С����  bullet_type==1
	if(Shoot_data.bullet_type == 1)
	{
	Shoot_Speed_Now_17mm = Shoot_data.bullet_speed;
	if(Shoot_Speed_Last_17mm != Shoot_Speed_Now_17mm)//��Ϊ��float�ͣ�������������ȫ���,�����ٶȲ���ʱ˵��������һ�ŵ�
	{
		ShootNum_17mm++;
		Shoot_Speed_Last_17mm = Shoot_Speed_Now_17mm;
	}
	shoot_time_17mm = xTaskGetTickCount();//��ȡ���跢��ʱ��ϵͳʱ��
//	shoot_ping_17mm = shoot_time_17mm - REVOL_uiGetRevolTime();//�����ӳ�
	}
}

/**
  * @brief  ͳ��42mm������
  * @param  void
  * @retval void
  * @attention  
*/
portTickType shoot_time_42mm;//������ʱ����
portTickType shoot_ping_42mm;//����������շ����ӳ�

float Shoot_Speed_Now_42mm = 0;
float Shoot_Speed_Last_42mm = 0;

void JUDGE_ShootNumCount_42mm(void)
{
	//��⵽�Ǵ���  bullet_type==2
	if(Shoot_data.bullet_type == 2)
	{
	Shoot_Speed_Now_42mm = Shoot_data.bullet_speed;
	if(Shoot_Speed_Last_42mm != Shoot_Speed_Now_42mm)//��Ϊ��float�ͣ�������������ȫ���,�����ٶȲ���ʱ˵��������һ�ŵ�
	{
		ShootNum_42mm++;
		Shoot_Speed_Last_42mm = Shoot_Speed_Now_42mm;
	}
	shoot_time_42mm = xTaskGetTickCount();//��ȡ���跢��ʱ��ϵͳʱ��
//	shoot_ping_42mm = shoot_time_42mm - REVOL_uiGetRevolTime();//�����ӳ�
	}
}


/**
  * @brief  ��ȡ17mm������
  * @param  void
  * @retval ������
  * @attention ��������˫ǹ��
*/
uint16_t JUDGE_usGetShootNum_17mm(void)
{
	return ShootNum_17mm;
}

/**
  * @brief  ��ȡ42mm������
  * @param  void
  * @retval ������
  * @attention ��������˫ǹ��
*/
uint16_t JUDGE_usGetShootNum_42mm(void)
{
	return ShootNum_42mm;
}
/**
	* @brief  17mm����������
  * @param  void
  * @retval void
  * @attention 
*/
void JUDGE_ShootNum_Clear_17mm(void)
{
	ShootNum_17mm = 0;
}
/**
  * @brief  42mm����������
	* @param  void
	* @retval void
	* @attention 
*/
void JUDGE_ShootNum_Clear_42mm(void)
{
	ShootNum_42mm = 0;
}

/**
  * @brief  ��ȡid1_17mmǹ������
  * @param  void
  * @retval ��ǰ�ȼ�17mm��������
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit_id1_17mm(void)
{
	return Robot_status.shooter_id1_17mm_cooling_limit;
}



/**
	* @brief  ��ȡ���̹������ֵ
	* @param  void
	* @retval ��ǰ�ȼ����̹�������
	* @attention  
*/
uint16_t JUDGE_usGetChassisPowerLimit(void)
{
	return Robot_status.chassis_power_limit;
}


/**
	* @brief  ��ȡ�����˵ȼ�
	* @param  void
	* @retval 
	* @attention  
*/
uint8_t JUDGE_ucRobotLevel(void)
{
	return Robot_status.robot_level;
}

/**
  * @brief  ��ǰ�ȼ�id1_17mm��Ӧ��ǹ��ÿ����ȴֵ
  * @param  void
  * @retval ��ǰ�ȼ�17mm��ȴ�ٶ�
  * @attention  
*/
uint16_t JUDGE_usGetShootCold_id1_17mm(void)
{
	return Robot_status. shooter_id1_17mm_cooling_rate;
}

/**
  * @brief  ��ǰ�ȼ�id2_17mm��Ӧ��ǹ��ÿ����ȴֵ
  * @param  void
  * @retval ��ǰ�ȼ�17mm��ȴ�ٶ�
  * @attention  
*/
uint16_t JUDGE_usGetShootCold_id2_17mm(void)
{
	return Robot_status.shooter_id1_17mm_cooling_rate;
}


/**
  * @brief  ����վ�� ID��
  * @param  void
  * @retval ��ǰ����վ�� ID��
  * @attention  Ϊ�ַ��ͱ���
	1�� 1 �Ų�����
	2�� 2 �Ų�����
*/
uint8_t JUDGE_usGetSupply_Id(void)
{
	return Supply_projectile_action.supply_projectile_id;
}

/**
  * @brief  ���������� ID��
  * @param  void
  * @retval ��ǰ�貹�������� ID��
  * @attention  Ϊ�ַ��ͱ���
	���������� ID�� 0 Ϊ��ǰ�޻����˲����� 1 Ϊ�췽Ӣ�ۻ����˲����� 2 Ϊ�췽���̻�
	���˲����� 3/4/5 Ϊ�췽���������˲����� 101 Ϊ����Ӣ�ۻ����˲����� 102 Ϊ������
	�̻����˲����� 103/104/105 Ϊ�������������˲���
*/
uint8_t JUDGE_usGetSupply_Robo_Id(void)
{
	return Supply_projectile_action.supply_projectile_id;
}


/**
  * @brief  �����ڿ���״̬ 
  * @param  void
  * @retval �����ڿ���״̬ 
  * @attention  Ϊ�ַ��ͱ���
	0 Ϊ�رգ�
	1 Ϊ�ӵ�׼���У�
	2 Ϊ�ӵ�����
*/
uint8_t JUDGE_usGetSupply_Mode(void)
{
	return Supply_projectile_action.supply_projectile_step ;
}

/**
  * @brief  ��ǰ��������
  * @param  void
  * @retval ��ǰ��������
  * @attention Ϊ�ַ��ͱ���
	50�� 50 ���ӵ���
	100�� 100 ���ӵ���
	150�� 150 ���ӵ���
	200�� 200 ���ӵ�
*/ 
uint8_t JUDGE_usGetSupply_Num(void)
{
	return Supply_projectile_action.supply_projectile_num;
}

/**
  * @brief  С��ͼ������ϢĿ������� ID(�״�վ)
  * @param  void
  * @retval ��ǰ��������
  * @attention 

*/ 
uint16_t JUDGE_usGetRadar_Station_to_robo_ID(void)
{
	return Client_map_command_t.target_robot_ID;
}

/**
  * @brief  С��ͼ������ϢĿ������� x λ�����꣬��λ m �� x,y ��������ʱ����ʾ
			(�״�վ)
  * @param  void
  * @retval ��ǰ��������
  * @attention 
*/ 
float JUDGE_usGetRadar_Station_to_robo_posX(void)
{
	return	Client_map_command_t.target_position_x;
}

/**
  * @brief  С��ͼ������ϢĿ������� y λ�����꣬��λ m �� x,y ��������ʱ����ʾ
			(�״�վ)
  * @param  void
  * @retval ��ǰ��������
  * @attention 

*/ 
float JUDGE_usGetRadar_Station_to_robo_posy(void)
{
	return	Client_map_command_t.target_position_y;
}

/**
  * @brief  ��ǰ������Ѫ��			
  * @param  void
  * @retval ��ǰ������Ѫ��
  * @attention 
*/
uint16_t JUDGE_usGetRadar_Robo_HP(void)
{
	return Robot_status.remain_HP;
}

/**
  * @brief  ��ǰ���������Ѫ��			
  * @param  void
  * @retval ��ǰ������Ѫ��
  * @attention 
*/
uint16_t JUDGE_usGetRadar_Robo_Max_HP(void)
{
	return Robot_status.max_HP;
}

/**
  * @brief  ��ǰ������17mm �ӵ�ʣ�෢������			
  * @param  void
  * @retval ��ǰ����17mm �ӵ�ʣ�෢������
  * @attention 
17mm �ӵ�ʣ�෢������
����˵��
					������ 														�Կ���
���������� 			ȫ�Ӳ�����Ӣ��ʣ��ɷ��� 17mm ��������			ȫ�� 17mm ����ʣ��ɶһ�����

Ӣ�ۻ����� 			ȫ�Ӳ�����Ӣ��ʣ��ɷ��� 17mm ��������			ȫ�� 17mm ����ʣ��ɶһ�����

���л����ˡ��ڱ������� �û�����ʣ��ɷ��� 17mm �������� 				�û�����ʣ��ɷ��� 17mm ��������

*/
uint16_t JUDGE_usGetRadar_Robo_remain_17mm(void)
{
	return Bullet_remaining.bullet_remaining_num_17mm;
}

uint16_t  JUDGE_usGetRadar_Robo_remain_42mm(void)
{
	return Bullet_remaining.bullet_remaining_num_42mm;
}

/**
  * @brief  shooter��Դ���
  * @param  void
  * @retval 1 �����   0û���
  * @attention  
  */
uint8_t JUDGE_usGetJudge_shooter_Power(void)
{
	return 	Robot_status.mains_power_shooter_output;
}



/****************�����Զ������ж���*******************/
/**
  * @brief  װ�װ��˺������Ƿ����
  * @param  void
  * @retval TRUE�Ѹ���   FALSEû����
  * @attention  
*/
bool JUDGE_IfArmorHurt(void)
{
	static portTickType ulCurrent = 0;
	static uint32_t ulDelay = 0;
	static bool IfHurt = FALSE;//Ĭ��װ�װ崦������״̬

	
	ulCurrent = xTaskGetTickCount();

	if (Hurt_Data_Update == TRUE)//װ�װ����ݸ���
	{
		Hurt_Data_Update = FALSE;//��֤���жϵ��´�װ�װ��˺�����
		ulDelay = ulCurrent + 200;//
		IfHurt = TRUE;
	}
	else if (ulCurrent > ulDelay)//
	{
		IfHurt = FALSE;
	}
	
	return IfHurt;
}

bool Judge_If_Death(void)
{
	if(Robot_status.remain_HP == 0 && JUDGE_sGetDataState() == TRUE)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

bool Judge_If_Chassis(void)
{
	if(Robot_status.mains_power_chassis_output == 1)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  װ�װ��ܴ��ID
  * @param  void
  * @retval   
  * @attention  
		bit 0-3�� ��Ѫ���仯����Ϊװ���˺�������װ�� ID��������ֵΪ 0-4 �Ŵ��������
		�����װ��Ƭ������Ѫ���仯���ͣ��ñ�����ֵΪ 0��
*/
uint8_t Judge_armor_id(void)
{
	return Robot_hurt.armor_id;
}

/**
  * @brief  װ�װ�����ģʽ
  * @param  void
  * @retval   
  * @attention  
		bit 4-7�� Ѫ���仯����
			0x0 װ���˺���Ѫ��
			0x1 ģ����߿�Ѫ��
			0x2 �����ٿ�Ѫ��
			0x3 ��ǹ��������Ѫ��
			0x4 �����̹��ʿ�Ѫ��
			0x5 װ��ײ����Ѫ
*/
uint8_t Judge_hurt_mode(void)
{
	return Robot_hurt.hurt_type;
}				
