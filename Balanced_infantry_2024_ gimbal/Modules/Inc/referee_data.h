#ifndef __REFEREE_DATA_H
#define __REFEREE_DATA_H

#include "bsp_usart.h"
#include "struct_typedef.h"
#include "CRC8_CRC16.h"

#define JUDGE_BUFFER_LEN	200 //����ϵͳ���ڽ��ճ���

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
//Ϊ�˼���һ���Ĵ�����  �˴��Խ�18 19 �����ϵͳ��������׼���� HAL�����ϵͳ�ļ���2021��ʼ

//2021����ϵͳ�ٷ��ӿ�Э��
//ͨ�ŷ�ʽ�Ǵ��ڣ�����Ϊ������ 115200�� 8 λ����λ�� 1 λֹͣλ����Ӳ�����أ���У��λ

//���ȸ���Э�鶨��,���ݶγ���Ϊn��Ҫ����֡ͷ�ڶ��ֽ�����ȡ
#define    LEN_HEADER    5        //֡ͷ��
#define    LEN_CMDID     2        //�����볤��
//#define    LEN_DATA      32        //���ݳ���
#define    LEN_TAIL      2	      //֡βCRC16

//��ʼ�ֽ�,Э��̶�Ϊ0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)

//ͨ��Э��ƫ��λ��
typedef enum 
{
	FRAME_HEADER         = 0,
	CMD_ID               = 5,
	DATA                 = 7,
	
}JudgeFrameOffset;

//5�ֽ�֡ͷ,ƫ��λ��
typedef enum
{
	SOF          = 0,//��ʼλ
	DATA_LENGTH  = 1,//֡�����ݳ���,�����������ȡ���ݳ���
	SEQ          = 3,//�����
	CRC8         = 4 //CRC8
}FrameHeaderOffset;

typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	
} xFrameHeader;

enum judge_robot_ID{
	hero_red       = 1,
	engineer_red   = 2,
	infantry3_red  = 3,
	infantry4_red  = 4,
	infantry5_red  = 5,
	plane_red      = 6,
	
	hero_blue      = 101,
	engineer_blue  = 102,
	infantry3_blue = 103,
	infantry4_blue = 104,
	infantry5_blue = 105,
	plane_blue     = 106,
};

/***************������ID********************/

/* 
   ID: 0x0001  Byte:  11    ����״̬���ݣ�                  1Hz ���ڷ���      
   ID: 0x0002  Byte:  1     ����������ݣ�                  ������������      
   ID: 0x0003  Byte:  32//ԭ28    ����������Ѫ�����ݣ�            1Hz ���ڷ���  
   ID: 0x0004  Byte:  3     ���ڷ���״̬��                  ���ڷ������
   ID: 0x0005  Byte:  11    �˹�������ս���ӳ���ͷ���״̬   1Hz ���ڷ��� 
   ID: 0X0101  Byte:  4    �����¼����ݣ�                  1Hz ���ڷ��� 
   ID: 0X0102  Byte:  3    ���ز���վ������ʶ���ݣ�        ������������
   ID: 0x0103  Byte:  2    	���󲹸�վ��������	          �ɲ����ӷ��ͣ����� 10Hz����RM �Կ�����δ���ţ�   
   ID: 0X0104  Byte:  3//ԭ2    ���о������ݣ�                  ���淢������        
   ID: 0x0105  Byte:  1    ���ڷ���ڵ���ʱ��              1Hz ���ڷ���


   ID: 0x0202  Byte:  16//ԭ14   ʵʱ�����������ݣ�              50Hz ���ڷ���
   ID: 0x0203  Byte:  16   ������λ�����ݣ�                10Hz ����
   ID: 0x0204  Byte:  1    �������������ݣ�                1Hz ���ڷ���
   ID: 0x0205  Byte:  3    ���л���������״̬���ݣ�        10Hz ���ڷ��ͣ�ֻ�п��л��������ط���
   ID: 0x0206  Byte:  1    �˺�״̬���ݣ�                  �˺���������
   ID: 0x0207  Byte:  7//ԭ6    ʵʱ������ݣ�                  �ӵ��������
	 ID: 0x0208  Byte: 6//ԭ2    ����ʣ�෢�����������л����ˣ��ڱ��������Լ� ICRA �����˷���  1Hz ���ڷ���
	 ID: 0x0209  Byte:  4    ������ RFID ״̬��              1Hz ���ڷ���
	 ID: 0x020A  Byte:  12   ���ڻ����˿ͻ���ָ�����ݣ�      10Hz ���ڷ���
	 ID: 0x0301  Byte:  n    �����˼佻�����ݣ�              ���ͷ��������ͣ�
	 ID: 0x0302  Byte:  n    �Զ���������������ݽӿ�,      ͨ���ͻ��˴������ͣ����� 30Hz
	 ID: 0x0303  Byte:  15   �ͻ���С��ͼ��������,  		   ��������
	 ID: 0x0304  Byte:  12	 ���̡������Ϣ,					ͨ��ͼ�����ڷ���
	 ID: 0X0305  Byte:  10	 �ͻ���С��ͼ������Ϣ		������Ƶ�ʣ� 10Hz �״�վ���͵�������Ϣ���Ա����м����������ڵ�һ�ӽ�С��ͼ������	

*/

//������ID,�����жϽ��յ���ʲô����
typedef enum
{ 
	ID_game_status                 = 0x0001,//����״̬����,1Hz���ڷ���
	ID_game_result 	               = 0x0002,//����������ݣ�������������
	ID_robot_HP                    = 0x0003,//����������Ѫ������,1Hz���ڷ���
	ID_dart_status                 = 0x0004,//���ڷ���״̬�����ڷ���ʱ����
	ID_ICRA_buff_debuff_zone_status= 0x0005,
	ID_event_data                  = 0x0101,//�����¼�����,1Hz ���ڷ��� 
	ID_supply_projectile_action    = 0x0102,//���ز���վ������ʶ����,������������
	ID_referee_warning             = 0x0104,//���о������ݣ����淢������ 
	ID_dart_remaining_time         = 0x0105,//���ڷ���ڵ���ʱ��1Hz���ڷ���
  ID_robot_status                = 0x0201,//������״̬����,10Hz���ڷ��� 
	ID_heat_data                   = 0x0202,//ʵʱ������������,50Hz���ڷ��� 
	ID_robot_pos                   = 0x0203,//������λ������,10Hz����
	ID_buff                        = 0x0204,//��������������,1Hz ���ڷ��� 
	ID_aerial_robot_energy         = 0x0205,//���л���������״̬����,10Hz���ڷ���,ֻ�п��л��������ط���
	ID_robot_hurt                  = 0x0206,//�˺�״̬����,���˺���������
	ID_shoot_data                  = 0x0207,//ʵʱ�������,�ӵ�������� 
	ID_bullet_remaining            = 0x0208,//����ʣ�෢�����������л����ˣ��ڱ��������Լ� ICRA �����˷��ͣ�1Hz ���ڷ���
	ID_rfid_status                 = 0x0209,//������ RFID ״̬��1Hz ���ڷ��� 
	ID_dart_client_cmd             = 0x020A,//���ڻ����˿ͻ���ָ�����ݣ�10Hz ���ڷ��� 
	ID_robot_interactive_header_data      = 0x0301, //�����˼佻�����ݣ����ͷ���������
	ID_customize_controller_interaction_data_interface     = 0x0302, //�Զ���������������ݽӿڣ�ͨ���ͻ��˴������ͣ����� 30Hz
  ID_minimap_interactive_data    = 0x0303, //�ͻ���С��ͼ�������ݣ���������
	ID_keybord_and_mouse_massage   = 0x0304, //���̡������Ϣ��ͨ��ͼ�����ڷ���
	ID_client_map_command     	   = 0x0305, //�״�վ���͵�������Ϣ���Ա����м����������ڵ�һ�ӽ�С��ͼ����

} CmdID;

/*δд
	  0x301
	  0x302
*/
//���������ݶγ�,���ݹٷ�Э�������峤��
typedef enum
{
		/* Std */
	LEN_FRAME_HEAD 	                 = 5,	// ֡ͷ����
	LEN_CMD_ID 		                   = 2,	// �����볤��
	LEN_FRAME_TAIL 	                 = 2,	// ֡βCRC16
	
	LEN_game_status                  =  11,	//0x0001
	LEN_game_result                  =  1,	//0x0002
	LEN_robot_HP                     =  32,	//0x0003 28
	LEN_dart_status                  =  3,	//0x0004
	LEN_ICRA_buff_debuff_zone_status =  11, //0x0005
	LEN_event_data                   =  4,	//0x0101
	LEN_supply_projectile_action     =  4,	//0x0102   3
	LEN_referee_warning              =  3,	//0x0104   2
	LEN_dart_remaining_time          =  1,	//0x0105
	LEN_robot_status                 = 27,	//0x0201
	LEN_heat_data                    = 16,	//0x0202  14
	LEN_robot_pos                    = 16,	//0x0203
	LEN_buff                         =  1,	//0x0204
	LEN_aerial_robot_energy          =  3,	//0x0205    
	LEN_robot_hurt                   =  1,	//0x0206  
	LEN_shoot_data                   =  7,	//0x0207  
	LEN_bullet_remaining             =  6,	//0x0208  2
	LEN_rfid_status                  =  4,	//0x0209 
	LEN_dart_client_cmd              = 12,	//0x020A 
	LEN_minimap_interactive_data 	 = 15,   //0x0303
	LEN_keybord_and_mouse_massage 	 = 12,   //0x0304
	LEN_client_map_command    		 = 10,  //0x0305
	
} JudgeDataLength;

//����״̬���ݣ� 0x0001      ���ͷ�Χ�����л����ˡ�
typedef __packed struct
{
//�﷨�����Զ���ı���������ϣ����� �����ñ�����ռ��λ��
uint8_t game_type : 4;
uint8_t game_progress : 4;
uint16_t stage_remain_time;
	
uint64_t SyncTimeStamp;	
} ext_game_status_t;

// 0-3 bit�� ��������
// 1�� RoboMaster ���״�ʦ����
// 2�� RoboMaster ���״�ʦ��������
// 3�� ICRA RoboMaster �˹�������ս��
// 4�� RoboMaster ������ 3V3
// 5�� RoboMaster ������ 1V1
// 4-7 bit�� ��ǰ�����׶�
// 0�� δ��ʼ������
// 1�� ׼���׶Σ�
// 2�� �Լ�׶Σ�
// 3�� 5s ����ʱ��
// 4�� ��ս�У�
// 5�� ����������

//����������ݣ� 0x0002     ���ͷ�Χ�����л����ˡ�
typedef __packed struct
{
uint8_t winner;
} ext_game_result_t;

//������Ѫ������ 0x0003     ���ͷ�Χ�����л����ˡ�
typedef __packed struct
{
uint16_t red_1_robot_HP;
uint16_t red_2_robot_HP;
uint16_t red_3_robot_HP;
uint16_t red_4_robot_HP;
uint16_t red_5_robot_HP;
uint16_t red_7_robot_HP;
uint16_t red_outpost_HP;
uint16_t red_base_HP;
uint16_t blue_1_robot_HP;
uint16_t blue_2_robot_HP;
uint16_t blue_3_robot_HP;
uint16_t blue_4_robot_HP;
uint16_t blue_5_robot_HP;
uint16_t blue_7_robot_HP;
uint16_t blue_outpost_HP;
uint16_t blue_base_HP;
} ext_game_robot_HP_t;

//���ڷ���״̬�� 0x0004          ���ͷ�Χ�����л����ˡ�
typedef __packed struct
{
uint8_t dart_belong;
uint16_t stage_remaining_time;
} ext_dart_status_t;

//�˹�������ս���ӳ���ͷ���״̬�� 0x0005   ���ͷ�Χ�����л����ˡ�
typedef __packed struct
{
uint8_t F1_zone_status:1;
uint8_t F1_zone_buff_debuff_status:3;
uint8_t F2_zone_status:1;
uint8_t F2_zone_buff_debuff_status:3;
uint8_t F3_zone_status:1;
uint8_t F3_zone_buff_debuff_status:3;
uint8_t F4_zone_status:1;
uint8_t F4_zone_buff_debuff_status:3;
uint8_t F5_zone_status:1;
uint8_t F5_zone_buff_debuff_status:3;
uint8_t F6_zone_status:1;
uint8_t F6_zone_buff_debuff_status:3;
uint16_t red1_bullet_left;
uint16_t red2_bullet_left;
uint16_t blue1_bullet_left;
uint16_t blue2_bullet_left;
} ext_ICRA_buff_debuff_zone_status_t;

//�����¼����ݣ� 0x0101      ���ͷ�Χ�� ����������
typedef __packed struct
{
uint32_t event_type;
} ext_event_data_t;

//����վ������ʶ�� 0x0102                ���ͷ�Χ������������
typedef __packed struct
{
uint8_t supply_projectile_id;
uint8_t supply_robot_id;
uint8_t supply_projectile_step;
uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

//���о�����Ϣ�� cmd_id (0x0104)          ���ͷ�Χ������������
typedef __packed struct
{
uint8_t level;
uint8_t foul_robot_id;
} ext_referee_warning_t;

//���ڷ���ڵ���ʱ�� cmd_id (0x0105)      ���ͷ�Χ����������
typedef __packed struct
{
uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

//����������״̬�� 0x0201                  ���ͷ�Χ����һ������
typedef __packed struct
{
uint8_t robot_id;
uint8_t robot_level;
uint16_t remain_HP;
uint16_t max_HP;
uint16_t shooter_id1_17mm_cooling_rate;
uint16_t shooter_id1_17mm_cooling_limit;
uint16_t chassis_power_limit;
uint8_t mains_power_gimbal_output : 1;
uint8_t mains_power_chassis_output : 1;
uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

//ʵʱ�����������ݣ� 0x0202                ���ͷ�Χ����һ������
typedef __packed struct
{
uint16_t chassis_volt;
uint16_t chassis_current;
float chassis_power;
uint16_t chassis_power_buffer;
uint16_t shooter_id1_17mm_cooling_heat;
uint16_t shooter_id2_17mm_cooling_heat;
uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

//������λ�ã� 0x0203                     ���ͷ�Χ����һ�����ˡ�
typedef __packed struct
{
float x;
float y;
float z;
float yaw;
} ext_game_robot_pos_t;

//���������棺 0x0204                      ���ͷ�Χ����һ�����ˡ�
typedef __packed struct
{
uint8_t power_rune_buff;
}ext_buff_t;

//���л���������״̬�� 0x0205              ���ͷ�Χ����һ������
typedef __packed struct
{
uint8_t attack_time;
} aerial_robot_energy_t;

//�˺�״̬�� 0x0206��                      ���ͷ�Χ����һ�����ˡ�
typedef __packed struct
{
uint8_t armor_id : 4;
uint8_t hurt_type : 4;
} ext_robot_hurt_t;

//ʵʱ�����Ϣ�� 0x0207                    ���ͷ�Χ����һ�����ˡ�
typedef __packed struct
{
uint8_t bullet_type;
uint8_t shooter_id;
uint8_t bullet_freq;
float bullet_speed;
} ext_shoot_data_t;

//�ӵ�ʣ�෢������ 0x0208                  ���ͷ�Χ����һ�����ˡ����л����ˣ��ڱ��������Լ� ICRA ������,���ط���
typedef __packed struct
{
uint16_t bullet_remaining_num_17mm;
uint16_t bullet_remaining_num_42mm;
uint16_t coin_remaining_num;
} ext_bullet_remaining_t;


//������ RFID ״̬�� 0x0209                ���ͷ�Χ����һ������
typedef __packed struct
{
uint32_t rfid_status;
} ext_rfid_status_t;

//���ڻ����˿ͻ���ָ�����ݣ� 0x020A        ���ͷ�Χ����һ������
typedef __packed struct
{
uint8_t dart_launch_opening_status;
uint8_t dart_attack_target;
uint16_t target_change_time;
uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;



/*�����˼佻������*/

/* 	
	�������ݰ���һ��ͳһ�����ݶ�ͷ�ṹ��
  ���ݶΰ���������ID���������Լ������ߵ�ID���������ݶΣ�
  �����������ݵİ��ܹ������Ϊ 128 ���ֽڣ�
  ��ȥ frame_header,cmd_id �� frame_tail �� 9 ���ֽ�
  �Լ����ݶ�ͷ�ṹ�� 6 ���ֽڣ��ʶ����͵��������ݶ����Ϊ 113
	������ ID��                                                                                                                                                                                                                                                                                                                                             
	1��Ӣ��(��)��
	2������(��)��
	3/4/5������(��)��
	6������(��)��
	7���ڱ�(��)��
  9���״�վ���죩
	101��Ӣ��(��)��
	102������(��)��
	103/104/105������(��)��
	106������(��)��
	107���ڱ�(��)��
  109���״�վ������	
	�ͻ��� ID�� 
	0x0101 ΪӢ�۲����ֿͻ���( ��) ��
	0x0102 �����̲����ֿͻ��� ((�� )��
	0x0103/0x0104/0x0105�����������ֿͻ���(��)��
	0x0106�����в����ֿͻ���((��)�� 
	0x0165��Ӣ�۲����ֿͻ���(��)��
	0x0166�����̲����ֿͻ���(��)��
	0x0167/0x0168/0x0169�������ֿͻ��˲���(��)��
	0x016A�����в����ֿͻ���(��)�� 
*/

/* �������ݽ�����Ϣ��0x0301  */
typedef __packed struct
{
uint16_t data_cmd_id;
uint16_t send_ID;
uint16_t receiver_ID;
}ext_student_interactive_header_data_t;

///* 
//	�ͻ��� �ͻ����Զ������ݣ�cmd_id:0x0301������ ID:0xD180F
//	����Ƶ�ʣ����� 10Hz

/*	1.	�ͻ��� �ͻ����Զ������ݣ�cmd_id:0x0301������ ID:0xD180������Ƶ�ʣ����� 10Hz 
	�ֽ�ƫ���� 	��С 	˵�� 				��ע 
	0 			2 		���ݵ����� ID 		0xD180 
	2 			2 		�����ߵ� ID 			��ҪУ�鷢���߻����˵� ID ��ȷ�� 
	4 			2 		�ͻ��˵� ID 		ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ��� 
	6 			4 		�Զ��帡������ 1 	 
	10 			4 		�Զ��帡������ 2 	 
	14 			4 		�Զ��帡������ 3 	 
	18 			1 		�Զ��� 8 λ���� 4 	 

*/
typedef __packed struct 
{ 
	float data1; 
	float data2; 
	float data3; 
	uint8_t masks; 
} client_custom_data_t;	

/* 
	ѧ�������˼�ͨ�� cmd_id 0x0301������ ID:0x0200~0x02FF
	�������� �����˼�ͨ�ţ�0x0301��
	����Ƶ�ʣ����� 10Hz  

	�ֽ�ƫ���� 	��С 	˵�� 			      ��ע 
	0 		     	2 		���ݵ����� ID 	0x0200~0x02FF ���������� ID ��ѡȡ������ ID �����ɲ������Զ��� 
	
	2 		    	2 		�����ߵ� ID 	  ��ҪУ�鷢���ߵ� ID ��ȷ�ԣ� 
	
	4 			    2 		�����ߵ� ID 	  ��ҪУ������ߵ� ID ��ȷ�ԣ����粻�ܷ��͵��жԻ����˵�ID 
	
	6 			    n 		���ݶ� 		     	n ��ҪС�� 113 

*/
typedef __packed struct
{
	float bullet;
} robot_interactive_data_t;

/*
	�Զ�����������ݰ���һ��ͳһ�����ݶ�ͷ�ṹ�����ݶ�Ϊ�������ݶΣ������������ݵİ��ܹ������Ϊ
	39 ���ֽڣ���ȥ frame_header,cmd_id �� frame_tail �� 9 ���ֽڣ��ʶ����͵��������ݶ����Ϊ 30 ��
	�ڡ������������� 0x0302 �İ�������Ƶ��Ϊ 30Hz��
*/

////�������ݽ�����Ϣ�� 0x0302�� ����Ƶ�ʣ����� 30Hz
//typedef __packed struct
//{
//uint8_t data[];
//} robot_interactive_data_t;

//С��ͼ������Ϣ��ʶ�� 0x0303�� ����Ƶ�ʣ�����ʱ���͡�
//�ͻ����·���Ϣ
//������ ID�� ������ ID�� 1��Ӣ��(��)�� 2������(��)�� 3/4/5������(��)�� 6������(��)�� 7���ڱ�(��)�� 9����
//��վ���죩�� 10��ǰ��վ���죩�� 11�����أ��죩�� 101��Ӣ��(��)�� 102������(��)�� 103/104/105������
//(��)�� 106������(��)�� 107���ڱ�(��)�� 109���״�վ�������� 110��ǰ��վ�������� 111�����أ����� ��

typedef __packed struct
{
float target_position_x;
float target_position_y;
float target_position_z;
uint8_t commd_keyboard;
uint16_t target_robot_ID;
} ext_robot_command_t;


//С��ͼ������Ϣ��ʶ�� 0x0305�� ������Ƶ�ʣ� 10Hz��
//�״�վ���͵�������Ϣ���Ա����м����������ڵ�һ�ӽ�С��ͼ����

typedef __packed struct
{ 
uint16_t target_robot_ID;
float target_position_x;
float target_position_y;
} ext_client_map_command_t;

//ͼ��ң����Ϣ��ʶ�� 0x0304������Ƶ�ʣ� 30Hz��
//�ͻ����·���Ϣ
//�ֽ�ƫ����      ��С          		˵��
//	  0			   2			��� X ����Ϣ
//	  2 		   2			��� Y ����Ϣ
//	  4			   2			��������Ϣ
//	  6			   1			������
//	  7			   1     		����Ҽ�����
//	  8			   2			������Ϣ
//bit 0������ W �Ƿ���
//bit 1������ S �Ƿ���
//bit 2������ A �Ƿ���
//bit 3������ D �Ƿ���
//bit 4������ SHIFT �Ƿ���
//bit 5������ CTRL �Ƿ���
//bit 6������ Q �Ƿ���
//bit 7������ E �Ƿ���
//bit 8������ R �Ƿ���
//bit 9������ F �Ƿ���
//bit 10������ G �Ƿ���
//bit 11������ Z �Ƿ���
//bit 12������ X �Ƿ���
//bit 13������ C �Ƿ���
//bit 14������ V �Ƿ���
//bit 15������ B �Ƿ���

//�Լ����Ĺ��ṹ������	ԭ   ext_robot_command_t;
typedef __packed struct
{
int16_t mouse_x;
int16_t mouse_y;
int16_t mouse_z;
int8_t left_button_down;
int8_t right_button_down;
uint16_t keyboard_value;
uint16_t reserved;
} ext_robot_command_t_change;

/*
	����
*/
//֡ͷ  ������   ���ݶ�ͷ�ṹ  ���ݶ�   ֡β
//�ϴ��ͻ���
typedef __packed struct
{
	xFrameHeader   				 			             txFrameHeader;//֡ͷ(5���ֽ�)
	uint16_t		 				 	                 	 CmdID;//������(2���ֽ�)
	ext_student_interactive_header_data_t    dataFrameHeader;//���ݶ�ͷ�ṹ(6���ֽ�)
	client_custom_data_t  					         clientData;//���ݶ�(14���ֽ�)
	uint16_t		 						                 FrameTail;//֡β(2���ֽ�)
}ext_SendClientData_t;

//�����˽�����Ϣ
typedef __packed struct
{
	xFrameHeader   					txFrameHeader;//֡ͷ
	uint16_t								CmdID;//������
	ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
	robot_interactive_data_t  interactData;//���ݶ�
	uint16_t		 						FrameTail;//֡β
}ext_CommunatianData_t;

typedef struct{
	uint16_t teammate_hero;
	uint16_t teammate_engineer;
	uint16_t teammate_infantry3;
	uint16_t teammate_infantry4;
	uint16_t teammate_infantry5;
	uint16_t teammate_plane;
	uint16_t teammate_sentry;
	
	uint16_t client_hero;
	uint16_t client_engineer;
	uint16_t client_infantry3;
	uint16_t client_infantry4;
	uint16_t client_infantry5;
	uint16_t client_plane;
} ext_interact_id_t;

typedef struct judge_info_struct {
	xFrameHeader							FrameHeader;				// ֡ͷ��Ϣ
	
	ext_game_status_t 							GameState;				// 0x0001           ����״̬����
	ext_game_result_t 							GameResult;				// 0x0002         �����������
	ext_game_robot_HP_t 						GameRobotHP;			// 0x0003         ������Ѫ������
	ext_dart_status_t								GameRobotmissile;				// 0x0004         ���ڷ���״̬
	ext_ICRA_buff_debuff_zone_status_t	Game_ICRA_buff;      //           �˹�������ս���ӳ���ͷ���״̬
	
	ext_event_data_t								EventData;					// 0x0101         �����¼�����
	ext_supply_projectile_action_t	SupplyProjectileAction;		// 0x0102 ����վ������ʶ
	ext_referee_warning_t						RefereeWarning;		// 0x0104         ���о�����Ϣ
	ext_dart_remaining_time_t				dart_remaining_time;// 0x0105         ���ڷ���ڵ���ʱ
	
	ext_game_robot_status_t					GameRobotStat;	// 0x0201         ����������״̬
	ext_power_heat_data_t						PowerHeatData;		// 0x0202         ʵʱ������������
	ext_game_robot_pos_t						GameRobotPos;			// 0x0203         ������λ��
	ext_buff_t											Buff;								// 0x0204     ����������
	aerial_robot_energy_t				AerialRobotEnergy;// 0x0205             ���л���������״̬
	ext_robot_hurt_t								RobotHurt;					// 0x0206         �˺�״̬
	ext_shoot_data_t								ShootData;					// 0x0207         ʵʱ�����Ϣ(��Ƶ  ����  �ӵ���Ϣ)
	ext_bullet_remaining_t					bullet_remaining;		// 0x0208	        �ӵ�ʣ�෢����
	ext_rfid_status_t								rfid_status;				// 0x0209	        RFID��Ϣ
	ext_dart_client_cmd_t           dart_client;        // 0x020A         ���ڿͻ���
	
	ext_interact_id_t								ids;								//�뱾�������Ļ�����id
	uint16_t                        self_client;        //�����ͻ���
	bool	 		IF_REF_ONL;

} Referee_info_t;

/**********����ϵͳ���ݶ�ȡ*************/
bool Judge_Read_Data(uint8_t *ReadFormUsart);//��ѭ��4ms����һ��
void JUDGE_Show_Data(void);
void Send_to_Teammate(void);
bool is_red_or_blue(void);
//void determine_ID(void);
void Determine_ID(void);//�ж��Լ����ĸ�����
bool Judge_If_Chassis(void);

/*****�������ݸ����жϺ���********/
bool JUDGE_sGetDataState(void);    //�ж��Ƿ���ܵ�����ϵͳ
float JUDGE_fGetChassisPower(void);
uint16_t JUDGE_fGetRemainEnergy(void);
uint8_t JUDGE_ucGetRobotLevel(void);
//��ǹ��������ȡǹ������
uint16_t JUDGE_usGetRemoteHeat_id1_17mm(void);
uint16_t JUDGE_usGetRemoteHeat_id2_17mm(void);
uint16_t JUDGE_usGetRemoteHeat_id1_42mm(void);
//��ȡ����
float JUDGE_usGetSpeedHeat(void);
//ͳ�Ʒ�����
//ATTENTION��������������������������˫ǹ�ܿ��ܻ��������������������������������
void JUDGE_ShootNumCount_17mm(void);
void JUDGE_ShootNumCount_42mm(void);
//��ȡ������
//ATTENTION��������������������������˫ǹ�ܿ��ܻ��������������������������������
uint16_t JUDGE_usGetShootNum_17mm(void);
uint16_t JUDGE_usGetShootNum_42mm(void);
//����������
//ATTENTION��������������������������˫ǹ�ܿ��ܻ��������������������������������
void JUDGE_ShootNum_Clear_17mm(void);
void JUDGE_ShootNum_Clear_42mm(void);
//��ǹ��������ȡǹ����������
uint16_t JUDGE_usGetHeatLimit_id1_17mm(void);
uint16_t JUDGE_usGetHeatLimit_id2_17mm(void);
uint16_t JUDGE_usGetHeatLimit_id1_42mm(void);
//��ǹ��������ȡǹ������ÿ����ȴֵ
uint16_t JUDGE_usGetShootCold_id1_17mm(void);
uint16_t JUDGE_usGetShootCold_id2_17mm(void);
uint16_t JUDGE_usGetShootCold_id1_42mm(void);
//����վ��
uint8_t JUDGE_usGetSupply_Id(void);
//���������� ID
uint8_t JUDGE_usGetSupply_Robo_Id(void);
//�����ڿ���״̬ 
uint8_t JUDGE_usGetSupply_Mode(void);
//���ص�ǰ����������Ϊ�ַ��ͱ�����
uint8_t JUDGE_usGetSupply_Num(void);
//С��ͼ������ϢĿ������� ID(�״�վ)
uint16_t JUDGE_usGetRadar_Station_to_robo_ID(void);
//С��ͼ������ϢĿ������� x λ������
float JUDGE_usGetRadar_Station_to_robo_posX(void);
//С��ͼ������ϢĿ������� y λ������
float JUDGE_usGetRadar_Station_to_robo_posy(void);

//��ǰ������Ѫ��
uint16_t JUDGE_usGetRadar_Robo_HP(void);
//��ǰ���������Ѫ��
uint16_t JUDGE_usGetRadar_Robo_Max_HP(void);
//17mm �ӵ�ʣ�෢������
uint16_t JUDGE_usGetRadar_Robo_remain_17mm(void);
uint16_t  JUDGE_usGetRadar_Robo_remain_42mm(void);
//shooter��Դ���
uint8_t JUDGE_usGetJudge_shooter_Power(void);
float JUDGE_fGetChassisVolt(void);

bool JUDGE_IfArmorHurt(void);
bool Judge_If_Death(void);

//���̹��ʺ���
uint16_t JUDGE_usGetChassisPowerLimit(void);//��ȡ��ǰ�ȼ����̹�������
uint8_t JUDGE_ucRobotLevel(void);//��ȡ�����˵ȼ�����

uint8_t Judge_armor_id(void);//װ�װ��ܴ��ID
uint8_t Judge_hurt_mode(void);//�˺�����
void JUDGE_Show_Graph(void); //�ϴ��ͻ���ͼ��

extern Referee_info_t 	REF;
void RefereeInit();
#endif
	
#ifdef __cplusplus
}
#endif

#endif
