/**
  ****************************(C) COPYRIGHT 2023 TUP****************************
  * @file       referee_data.cpp/h
  * @brief      Judge system data reading, C board through the serial port one for data transmission,
  *							A board through the serial port three.For data transmission, the specific protocol 
	*							reference serial port the appendix.
  *							This file is forbidden to be modified.
  *             裁判系统数据读取，C板通过串口一进行数据传递，A板通过串口三
	*							进行数据传递，具体协议参考串口协议附录
	*							正常此文件禁止修改
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
/*****************串口实例定义**********************/
Usart_Instance_t referee_usart; //裁判系统串口实例

//串口接收回调函数用以解析数据
void DecodeRefereeData()
{
	Judge_Read_Data(referee_usart.rx_buff);  //读取信息
}

//串口初始化
void RefereeInit()
{
	USARTInit(&referee_usart,&huart6,USART_RX_DMA,JUDGE_BUFFER_LEN,DecodeRefereeData); //初始化设置串口接收
	UsartRegister(&referee_usart);  //登记注册
}

/*****************系统数据定义**********************/
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


xFrameHeader              FrameHeader;		//发送帧头信息
//未用
//ext_SendClientData_t      ShowData;			//客户端信息
//ext_CommunatianData_t     CommuData;		//队友通信信息
//ext_SendClientGraph_t     ClientGraph;  //图形通信
//ext_SendClientGraphDelete_t ClientGraphDelete; //图形删除


/****************************************************/
bool Judge_Data_TF = FALSE;//裁判数据是否可用,辅助函数调用
uint8_t Judge_Self_ID;//当前机器人的ID
uint16_t Judge_SelfClient_ID;//发送者机器人对应的客户端ID

/**************裁判系统数据辅助****************/
uint16_t ShootNum_17mm;//统计发弹量,0x0003触发一次则认为发射了一颗
uint16_t ShootNum_42mm;//统计发弹量,0x0003触发一次则认为发射了一颗
bool Hurt_Data_Update = FALSE;//装甲板伤害数据是否更新,每受一次伤害置TRUE,然后立即置FALSE,给底盘闪避用

#define BLUE  0
#define RED   1

Referee_info_t 	REF; // 裁判系统信息

/**
  * @brief  读取裁判数据,loop中循环调用此函数来读取数据
  * @param  缓存数据
  * @retval 是否对正误判断做处理
  * @attention  在此判断帧头和CRC校验,无误再写入数据
  */
bool Judge_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE;//数据正确与否标志,每次调用读取裁判系统数据函数都先默认为错误
	
	uint16_t judge_length;//统计一帧数据长度 
	
	int CmdID = 0;//数据命令码解析
	
	/***------------------*****/
	//无数据包，则不作任何处理
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	
	//写入帧头数据,用于判断是否开始存储裁判数据
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
	memcpy(&REF.FrameHeader,ReadFromUsart,LEN_HEADER);   //储存帧头数据
	
		//判断帧头数据是否为0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		//帧头CRC8校验
		if (verify_CRC8_check_sum( ReadFromUsart, LEN_HEADER ) == TRUE)
		{
			//统计一帧数据长度,用于CR16校验
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			
			//帧尾CRC16校验
			if(verify_CRC16_check_sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//都校验过了则说明数据可用
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
			
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
					  if(Robot_hurt.hurt_type == 0)//非装甲板离线造成伤害
						{	Hurt_Data_Update = TRUE;	}//装甲数据每更新一次则判定为受到一次伤害
					break;
					
					case ID_shoot_data:      //0x0207
						memcpy(&Shoot_data, (ReadFromUsart + DATA), LEN_shoot_data);
//					    JUDGE_ShootNumCount_17mm();//发弹量统计,不适用于双枪管,不准
//					    JUDGE_ShootNumCount_42mm();//发弹量统计,不适用于双枪管,不准
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
				
				//首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
				if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
				{
					//如果一个数据包出现了多帧数据,则再次读取
					Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
				}
			}					
		}		
	}
	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE;//辅助函数用
	}
	else		//只要CRC16校验不通过就为FALSE
	{
		Judge_Data_TF = FALSE;//辅助函数用
	}

	return retval_tf;//对数据正误做处理	

}

/**
  * @brief  判断自己红蓝方
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
bool Color;
bool is_red_or_blue(void)
{
	Judge_Self_ID = Robot_status.robot_id;//读取当前机器人ID
	
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
  * @brief  判断自身ID，选择客户端ID
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
//void determine_ID(void)
//{
//	Color = is_red_or_blue();
//	if(Color == BLUE)
//	{
//		Judge_SelfClient_ID = 0x0110 + (Judge_Self_ID-10);//计算客户端ID
//	}
//	else if(Color == RED)
//	{
//		Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//计算客户端ID
//	}
//}
void Determine_ID(void)//判断自己是哪个队伍
{
	if(REF.GameRobotStat.robot_id < 10)//本机器人的ID，红方
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
		
		if     (REF.GameRobotStat.robot_id == hero_red)//不断刷新放置在比赛中更改颜色
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
	else //蓝方
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

/********************裁判数据辅助判断函数***************************/

/**
  * @brief  数据是否可用
  * @param  void
  * @retval  TRUE可用   FALSE不可用
  * @attention  在裁判读取函数中实时改变返回值
*/
bool JUDGE_sGetDataState(void)
{
	return Judge_Data_TF;
}

/**
  * @brief  读取瞬时功率
  * @param  void
  * @retval 实时功率值
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
	* @brief  读取剩余焦耳能量
	* @param  void
	* @retval 剩余缓冲焦耳能量(最大60)
	* @attention  
*/
uint16_t JUDGE_fGetRemainEnergy(void)
{
	return (Power_heat_data.chassis_power_buffer);
}

/**
  * @brief  读取当前等级
  * @param  void
  * @retval 当前等级
  * @attention  
*/
uint8_t JUDGE_ucGetRobotLevel(void)
{
    return	Robot_status.robot_level;
}

/**
  * @brief  读取id1_17mm枪口热量
  * @param  void
  * @retval 17mm
  * @attention  
*/
uint16_t JUDGE_usGetRemoteHeat_id1_17mm(void)
{
	return Power_heat_data.shooter_id1_17mm_cooling_heat;
}

/**
  * @brief  读取id2_17mm枪口热量
  * @param  void
  * @retval 17mm
  * @attention  
*/
uint16_t JUDGE_usGetRemoteHeat_id2_17mm(void)
{
	return Power_heat_data.shooter_id2_17mm_cooling_heat;
}

/**
  * @brief  读取id1_42mm枪口热量
  * @param  void
  * @retval 42mm
  * @attention  
*/
uint16_t JUDGE_usGetRemoteHeat_id1_42mm(void)
{
	return Power_heat_data.shooter_id1_42mm_cooling_heat;
}


/**
  * @brief  读取射速
  * @param  void
  * @retval 17mm
  * @attention  实时热量
*/
float JUDGE_usGetSpeedHeat(void)
{
	return Shoot_data.bullet_speed;
}

/**
  * @brief  统计17mm发弹量
  * @param  void
  * @retval void
  * @attention  
*/
portTickType shoot_time_17mm;//发射延时测试
portTickType shoot_ping_17mm;//计算出的最终发弹延迟

float Shoot_Speed_Now_17mm = 0;
float Shoot_Speed_Last_17mm = 0;

void JUDGE_ShootNumCount_17mm(void)
{
	//检测到是小弹丸  bullet_type==1
	if(Shoot_data.bullet_type == 1)
	{
	Shoot_Speed_Now_17mm = Shoot_data.bullet_speed;
	if(Shoot_Speed_Last_17mm != Shoot_Speed_Now_17mm)//因为是float型，几乎不可能完全相等,所以速度不等时说明发射了一颗弹
	{
		ShootNum_17mm++;
		Shoot_Speed_Last_17mm = Shoot_Speed_Now_17mm;
	}
	shoot_time_17mm = xTaskGetTickCount();//获取弹丸发射时的系统时间
//	shoot_ping_17mm = shoot_time_17mm - REVOL_uiGetRevolTime();//计算延迟
	}
}

/**
  * @brief  统计42mm发弹量
  * @param  void
  * @retval void
  * @attention  
*/
portTickType shoot_time_42mm;//发射延时测试
portTickType shoot_ping_42mm;//计算出的最终发弹延迟

float Shoot_Speed_Now_42mm = 0;
float Shoot_Speed_Last_42mm = 0;

void JUDGE_ShootNumCount_42mm(void)
{
	//检测到是大弹丸  bullet_type==2
	if(Shoot_data.bullet_type == 2)
	{
	Shoot_Speed_Now_42mm = Shoot_data.bullet_speed;
	if(Shoot_Speed_Last_42mm != Shoot_Speed_Now_42mm)//因为是float型，几乎不可能完全相等,所以速度不等时说明发射了一颗弹
	{
		ShootNum_42mm++;
		Shoot_Speed_Last_42mm = Shoot_Speed_Now_42mm;
	}
	shoot_time_42mm = xTaskGetTickCount();//获取弹丸发射时的系统时间
//	shoot_ping_42mm = shoot_time_42mm - REVOL_uiGetRevolTime();//计算延迟
	}
}


/**
  * @brief  读取17mm发弹量
  * @param  void
  * @retval 发弹量
  * @attention 不适用于双枪管
*/
uint16_t JUDGE_usGetShootNum_17mm(void)
{
	return ShootNum_17mm;
}

/**
  * @brief  读取42mm发弹量
  * @param  void
  * @retval 发弹量
  * @attention 不适用于双枪管
*/
uint16_t JUDGE_usGetShootNum_42mm(void)
{
	return ShootNum_42mm;
}
/**
	* @brief  17mm发弹量清零
  * @param  void
  * @retval void
  * @attention 
*/
void JUDGE_ShootNum_Clear_17mm(void)
{
	ShootNum_17mm = 0;
}
/**
  * @brief  42mm发弹量清零
	* @param  void
	* @retval void
	* @attention 
*/
void JUDGE_ShootNum_Clear_42mm(void)
{
	ShootNum_42mm = 0;
}

/**
  * @brief  读取id1_17mm枪口热量
  * @param  void
  * @retval 当前等级17mm热量上限
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit_id1_17mm(void)
{
	return Robot_status.shooter_id1_17mm_cooling_limit;
}



/**
	* @brief  读取底盘功率最大值
	* @param  void
	* @retval 当前等级底盘功率上限
	* @attention  
*/
uint16_t JUDGE_usGetChassisPowerLimit(void)
{
	return Robot_status.chassis_power_limit;
}


/**
	* @brief  读取机器人等级
	* @param  void
	* @retval 
	* @attention  
*/
uint8_t JUDGE_ucRobotLevel(void)
{
	return Robot_status.robot_level;
}

/**
  * @brief  当前等级id1_17mm对应的枪口每秒冷却值
  * @param  void
  * @retval 当前等级17mm冷却速度
  * @attention  
*/
uint16_t JUDGE_usGetShootCold_id1_17mm(void)
{
	return Robot_status. shooter_id1_17mm_cooling_rate;
}

/**
  * @brief  当前等级id2_17mm对应的枪口每秒冷却值
  * @param  void
  * @retval 当前等级17mm冷却速度
  * @attention  
*/
uint16_t JUDGE_usGetShootCold_id2_17mm(void)
{
	return Robot_status.shooter_id1_17mm_cooling_rate;
}


/**
  * @brief  补给站口 ID：
  * @param  void
  * @retval 当前补给站口 ID：
  * @attention  为字符型变量
	1： 1 号补给口
	2： 2 号补给口
*/
uint8_t JUDGE_usGetSupply_Id(void)
{
	return Supply_projectile_action.supply_projectile_id;
}

/**
  * @brief  补给机器人 ID：
  * @param  void
  * @retval 当前需补给机器人 ID：
  * @attention  为字符型变量
	补弹机器人 ID： 0 为当前无机器人补弹， 1 为红方英雄机器人补弹， 2 为红方工程机
	器人补弹， 3/4/5 为红方步兵机器人补弹， 101 为蓝方英雄机器人补弹， 102 为蓝方工
	程机器人补弹， 103/104/105 为蓝方步兵机器人补弹
*/
uint8_t JUDGE_usGetSupply_Robo_Id(void)
{
	return Supply_projectile_action.supply_projectile_id;
}


/**
  * @brief  出弹口开闭状态 
  * @param  void
  * @retval 出弹口开闭状态 
  * @attention  为字符型变量
	0 为关闭，
	1 为子弹准备中，
	2 为子弹下落
*/
uint8_t JUDGE_usGetSupply_Mode(void)
{
	return Supply_projectile_action.supply_projectile_step ;
}

/**
  * @brief  当前补弹数量
  * @param  void
  * @retval 当前补弹数量
  * @attention 为字符型变量
	50： 50 颗子弹；
	100： 100 颗子弹；
	150： 150 颗子弹；
	200： 200 颗子弹
*/ 
uint8_t JUDGE_usGetSupply_Num(void)
{
	return Supply_projectile_action.supply_projectile_num;
}

/**
  * @brief  小地图接收信息目标机器人 ID(雷达站)
  * @param  void
  * @retval 当前补弹数量
  * @attention 

*/ 
uint16_t JUDGE_usGetRadar_Station_to_robo_ID(void)
{
	return Client_map_command_t.target_robot_ID;
}

/**
  * @brief  小地图接收信息目标机器人 x 位置坐标，单位 m 当 x,y 超出界限时则不显示
			(雷达站)
  * @param  void
  * @retval 当前补弹数量
  * @attention 
*/ 
float JUDGE_usGetRadar_Station_to_robo_posX(void)
{
	return	Client_map_command_t.target_position_x;
}

/**
  * @brief  小地图接收信息目标机器人 y 位置坐标，单位 m 当 x,y 超出界限时则不显示
			(雷达站)
  * @param  void
  * @retval 当前补弹数量
  * @attention 

*/ 
float JUDGE_usGetRadar_Station_to_robo_posy(void)
{
	return	Client_map_command_t.target_position_y;
}

/**
  * @brief  当前机器人血量			
  * @param  void
  * @retval 当前机器人血量
  * @attention 
*/
uint16_t JUDGE_usGetRadar_Robo_HP(void)
{
	return Robot_status.remain_HP;
}

/**
  * @brief  当前机器人最大血量			
  * @param  void
  * @retval 当前机器人血量
  * @attention 
*/
uint16_t JUDGE_usGetRadar_Robo_Max_HP(void)
{
	return Robot_status.max_HP;
}

/**
  * @brief  当前机器人17mm 子弹剩余发射数量			
  * @param  void
  * @retval 当前机器17mm 子弹剩余发射数量
  * @attention 
17mm 子弹剩余发射数量
含义说明
					联盟赛 														对抗赛
步兵机器人 			全队步兵与英雄剩余可发射 17mm 弹丸总量			全队 17mm 弹丸剩余可兑换数量

英雄机器人 			全队步兵与英雄剩余可发射 17mm 弹丸总量			全队 17mm 弹丸剩余可兑换数量

空中机器人、哨兵机器人 该机器人剩余可发射 17mm 弹丸总量 				该机器人剩余可发射 17mm 弹丸总量

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
  * @brief  shooter电源输出
  * @param  void
  * @retval 1 已输出   0没输出
  * @attention  
  */
uint8_t JUDGE_usGetJudge_shooter_Power(void)
{
	return 	Robot_status.mains_power_shooter_output;
}



/****************底盘自动闪避判断用*******************/
/**
  * @brief  装甲板伤害数据是否更新
  * @param  void
  * @retval TRUE已更新   FALSE没更新
  * @attention  
*/
bool JUDGE_IfArmorHurt(void)
{
	static portTickType ulCurrent = 0;
	static uint32_t ulDelay = 0;
	static bool IfHurt = FALSE;//默认装甲板处于离线状态

	
	ulCurrent = xTaskGetTickCount();

	if (Hurt_Data_Update == TRUE)//装甲板数据更新
	{
		Hurt_Data_Update = FALSE;//保证能判断到下次装甲板伤害更新
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
  * @brief  装甲板受打击ID
  * @param  void
  * @retval   
  * @attention  
		bit 0-3： 当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人
		的五个装甲片，其他血量变化类型，该变量数值为 0。
*/
uint8_t Judge_armor_id(void)
{
	return Robot_hurt.armor_id;
}

/**
  * @brief  装甲板受伤模式
  * @param  void
  * @retval   
  * @attention  
		bit 4-7： 血量变化类型
			0x0 装甲伤害扣血；
			0x1 模块掉线扣血；
			0x2 超射速扣血；
			0x3 超枪口热量扣血；
			0x4 超底盘功率扣血；
			0x5 装甲撞击扣血
*/
uint8_t Judge_hurt_mode(void)
{
	return Robot_hurt.hurt_type;
}				
