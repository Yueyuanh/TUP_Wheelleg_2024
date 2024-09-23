/**
 ******************************************************************************
 * @file    sys_task.cpp
 * @author  Xushuang
 * @version V1.0.0 Xushuang 基本完成
 *					V1.1.0 Xushuang 优化封装
 *					V2.0.0 Xushuang 优化模式设置
 * @date    2024/1/11
 * @brief		此处为模式设置和各按键处理
 ******************************************************************************
 * @attention
 *实际比赛有必要为飞坡单独写一个模式-----对准后禁止YAW轴移动
 *调试时
 *步兵、英雄统一规定遥控器对应模式：
 *		左↓	右↓：无力	摩擦轮不动
 *		左↓	右-：正常控制（底盘跟随云台）/单控底盘
 *		左↓ 右↑：小陀螺 摩擦轮不动
 *		左-	右-：开启摩擦轮
 *		左↑ 右-↑：拨盘转动打弹
 *		右-：左↙ 右↗：开启自瞄（单按模式）
 *		拨轮：向正方向开启自瞄（松开即退出自瞄）
 *					向负方向开启超级电容
 ******************************************************************************
 */
#include "system.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "tim.h"

//系统实例
system_t sys;

/**
	* @brief          系统类构造函数
  * @param[in]      NULL
  * @retval         NULL
  */
system_t::system_t()
{
	/*****初始模式设置*****/
	sys_pub.mode = ZERO_FORCE;
	sys_pub.fir_mode = FORBID_FIRE;
	sys_pub.stir_mode = NO_MOVE;
	/*****初始发弹模式设置*****/
	key_shoot_mode_set = 1;
	debug_shoot_mode_set = 1;
	/*****初始控制量设置*****/
	vx_ramp_set = vy_ramp_set = vx_set_channel = vy_set_channel =0.0f;
	mouse_yaw = mouse_pitch = key_yaw = 0.0f;
	yaw_channel = pitch_channel = 0.0f;

	sys_pub.offset_firc_v=0;

}

/**
	* @brief          系统初始化
  * @param[in]      NULL
  * @retval         NULL
  */
void SysInit()
{
	/******遥控器指针获取******/
	sys.system_rc_ctrl = GetRemoteControlPoint();
	/******信息中心实例建立******/
	CenterPointer()->PointerInit(&sys.sys_pub,SYSPUB);
}

/**
	* @brief          系统主任务
  * @param[in]      NULL
  * @retval         NULL
  */
void SystemTask()
{
	/*****模式设置*****/
	sys.KeyBoardModeSet(); //键盘设置
	sys.ThumbWheelModeSet(); 	 //拨轮设置
	sys.RobotModeSet();  //机器人模式设置
	/*****操作量设置*****/
	sys.CalControlQuantity();
}

/**
	* @brief          机器人模式设置
  * @param[in]      NULL
  * @retval         NULL
  */
void system_t::RobotModeSet()
{
	/*****遥控器失联模式判断*****/
	if(!MonitorRc())
	{
		//失联模式进行保护
		sys_pub.mode = DT7_MISSING;
		sys_pub.fir_mode = CLOSE;
		sys_pub.stir_mode = NO_MOVE;
		return;
	}
	
	/*****正常模式控制*****/
	if(sys_pub.mode == INIT)
	{
		//防止拨杆突然到最底下的情况
		if(switch_is_down(system_rc_ctrl->rc.s[RIGTH_CHANNEL]))
			sys_pub.mode = ZERO_FORCE;
		return;
		
	}else
	{
		/**********运动模式设置**********/
		if(switch_is_down(system_rc_ctrl->rc.s[RIGTH_CHANNEL]) )
		{  
			//无力模式
			sys_pub.mode = ZERO_FORCE;			
		}else if(switch_is_mid(system_rc_ctrl->rc.s[RIGTH_CHANNEL]))
		{ 
			//正常控制模式-键盘同时可以控制
			sys_pub.mode = NORMAL;
			//进入自瞄模式
			if(sys_pub.key_flag.auto_aim_flag == 1)
				sys_pub.mode = AUTO;
		}else if(switch_is_up(system_rc_ctrl->rc.s[RIGTH_CHANNEL]))
		{  
			//小陀螺模式
			sys_pub.mode = SPIN;
			//进入小陀螺自瞄模式
			if(sys_pub.key_flag.auto_aim_flag == 1)
				sys_pub.mode = SPIN_AUTO;
		}
		
		/**********初始化模式判断**********/
		if(last_mode == ZERO_FORCE && sys_pub.mode != ZERO_FORCE )
			sys_pub.mode = INIT;

		/**********模式切换判断**********/
		if(last_mode != sys_pub.mode)
			sys_pub.change_mode_flag = 1;
		//保存上次模式，用以判断模式切换
		last_mode = sys_pub.mode; 
		




		/**********射击模式设置**********/
		//右拨杆设置（中间或者上方进行正常设置，下方时禁止发射）
		if(!switch_is_down(system_rc_ctrl->rc.s[RIGTH_CHANNEL]))
		{
			//左拨杆设置
			if(switch_is_down(system_rc_ctrl->rc.s[LEFT_CHANNEL]))
			{
					//摩擦轮关闭，拨盘禁止运动
					sys_pub.fir_mode = CLOSE;
					sys_pub.stir_mode = NO_MOVE;

			}else
			{


				//摩擦轮开启，拨盘允许转动（即默认为角度闭环模式）
				sys_pub.fir_mode = OPEN;
				sys_pub.stir_mode = ALLOW_MOVE;

				
				if(RevolverPointer()->rest_heat<=45) 
				{
						sys_pub.stir_mode = ALLOW_MOVE; 
						return;
				}

				//拨盘拨弹模式设置（0，1，2，3：设为单发、三发、连发模式）
				if(key_shoot_flag == 1 || debug_shoot_flag == 1)
				{
					 sys_pub.stir_mode = SHOOT_1_BULLET;

				}else if(key_shoot_flag == 2 || debug_shoot_flag == 2)
				{
					 sys_pub.stir_mode = SHOOT_3_BULLET;

				}else if(key_shoot_flag == 3 || debug_shoot_flag == 3)
				{
					//连发速度环模式---卡弹在里面
					sys_pub.stir_mode = SHOOT_FIXED_FIRE_RATE;
					if(RevolverPointer()->if_stuck)
					{
						if(RevolverPointer()->reserve_time == 0)
						{
							sys_pub.stir_mode=REVERSE_MOVE;
							RevolverPointer()->reserve_time = 1;
						}
						else
						{
							sys_pub.stir_mode=ALLOW_MOVE;
						}							
						
						if(RevolverPointer()->shoot_finish)
						{
							sys_pub.stir_mode = SHOOT_FIXED_FIRE_RATE;
							RevolverPointer()->if_stuck=0;
							RevolverPointer()->reserve_time = 0;		
						}
					}
				}else if(key_shoot_flag == 0 || debug_shoot_flag ==0)
				{
					 sys_pub.stir_mode = ALLOW_MOVE;
				}




			}
		}else   
		{
			//摩擦轮关闭，拨盘禁止运动
			sys_pub.fir_mode = CLOSE;
			sys_pub.stir_mode = NO_MOVE;
		}
	}
}

/**
	* @brief          拨轮模式设置
  * @param[in]      NULL
  * @retval         NULL
  */
void system_t::ThumbWheelModeSet()
{
	/*****下半边拨轮的状态使用*****/
	//用于拨盘模式设置
	
	stand_state=RC_S_LEFT;
	
	if(sys_pub.stir_mode == ALLOW_MOVE) //拨轮设置为此种模式下才可进行模式设置
	{

		if(RC_STAND_UP)
			stand_up_state_time++;//上拨状态计时
	}

	if(stand_state == 1 && stand_state_last == 3 && debug_shoot_mode_set)
	{
			debug_shoot_flag=1;//单发
	}
	else if(stand_up_state_time>500 && debug_shoot_mode_set)
	{
			debug_shoot_flag=3;//连发
	}
	else
	{
			debug_shoot_flag=0;
	}

	if(RC_STAND)
	{
			debug_shoot_flag=0;//清零
			stand_up_state_time=0;
	}

	stand_up_state_time_last=stand_up_state_time;//更新
	stand_state_last=stand_state;



}

/**
	* @brief          操作设置
  * @param[in]      NULL
  * @retval         NULL
  */
void system_t::CalControlQuantity()
{
	//遥控器控制量设置
	RemoteQuantitySet();
	//按键控制量设置
	KeyBoardQuantitySet();
	
	//发布的信息基础解算
	sys_pub.add_yaw = (yaw_channel * YAW_RC_SEN - 
												 mouse_yaw +
												 key_yaw*0.00003);
	
	sys_pub.add_pit = (pitch_channel * PITCH_RC_SEN - 
												 mouse_pitch 
												);
	
	vx_ramp_set = RAMP_float(vx_set_channel,vx_ramp_set,20);
	vy_ramp_set = RAMP_float(vy_set_channel,vy_ramp_set,20);
	
	//停止信号，不需要缓慢加速，直接减速到零
	if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
		vx_ramp_set = 0.0f;

	if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
		vy_ramp_set = 0.0f;
	
	//斜坡赋值
	sys_pub.vx_set = vx_ramp_set;
	sys_pub.vy_set = vy_ramp_set;
}

/**
	* @brief          遥控器控制量设置
  * @param[in]      NULL
  * @retval         NULL
  */
void system_t::RemoteQuantitySet()
{
	int16_t vx_channel, vy_channel;
	
	//将遥控器的数据处理死区 int16_t yaw_channel,pitch_channel
	rc_deadband_limit(system_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
  rc_deadband_limit(system_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
	
//	//遥控器死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
//    rc_deadband_limit(system_rc_ctrl->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
//    rc_deadband_limit(system_rc_ctrl->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
//	
//	//通道值设置，缓慢上升
//	vx_set_channel = vx_channel * -CHASSIS_VX_RC_SEN;
//	vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;
}


/**
	* @brief          键盘基础运动设置-不允许修改
  * @param[in]      NULL
  * @retval         NULL
  */
void system_t::KeyBoardQuantitySet()
{
	//鼠标对应YAW,PITCH的灵敏度
	mouse_yaw = LPF(&yaw_lpf ,0.002,system_rc_ctrl->mouse.x * YAW_MOUSE_SEN, 14);
	mouse_pitch = LPF(&pitch_lpf ,0.002,system_rc_ctrl->mouse.y * PITCH_MOUSE_SEN, 30);
	
	//按键Q、E进行YAW轴灵敏度
	if(IF_KEY_PRESSED_Q)
		key_yaw = 100;
	else if(IF_KEY_PRESSED_E)
		key_yaw = -100;
	else
    key_yaw = 0;
	
	/***基础运动设置前后左右运动W、S、A、D***/
	if(IF_KEY_PRESSED_W)
		vx_set_channel = -NORMAL_MAX_CHASSIS_SPEED_X;
	else if(IF_KEY_PRESSED_S)
		vx_set_channel = NORMAL_MAX_CHASSIS_SPEED_X;
	else
		vx_set_channel = 0;

	if(IF_KEY_PRESSED_D)
		vy_set_channel = NORMAL_MAX_CHASSIS_SPEED_Y;
	else if(IF_KEY_PRESSED_A)
		vy_set_channel = -NORMAL_MAX_CHASSIS_SPEED_Y;
	else
		vy_set_channel = 0;


	
}

/**
	* @brief          其余按键设置-不同车可依据操作手需要更改
  * @param[in]      NULL
  * @retval         NULL
  */
void system_t::KeyBoardModeSet()
{
	static uint16_t last_keyboard = 0;

	/*****所有按键操作设置都在此处定义（额外多加也需在此处*****/
	
	/***自瞄模式设置（鼠标右键）***/
	if(IF_MOUSE_PRESSED_RIGH)
		sys_pub.key_flag.auto_aim_flag = 1;
	else
		sys_pub.key_flag.auto_aim_flag = 0;
	


	/***发射模式设置（鼠标左键射击、F键切换射击模式）***/



	if(key_shoot_mode_set == 1 && (IF_MOUSE_PRESSED_LEFT && last_mouse_l_press==0)) //单击鼠标左键：单发射击
	{
		key_shoot_flag = 1;  

	}else if(key_shoot_mode_set == 2 && (IF_MOUSE_PRESSED_LEFT && last_mouse_l_press==0)) //单击鼠标左键：三发射击
	{
		key_shoot_flag = 2;
	}else if(key_shoot_mode_set == 3 && IF_MOUSE_PRESSED_LEFT) //长按鼠标左键：连续射击
	{
		if(press_shoot_time++ >500)  //按键时间大于阈值进行连发防止误触
			key_shoot_flag = 3;
	}else
	{
		key_shoot_flag = 0;
	}

		/***底盘模式设置（ shift小陀螺  v侧身  ctrl超级电容 ）***/
		if(IF_KEY_PRESSED_SHIFT && !last_key_shift_press) 	sys_pub.rc_spin_flag=1;
		else	                                              sys_pub.rc_spin_flag=0;

		

		if(IF_KEY_PRESSED_V && !last_key_v_press) sys_pub.rc_sideway_flag=1;
		else 																			sys_pub.rc_sideway_flag=0;

		if(IF_KEY_PRESSED_F && !last_key_f_press) 
		{
				if(key_shoot_mode_set++ >=3) key_shoot_mode_set=1;
		}		
	

		if(IF_KEY_PRESSED_C) sys_pub.rc_super_flag=1;
		else 								 sys_pub.rc_super_flag=0;


/***调整摩擦轮转速***/
	if((system_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Z) && !(last_keyboard & KEY_PRESSED_OFFSET_Z))
	{
		if(IF_KEY_PRESSED_CTRL)
			sys_pub.offset_firc_v-=100;
	}

	if((system_rc_ctrl->key.v & KEY_PRESSED_OFFSET_X) && !(last_keyboard & KEY_PRESSED_OFFSET_X))
	{
		if(IF_KEY_PRESSED_CTRL)
			sys_pub.offset_firc_v+=100;
	}

		

	//上次按键值，用于模式判断
	last_mouse_l_press = IF_MOUSE_PRESSED_LEFT;
	last_key_g_press = IF_KEY_PRESSED_G;
	last_key_shift_press = IF_KEY_PRESSED_SHIFT;
	last_key_v_press = IF_KEY_PRESSED_V;
	last_key_f_press = IF_KEY_PRESSED_F;
	last_keyboard = system_rc_ctrl->key.v;




	/************************************弹舱盖舵机控制*************************************/
static uint32_t rc_keyZ_time=0;    							//键盘Z的按键时间

			//长按Z在弹舱盖开关间切换		
if ( IF_KEY_PRESSED_Z||(ChassisPointer()->rc_ctrl_down && RC_SIDEWAY<-300))
			{				
				if(rc_keyZ_time<100)//弹舱盖开启时间为500周期
				 {
					 rc_keyZ_time++;
					 
					 if(rc_keyZ_time==100 && steering_mode==0)
					 {
						 steering_mode=1;  //打开
					 }
					 else if(rc_keyZ_time==100 && steering_mode==1)
					 {
						 steering_mode=0;  //关闭
					 }
					 
				 }
			}
			else
			{
				rc_keyZ_time=0;
			}
			
			if(steering_mode==1)
			{
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 800 );			
			}				
			else if(steering_mode==0)
			{ 
				 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1850);
			}
			sys_pub.key_flag.ster_flag=steering_mode;


	//重启操作
	if(RC_STAND_DOWN&&RC_ZERO_FORCE)
	{
			C_RESET();
	}

}


/**
  * @brief          一键C板重启(上下板同时)
  * @param[in]      none
  * @retval         none
  */
int rc_keyB_time;
int rc_keyR_time;
int rc_reset_flag_gimbal;// 上内八
int rc_reset_flag_chassis;//下内八

static void C_RESET(void)
{

rc_reset_flag_gimbal=(RC_FORWARD>=650&&RC_SIDEWAY<=-650&&RC_YAW>=650&&RC_PITCH>=650);
rc_reset_flag_chassis=(RC_FORWARD<=-650&&RC_SIDEWAY<=-650&&RC_YAW>=650&&RC_PITCH<=-650);



if ( IF_KEY_PRESSED_B||rc_reset_flag_gimbal)//
		{				
				if(rc_keyB_time<500)//模式转换时间
				 {
					 rc_keyB_time++;
					 
					 if(rc_keyB_time==500)
					 {
								HAL_NVIC_SystemReset();
								rc_keyB_time=0;
					 }
					}
		}		
else
		{
				rc_keyB_time=0;
		}




//下板
if ( rc_reset_flag_chassis)//
		{				
				if(rc_keyR_time<500)//模式转换时间
				 {
					 rc_keyR_time++;
					 
					 if(rc_keyR_time==500)
					 {
								sys.sys_pub.rc_restart=1;
								rc_keyR_time=0;
					 }
					}
		}		
else
		{
				rc_keyR_time=0;
				sys.sys_pub.rc_restart=0;
				return;
		}
}



