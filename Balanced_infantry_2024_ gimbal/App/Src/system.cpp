/**
 ******************************************************************************
 * @file    sys_task.cpp
 * @author  Xushuang
 * @version V1.0.0 Xushuang �������
 *					V1.1.0 Xushuang �Ż���װ
 *					V2.0.0 Xushuang �Ż�ģʽ����
 * @date    2024/1/11
 * @brief		�˴�Ϊģʽ���ú͸���������
 ******************************************************************************
 * @attention
 *ʵ�ʱ����б�ҪΪ���µ���дһ��ģʽ-----��׼���ֹYAW���ƶ�
 *����ʱ
 *������Ӣ��ͳһ�涨ң������Ӧģʽ��
 *		���	�ҡ�������	Ħ���ֲ���
 *		���	��-���������ƣ����̸�����̨��/���ص���
 *		��� �ҡ���С���� Ħ���ֲ���
 *		��-	��-������Ħ����
 *		��� ��-��������ת����
 *		��-����L �ҨJ���������飨����ģʽ��
 *		���֣��������������飨�ɿ����˳����飩
 *					�򸺷�������������
 ******************************************************************************
 */
#include "system.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "tim.h"

//ϵͳʵ��
system_t sys;

/**
	* @brief          ϵͳ�๹�캯��
  * @param[in]      NULL
  * @retval         NULL
  */
system_t::system_t()
{
	/*****��ʼģʽ����*****/
	sys_pub.mode = ZERO_FORCE;
	sys_pub.fir_mode = FORBID_FIRE;
	sys_pub.stir_mode = NO_MOVE;
	/*****��ʼ����ģʽ����*****/
	key_shoot_mode_set = 1;
	debug_shoot_mode_set = 1;
	/*****��ʼ����������*****/
	vx_ramp_set = vy_ramp_set = vx_set_channel = vy_set_channel =0.0f;
	mouse_yaw = mouse_pitch = key_yaw = 0.0f;
	yaw_channel = pitch_channel = 0.0f;

	sys_pub.offset_firc_v=0;

}

/**
	* @brief          ϵͳ��ʼ��
  * @param[in]      NULL
  * @retval         NULL
  */
void SysInit()
{
	/******ң����ָ���ȡ******/
	sys.system_rc_ctrl = GetRemoteControlPoint();
	/******��Ϣ����ʵ������******/
	CenterPointer()->PointerInit(&sys.sys_pub,SYSPUB);
}

/**
	* @brief          ϵͳ������
  * @param[in]      NULL
  * @retval         NULL
  */
void SystemTask()
{
	/*****ģʽ����*****/
	sys.KeyBoardModeSet(); //��������
	sys.ThumbWheelModeSet(); 	 //��������
	sys.RobotModeSet();  //������ģʽ����
	/*****����������*****/
	sys.CalControlQuantity();
}

/**
	* @brief          ������ģʽ����
  * @param[in]      NULL
  * @retval         NULL
  */
void system_t::RobotModeSet()
{
	/*****ң����ʧ��ģʽ�ж�*****/
	if(!MonitorRc())
	{
		//ʧ��ģʽ���б���
		sys_pub.mode = DT7_MISSING;
		sys_pub.fir_mode = CLOSE;
		sys_pub.stir_mode = NO_MOVE;
		return;
	}
	
	/*****����ģʽ����*****/
	if(sys_pub.mode == INIT)
	{
		//��ֹ����ͻȻ������µ����
		if(switch_is_down(system_rc_ctrl->rc.s[RIGTH_CHANNEL]))
			sys_pub.mode = ZERO_FORCE;
		return;
		
	}else
	{
		/**********�˶�ģʽ����**********/
		if(switch_is_down(system_rc_ctrl->rc.s[RIGTH_CHANNEL]) )
		{  
			//����ģʽ
			sys_pub.mode = ZERO_FORCE;			
		}else if(switch_is_mid(system_rc_ctrl->rc.s[RIGTH_CHANNEL]))
		{ 
			//��������ģʽ-����ͬʱ���Կ���
			sys_pub.mode = NORMAL;
			//��������ģʽ
			if(sys_pub.key_flag.auto_aim_flag == 1)
				sys_pub.mode = AUTO;
		}else if(switch_is_up(system_rc_ctrl->rc.s[RIGTH_CHANNEL]))
		{  
			//С����ģʽ
			sys_pub.mode = SPIN;
			//����С��������ģʽ
			if(sys_pub.key_flag.auto_aim_flag == 1)
				sys_pub.mode = SPIN_AUTO;
		}
		
		/**********��ʼ��ģʽ�ж�**********/
		if(last_mode == ZERO_FORCE && sys_pub.mode != ZERO_FORCE )
			sys_pub.mode = INIT;

		/**********ģʽ�л��ж�**********/
		if(last_mode != sys_pub.mode)
			sys_pub.change_mode_flag = 1;
		//�����ϴ�ģʽ�������ж�ģʽ�л�
		last_mode = sys_pub.mode; 
		




		/**********���ģʽ����**********/
		//�Ҳ������ã��м�����Ϸ������������ã��·�ʱ��ֹ���䣩
		if(!switch_is_down(system_rc_ctrl->rc.s[RIGTH_CHANNEL]))
		{
			//�󲦸�����
			if(switch_is_down(system_rc_ctrl->rc.s[LEFT_CHANNEL]))
			{
					//Ħ���ֹرգ����̽�ֹ�˶�
					sys_pub.fir_mode = CLOSE;
					sys_pub.stir_mode = NO_MOVE;

			}else
			{


				//Ħ���ֿ�������������ת������Ĭ��Ϊ�Ƕȱջ�ģʽ��
				sys_pub.fir_mode = OPEN;
				sys_pub.stir_mode = ALLOW_MOVE;

				
				if(RevolverPointer()->rest_heat<=45) 
				{
						sys_pub.stir_mode = ALLOW_MOVE; 
						return;
				}

				//���̲���ģʽ���ã�0��1��2��3����Ϊ����������������ģʽ��
				if(key_shoot_flag == 1 || debug_shoot_flag == 1)
				{
					 sys_pub.stir_mode = SHOOT_1_BULLET;

				}else if(key_shoot_flag == 2 || debug_shoot_flag == 2)
				{
					 sys_pub.stir_mode = SHOOT_3_BULLET;

				}else if(key_shoot_flag == 3 || debug_shoot_flag == 3)
				{
					//�����ٶȻ�ģʽ---����������
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
			//Ħ���ֹرգ����̽�ֹ�˶�
			sys_pub.fir_mode = CLOSE;
			sys_pub.stir_mode = NO_MOVE;
		}
	}
}

/**
	* @brief          ����ģʽ����
  * @param[in]      NULL
  * @retval         NULL
  */
void system_t::ThumbWheelModeSet()
{
	/*****�°�߲��ֵ�״̬ʹ��*****/
	//���ڲ���ģʽ����
	
	stand_state=RC_S_LEFT;
	
	if(sys_pub.stir_mode == ALLOW_MOVE) //��������Ϊ����ģʽ�²ſɽ���ģʽ����
	{

		if(RC_STAND_UP)
			stand_up_state_time++;//�ϲ�״̬��ʱ
	}

	if(stand_state == 1 && stand_state_last == 3 && debug_shoot_mode_set)
	{
			debug_shoot_flag=1;//����
	}
	else if(stand_up_state_time>500 && debug_shoot_mode_set)
	{
			debug_shoot_flag=3;//����
	}
	else
	{
			debug_shoot_flag=0;
	}

	if(RC_STAND)
	{
			debug_shoot_flag=0;//����
			stand_up_state_time=0;
	}

	stand_up_state_time_last=stand_up_state_time;//����
	stand_state_last=stand_state;



}

/**
	* @brief          ��������
  * @param[in]      NULL
  * @retval         NULL
  */
void system_t::CalControlQuantity()
{
	//ң��������������
	RemoteQuantitySet();
	//��������������
	KeyBoardQuantitySet();
	
	//��������Ϣ��������
	sys_pub.add_yaw = (yaw_channel * YAW_RC_SEN - 
												 mouse_yaw +
												 key_yaw*0.00003);
	
	sys_pub.add_pit = (pitch_channel * PITCH_RC_SEN - 
												 mouse_pitch 
												);
	
	vx_ramp_set = RAMP_float(vx_set_channel,vx_ramp_set,20);
	vy_ramp_set = RAMP_float(vy_set_channel,vy_ramp_set,20);
	
	//ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
	if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
		vx_ramp_set = 0.0f;

	if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
		vy_ramp_set = 0.0f;
	
	//б�¸�ֵ
	sys_pub.vx_set = vx_ramp_set;
	sys_pub.vy_set = vy_ramp_set;
}

/**
	* @brief          ң��������������
  * @param[in]      NULL
  * @retval         NULL
  */
void system_t::RemoteQuantitySet()
{
	int16_t vx_channel, vy_channel;
	
	//��ң���������ݴ������� int16_t yaw_channel,pitch_channel
	rc_deadband_limit(system_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
  rc_deadband_limit(system_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
	
//	//ң�����������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
//    rc_deadband_limit(system_rc_ctrl->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
//    rc_deadband_limit(system_rc_ctrl->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
//	
//	//ͨ��ֵ���ã���������
//	vx_set_channel = vx_channel * -CHASSIS_VX_RC_SEN;
//	vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;
}


/**
	* @brief          ���̻����˶�����-�������޸�
  * @param[in]      NULL
  * @retval         NULL
  */
void system_t::KeyBoardQuantitySet()
{
	//����ӦYAW,PITCH��������
	mouse_yaw = LPF(&yaw_lpf ,0.002,system_rc_ctrl->mouse.x * YAW_MOUSE_SEN, 14);
	mouse_pitch = LPF(&pitch_lpf ,0.002,system_rc_ctrl->mouse.y * PITCH_MOUSE_SEN, 30);
	
	//����Q��E����YAW��������
	if(IF_KEY_PRESSED_Q)
		key_yaw = 100;
	else if(IF_KEY_PRESSED_E)
		key_yaw = -100;
	else
    key_yaw = 0;
	
	/***�����˶�����ǰ�������˶�W��S��A��D***/
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
	* @brief          ���ఴ������-��ͬ�������ݲ�������Ҫ����
  * @param[in]      NULL
  * @retval         NULL
  */
void system_t::KeyBoardModeSet()
{
	static uint16_t last_keyboard = 0;

	/*****���а����������ö��ڴ˴����壨������Ҳ���ڴ˴�*****/
	
	/***����ģʽ���ã�����Ҽ���***/
	if(IF_MOUSE_PRESSED_RIGH)
		sys_pub.key_flag.auto_aim_flag = 1;
	else
		sys_pub.key_flag.auto_aim_flag = 0;
	


	/***����ģʽ���ã������������F���л����ģʽ��***/



	if(key_shoot_mode_set == 1 && (IF_MOUSE_PRESSED_LEFT && last_mouse_l_press==0)) //�������������������
	{
		key_shoot_flag = 1;  

	}else if(key_shoot_mode_set == 2 && (IF_MOUSE_PRESSED_LEFT && last_mouse_l_press==0)) //�������������������
	{
		key_shoot_flag = 2;
	}else if(key_shoot_mode_set == 3 && IF_MOUSE_PRESSED_LEFT) //�������������������
	{
		if(press_shoot_time++ >500)  //����ʱ�������ֵ����������ֹ��
			key_shoot_flag = 3;
	}else
	{
		key_shoot_flag = 0;
	}

		/***����ģʽ���ã� shiftС����  v����  ctrl�������� ��***/
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


/***����Ħ����ת��***/
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

		

	//�ϴΰ���ֵ������ģʽ�ж�
	last_mouse_l_press = IF_MOUSE_PRESSED_LEFT;
	last_key_g_press = IF_KEY_PRESSED_G;
	last_key_shift_press = IF_KEY_PRESSED_SHIFT;
	last_key_v_press = IF_KEY_PRESSED_V;
	last_key_f_press = IF_KEY_PRESSED_F;
	last_keyboard = system_rc_ctrl->key.v;




	/************************************���ոǶ������*************************************/
static uint32_t rc_keyZ_time=0;    							//����Z�İ���ʱ��

			//����Z�ڵ��ոǿ��ؼ��л�		
if ( IF_KEY_PRESSED_Z||(ChassisPointer()->rc_ctrl_down && RC_SIDEWAY<-300))
			{				
				if(rc_keyZ_time<100)//���ոǿ���ʱ��Ϊ500����
				 {
					 rc_keyZ_time++;
					 
					 if(rc_keyZ_time==100 && steering_mode==0)
					 {
						 steering_mode=1;  //��
					 }
					 else if(rc_keyZ_time==100 && steering_mode==1)
					 {
						 steering_mode=0;  //�ر�
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


	//��������
	if(RC_STAND_DOWN&&RC_ZERO_FORCE)
	{
			C_RESET();
	}

}


/**
  * @brief          һ��C������(���°�ͬʱ)
  * @param[in]      none
  * @retval         none
  */
int rc_keyB_time;
int rc_keyR_time;
int rc_reset_flag_gimbal;// ���ڰ�
int rc_reset_flag_chassis;//���ڰ�

static void C_RESET(void)
{

rc_reset_flag_gimbal=(RC_FORWARD>=650&&RC_SIDEWAY<=-650&&RC_YAW>=650&&RC_PITCH>=650);
rc_reset_flag_chassis=(RC_FORWARD<=-650&&RC_SIDEWAY<=-650&&RC_YAW>=650&&RC_PITCH<=-650);



if ( IF_KEY_PRESSED_B||rc_reset_flag_gimbal)//
		{				
				if(rc_keyB_time<500)//ģʽת��ʱ��
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




//�°�
if ( rc_reset_flag_chassis)//
		{				
				if(rc_keyR_time<500)//ģʽת��ʱ��
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



