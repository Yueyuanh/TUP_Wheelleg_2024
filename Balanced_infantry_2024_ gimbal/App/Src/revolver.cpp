/**
 ******************************************************************************
 * @file    revolver.cpp
 * @author  Xushuang
 * @version V1.0.0 基本完成
 * @date    2023/9/22
 * @brief		此处为拨盘各模式控制
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "arm_math.h"
#include "revolver.h"
#include "referee_data.h"

//发射机构实例
Revolver_t revolver;

/********电机初始化设置*********/
Motor_Setting_t firc_l_config = {"fir_l",ON_CAN2,1,M3508,POSITIVE_DIRECT,DIRECT_DRIVE,SINGLE_LOOP};
Motor_Setting_t firc_r_config = {"fir_r",ON_CAN1,2,M3508,POSITIVE_DIRECT,DIRECT_DRIVE,SINGLE_LOOP};
Motor_Setting_t stir_config = {"stir",ON_CAN1,3,M2006,POSITIVE_DIRECT,RATIO_1_TO_36,CASCADE_LOOP};

/**
  * @brief          发射机构类构造函数
  * @param[in]      NULL
  * @retval         NULL
  */
Revolver_t::Revolver_t()
{
	/****设定初始速度值——为后续方便调试，仿真时修改此变量即可****/
	vfic_set = FIRST_STAGE_RPM_SET;
	/*****连发射频设置*****/
	revolver_msg.shoot_rate = INIT_FREQUENCE;
}

/**
  * @brief          发射机构初始化
  * @param[in]      NULL
  * @retval         NULL
  */
void RevolverInit()
{
	/***************摩擦轮初始化***************/
	//电机初始化
	revolver.firc_l[FIRST_STAGE].DJIMotorInit(&firc_l_config);  
	revolver.firc_r[FIRST_STAGE].DJIMotorInit(&firc_r_config);
	
#if FIRC_STAGE == SECONE_STAGE+1  //第二级摩擦轮设置
	revolver.firc_l[SECONE_STAGE].DJIMotorInit(&firc_l_config);
	revolver.firc_r[SECONE_STAGE].DJIMotorInit(&firc_r_config);
#endif
	//发射机构速度环PID控制器初始化
    const fp32 Firc_speed_pid[3] = {FIRC_SPEED_PID_KP, FIRC_SPEED_PID_KI, FIRC_SPEED_PID_KD};
	for(uint8_t i = 0;i<FIRC_STAGE;i++)
	{
		revolver.firc_l[i].controller.speed_PID.Init(PID_POSITION,Firc_speed_pid,NULL,FIRC_SPEED_PID_MAX_IOUT);
		revolver.firc_r[i].controller.speed_PID.Init(PID_POSITION,Firc_speed_pid,NULL,FIRC_SPEED_PID_MAX_IOUT);
	}
	
	/***************拨盘电机初始化***************/
	revolver.fire_time=0;
	revolver.fire_Hz=20;

	revolver.stir_motor_gun[MAIN_REVOLVER].DJIMotorInit(&stir_config);

#if REVOLVER_NUM != 1  //第二个拨盘的初始化
	revolver.stir_motor_gun[MAIN_REVOLVER].DJIMotorInit(&stir_config);  //需要改参数
#endif
	
	//拨盘PID控制器
	const fp32 Stir_speed_pid[3] = {STIR_SPEED_PID_KP, STIR_SPEED_PID_KI, STIR_SPEED_PID_KD};
	const fp32 Stir_position_pid[3] = {STIR_POSITION_PID_KP, STIR_POSITION_PID_KI, STIR_POSITION_PID_KD};
	
	//主要拨盘控制器参数初始化（如有辅助拨盘）
	for(uint8_t u = 1;u<=REVOLVER_NUM;u++)
	{
		revolver.stir_motor_gun[u-1].controller.angle_PID.Init(PID_POSITION, Stir_position_pid,STIR_POSITION_PID_MAX_OUT, STIR_POSITION_PID_MAX_IOUT);
		revolver.stir_motor_gun[u-1].controller.speed_PID.Init(PID_POSITION, Stir_speed_pid,NULL, STIR_SPEED_PID_MAX_IOUT);
 	}
	/******信息中心实例建立******/
	CenterPointer()->PointerInit(&revolver.revolver_msg,REVOLVERPUB);
}

/**
  * @brief          发射机构主任务
  * @param[in]      NULL
  * @retval         NULL
  */
void RevolverTask()
{
	revolver.InfoUpdate();  //信息更新
	revolver.ControlSet();  //控制设置
}

/**
  * @brief          信息更新
  * @param[in]      NULL
  * @retval         NULL
  */
void Revolver_t::InfoUpdate()
{
	/*****裁判系统信息更新*****/
	#if ROBOT_SHOOT_TYPE == BULLET_17
	//剩余热量=上限-当前热量（可能id是枪口2，如果没数据要改成id2）
	revolver_msg.rest_heat = JUDGE_usGetHeatLimit_id1_17mm() - JUDGE_usGetRemoteHeat_id1_17mm(); 
	
	#elif ROBOT_SHOOT_TYPE == BULLET_42
	
	//剩余热量=上限-当前热量
	revolver_msg.rest_heat = JUDGE_usGetHeatLimit_id1_42mm() - JUDGE_usGetRemoteHeat_id1_42mm(); 
	
	#endif
	
	/*****拨盘信息更新*****/
	//拨盘电机速度更新（滤波一下）
    fp32 revolver_fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
	stir_speed = stir_motor_gun->FilterSpeed(&revolver_fliter_num[0]);
	
	//拨盘电机角度更新（调用API）
	stir_angle = stir_motor_gun[MAIN_REVOLVER].GetOutputShaftRad();

	//判断是否达到误差角内（需改0）
	if(fabs(stir_angle_set - stir_angle)<AN_BULLET*0.1f)
		revolver_msg.shoot_finish = 1;
	else
		revolver_msg.shoot_finish = 0;
	
	/*****拨盘反转逻辑判断*****/
		//new
	if(stir_motor_gun[MAIN_REVOLVER].GetGivenCurrent() >= 9000 ||
		 stir_motor_gun[MAIN_REVOLVER].GetGivenCurrent() <= -9000)
	{
		if(unfinish_time++ >100)
		{
			revolver_msg.if_stuck=1;
		}
	}else
	{
		unfinish_time = 0;
	}
	/*****单发连发模式切换时信息更新*****/
	if(last_stir_mode != SHOOT_FIXED_FIRE_RATE && SysPointer()->stir_mode == SHOOT_FIXED_FIRE_RATE)
	{
		//此时改为速度单环//改成角度环
		stir_motor_gun[MAIN_REVOLVER].ChangeControlMode(CASCADE_LOOP);
		//控制器参数重置——仅需一次即可（若角度闭环时的速度环效果较好可以直接用，即省略控制器参数重置）
		
	}else if(last_stir_mode == SHOOT_FIXED_FIRE_RATE && SysPointer()->stir_mode != SHOOT_FIXED_FIRE_RATE)
	{
		stir_motor_gun[MAIN_REVOLVER].ChangeControlMode(CASCADE_LOOP);
		//stir_angle_set = stir_angle;
	}
	last_stir_mode = SysPointer()->stir_mode;  //上次模式读取

	//UI
	revolver_msg.firc_speed_set=(int16_t)FIRST_STAGE_RPM_SET+SysPointer()->offset_firc_v;


}

/**
  * @brief          控制设置
  * @param[in]      NULL
  * @retval         NULL
  */
void Revolver_t::ControlSet()
{
	//摩擦轮设置
	FircControl();
	//拨盘设置-只有摩擦轮开启的情况下才允许拨盘转动
	StirControl();
}

/**
	* @brief          摩擦轮速度设定
  * @param[in]      NULL
  * @retval         NULL
  */
void Revolver_t::FircSpeedSet()
{
#if ROBOT_SHOOT_TYPE == BULLET_42
	//一级摩擦轮转速设置
	l_firc_speed_set[FIRST_STAGE] = vfic_set;
	r_firc_speed_set[FIRST_STAGE] = vfic_set;
	//一级摩擦轮斜坡函数
	l_firc_ramp_set[FIRST_STAGE] = RAMP_float(l_firc_speed_set[FIRST_STAGE],l_firc_ramp_set[FIRST_STAGE],10);
	r_firc_ramp_set[FIRST_STAGE] = RAMP_float(r_firc_speed_set[FIRST_STAGE],r_firc_ramp_set[FIRST_STAGE],10);
#elif ROBOT_SHOOT_TYPE == BULLET_17

	//一级摩擦轮转速设置
	vfic_set=FIRST_STAGE_RPM_SET+SysPointer()->offset_firc_v;


	l_firc_speed_set[FIRST_STAGE] =  vfic_set;
	r_firc_speed_set[FIRST_STAGE] = -vfic_set;

	l_firc_ramp_set[FIRST_STAGE] = RAMP_float(l_firc_speed_set[FIRST_STAGE],l_firc_ramp_set[FIRST_STAGE],10);
	r_firc_ramp_set[FIRST_STAGE] = RAMP_float(r_firc_speed_set[FIRST_STAGE],r_firc_ramp_set[FIRST_STAGE],10);
#endif
}

/**
	* @brief          不同转速选择（新赛季用不了哩呜呜呜）
  * @param[in]      NULL
  * @retval         NULL
  */
void Revolver_t::DifferentFirSpeedSet(FircSpeed_type_t speed_type)
{
	switch(speed_type)
	{
		case LOW_SPEED:
			l_firc_speed_set[FIRST_STAGE] = LOW_RPM_SET;
			r_firc_speed_set[FIRST_STAGE] = -LOW_RPM_SET;
			break;
		case MID_SPEED:
			l_firc_speed_set[FIRST_STAGE] = MID_RPM_SET;
			r_firc_speed_set[FIRST_STAGE] = -MID_RPM_SET;
			break;
		case HIGH_SPEED:
			l_firc_speed_set[FIRST_STAGE] = HIGH_RPM_SET;
			r_firc_speed_set[FIRST_STAGE] = -HIGH_RPM_SET;
			break;
	}
}

/**
	* @brief          摩擦轮控制
  * @param[in]      NULL
  * @retval         NULL
  */
void Revolver_t::FircControl()
{
	switch(SysPointer()->fir_mode)
	{
		case FORBID_FIRE:
		case CLOSE: 
			//关闭摩擦轮
			for(uint8_t i = 0;i<FIRC_STAGE;i++)
			{
				l_firc_ramp_set[i] = 0;
				r_firc_ramp_set[i] = 0;
				firc_l[i].MotorZeroForce();
				firc_r[i].MotorZeroForce();
			}
			break;
		case OPEN:
			//开启摩擦轮设置
			FircSpeedSet();
			break;
	}
	//多级摩擦轮控制
	for(uint8_t i = 0;i<FIRC_STAGE;i++)
	{
		firc_l[i].DJIMotorControl(NULL,&l_firc_ramp_set[i],NULL,OFF_SET);
		firc_r[i].DJIMotorControl(NULL,&r_firc_ramp_set[i],NULL,OFF_SET);
	}
		//UI传递
		SysPointer()->key_flag.fir_flag=SysPointer()->fir_mode;



}

/**
  * @brief          拨盘控制
  * @param[in]      NULL
  * @retval         NULL
  */
void Revolver_t::StirControl()
{
	/*****此处仅根据各种模式进行对应控制量设置*****/
	switch(SysPointer()->stir_mode)
	{
		//无力模式
		case NO_MOVE:
			//无力时控制器不起作用，可以随意动拨盘
			stir_angle_set = stir_angle;
			//没开摩擦轮不允许控制-按键保险发电流值为零
			stir_motor_gun[MAIN_REVOLVER].MotorZeroForce(); 

			break;
		
		//拨盘反转模式
		case REVERSE_MOVE:
			//加入拨盘反转判断逻辑即（if~）//反转逻辑待加入
			
			stir_angle_set += AN_BULLET;  //小弹丸需修改AN_BULLET
			break;
		
		//单发模式
		case SHOOT_1_BULLET:
			if(SysPointer()->fir_mode!= OPEN)
				return;

			stir_angle_set -= AN_BULLET;  //小弹丸需修改AN_BULLET
			SysPointer()->stir_mode = ALLOW_MOVE;
			break;
			
		//三连发模式
		case SHOOT_3_BULLET:
			if(SysPointer()->fir_mode!= OPEN)
				return;

			stir_angle_set -= 3*AN_BULLET;  //小弹丸需修改AN_BULLET
			SysPointer()->stir_mode = ALLOW_MOVE;
			break;
			
		//固定射频发射模式
		case SHOOT_FIXED_FIRE_RATE:

			fire_time++;
			//运行频率1000Hz 
			if((fire_time%(1000/fire_Hz))==1  &&  (fire_time_last%(1000/fire_Hz))==0)
			{
					stir_angle_set -= AN_BULLET;
			}
			
			//stir_angle_set -= 0;
			stir_motor_gun[MAIN_REVOLVER].DJIMotorControl(&stir_angle,&stir_angle_set,NULL,ON_SET);

			fire_time_last=fire_time;
			//射频参数赋值（设定射频（颗/s） * 拨盘一整圈角度（rad） / 拨轮数（颗） = rad/s）
			//stir_speed_set = -revolver_msg.shoot_rate * 2.0f * PI / WHOLE_CIRCLE_BULLET_NUM; 
			//对输出轴进行闭环控制（控速）
			//stir_motor_gun[MAIN_REVOLVER].DJIMotorControl(&stir_speed,&stir_speed_set,NULL,ON_SET);
			break;
		
		//控制器起作用
		case ALLOW_MOVE:
			#if USE_SENSOR == OFF_SET

						//以输出轴为准
				stir_motor_gun[MAIN_REVOLVER].DJIMotorControl(&stir_angle,&stir_angle_set,NULL,ON_SET);	
		
				fire_time=0;

				//连发清零
				

			#else
				//单速度环旋转到传感器检测到变化值
				stir_motor_gun[MAIN_REVOLVER].DJIMotorControl(&stir_angle,&stir_angle_set,NULL,ON_SET);
			#endif
			break;
	}
	
}

