/**
 ******************************************************************************
 * @file    gimbal.cpp
 * @author  Xushuang
 * @version V1.0.0 基本完成
 * @date    2023/9/20
 * @brief		此处为云台各模式控制
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "arm_math.h"
#include "gimbal.h"
#include "stm32f4xx_hal.h"
#include "bsp_dwt.h"

//车的后方为X轴正方向，右方为Y轴正方向，上方为Z轴正方向

//创建实例
gimbal_t gimbal;

/********电机初始化设置*********/
Motor_Setting_t yaw_setting = {"yaw",ON_CAN1,1,GM6020,POSITIVE_DIRECT,DIRECT_DRIVE,LADRC_FDW_CONTROL};
Motor_Setting_t pit_setting = {"pit",ON_CAN1,2,GM6020,POSITIVE_DIRECT,DIRECT_DRIVE,LADRC_FDW_CONTROL};

/********电机控制器参数设置*********/
//WC-B0-WO-W-GAIN
fp32 yaw_init_config[]={15,0.01f,100,0,0};
fp32 pit_init_config[]={15,0.01f,100,0,0};

fp32 yaw_normal_config[]={15,0.007f,100,0,0};
fp32 pit_normal_config[]={15,0.007f,100,0,0};

fp32 yaw_auto_config[]={15,0.005f,80,0,0};
fp32 pit_auto_config[]={15,0.007f,80,0,0};

/**
	* @brief          gimbal类构造函数
  * @param[in]      NULL
  * @retval         NULL
  */
gimbal_t::gimbal_t()
{
	yaw_absolute_set_rad = yaw_absolute_rad = 0.0f;
	pit_absolute_set_rad = pit_absolute_rad = 0.0f;

	gimbal_msg.yaw_num=0;

}

/**
	* @brief          云台初始化
  * @param[in]      NULL
  * @retval         NULL
  */
void GimbalInit()
{
	/******电机初始化******/
	gimbal.yaw_motor.DJIMotorInit(&yaw_setting);
	gimbal.pit_motor.DJIMotorInit(&pit_setting);
	/******信息中心实例建立******/
	CenterPointer()->PointerInit(&gimbal.gimbal_msg,GIMBALPUB);
}

/**
	* @brief          云台控制器的初始化
  * @param[in]      type：控制器类型
  * @retval         NULL
  */
void gimbal_t::GimbalControllerInit(uint8_t type)
{
	if(SysPointer()->change_mode_flag)  //根据不同模式更改控制器参数
	{
		SysPointer()->change_mode_flag = 0;
		//各个模式控制器初始化
		if(type == INIT_PARAM)
		{
			yaw_motor.controller.ladrc_fdw.Init(yaw_init_config,NULL);
			pit_motor.controller.ladrc_fdw.Init(pit_init_config,NULL);
		}else if(type == NORMAL_PARAM)
		{
			yaw_absolute_set_rad = yaw_absolute_rad;
			pit_absolute_set_rad = pit_absolute_rad;
			yaw_motor.controller.ladrc_fdw.Init(yaw_normal_config,NULL);
			pit_motor.controller.ladrc_fdw.Init(pit_normal_config,NULL);
		}else if(type == AUTO_PARAM)
		{
			yaw_motor.controller.ladrc_fdw.Init(yaw_auto_config,NULL);
			pit_motor.controller.ladrc_fdw.Init(pit_auto_config,NULL);
		}
	}
}


/**
	* @brief          云台主任务
  * @param[in]      NULL
  * @retval         NULL
  */
void GimbalTask()
{
	//基础信息更新
	gimbal.BasicInfoUpdate();


	//根据不同模式选择不同控制量信息来源
	switch(SysPointer()->mode)
	{
		//遥控器作为输入量（操作模式）
		case NORMAL:
		case SPIN:
			gimbal.GimbalControllerInit(NORMAL_PARAM); 					//控制器初始化
			gimbal.OperationInfoUpdate();      		   					//操作时信息更新
			gimbal.NormalControl();                    					//进入控制器
//			MonitorPointer()->state = SetLEDWorkType(GREEN_SLOW);
			BlinkLEDByCount(0xFFFF0000,500);
			break;


		//USBCDC串口作为输入量（自瞄模式）
		case AUTO:
		case SPIN_AUTO:
			gimbal.GimbalControllerInit(AUTO_PARAM); 			       	//控制器初始化
			gimbal.AutoInfoUpdate(); 								   	//自瞄信息更新
			gimbal.NormalControl(); 									//进入控制器
//			MonitorPointer()->state = SetLEDWorkType(BLUE_SLOW); 		//设置灯颜色为蓝色慢闪
			BlinkLEDByCount(0xFF00FF00,500);
			break;


		//初始化模式
		case INIT:
			gimbal.GimbalControllerInit(INIT_PARAM); 					//控制器初始化
			gimbal.InitInfoUpdate();  									//初始化信息更新
			gimbal.JudgeInitState();									//判断初始化状态
			gimbal.RelativeControl();									//进入控制器
//			MonitorPointer()->state = SetLEDWorkType(BLUE_TWO_BLINK);	//设置灯颜色为蓝色双闪
			BlinkLEDByCount(0xFFFFFF00,500);
			break;


		//无力及丢失遥控器失控模式
		case DT7_MISSING:
//			MonitorPointer()->state = SetLEDWorkType(YELLOW_TWO_BLINK); //设置灯颜色为黄色双闪


		case ZERO_FORCE:
			gimbal.ZeroForceControl();									//电机无力控制
//			MonitorPointer()->state = SetLEDWorkType(CYAN_SLOW);	    //设置灯颜色为青色闪烁
		BlinkLEDByCount(0xFF0000FF,500);
			break;


		//相对角度控制模式（底盘），不对云台进行控制
		case RELATIVE_ANGLE:
		case NO_FOLLOW_YAW: 
			BlinkLEDByCount(0xFF00FF00,500);
			//不进行控制
			break;
	}
}

/**
	* @brief          基础信息更新
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::BasicInfoUpdate()
{
	/**************绝对角度（陀螺仪）信息更新****************/
	//陀螺仪绝对角度转化成弧度制
	yaw_absolute_rad = *(get_INS_angle_point() + INS_YAW_ADDRESS_OFFSET);
	pit_absolute_rad = *(get_INS_angle_point() + INS_PITCH_ADDRESS_OFFSET);
	//陀螺仪角速度
	roll_gyro = *(get_gyro_data_point() + INS_GYRO_X_ADDRESS_OFFSET);
	pitch_gyro = *(get_gyro_data_point() + INS_GYRO_Y_ADDRESS_OFFSET);
	yaw_gyro = arm_cos_f32(gimbal_msg.pit_relative_angle) * (*(get_gyro_data_point() + INS_GYRO_Z_ADDRESS_OFFSET))-
						 arm_sin_f32(gimbal_msg.pit_relative_angle) * (*(get_gyro_data_point() + INS_GYRO_X_ADDRESS_OFFSET));
	
	/**************相对角度（电机）信息更新****************/
	//不是一比一，相对角度不能直接使用电机设定值差--弧度制
	yaw_ecd_angle = yaw_motor.CalcTotalecd();
	//电机相对角度
	gimbal_msg.yaw_relative_angle = yaw_motor.MotorEcdToAngle(&yaw_ecd_angle,YAW_OFFSET,MOTOR_TO_YAW_RADIO);
	gimbal_msg.pit_relative_angle = -1.0f * PIT_TRANSMISSION_RADIO * pit_motor.MotorEcdToAngle(NULL,PIT_OFFSET,MOTOR_TO_PIT_RADIO);
	
	if(gimbal_msg.yaw_relative_angle-gimbal_msg.yaw_relative_angle_last>3.1415f)
	{
			gimbal_msg.yaw_num--;
	}
	else if(gimbal_msg.yaw_relative_angle-gimbal_msg.yaw_relative_angle_last<-3.1415f)
	{
			gimbal_msg.yaw_num++;
	}

//	gimbal_msg.yaw_num=yaw_motor.motor_measure.difference_num;
	gimbal_msg.yaw_relative_angle_sum=gimbal_msg.yaw_relative_angle+gimbal_msg.yaw_num*2*3.1415926f;

	gimbal_msg.yaw_relative_angle_last=gimbal_msg.yaw_relative_angle;
}

/**
	* @brief          操作时信息更新
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::OperationInfoUpdate()
{
//	if(SysPointer()->mode == NORMAL)     //底盘跟随云台模式
//	{
//		//add值更新并限幅
//		add_yaw = AbsoluteControlAddLimit((yaw_absolute_set_rad-yaw_absolute_rad),SysPointer()->add_yaw,gimbal_msg.yaw_relative_angle,MAX_YAW_RELATIVE,MIN_YAW_RELATIVE);
//	}else if(SysPointer()->mode == SPIN) //小陀螺模式
//	{
//		//add值更新，不进行限幅
//		add_yaw = SysPointer()->add_yaw;
//	}
	
		add_yaw = SysPointer()->add_yaw;



	//add值更新并限幅
	add_pit=AbsoluteControlAddLimit((pit_absolute_set_rad-pit_absolute_rad),SysPointer()->add_pit,gimbal_msg.pit_relative_angle,MAX_PIT_RELATIVE,MIN_PIT_RELATIVE);
	
	//弧度制赋值并限制
	yaw_absolute_set_rad = rad_format(yaw_absolute_set_rad + add_yaw);
	pit_absolute_set_rad = rad_format(pit_absolute_set_rad + add_pit);
}

/**
	* @brief          自瞄信息更新
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::AutoInfoUpdate()
{
	fp32 auto_error_yaw,auto_error_pit;
	/*****获取相对角度值*****/
	VisionErrorAngleYaw(&auto_error_yaw);
	VisionErrorAnglePit(&auto_error_pit);
	
	/*****数据处理方式*****/
#if Auto_Type == HANDLE_LPF
	//瞄准到目标进行控制
	if( VisionGetIfTarget() )
	{
		//一节低通滤波
		add_yaw = LPF(&yaw_vision_lpf ,0.001,auto_error_yaw,550); //800 812 875 750 843 781 769 687
		add_pit = LPF(&pitch_vision_lpf ,0.001,auto_error_pit,1300);
		
		//数据异常处理
		if(isnan(add_yaw) || isinf(add_yaw))
		{
			add_yaw = 0.0;
		}
		if(isnan(add_pit) || isinf(add_pit))
		{ 
			add_pit = 0.0;
		}
	}else
	{
		add_yaw = add_pit = 0.0f;
	}
#elif Auto_Type == HANDLE_KALMAN
	
#endif
	
	/*****处理数据后进行限幅处理*****/
	if(SysPointer()->mode == AUTO)  //自瞄模式
	{
		//add值更新并限幅
		add_yaw=AbsoluteControlAddLimit((yaw_absolute_set_rad-yaw_absolute_rad),add_yaw,gimbal_msg.yaw_relative_angle,MAX_YAW_RELATIVE,MIN_YAW_RELATIVE);
	
  }else if(SysPointer()->mode == SPIN_AUTO)  //小陀螺自瞄模式
	{
		//add值更新，不进行限幅
		add_yaw = SysPointer()->add_yaw;
	}
	
	//角度值更新及限制
	add_pit=AbsoluteControlAddLimit((pit_absolute_set_rad-pit_absolute_rad),add_pit,gimbal_msg.pit_relative_angle,MAX_PIT_RELATIVE,MIN_PIT_RELATIVE);
	
	//进行赋值
	yaw_absolute_set_rad = rad_format(yaw_absolute_rad + add_yaw);
	pit_absolute_set_rad = rad_format(pit_absolute_rad + add_pit);
}

/**
	* @brief          正常绝对角度控制--正常
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::NormalControl()
{
	yaw_motor.DJIMotorControl(&yaw_absolute_rad,&yaw_absolute_set_rad,&yaw_gyro,OFF_SET);
	pit_motor.DJIMotorControl(&pit_absolute_rad,&pit_absolute_set_rad,&pitch_gyro,OFF_SET);
}

/**
	* @brief          无力模式控制
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::ZeroForceControl()
{
	yaw_motor.MotorZeroForce();
	pit_motor.MotorZeroForce();
}

/**
	* @brief          相对角度控制
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::RelativeControl()
{
	//正常相对角度控制角速度应该也使用电机的，不使用陀螺仪的角速度
	yaw_relative_set=yaw_motor.MotorWorkSpaceLimit(yaw_relative_set,add_yaw,MAX_YAW_RELATIVE,MIN_YAW_RELATIVE);
	pit_relative_set=pit_motor.MotorWorkSpaceLimit(pit_relative_set,add_pit,MAX_PIT_RELATIVE,MIN_PIT_RELATIVE);
	
	yaw_motor.DJIMotorControl(&gimbal_msg.yaw_relative_angle,&yaw_relative_set,&yaw_gyro,OFF_SET);
	pit_motor.DJIMotorControl(&gimbal_msg.pit_relative_angle,&pit_relative_set,&pitch_gyro,OFF_SET);
}

/**
	* @brief          初始化信息更新
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::InitInfoUpdate()
{
	if(fabs(INIT_PITCH_SET - pit_absolute_rad) > GIMBAL_INIT_ANGLE_ERROR)  //先PITCH轴初始化
	{
		add_pit = (INIT_PITCH_SET - pit_absolute_rad) * GIMBAL_INIT_PITCH_SPEED;
		add_yaw = 0.0f;
	}else                                                                  //后YAW轴初始化
	{
		add_pit = (INIT_PITCH_SET - pit_absolute_rad) * GIMBAL_INIT_PITCH_SPEED;
		add_yaw = (INIT_YAW_SET - gimbal_msg.yaw_relative_angle) * GIMBAL_INIT_YAW_SPEED;
	}
}

/**
	* @brief          判断初始化状态
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::JudgeInitState()
{
  static uint16_t init_time = 0;
  static uint16_t init_stop_time = 0;
  init_time++;
        
	//目标值与当前值之差小于阈值超过一定时间，则判断初始化完成
  if((fabs(gimbal_msg.yaw_relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
      fabs(pit_absolute_rad - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR))
  {        
    if(init_stop_time < GIMBAL_INIT_STOP_TIME)
      init_stop_time++;
  }else
  {     
    if(init_time < GIMBAL_INIT_TIME)
	  init_time++;
  }

	//超过初始化最大时间，或者已经稳定到中值一段时间，退出初始化状态开关打下档，或者掉线
  if(init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME)
  {
    return;
  }else  //初始化完成
  {
    init_stop_time = 0;
    init_time = 0;
	SysPointer()->mode = NORMAL;
  }
}
