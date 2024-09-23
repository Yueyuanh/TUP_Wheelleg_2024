/**
 ******************************************************************************
 * @file    gimbal.cpp
 * @author  Xushuang
 * @version V1.0.0 �������
 * @date    2023/9/20
 * @brief		�˴�Ϊ��̨��ģʽ����
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "arm_math.h"
#include "gimbal.h"
#include "stm32f4xx_hal.h"
#include "bsp_dwt.h"

//���ĺ�ΪX���������ҷ�ΪY���������Ϸ�ΪZ��������

//����ʵ��
gimbal_t gimbal;

/********�����ʼ������*********/
Motor_Setting_t yaw_setting = {"yaw",ON_CAN1,1,GM6020,POSITIVE_DIRECT,DIRECT_DRIVE,LADRC_FDW_CONTROL};
Motor_Setting_t pit_setting = {"pit",ON_CAN1,2,GM6020,POSITIVE_DIRECT,DIRECT_DRIVE,LADRC_FDW_CONTROL};

/********�����������������*********/
//WC-B0-WO-W-GAIN
fp32 yaw_init_config[]={15,0.01f,100,0,0};
fp32 pit_init_config[]={15,0.01f,100,0,0};

fp32 yaw_normal_config[]={15,0.007f,100,0,0};
fp32 pit_normal_config[]={15,0.007f,100,0,0};

fp32 yaw_auto_config[]={15,0.005f,80,0,0};
fp32 pit_auto_config[]={15,0.007f,80,0,0};

/**
	* @brief          gimbal�๹�캯��
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
	* @brief          ��̨��ʼ��
  * @param[in]      NULL
  * @retval         NULL
  */
void GimbalInit()
{
	/******�����ʼ��******/
	gimbal.yaw_motor.DJIMotorInit(&yaw_setting);
	gimbal.pit_motor.DJIMotorInit(&pit_setting);
	/******��Ϣ����ʵ������******/
	CenterPointer()->PointerInit(&gimbal.gimbal_msg,GIMBALPUB);
}

/**
	* @brief          ��̨�������ĳ�ʼ��
  * @param[in]      type������������
  * @retval         NULL
  */
void gimbal_t::GimbalControllerInit(uint8_t type)
{
	if(SysPointer()->change_mode_flag)  //���ݲ�ͬģʽ���Ŀ���������
	{
		SysPointer()->change_mode_flag = 0;
		//����ģʽ��������ʼ��
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
	* @brief          ��̨������
  * @param[in]      NULL
  * @retval         NULL
  */
void GimbalTask()
{
	//������Ϣ����
	gimbal.BasicInfoUpdate();


	//���ݲ�ͬģʽѡ��ͬ��������Ϣ��Դ
	switch(SysPointer()->mode)
	{
		//ң������Ϊ������������ģʽ��
		case NORMAL:
		case SPIN:
			gimbal.GimbalControllerInit(NORMAL_PARAM); 					//��������ʼ��
			gimbal.OperationInfoUpdate();      		   					//����ʱ��Ϣ����
			gimbal.NormalControl();                    					//���������
//			MonitorPointer()->state = SetLEDWorkType(GREEN_SLOW);
			BlinkLEDByCount(0xFFFF0000,500);
			break;


		//USBCDC������Ϊ������������ģʽ��
		case AUTO:
		case SPIN_AUTO:
			gimbal.GimbalControllerInit(AUTO_PARAM); 			       	//��������ʼ��
			gimbal.AutoInfoUpdate(); 								   	//������Ϣ����
			gimbal.NormalControl(); 									//���������
//			MonitorPointer()->state = SetLEDWorkType(BLUE_SLOW); 		//���õ���ɫΪ��ɫ����
			BlinkLEDByCount(0xFF00FF00,500);
			break;


		//��ʼ��ģʽ
		case INIT:
			gimbal.GimbalControllerInit(INIT_PARAM); 					//��������ʼ��
			gimbal.InitInfoUpdate();  									//��ʼ����Ϣ����
			gimbal.JudgeInitState();									//�жϳ�ʼ��״̬
			gimbal.RelativeControl();									//���������
//			MonitorPointer()->state = SetLEDWorkType(BLUE_TWO_BLINK);	//���õ���ɫΪ��ɫ˫��
			BlinkLEDByCount(0xFFFFFF00,500);
			break;


		//��������ʧң����ʧ��ģʽ
		case DT7_MISSING:
//			MonitorPointer()->state = SetLEDWorkType(YELLOW_TWO_BLINK); //���õ���ɫΪ��ɫ˫��


		case ZERO_FORCE:
			gimbal.ZeroForceControl();									//�����������
//			MonitorPointer()->state = SetLEDWorkType(CYAN_SLOW);	    //���õ���ɫΪ��ɫ��˸
		BlinkLEDByCount(0xFF0000FF,500);
			break;


		//��ԽǶȿ���ģʽ�����̣���������̨���п���
		case RELATIVE_ANGLE:
		case NO_FOLLOW_YAW: 
			BlinkLEDByCount(0xFF00FF00,500);
			//�����п���
			break;
	}
}

/**
	* @brief          ������Ϣ����
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::BasicInfoUpdate()
{
	/**************���ԽǶȣ������ǣ���Ϣ����****************/
	//�����Ǿ��ԽǶ�ת���ɻ�����
	yaw_absolute_rad = *(get_INS_angle_point() + INS_YAW_ADDRESS_OFFSET);
	pit_absolute_rad = *(get_INS_angle_point() + INS_PITCH_ADDRESS_OFFSET);
	//�����ǽ��ٶ�
	roll_gyro = *(get_gyro_data_point() + INS_GYRO_X_ADDRESS_OFFSET);
	pitch_gyro = *(get_gyro_data_point() + INS_GYRO_Y_ADDRESS_OFFSET);
	yaw_gyro = arm_cos_f32(gimbal_msg.pit_relative_angle) * (*(get_gyro_data_point() + INS_GYRO_Z_ADDRESS_OFFSET))-
						 arm_sin_f32(gimbal_msg.pit_relative_angle) * (*(get_gyro_data_point() + INS_GYRO_X_ADDRESS_OFFSET));
	
	/**************��ԽǶȣ��������Ϣ����****************/
	//����һ��һ����ԽǶȲ���ֱ��ʹ�õ���趨ֵ��--������
	yaw_ecd_angle = yaw_motor.CalcTotalecd();
	//�����ԽǶ�
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
	* @brief          ����ʱ��Ϣ����
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::OperationInfoUpdate()
{
//	if(SysPointer()->mode == NORMAL)     //���̸�����̨ģʽ
//	{
//		//addֵ���²��޷�
//		add_yaw = AbsoluteControlAddLimit((yaw_absolute_set_rad-yaw_absolute_rad),SysPointer()->add_yaw,gimbal_msg.yaw_relative_angle,MAX_YAW_RELATIVE,MIN_YAW_RELATIVE);
//	}else if(SysPointer()->mode == SPIN) //С����ģʽ
//	{
//		//addֵ���£��������޷�
//		add_yaw = SysPointer()->add_yaw;
//	}
	
		add_yaw = SysPointer()->add_yaw;



	//addֵ���²��޷�
	add_pit=AbsoluteControlAddLimit((pit_absolute_set_rad-pit_absolute_rad),SysPointer()->add_pit,gimbal_msg.pit_relative_angle,MAX_PIT_RELATIVE,MIN_PIT_RELATIVE);
	
	//�����Ƹ�ֵ������
	yaw_absolute_set_rad = rad_format(yaw_absolute_set_rad + add_yaw);
	pit_absolute_set_rad = rad_format(pit_absolute_set_rad + add_pit);
}

/**
	* @brief          ������Ϣ����
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::AutoInfoUpdate()
{
	fp32 auto_error_yaw,auto_error_pit;
	/*****��ȡ��ԽǶ�ֵ*****/
	VisionErrorAngleYaw(&auto_error_yaw);
	VisionErrorAnglePit(&auto_error_pit);
	
	/*****���ݴ���ʽ*****/
#if Auto_Type == HANDLE_LPF
	//��׼��Ŀ����п���
	if( VisionGetIfTarget() )
	{
		//һ�ڵ�ͨ�˲�
		add_yaw = LPF(&yaw_vision_lpf ,0.001,auto_error_yaw,550); //800 812 875 750 843 781 769 687
		add_pit = LPF(&pitch_vision_lpf ,0.001,auto_error_pit,1300);
		
		//�����쳣����
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
	
	/*****�������ݺ�����޷�����*****/
	if(SysPointer()->mode == AUTO)  //����ģʽ
	{
		//addֵ���²��޷�
		add_yaw=AbsoluteControlAddLimit((yaw_absolute_set_rad-yaw_absolute_rad),add_yaw,gimbal_msg.yaw_relative_angle,MAX_YAW_RELATIVE,MIN_YAW_RELATIVE);
	
  }else if(SysPointer()->mode == SPIN_AUTO)  //С��������ģʽ
	{
		//addֵ���£��������޷�
		add_yaw = SysPointer()->add_yaw;
	}
	
	//�Ƕ�ֵ���¼�����
	add_pit=AbsoluteControlAddLimit((pit_absolute_set_rad-pit_absolute_rad),add_pit,gimbal_msg.pit_relative_angle,MAX_PIT_RELATIVE,MIN_PIT_RELATIVE);
	
	//���и�ֵ
	yaw_absolute_set_rad = rad_format(yaw_absolute_rad + add_yaw);
	pit_absolute_set_rad = rad_format(pit_absolute_rad + add_pit);
}

/**
	* @brief          �������ԽǶȿ���--����
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::NormalControl()
{
	yaw_motor.DJIMotorControl(&yaw_absolute_rad,&yaw_absolute_set_rad,&yaw_gyro,OFF_SET);
	pit_motor.DJIMotorControl(&pit_absolute_rad,&pit_absolute_set_rad,&pitch_gyro,OFF_SET);
}

/**
	* @brief          ����ģʽ����
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::ZeroForceControl()
{
	yaw_motor.MotorZeroForce();
	pit_motor.MotorZeroForce();
}

/**
	* @brief          ��ԽǶȿ���
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::RelativeControl()
{
	//������ԽǶȿ��ƽ��ٶ�Ӧ��Ҳʹ�õ���ģ���ʹ�������ǵĽ��ٶ�
	yaw_relative_set=yaw_motor.MotorWorkSpaceLimit(yaw_relative_set,add_yaw,MAX_YAW_RELATIVE,MIN_YAW_RELATIVE);
	pit_relative_set=pit_motor.MotorWorkSpaceLimit(pit_relative_set,add_pit,MAX_PIT_RELATIVE,MIN_PIT_RELATIVE);
	
	yaw_motor.DJIMotorControl(&gimbal_msg.yaw_relative_angle,&yaw_relative_set,&yaw_gyro,OFF_SET);
	pit_motor.DJIMotorControl(&gimbal_msg.pit_relative_angle,&pit_relative_set,&pitch_gyro,OFF_SET);
}

/**
	* @brief          ��ʼ����Ϣ����
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::InitInfoUpdate()
{
	if(fabs(INIT_PITCH_SET - pit_absolute_rad) > GIMBAL_INIT_ANGLE_ERROR)  //��PITCH���ʼ��
	{
		add_pit = (INIT_PITCH_SET - pit_absolute_rad) * GIMBAL_INIT_PITCH_SPEED;
		add_yaw = 0.0f;
	}else                                                                  //��YAW���ʼ��
	{
		add_pit = (INIT_PITCH_SET - pit_absolute_rad) * GIMBAL_INIT_PITCH_SPEED;
		add_yaw = (INIT_YAW_SET - gimbal_msg.yaw_relative_angle) * GIMBAL_INIT_YAW_SPEED;
	}
}

/**
	* @brief          �жϳ�ʼ��״̬
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::JudgeInitState()
{
  static uint16_t init_time = 0;
  static uint16_t init_stop_time = 0;
  init_time++;
        
	//Ŀ��ֵ�뵱ǰֵ֮��С����ֵ����һ��ʱ�䣬���жϳ�ʼ�����
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

	//������ʼ�����ʱ�䣬�����Ѿ��ȶ�����ֵһ��ʱ�䣬�˳���ʼ��״̬���ش��µ������ߵ���
  if(init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME)
  {
    return;
  }else  //��ʼ�����
  {
    init_stop_time = 0;
    init_time = 0;
	SysPointer()->mode = NORMAL;
  }
}
