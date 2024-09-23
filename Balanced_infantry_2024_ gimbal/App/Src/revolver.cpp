/**
 ******************************************************************************
 * @file    revolver.cpp
 * @author  Xushuang
 * @version V1.0.0 �������
 * @date    2023/9/22
 * @brief		�˴�Ϊ���̸�ģʽ����
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "arm_math.h"
#include "revolver.h"
#include "referee_data.h"

//�������ʵ��
Revolver_t revolver;

/********�����ʼ������*********/
Motor_Setting_t firc_l_config = {"fir_l",ON_CAN2,1,M3508,POSITIVE_DIRECT,DIRECT_DRIVE,SINGLE_LOOP};
Motor_Setting_t firc_r_config = {"fir_r",ON_CAN1,2,M3508,POSITIVE_DIRECT,DIRECT_DRIVE,SINGLE_LOOP};
Motor_Setting_t stir_config = {"stir",ON_CAN1,3,M2006,POSITIVE_DIRECT,RATIO_1_TO_36,CASCADE_LOOP};

/**
  * @brief          ��������๹�캯��
  * @param[in]      NULL
  * @retval         NULL
  */
Revolver_t::Revolver_t()
{
	/****�趨��ʼ�ٶ�ֵ����Ϊ����������ԣ�����ʱ�޸Ĵ˱�������****/
	vfic_set = FIRST_STAGE_RPM_SET;
	/*****������Ƶ����*****/
	revolver_msg.shoot_rate = INIT_FREQUENCE;
}

/**
  * @brief          ���������ʼ��
  * @param[in]      NULL
  * @retval         NULL
  */
void RevolverInit()
{
	/***************Ħ���ֳ�ʼ��***************/
	//�����ʼ��
	revolver.firc_l[FIRST_STAGE].DJIMotorInit(&firc_l_config);  
	revolver.firc_r[FIRST_STAGE].DJIMotorInit(&firc_r_config);
	
#if FIRC_STAGE == SECONE_STAGE+1  //�ڶ���Ħ��������
	revolver.firc_l[SECONE_STAGE].DJIMotorInit(&firc_l_config);
	revolver.firc_r[SECONE_STAGE].DJIMotorInit(&firc_r_config);
#endif
	//��������ٶȻ�PID��������ʼ��
    const fp32 Firc_speed_pid[3] = {FIRC_SPEED_PID_KP, FIRC_SPEED_PID_KI, FIRC_SPEED_PID_KD};
	for(uint8_t i = 0;i<FIRC_STAGE;i++)
	{
		revolver.firc_l[i].controller.speed_PID.Init(PID_POSITION,Firc_speed_pid,NULL,FIRC_SPEED_PID_MAX_IOUT);
		revolver.firc_r[i].controller.speed_PID.Init(PID_POSITION,Firc_speed_pid,NULL,FIRC_SPEED_PID_MAX_IOUT);
	}
	
	/***************���̵����ʼ��***************/
	revolver.fire_time=0;
	revolver.fire_Hz=20;

	revolver.stir_motor_gun[MAIN_REVOLVER].DJIMotorInit(&stir_config);

#if REVOLVER_NUM != 1  //�ڶ������̵ĳ�ʼ��
	revolver.stir_motor_gun[MAIN_REVOLVER].DJIMotorInit(&stir_config);  //��Ҫ�Ĳ���
#endif
	
	//����PID������
	const fp32 Stir_speed_pid[3] = {STIR_SPEED_PID_KP, STIR_SPEED_PID_KI, STIR_SPEED_PID_KD};
	const fp32 Stir_position_pid[3] = {STIR_POSITION_PID_KP, STIR_POSITION_PID_KI, STIR_POSITION_PID_KD};
	
	//��Ҫ���̿�����������ʼ�������и������̣�
	for(uint8_t u = 1;u<=REVOLVER_NUM;u++)
	{
		revolver.stir_motor_gun[u-1].controller.angle_PID.Init(PID_POSITION, Stir_position_pid,STIR_POSITION_PID_MAX_OUT, STIR_POSITION_PID_MAX_IOUT);
		revolver.stir_motor_gun[u-1].controller.speed_PID.Init(PID_POSITION, Stir_speed_pid,NULL, STIR_SPEED_PID_MAX_IOUT);
 	}
	/******��Ϣ����ʵ������******/
	CenterPointer()->PointerInit(&revolver.revolver_msg,REVOLVERPUB);
}

/**
  * @brief          �������������
  * @param[in]      NULL
  * @retval         NULL
  */
void RevolverTask()
{
	revolver.InfoUpdate();  //��Ϣ����
	revolver.ControlSet();  //��������
}

/**
  * @brief          ��Ϣ����
  * @param[in]      NULL
  * @retval         NULL
  */
void Revolver_t::InfoUpdate()
{
	/*****����ϵͳ��Ϣ����*****/
	#if ROBOT_SHOOT_TYPE == BULLET_17
	//ʣ������=����-��ǰ����������id��ǹ��2�����û����Ҫ�ĳ�id2��
	revolver_msg.rest_heat = JUDGE_usGetHeatLimit_id1_17mm() - JUDGE_usGetRemoteHeat_id1_17mm(); 
	
	#elif ROBOT_SHOOT_TYPE == BULLET_42
	
	//ʣ������=����-��ǰ����
	revolver_msg.rest_heat = JUDGE_usGetHeatLimit_id1_42mm() - JUDGE_usGetRemoteHeat_id1_42mm(); 
	
	#endif
	
	/*****������Ϣ����*****/
	//���̵���ٶȸ��£��˲�һ�£�
    fp32 revolver_fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
	stir_speed = stir_motor_gun->FilterSpeed(&revolver_fliter_num[0]);
	
	//���̵���Ƕȸ��£�����API��
	stir_angle = stir_motor_gun[MAIN_REVOLVER].GetOutputShaftRad();

	//�ж��Ƿ�ﵽ�����ڣ����0��
	if(fabs(stir_angle_set - stir_angle)<AN_BULLET*0.1f)
		revolver_msg.shoot_finish = 1;
	else
		revolver_msg.shoot_finish = 0;
	
	/*****���̷�ת�߼��ж�*****/
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
	/*****��������ģʽ�л�ʱ��Ϣ����*****/
	if(last_stir_mode != SHOOT_FIXED_FIRE_RATE && SysPointer()->stir_mode == SHOOT_FIXED_FIRE_RATE)
	{
		//��ʱ��Ϊ�ٶȵ���//�ĳɽǶȻ�
		stir_motor_gun[MAIN_REVOLVER].ChangeControlMode(CASCADE_LOOP);
		//�������������á�������һ�μ��ɣ����Ƕȱջ�ʱ���ٶȻ�Ч���Ϻÿ���ֱ���ã���ʡ�Կ������������ã�
		
	}else if(last_stir_mode == SHOOT_FIXED_FIRE_RATE && SysPointer()->stir_mode != SHOOT_FIXED_FIRE_RATE)
	{
		stir_motor_gun[MAIN_REVOLVER].ChangeControlMode(CASCADE_LOOP);
		//stir_angle_set = stir_angle;
	}
	last_stir_mode = SysPointer()->stir_mode;  //�ϴ�ģʽ��ȡ

	//UI
	revolver_msg.firc_speed_set=(int16_t)FIRST_STAGE_RPM_SET+SysPointer()->offset_firc_v;


}

/**
  * @brief          ��������
  * @param[in]      NULL
  * @retval         NULL
  */
void Revolver_t::ControlSet()
{
	//Ħ��������
	FircControl();
	//��������-ֻ��Ħ���ֿ���������²�������ת��
	StirControl();
}

/**
	* @brief          Ħ�����ٶ��趨
  * @param[in]      NULL
  * @retval         NULL
  */
void Revolver_t::FircSpeedSet()
{
#if ROBOT_SHOOT_TYPE == BULLET_42
	//һ��Ħ����ת������
	l_firc_speed_set[FIRST_STAGE] = vfic_set;
	r_firc_speed_set[FIRST_STAGE] = vfic_set;
	//һ��Ħ����б�º���
	l_firc_ramp_set[FIRST_STAGE] = RAMP_float(l_firc_speed_set[FIRST_STAGE],l_firc_ramp_set[FIRST_STAGE],10);
	r_firc_ramp_set[FIRST_STAGE] = RAMP_float(r_firc_speed_set[FIRST_STAGE],r_firc_ramp_set[FIRST_STAGE],10);
#elif ROBOT_SHOOT_TYPE == BULLET_17

	//һ��Ħ����ת������
	vfic_set=FIRST_STAGE_RPM_SET+SysPointer()->offset_firc_v;


	l_firc_speed_set[FIRST_STAGE] =  vfic_set;
	r_firc_speed_set[FIRST_STAGE] = -vfic_set;

	l_firc_ramp_set[FIRST_STAGE] = RAMP_float(l_firc_speed_set[FIRST_STAGE],l_firc_ramp_set[FIRST_STAGE],10);
	r_firc_ramp_set[FIRST_STAGE] = RAMP_float(r_firc_speed_set[FIRST_STAGE],r_firc_ramp_set[FIRST_STAGE],10);
#endif
}

/**
	* @brief          ��ͬת��ѡ���������ò����������أ�
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
	* @brief          Ħ���ֿ���
  * @param[in]      NULL
  * @retval         NULL
  */
void Revolver_t::FircControl()
{
	switch(SysPointer()->fir_mode)
	{
		case FORBID_FIRE:
		case CLOSE: 
			//�ر�Ħ����
			for(uint8_t i = 0;i<FIRC_STAGE;i++)
			{
				l_firc_ramp_set[i] = 0;
				r_firc_ramp_set[i] = 0;
				firc_l[i].MotorZeroForce();
				firc_r[i].MotorZeroForce();
			}
			break;
		case OPEN:
			//����Ħ��������
			FircSpeedSet();
			break;
	}
	//�༶Ħ���ֿ���
	for(uint8_t i = 0;i<FIRC_STAGE;i++)
	{
		firc_l[i].DJIMotorControl(NULL,&l_firc_ramp_set[i],NULL,OFF_SET);
		firc_r[i].DJIMotorControl(NULL,&r_firc_ramp_set[i],NULL,OFF_SET);
	}
		//UI����
		SysPointer()->key_flag.fir_flag=SysPointer()->fir_mode;



}

/**
  * @brief          ���̿���
  * @param[in]      NULL
  * @retval         NULL
  */
void Revolver_t::StirControl()
{
	/*****�˴������ݸ���ģʽ���ж�Ӧ����������*****/
	switch(SysPointer()->stir_mode)
	{
		//����ģʽ
		case NO_MOVE:
			//����ʱ�������������ã��������⶯����
			stir_angle_set = stir_angle;
			//û��Ħ���ֲ��������-�������շ�����ֵΪ��
			stir_motor_gun[MAIN_REVOLVER].MotorZeroForce(); 

			break;
		
		//���̷�תģʽ
		case REVERSE_MOVE:
			//���벦�̷�ת�ж��߼�����if~��//��ת�߼�������
			
			stir_angle_set += AN_BULLET;  //С�������޸�AN_BULLET
			break;
		
		//����ģʽ
		case SHOOT_1_BULLET:
			if(SysPointer()->fir_mode!= OPEN)
				return;

			stir_angle_set -= AN_BULLET;  //С�������޸�AN_BULLET
			SysPointer()->stir_mode = ALLOW_MOVE;
			break;
			
		//������ģʽ
		case SHOOT_3_BULLET:
			if(SysPointer()->fir_mode!= OPEN)
				return;

			stir_angle_set -= 3*AN_BULLET;  //С�������޸�AN_BULLET
			SysPointer()->stir_mode = ALLOW_MOVE;
			break;
			
		//�̶���Ƶ����ģʽ
		case SHOOT_FIXED_FIRE_RATE:

			fire_time++;
			//����Ƶ��1000Hz 
			if((fire_time%(1000/fire_Hz))==1  &&  (fire_time_last%(1000/fire_Hz))==0)
			{
					stir_angle_set -= AN_BULLET;
			}
			
			//stir_angle_set -= 0;
			stir_motor_gun[MAIN_REVOLVER].DJIMotorControl(&stir_angle,&stir_angle_set,NULL,ON_SET);

			fire_time_last=fire_time;
			//��Ƶ������ֵ���趨��Ƶ����/s�� * ����һ��Ȧ�Ƕȣ�rad�� / ���������ţ� = rad/s��
			//stir_speed_set = -revolver_msg.shoot_rate * 2.0f * PI / WHOLE_CIRCLE_BULLET_NUM; 
			//���������бջ����ƣ����٣�
			//stir_motor_gun[MAIN_REVOLVER].DJIMotorControl(&stir_speed,&stir_speed_set,NULL,ON_SET);
			break;
		
		//������������
		case ALLOW_MOVE:
			#if USE_SENSOR == OFF_SET

						//�������Ϊ׼
				stir_motor_gun[MAIN_REVOLVER].DJIMotorControl(&stir_angle,&stir_angle_set,NULL,ON_SET);	
		
				fire_time=0;

				//��������
				

			#else
				//���ٶȻ���ת����������⵽�仯ֵ
				stir_motor_gun[MAIN_REVOLVER].DJIMotorControl(&stir_angle,&stir_angle_set,NULL,ON_SET);
			#endif
			break;
	}
	
}

