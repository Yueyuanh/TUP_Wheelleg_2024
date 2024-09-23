/**
 ******************************************************************************
 * @file    motor.cpp
 * @author  Xushuang
 * @version V1.0.0 Xushuang ������� 2023/8/22
 *					V2.0.0 Xushuang �޸�CAN���ֽ��� 2023/10/30
 * @date    2023/10/30
 * @brief		�˴�Ϊ�����
 *					�󽮵���Լ�AM3508
 ******************************************************************************
 * @attention
 *			�����˵�����ݶ�ȡ��Э�飬������PID��������LADRC�����ڳ�ʼ��ʱ����ѡ���
 *	�����͵���Ϣ�Լ���ȡ�Ŀ��������˵���豸������CAN���ϣ��ɸ������д��룬���
 *  ���ʱ�����ٵ�CAN�߽��ջص������������µ�case�����豸��Ϣ����CAN���Զ��Ǽǣ�
 *  ������DJIMotorInit(**)������Ϳ�������ȡ������ݺͿ���
 ******************************************************************************
 */
#include "motor.h"
#include "bsp_dwt.h"

//M2006,M3508:���վ�Ϊ0x200+ID������1-4Ϊ0x200��5-8Ϊ0x1FF
//GM6020:����Ϊ0x204+ID,����1-4Ϊ0x1FF��5-7Ϊ0x2FF
//AM3508:�������ԽǶȽ���wΪ0x300+ID


////����ӿ��������ͼ��
//������ݶ�ȡ
#define get_basic_motor_measure(ptr, data)                          \
{                                                                   \
	(ptr)->last_ecd = (ptr)->ecd;                                     \
  (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);              \
  (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);        \
  (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);    \
  (ptr)->temperate = (data)[6];                                     \
	if (abs((ptr)->ecd - (ptr)->last_ecd) > 4096)											\
	{																																	\
		if ((ptr)->ecd  >= 6000 && (ptr)->last_ecd <= 1000)							\
		{																																\
			(ptr)->difference_num -= 1;																					  	\
		}																																\
		else	if((ptr)->ecd  <= 1000 && (ptr)->last_ecd >= 6000)					\
		{																																\
			(ptr)->difference_num += 1;																					  	\
		}																															  \
	}																																	\
}

//MK20��ȡ�������ԽǶ�
#define AM3508_output_absolute_angle(ptr, data)                              												   \
{                                                                  																 \
  (ptr)->output_encoder_num = (int32_t)((data)[0] << 24 | (data)[1] << 16 | (data)[2] << 8 | (data)[3]);   \
  (ptr)->output_encoder_ecd = (uint16_t)((data)[4] << 8 | (data)[5]);      														 \
}	

//����ʵ����ַ
DJIMotorInstance *MotorInstances[TOTAL_MOTOR_SUM];
//ע��������
uint8_t dji_motor_num = 0;

/**
	* @brief          ������ԽǶ�
	* @param[in]     	*ecd_angle�����ۻ��Ƕ�ֵ
	* @param[in]     	offset_ecd����ֵ����
	* @param[in]     	drive_radio��������
  * @retval         fp32������Ƕ�
  */
fp32 DJIMotorInstance::MotorEcdToAngle(uint16_t *ecd_angle,uint16_t offset_ecd,int8_t drive_radio)
{
  int32_t relative_ecd;
  if(ecd_angle == NULL)
  {
	relative_ecd = motor_measure.ecd - offset_ecd;
  }else
  {
	relative_ecd = *ecd_angle - offset_ecd;
  }
	
  if(relative_ecd > HALF_ECD_RANGE * drive_radio)
  {
	relative_ecd -= ECD_RANGE * drive_radio;
  }else if (relative_ecd < -HALF_ECD_RANGE * drive_radio)
  {
	relative_ecd += ECD_RANGE * drive_radio;
  }
	
  return relative_ecd * MOTOR_ECD_TO_RAD / drive_radio;
}

/**
	* @brief          ���Ƶ����ԽǶ��˶��ռ�
	* @param[in]     	relative_angle_set����ԽǶ��趨ֵ
	* @param[in]     	add�����ֵ
	* @param[in]     	max_limit�����Ƕ�
	* @param[in]     	min_limit����С�Ƕ�
  * @retval         Null
  */
fp32 DJIMotorInstance::MotorWorkSpaceLimit(fp32 relative_angle_set,fp32 add,fp32 max_limit,fp32 min_limit)
{
	relative_angle_set += add;
	if(relative_angle_set > max_limit)
	{
		relative_angle_set = max_limit;
	}else if(relative_angle_set < min_limit)
	{
		relative_angle_set = min_limit;
	}
	return relative_angle_set;
}

/**
	* @brief          ���ԽǶ�ʱ��addֵ���޷������������С��ԽǶ�������
	* @param[in]     	error_angle�����Ƕ�
	* @param[in]     	add�����ֵ
	* @param[in]     	relative_angle����ԽǶ�
	* @param[in]     	max_limit�����Ƕ�
	* @param[in]     	min_limit����С�Ƕ�
  * @retval         Null
  */
fp32 AbsoluteControlAddLimit(fp32 error_angle,fp32 add,fp32 relative_angle,fp32 max_limit,fp32 min_limit)
{
	static fp32 bias_angle;
	//��ǰ���ƽǶ�
	bias_angle = rad_format(error_angle);
	//relative angle + angle error + add_angle > max_relative angle
	//��̨��ԽǶ�+ ���Ƕ� + �����Ƕ� ������� ����е�Ƕ�
    if (relative_angle + bias_angle + add > max_limit)
	{
		//calculate max add_angle
    //�����һ��������ӽǶ�
		if(add > 0.0f)
			add = max_limit - relative_angle - bias_angle;
	}else if (relative_angle + bias_angle + add < min_limit)
	{
		if(add < 0.0f)
			add = min_limit - relative_angle - bias_angle;
	}
	return add;
	
}

/**
	* @brief          ����๹�캯��
	* @param[in]     	Null
  * @retval         Null
  */
DJIMotorInstance::DJIMotorInstance()
{
  motor_watch.error_code = NO_MOTOR_ERROR;
  motor_settings = NULL;
  //һϵ�е�ַ�洢
  // ����ǰʵ����ָ����ӵ�������
  for (int i = 0; i < TOTAL_MOTOR_SUM; i++) 
  {
	if (MotorInstances[i] == NULL) 
	{
		MotorInstances[i] = this;
		dji_motor_num++;
      break;
    }
  }
}

/**
	* @brief          �󽮵�����ݽ���
  * @param[in]      *rx_instance�����յ�ʵ��
  * @retval         Null
  */
static void DecodeDJIMotor(CAN_Rx_Instance_t *rx_instance)
{
	uint8_t *rxbuff = rx_instance->rx_buff;
	//address��void*��ǿ��ת��
	DJIMotorInstance *motor = (DJIMotorInstance *)rx_instance->module_address;
	Dji_Motor_Measure_t *measure = &motor->motor_measure;
	get_basic_motor_measure(measure,rxbuff);
}

static void DecodeExtraAM3508Data(CAN_Rx_Instance_t *rx_instance)
{
	uint8_t *rxbuff = rx_instance->rx_buff;
	//address��void*��ǿ��ת��
	DJIMotorInstance *motor = (DJIMotorInstance *)rx_instance->module_address;
	Dji_Motor_Measure_t *measure = &motor->motor_measure;
	AM3508_output_absolute_angle(measure,rxbuff);
}

/**
	* @brief          �󽮵��ʾ����ʼ��
  * @param[in]      input_type���������
	* @param[in]      can_num��CAN�߹��ص�ID
	* @param[in]      input_id�����ID������
	* @param[in]      input_control_type�����Ʒ�ʽ
  * @retval         Null
  */
void DJIMotorInstance::DJIMotorInit(Motor_Setting_t *config)
{
	fp32 max_out;
	uint32_t rx_id,am_rx_id;
	//���һ����û������
	//ID�����ڷ�Χ��ErrorCode
	if(config->set_id <= 0 || config->set_id > 8)
		motor_watch.error_code =  ID_RANGE_ERROR;
	
	motor_settings = config;
	//��ʼ���������
	switch(motor_settings->motor_type)
	{
		case M3508:
			max_out = 16384;
			rx_id = 0x200 + motor_settings->set_id;//idΪ1-8
			break;
		case AM3508:
			max_out = 16384;
			rx_id = 0x200 + motor_settings->set_id;//idΪ1-8
			am_rx_id = 0x300 + motor_settings->set_id; //0x300+�����
			CANRxInitSet(&motor_settings->AMExtra_can,motor_settings->can_id,am_rx_id,this,DecodeExtraAM3508Data);
			CANRxRegister(&motor_settings->AMExtra_can);
			break;
		case GM6020:
			max_out = 30000;
			rx_id = 0x204 + motor_settings->set_id;//idΪ1-7
			break;
		case M2006:
			max_out = 10000;
			rx_id = 0x200 + motor_settings->set_id;//idΪ1-8
			break;
		default:
			//ErrorCode
			motor_watch.error_code = MOTOR_TYPE_ERROR;
			break;
	}
	//����CAN�ߵǼ�ע��
	CANRxInitSet(&motor_settings->motor_can,motor_settings->can_id,rx_id,this,DecodeDJIMotor);
	CANRxRegister(&motor_settings->motor_can);
	
	switch(motor_settings->control_type)
	{
		case LADRC_FDW_CONTROL:
			controller.ladrc_fdw.MaxOutInit(max_out);
			break;
		case LADRC_CONTROL:
			controller.ladrc.MaxOutInit(max_out);
			break;
		case CASCADE_LOOP:
			controller.angle_PID.MaxOutInit(max_out);
			controller.speed_PID.MaxOutInit(max_out);
			break;
		case SINGLE_LOOP:
			controller.speed_PID.MaxOutInit(max_out);
			break;
		case OPEN_LOOP:
			break;
	}
	
	//���ٱ��趨--��ֱ�Ӹ�enum��ֵ
	switch(motor_settings->reduction_type)
	{
		case DIRECT_DRIVE:
			motor_measure.reduction_ratio = 1.0f;
			break;
		case RATIO_1_TO_14:
			motor_measure.reduction_ratio = ONE_FOURTEEN;
			break;
		case RATIO_1_TO_19:
			motor_measure.reduction_ratio = ONE_NINETEEN;
			break;
		case RATIO_1_TO_36:
			motor_measure.reduction_ratio = ONE_OUT_OF_THIRTYSIX;
			break;
		case RATIO_1_TO_71:
			motor_measure.reduction_ratio = ONE_OUT_OF_SEVENTYONE;
			break;
	}
	
	motor_watch.InitDeviceStatus(1000);
}

/**
	* @brief          �ı����ģʽ
  * @param[in]      change_type����ı�Ŀ���ģʽ
  * @retval         Null
  */
void DJIMotorInstance::ChangeControlMode(Control_Type_e change_type)
{
	motor_settings->control_type = change_type;
}

/**
	* @brief          ���λ�����ã�δ���ƣ�������ʹ�ã�
	* @param[in]      *flag����ʼ�仯�ı�־λ
  * @param[in]      *set_positon������λ��
  * @param[in]      add_position���ۼ�λ��ֵ
  * @retval         null
  */
void SetMotorPosition(uint8_t *flag,fp32 *set_position,fp32 add_position)
{
	if(*flag)
	{
		*set_position += add_position;
		*flag = 0;
	}
}

/**
	* @brief          ���ص�����ý��յ�id
  * @param[in]      NUll
	* @retval         motor_rx_id������id
  */
uint32_t DJIMotorInstance::GetRxID()
{
	return motor_settings->motor_can.rx_id;
}

/**
	* @brief          ��������CAN�ߵ�id
  * @param[in]      NUll
	* @retval         can_id��can�ߺ�
  */
CAN_NAME_e DJIMotorInstance::GetCANID()
{
	return motor_settings->motor_can.can_id;
}

/**
	* @brief          ����Ƿ��ظ�ע��id
  * @param[in]      NUll
	* @retval         true or false
  */
uint8_t CheckSameID()
{
	for(uint8_t i=0; i< dji_motor_num; i++)
	{
		for(uint8_t j=i+1; j< dji_motor_num; j++)
		{
			if( MotorInstances[j]->GetCANID() == MotorInstances[i]->GetCANID() && MotorInstances[j]->GetRxID() == MotorInstances[i]->GetRxID())
			{
				return false;
			}
		}
	}
	return true;
}

//�����˲��źű�־�Ƿ���Ҫɾ��
/**
	* @brief          �󽮵������
  * @param[in]      *ref ��ȡ��ǰֵ��ַ
	* @param[in]      *set ��ȡ�趨ֵ��ַ
	* @param[in]      *motor_gyro ��ȡ���ٶ�ֵ��ַ
  * @retval         ���ֵ
  */
void DJIMotorInstance::DJIMotorControl(fp32 *ref,fp32 *set,fp32 *motor_gyro,uint8_t filter_flag)
{
	fp32 output;
	motor_watch.RecordStartTime(); //��ʼ��ʱ
	//���⸳ֵ��Ϊ������Թ۲�
	controller.target_value = *set;//*motor_settings->direction;  //�˴�����ʱ���ڲ�����
	controller.now_value = *ref;
	switch(motor_settings->control_type)
	{
		case LADRC_FDW_CONTROL:
			//������
			if(motor_gyro == NULL)
			{
				motor_watch.error_code = INPUT_PARAM_ERROR;
			}else
			{
				output=controller.ladrc_fdw.FDW_Calc(controller.now_value,controller.target_value,*motor_gyro);
			}
			break;
		case LADRC_CONTROL:
			//������
			if(motor_gyro == NULL)
			{
				motor_watch.error_code = INPUT_PARAM_ERROR;
			}else
			{
				output=controller.ladrc.Calc(controller.now_value,controller.target_value,*motor_gyro);
			}
			break;
		case SINGLE_LOOP:
		case CASCADE_LOOP:
				output=PIDControl(ref,set,filter_flag);
			break;
		case OPEN_LOOP:
			break;
	}
	controller.send_current = (int16_t)output*motor_settings->direction;
	motor_watch.CalcExcutePeriod(); //������ʱ
}

/**
	* @brief          ���PID���ƺ���
	*                 ˫��ʱ��ref���趨�Ƕ� set��Ŀ��Ƕ� filter_flag���Ƿ���Ҫ�˲�
  *                 ����ʱ��ref������Ҫ		set��Ŀ���ٶ� filter_flag���Ƿ���Ҫ�˲�
  * @param[in]      *ref ��ȡ��ǰֵ��ַ
	* @param[in]      *set ��ȡ�趨ֵ��ַ
	* @param[in]      *filter_flag: �Ƿ���Ҫ�˲���־
  * @retval         ���ֵ
  */
fp32 DJIMotorInstance::PIDControl(fp32 *ref,fp32 *set,uint8_t filter_flag)
{
	fp32 angle_out,speed_set,pid_out;
	//�ٶȵ���ʱֱ�Ӹ�ֵ�趨ֵ��ַ
	speed_set = *set * motor_settings->direction;
	//�ǶȻ�
	if(motor_settings->control_type == CASCADE_LOOP )
	{
		angle_out	=	controller.angle_PID.Calc(*ref,*set);
		speed_set = angle_out;
	}
	//����ʱֱ�ӽ���
	//�ٶȷ����Ƿ���Ҫ�˲�
	if(filter_flag)
	{
		pid_out	=	controller.speed_PID.Calc(controller.pid_speed,speed_set);
	}else
	{
		if(ref != NULL)
			pid_out	=	controller.speed_PID.Calc(*ref,speed_set);
		else
			pid_out	=	controller.speed_PID.Calc(motor_measure.speed_rpm,speed_set);
	}
	return pid_out;
}

/**
	* @brief          �������ģʽ����
  * @retval         null
  */
void DJIMotorInstance::MotorZeroForce()
{
	controller.send_current = 0;
}

fp32 DJIMotorInstance::CalcTotalecd()
{
	fp32 total_ecd;
	if(motor_measure.difference_num%2 == 1 || motor_measure.difference_num%2 == -1)
	{
		if(motor_measure.ecd < motor_measure.last_ecd)
			  total_ecd = motor_measure.ecd + 8192;
		else
				total_ecd = motor_measure.ecd + 8192;
	}else
	{
		total_ecd = motor_measure.ecd;
	}
	return total_ecd;
}

//�����е����---�������Կ���дһ���˲���
/**
	* @brief          ����˲��ٶȸ��º���
	* @param[in]      *filter_num: �˲�����
  * @retval         null
  */
fp32 DJIMotorInstance::FilterSpeed(fp32 *fliter_num)
{
	static fp32 speed_fliter_1 = 0.0f;
	static fp32 speed_fliter_2 = 0.0f;
  static fp32 speed_fliter_3 = 0.0f;
    
	if(fliter_num == NULL)
	{
		return false;
	}else
	{
		//���׵�ͨ�˲�
		speed_fliter_1 = speed_fliter_2;
		speed_fliter_2 = speed_fliter_3;
		speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (motor_measure.speed_rpm * RPM_TO_RAD_S * motor_measure.reduction_ratio) * fliter_num[2];
		controller.pid_speed = speed_fliter_3;
		return speed_fliter_3;
	}
}

/**
	* @brief          ��ȡ������ٶȵ�API
	* @param[in]      NULL
  * @retval         fp32
  */
fp32 DJIMotorInstance::GetRotorW() 
{
	return motor_measure.speed_rpm * RPM_TO_RAD_S;
}

/**
	* @brief          ����������ٶȵ�API
	* @param[in]      NULL
  * @retval         fp32
  */
fp32 DJIMotorInstance::GetOutputShaftW() 
{
	return motor_measure.speed_rpm * motor_measure.reduction_ratio * RPM_TO_RAD_S;
}

/**
	* @brief          ���ת�ӵ��ۻ��Ƕȵ�API
	* @param[in]      NULL
  * @retval         fp32
  */
fp32 DJIMotorInstance::GetRotorRad()  
{
	if(motor_settings->motor_type == AM3508)
		return (motor_measure.output_encoder_num * ECD_RANGE + motor_measure.ecd) * MOTOR_ECD_TO_RAD;
	else
		return (motor_measure.difference_num * ECD_RANGE + motor_measure.ecd) * MOTOR_ECD_TO_RAD;
}

/**
	* @brief          ����������ۻ��Ƕȵ�API
	* @param[in]      NULL
  * @retval         fp32
  */
fp32 DJIMotorInstance::GetOutputShaftRad() 
{
	if(motor_settings->motor_type == AM3508)
		return (motor_measure.output_encoder_num * ECD_RANGE + motor_measure.ecd) * motor_measure.reduction_ratio * MOTOR_ECD_TO_RAD;
	else
		return (motor_measure.difference_num * ECD_RANGE + motor_measure.ecd) * motor_measure.reduction_ratio * MOTOR_ECD_TO_RAD;
}

/**
	* @brief          ��÷��͵�����API
	* @param[in]      NULL
  * @retval         fp32
  */
int16_t *DJIMotorInstance::SendCurrentPointer()
{
	return &controller.send_current;
}

/**
	* @brief          ���ת��ת�ٵ�API
	* @param[in]      NULL
  * @retval         fp32
  */
int16_t DJIMotorInstance::GetRotorRpm()
{
	return motor_measure.speed_rpm;
}

/**
	* @brief          ���ת�ӷ�����Ϣ��API
	* @param[in]      NULL
  * @retval         fp32
  */
Dji_Motor_Measure_t *DJIMotorInstance::GetMotorMeasure()
{
	return &motor_measure;
}

/**
	* @brief          ��õ�����ò�����API
	* @param[in]      NULL
  * @retval         fp32
  */
Motor_Setting_t *DJIMotorInstance::SetParamPointer()
{
	return motor_settings;
}

int16_t  DJIMotorInstance::GetGivenCurrent()
{
	return motor_measure.given_current;
}
/**
	* @brief          ��ȡ���ָ��
  * @retval         null
  */
DJIMotorInstance *DJIMotorInstancePointer(uint8_t cnt)
{
	return MotorInstances[cnt];
}

/**
	* @brief          ����ʼ��״̬
  * @retval         null
  */
void DJIMotorInstance::MonitorInitState()
{
	switch(motor_settings->control_type)
	{
		case LADRC_FDW_CONTROL:
			if(!controller.ladrc_fdw.init_flag)
				MotorMonitorDisplay(MOTOR_PARAM); //���嶨�岻�ڴ˴�
			break;
		case LADRC_CONTROL:
			if(!controller.ladrc.init_flag)
				MotorMonitorDisplay(MOTOR_PARAM);
			break;
		case SINGLE_LOOP:
			if(!controller.speed_PID.init_flag )
				MotorMonitorDisplay(MOTOR_PARAM);
			break;
		case CASCADE_LOOP:
			if(!controller.angle_PID.init_flag || !controller.speed_PID.init_flag )
				MotorMonitorDisplay(MOTOR_PARAM);
			break;
		case OPEN_LOOP:
			break;
	}
}

/**
	* @brief          ������¶�
  * @retval         null
  */
void DJIMotorInstance::MonitorTem()
{
	if(motor_measure.temperate > 48.0f)
		MotorMonitorDisplay(MOTOR_TEM);
}

/**
	* @brief          ������Ƿ�����
  * @retval         null
  */
void DJIMotorInstance::MonitorOnlineState()
{
	if(!motor_watch.UpdateDeviceStatus())
		MotorMonitorDisplay(MOTOR_TEM);
}

/**
	* @brief          ���е�����
  * @retval         null
  */
void MonitorMotor()
{
	for (int i = 0; i < TOTAL_MOTOR_SUM; i++) 
	{
		if (MotorInstances[i] != NULL) 
		{
			MotorInstances[i]->MonitorInitState();
			MotorInstances[i]->MonitorTem();
			MotorInstances[i]->MonitorOnlineState();
		}else
		{
			break;
		}
	}
}
