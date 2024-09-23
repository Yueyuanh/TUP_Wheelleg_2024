/**
 ******************************************************************************
 * @file    motor.cpp
 * @author  Xushuang
 * @version V1.0.0 Xushuang 基本完成 2023/8/22
 *					V2.0.0 Xushuang 修改CAN部分解码 2023/10/30
 * @date    2023/10/30
 * @brief		此处为电机类
 *					大疆电机以及AM3508
 ******************************************************************************
 * @attention
 *			包含了电机数据读取的协议，集合了PID控制器和LADRC，可在初始化时自行选择电
 *	机类型等信息以及采取的控制器，此电机设备挂载在CAN线上，可根据两行代码，多加
 *  电机时无需再到CAN线接收回调函数中增添新的case，其设备信息会在CAN线自动登记，
 *  仅调用DJIMotorInit(**)函数后就可正常读取电机数据和控制
 ******************************************************************************
 */
#include "motor.h"
#include "bsp_dwt.h"

//M2006,M3508:接收均为0x200+ID，发送1-4为0x200，5-8为0x1FF
//GM6020:接收为0x204+ID,发送1-4为0x1FF，5-7为0x2FF
//AM3508:输出轴绝对角度接收w为0x300+ID


////需添加控制器类型检测
//电机数据读取
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

//MK20获取输出轴绝对角度
#define AM3508_output_absolute_angle(ptr, data)                              												   \
{                                                                  																 \
  (ptr)->output_encoder_num = (int32_t)((data)[0] << 24 | (data)[1] << 16 | (data)[2] << 8 | (data)[3]);   \
  (ptr)->output_encoder_ecd = (uint16_t)((data)[4] << 8 | (data)[5]);      														 \
}	

//保存实例地址
DJIMotorInstance *MotorInstances[TOTAL_MOTOR_SUM];
//注册电机个数
uint8_t dji_motor_num = 0;

/**
	* @brief          返回相对角度
	* @param[in]     	*ecd_angle：总累积角度值
	* @param[in]     	offset_ecd：中值编码
	* @param[in]     	drive_radio：传动比
  * @retval         fp32：输出角度
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
	* @brief          限制电机相对角度运动空间
	* @param[in]     	relative_angle_set：相对角度设定值
	* @param[in]     	add：添加值
	* @param[in]     	max_limit：最大角度
	* @param[in]     	min_limit：最小角度
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
	* @brief          绝对角度时对add值的限幅限制在最大最小相对角度限制内
	* @param[in]     	error_angle：误差角度
	* @param[in]     	add：添加值
	* @param[in]     	relative_angle：相对角度
	* @param[in]     	max_limit：最大角度
	* @param[in]     	min_limit：最小角度
  * @retval         Null
  */
fp32 AbsoluteControlAddLimit(fp32 error_angle,fp32 add,fp32 relative_angle,fp32 max_limit,fp32 min_limit)
{
	static fp32 bias_angle;
	//当前控制角度
	bias_angle = rad_format(error_angle);
	//relative angle + angle error + add_angle > max_relative angle
	//云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
    if (relative_angle + bias_angle + add > max_limit)
	{
		//calculate max add_angle
    //计算出一个最大的添加角度
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
	* @brief          电机类构造函数
	* @param[in]     	Null
  * @retval         Null
  */
DJIMotorInstance::DJIMotorInstance()
{
  motor_watch.error_code = NO_MOTOR_ERROR;
  motor_settings = NULL;
  //一系列地址存储
  // 将当前实例的指针添加到数组中
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
	* @brief          大疆电机数据解码
  * @param[in]      *rx_instance：接收的实例
  * @retval         Null
  */
static void DecodeDJIMotor(CAN_Rx_Instance_t *rx_instance)
{
	uint8_t *rxbuff = rx_instance->rx_buff;
	//address是void*需强制转化
	DJIMotorInstance *motor = (DJIMotorInstance *)rx_instance->module_address;
	Dji_Motor_Measure_t *measure = &motor->motor_measure;
	get_basic_motor_measure(measure,rxbuff);
}

static void DecodeExtraAM3508Data(CAN_Rx_Instance_t *rx_instance)
{
	uint8_t *rxbuff = rx_instance->rx_buff;
	//address是void*需强制转化
	DJIMotorInstance *motor = (DJIMotorInstance *)rx_instance->module_address;
	Dji_Motor_Measure_t *measure = &motor->motor_measure;
	AM3508_output_absolute_angle(measure,rxbuff);
}

/**
	* @brief          大疆电机示例初始化
  * @param[in]      input_type：电机类型
	* @param[in]      can_num：CAN线挂载的ID
	* @param[in]      input_id：电调ID号设置
	* @param[in]      input_control_type：控制方式
  * @retval         Null
  */
void DJIMotorInstance::DJIMotorInit(Motor_Setting_t *config)
{
	fp32 max_out;
	uint32_t rx_id,am_rx_id;
	//检查一下有没有问题
	//ID错误不在范围内ErrorCode
	if(config->set_id <= 0 || config->set_id > 8)
		motor_watch.error_code =  ID_RANGE_ERROR;
	
	motor_settings = config;
	//初始化电机设置
	switch(motor_settings->motor_type)
	{
		case M3508:
			max_out = 16384;
			rx_id = 0x200 + motor_settings->set_id;//id为1-8
			break;
		case AM3508:
			max_out = 16384;
			rx_id = 0x200 + motor_settings->set_id;//id为1-8
			am_rx_id = 0x300 + motor_settings->set_id; //0x300+电调号
			CANRxInitSet(&motor_settings->AMExtra_can,motor_settings->can_id,am_rx_id,this,DecodeExtraAM3508Data);
			CANRxRegister(&motor_settings->AMExtra_can);
			break;
		case GM6020:
			max_out = 30000;
			rx_id = 0x204 + motor_settings->set_id;//id为1-7
			break;
		case M2006:
			max_out = 10000;
			rx_id = 0x200 + motor_settings->set_id;//id为1-8
			break;
		default:
			//ErrorCode
			motor_watch.error_code = MOTOR_TYPE_ERROR;
			break;
	}
	//进行CAN线登记注册
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
	
	//减速比设定--可直接给enum赋值
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
	* @brief          改变控制模式
  * @param[in]      change_type：需改变的控制模式
  * @retval         Null
  */
void DJIMotorInstance::ChangeControlMode(Control_Type_e change_type)
{
	motor_settings->control_type = change_type;
}

/**
	* @brief          电机位置设置（未完善，但可以使用）
	* @param[in]      *flag：开始变化的标志位
  * @param[in]      *set_positon：设置位置
  * @param[in]      add_position：累加位置值
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
	* @brief          返回电机设置接收的id
  * @param[in]      NUll
	* @retval         motor_rx_id：接收id
  */
uint32_t DJIMotorInstance::GetRxID()
{
	return motor_settings->motor_can.rx_id;
}

/**
	* @brief          返回设置CAN线的id
  * @param[in]      NUll
	* @retval         can_id：can线号
  */
CAN_NAME_e DJIMotorInstance::GetCANID()
{
	return motor_settings->motor_can.can_id;
}

/**
	* @brief          检查是否重复注册id
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

//考虑滤波信号标志是否需要删除
/**
	* @brief          大疆电机控制
  * @param[in]      *ref 获取当前值地址
	* @param[in]      *set 获取设定值地址
	* @param[in]      *motor_gyro 获取角速度值地址
  * @retval         输出值
  */
void DJIMotorInstance::DJIMotorControl(fp32 *ref,fp32 *set,fp32 *motor_gyro,uint8_t filter_flag)
{
	fp32 output;
	motor_watch.RecordStartTime(); //开始计时
	//在这赋值仅为方便调试观察
	controller.target_value = *set;//*motor_settings->direction;  //此处负号时仍在测试中
	controller.now_value = *ref;
	switch(motor_settings->control_type)
	{
		case LADRC_FDW_CONTROL:
			//错误处理
			if(motor_gyro == NULL)
			{
				motor_watch.error_code = INPUT_PARAM_ERROR;
			}else
			{
				output=controller.ladrc_fdw.FDW_Calc(controller.now_value,controller.target_value,*motor_gyro);
			}
			break;
		case LADRC_CONTROL:
			//错误处理
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
	motor_watch.CalcExcutePeriod(); //结束计时
}

/**
	* @brief          电机PID控制函数
	*                 双环时：ref：设定角度 set：目标角度 filter_flag：是否需要滤波
  *                 单环时：ref：不需要		set：目标速度 filter_flag：是否需要滤波
  * @param[in]      *ref 获取当前值地址
	* @param[in]      *set 获取设定值地址
	* @param[in]      *filter_flag: 是否需要滤波标志
  * @retval         输出值
  */
fp32 DJIMotorInstance::PIDControl(fp32 *ref,fp32 *set,uint8_t filter_flag)
{
	fp32 angle_out,speed_set,pid_out;
	//速度单环时直接赋值设定值地址
	speed_set = *set * motor_settings->direction;
	//角度环
	if(motor_settings->control_type == CASCADE_LOOP )
	{
		angle_out	=	controller.angle_PID.Calc(*ref,*set);
		speed_set = angle_out;
	}
	//单环时直接进入
	//速度反馈是否需要滤波
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
	* @brief          电机无力模式设置
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

//好像有点多余---后续可以考虑写一个滤波类
/**
	* @brief          电机滤波速度更新函数
	* @param[in]      *filter_num: 滤波数组
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
		//二阶低通滤波
		speed_fliter_1 = speed_fliter_2;
		speed_fliter_2 = speed_fliter_3;
		speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (motor_measure.speed_rpm * RPM_TO_RAD_S * motor_measure.reduction_ratio) * fliter_num[2];
		controller.pid_speed = speed_fliter_3;
		return speed_fliter_3;
	}
}

/**
	* @brief          获取电机角速度的API
	* @param[in]      NULL
  * @retval         fp32
  */
fp32 DJIMotorInstance::GetRotorW() 
{
	return motor_measure.speed_rpm * RPM_TO_RAD_S;
}

/**
	* @brief          获得输出轴角速度的API
	* @param[in]      NULL
  * @retval         fp32
  */
fp32 DJIMotorInstance::GetOutputShaftW() 
{
	return motor_measure.speed_rpm * motor_measure.reduction_ratio * RPM_TO_RAD_S;
}

/**
	* @brief          获得转子的累积角度的API
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
	* @brief          获得输出轴的累积角度的API
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
	* @brief          获得发送电流的API
	* @param[in]      NULL
  * @retval         fp32
  */
int16_t *DJIMotorInstance::SendCurrentPointer()
{
	return &controller.send_current;
}

/**
	* @brief          获得转子转速的API
	* @param[in]      NULL
  * @retval         fp32
  */
int16_t DJIMotorInstance::GetRotorRpm()
{
	return motor_measure.speed_rpm;
}

/**
	* @brief          获得转子反馈信息的API
	* @param[in]      NULL
  * @retval         fp32
  */
Dji_Motor_Measure_t *DJIMotorInstance::GetMotorMeasure()
{
	return &motor_measure;
}

/**
	* @brief          获得电机设置参数的API
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
	* @brief          获取电机指针
  * @retval         null
  */
DJIMotorInstance *DJIMotorInstancePointer(uint8_t cnt)
{
	return MotorInstances[cnt];
}

/**
	* @brief          监测初始化状态
  * @retval         null
  */
void DJIMotorInstance::MonitorInitState()
{
	switch(motor_settings->control_type)
	{
		case LADRC_FDW_CONTROL:
			if(!controller.ladrc_fdw.init_flag)
				MotorMonitorDisplay(MOTOR_PARAM); //具体定义不在此处
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
	* @brief          监测电机温度
  * @retval         null
  */
void DJIMotorInstance::MonitorTem()
{
	if(motor_measure.temperate > 48.0f)
		MotorMonitorDisplay(MOTOR_TEM);
}

/**
	* @brief          监测电机是否在线
  * @retval         null
  */
void DJIMotorInstance::MonitorOnlineState()
{
	if(!motor_watch.UpdateDeviceStatus())
		MotorMonitorDisplay(MOTOR_TEM);
}

/**
	* @brief          所有电机监测
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
