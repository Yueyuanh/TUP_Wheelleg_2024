/**
 ******************************************************************************
 * @file    can_send.cpp
 * @author  Xushuang
 * @version V1.0.0 Xushuang 基本完成 2023/9/21
						V2.0.0 Xushuang 更改框架 2023/10/28
						V2.1.0 Xushuang 更改DJIMotorAddrInit 2023/12/7
 * @date    2023/10/28
 * @brief		此处为CAN线信息发送处
 ******************************************************************************
 * @attention
 *	！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
 *	CANCommTxInstance_t相关的函数因写完后发现过于冗余，没有必要，直接调用
 *	bsp_can中的发送函数即可实现，故暂未使用相关内容
 *
 *	CANTxInstance[]新建实例，一般如果仅使用大疆电机，此处无需修改
 *	如有双板或与超电等模块有信息交换需求时在此处对应函数内进行新增CAN线发送函数即可
 ******************************************************************************
 */
#include "can_send.h"
#include "cmsis_os.h"
#include "string.h"
#include "system.h"
static int16_t zero_current = 0;

/*浙纺超电控制协议（1：是否开超电     ；2：控制模式        ；   3：缓冲能量高八位 ； 4：缓冲能量低八位 ；
 *					5：缓冲能量高八位 ；6：缓冲能量低八位）；   7：功率上限高八位 ； 8：功率上限低八位）
 */					

//CAN线电机实例
CANTxInstance_t CANTxInstance[] = 
{
	//四组电机 标识符
	{&hcan1,{0x200,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},  //0
	{&hcan1,{0x1FF,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},	 //1
	{&hcan1,{0x2FF,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},	 //2

	//add more		//平衡云台底盘通信

	{&hcan2,{0x301,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},	 //3
	{&hcan2,{0x302,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},	 //4
	{&hcan2,{0x303,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},	 //5
	{&hcan2,{0x304,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},	 //6
	{&hcan2,{0x305,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},	 //7
	{&hcan2,{0x306,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},	 //8
	{&hcan2,{0x307,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},	 //9

};



//未用
//CAN线信息交流发送实例
CANCommTxInstance_t CANCommuInstance[] = 
{
	//原紫色超电发送控制协议
	{&hcan1,{0x210,0,CAN_ID_STD,CAN_RTR_DATA,0x08},NULL,NULL,NULL},	 //0
	//浙纺超电发送控制协议
	{&hcan1,{0x100,0,CAN_ID_STD,CAN_RTR_DATA,0x08},NULL,NULL,NULL},	 //1
	//双板通信
	{&hcan1,{0x301,0,CAN_ID_STD,CAN_RTR_DATA,0x08},NULL,NULL,NULL},	 //2
	{&hcan1,{0x1FF,0,CAN_ID_STD,CAN_RTR_DATA,0x08},NULL,NULL,NULL},	 //3
};

//未用
uint8_t *send_uint8_msg[MAX_UINT8_GROUP][8] = {NULL};
uint16_t *send_uint16_msg[MAX_UINT16_GROUP][4] = {NULL};
fp32 *send_fp32_msg[MAX_FP32_GROUP][2] = {NULL};
//未用
//CAN交互信息发送
void CANCommuAddrInit(CAN_CommuInfo_Type_e commu_type,uint32_t std_id,void *commu_msg)
{
	uint8_t num = 0;
	//找到对应添加的STDID
	for(int i = 0; i < MAX_CAN_COMMU_NUM; i++){
		if(CANCommuInstance[i].tx_message_data.StdId == std_id){
			num = i;
		}
	}
	switch(commu_type){
		case ALL_UINT8:
			CANCommuInstance[num].uint8_msg = (uint8_t*)commu_msg;
			break;
		case ALL_FP32:
			CANCommuInstance[num].fp32_msg = (fp32*)commu_msg;
			break;
		case ALL_UINT16:
			CANCommuInstance[num].uint16_msg = (uint16_t*)commu_msg;
			break;
		default:
			break;
	}
}






/**
	* @brief          电机发送地址初始化
  * @param[in]      Null
  * @retval         Null
  */
void DJIMotorAddrInit()
{
	for (int i = 0; i < TOTAL_MOTOR_SUM; i++) {
		//判断是否有地址
		if(DJIMotorInstancePointer(i) == NULL)
			break;
		Motor_Type_e motorType = DJIMotorInstancePointer(i)->SetParamPointer()->motor_type;
    uint8_t setID = DJIMotorInstancePointer(i)->SetParamPointer()->set_id;
    uint8_t num;
		uint32_t targetStdId;
		
		//设置无电机时直接进入下一次循环
		if(motorType == NO_MOTOR)
			continue;
		
		//根据id进行分组
		if(setID >= 1 && setID <= 4){
			num = setID - 1;
			targetStdId = (motorType == GM6020) ? 0x1FF : 0x200;
		}else if(setID >= 5 && setID <= 7){
			num = setID - 5;
			targetStdId = (motorType == GM6020) ? 0x2FF : 0x1FF;
		}

		//寻找目标Stdid并且放入其电机id对应发送部分的数组中
    for (int j = 0; j < 10; j++) {
			if (CANTxInstance[j].tx_message_data.StdId == targetStdId &&
					CANTxInstance[j].can_handle == ((DJIMotorInstancePointer(i)->GetCANID() == ON_CAN1) ? &hcan1 : &hcan2)) {
        CANTxInstance[j].set_current[num] = DJIMotorInstancePointer(i)->SendCurrentPointer();
        break;
      }
    }
  }
	//无实例则赋0
	for(int u = 0; u<=10; u++) {
		for(int num = 0; num<=3; num++){
			if(CANTxInstance[u].set_current[num] == NULL)
				CANTxInstance[u].set_current[num] = &zero_current;
		}
	}
}

/**
  * @brief          发送数据地址获取
  * @param[in]      Null
  * @retval         Null
  */
uint8_t state[8];
void DataAddressInit()
{
	//电机电流值获取
	DJIMotorAddrInit();
	
}

/**
  * @brief          电机信息发送
  * @param[in]      Null
  * @retval         Null
  */

int current_set=0;
void MotorSendTask()
{
	//CAN1 ID:1-4
	//fir1 fir2 stir3
	CANSendToMotor(&CANTxInstance[0],*CANTxInstance[0].set_current[0],*CANTxInstance[0].set_current[1],*CANTxInstance[0].set_current[2],*CANTxInstance[0].set_current[3]);

		//不转摩擦轮
	//CANSendToMotor(&CANTxInstance[0],current_set,-current_set,*CANTxInstance[0].set_current[2],*CANTxInstance[0].set_current[3]);

	//CAN1 ID:5-8 云台电机 yaw 5 pitch 6
	CANSendToMotor(&CANTxInstance[1],*CANTxInstance[1].set_current[0],*CANTxInstance[1].set_current[1],*CANTxInstance[1].set_current[2],*CANTxInstance[1].set_current[3]);

	//底盘信息
	CANSendFpMessage(&CANTxInstance[3],&ChassisPointer()->chassis_x_speed_set,&ChassisPointer()->chassis_y_speed_set);//301


}

/**
  * @brief          其他信息发送
  * @param[in]      Null
  * @retval         Null
  */
void OthersSendTask()
{

}

/**
  * @brief          其他信息发送
  * @param[in]      Null
  * @retval         Null
  */
void YawSendTask()
{
}

/**
  * @brief          云台指令信息发送
  * @param[in]      Null
  * @retval         Null
  */
void GimbalToChassisTask()
{

  	CANSendFpMessage(&CANTxInstance[4],&ChassisPointer()->chassis_z_angle_set,&ChassisPointer()->chassis_z_num);//302

		uint8_t chassis_flag_bag[8]={ ChassisPointer()->chassis_init,
																	ChassisPointer()->leg_state,
																	ChassisPointer()->restart,
																	ChassisPointer()->lying_flag,

																	ChassisPointer()->sideway_flag,
																	ChassisPointer()->spin_flag,
																	ChassisPointer()->jump_flag,//jump_flag
																	ChassisPointer()->chassis_power_limit
																};

		CANSendU8Message(&CANTxInstance[5],chassis_flag_bag);//303
		//CANSendFpMessage(&CANTxInstance[6],&ChassisPointer()->chassis_real_power,&ChassisPointer()->chassis_surplus_energy);//304



}