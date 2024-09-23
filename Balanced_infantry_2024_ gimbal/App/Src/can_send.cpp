/**
 ******************************************************************************
 * @file    can_send.cpp
 * @author  Xushuang
 * @version V1.0.0 Xushuang ������� 2023/9/21
						V2.0.0 Xushuang ���Ŀ�� 2023/10/28
						V2.1.0 Xushuang ����DJIMotorAddrInit 2023/12/7
 * @date    2023/10/28
 * @brief		�˴�ΪCAN����Ϣ���ʹ�
 ******************************************************************************
 * @attention
 *	��������������������������������������������������������������
 *	CANCommTxInstance_t��صĺ�����д����ֹ������࣬û�б�Ҫ��ֱ�ӵ���
 *	bsp_can�еķ��ͺ�������ʵ�֣�����δʹ���������
 *
 *	CANTxInstance[]�½�ʵ����һ�������ʹ�ô󽮵�����˴������޸�
 *	����˫����볬���ģ������Ϣ��������ʱ�ڴ˴���Ӧ�����ڽ�������CAN�߷��ͺ�������
 ******************************************************************************
 */
#include "can_send.h"
#include "cmsis_os.h"
#include "string.h"
#include "system.h"
static int16_t zero_current = 0;

/*��ĳ������Э�飨1���Ƿ񿪳���     ��2������ģʽ        ��   3�����������߰�λ �� 4�����������Ͱ�λ ��
 *					5�����������߰�λ ��6�����������Ͱ�λ����   7���������޸߰�λ �� 8���������޵Ͱ�λ��
 */					

//CAN�ߵ��ʵ��
CANTxInstance_t CANTxInstance[] = 
{
	//������ ��ʶ��
	{&hcan1,{0x200,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},  //0
	{&hcan1,{0x1FF,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},	 //1
	{&hcan1,{0x2FF,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},	 //2

	//add more		//ƽ����̨����ͨ��

	{&hcan2,{0x301,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},	 //3
	{&hcan2,{0x302,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},	 //4
	{&hcan2,{0x303,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},	 //5
	{&hcan2,{0x304,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},	 //6
	{&hcan2,{0x305,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},	 //7
	{&hcan2,{0x306,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},	 //8
	{&hcan2,{0x307,0,CAN_ID_STD,CAN_RTR_DATA,0x08},{&zero_current}},	 //9

};



//δ��
//CAN����Ϣ��������ʵ��
CANCommTxInstance_t CANCommuInstance[] = 
{
	//ԭ��ɫ���緢�Ϳ���Э��
	{&hcan1,{0x210,0,CAN_ID_STD,CAN_RTR_DATA,0x08},NULL,NULL,NULL},	 //0
	//��ĳ��緢�Ϳ���Э��
	{&hcan1,{0x100,0,CAN_ID_STD,CAN_RTR_DATA,0x08},NULL,NULL,NULL},	 //1
	//˫��ͨ��
	{&hcan1,{0x301,0,CAN_ID_STD,CAN_RTR_DATA,0x08},NULL,NULL,NULL},	 //2
	{&hcan1,{0x1FF,0,CAN_ID_STD,CAN_RTR_DATA,0x08},NULL,NULL,NULL},	 //3
};

//δ��
uint8_t *send_uint8_msg[MAX_UINT8_GROUP][8] = {NULL};
uint16_t *send_uint16_msg[MAX_UINT16_GROUP][4] = {NULL};
fp32 *send_fp32_msg[MAX_FP32_GROUP][2] = {NULL};
//δ��
//CAN������Ϣ����
void CANCommuAddrInit(CAN_CommuInfo_Type_e commu_type,uint32_t std_id,void *commu_msg)
{
	uint8_t num = 0;
	//�ҵ���Ӧ��ӵ�STDID
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
	* @brief          ������͵�ַ��ʼ��
  * @param[in]      Null
  * @retval         Null
  */
void DJIMotorAddrInit()
{
	for (int i = 0; i < TOTAL_MOTOR_SUM; i++) {
		//�ж��Ƿ��е�ַ
		if(DJIMotorInstancePointer(i) == NULL)
			break;
		Motor_Type_e motorType = DJIMotorInstancePointer(i)->SetParamPointer()->motor_type;
    uint8_t setID = DJIMotorInstancePointer(i)->SetParamPointer()->set_id;
    uint8_t num;
		uint32_t targetStdId;
		
		//�����޵��ʱֱ�ӽ�����һ��ѭ��
		if(motorType == NO_MOTOR)
			continue;
		
		//����id���з���
		if(setID >= 1 && setID <= 4){
			num = setID - 1;
			targetStdId = (motorType == GM6020) ? 0x1FF : 0x200;
		}else if(setID >= 5 && setID <= 7){
			num = setID - 5;
			targetStdId = (motorType == GM6020) ? 0x2FF : 0x1FF;
		}

		//Ѱ��Ŀ��Stdid���ҷ�������id��Ӧ���Ͳ��ֵ�������
    for (int j = 0; j < 10; j++) {
			if (CANTxInstance[j].tx_message_data.StdId == targetStdId &&
					CANTxInstance[j].can_handle == ((DJIMotorInstancePointer(i)->GetCANID() == ON_CAN1) ? &hcan1 : &hcan2)) {
        CANTxInstance[j].set_current[num] = DJIMotorInstancePointer(i)->SendCurrentPointer();
        break;
      }
    }
  }
	//��ʵ����0
	for(int u = 0; u<=10; u++) {
		for(int num = 0; num<=3; num++){
			if(CANTxInstance[u].set_current[num] == NULL)
				CANTxInstance[u].set_current[num] = &zero_current;
		}
	}
}

/**
  * @brief          �������ݵ�ַ��ȡ
  * @param[in]      Null
  * @retval         Null
  */
uint8_t state[8];
void DataAddressInit()
{
	//�������ֵ��ȡ
	DJIMotorAddrInit();
	
}

/**
  * @brief          �����Ϣ����
  * @param[in]      Null
  * @retval         Null
  */

int current_set=0;
void MotorSendTask()
{
	//CAN1 ID:1-4
	//fir1 fir2 stir3
	CANSendToMotor(&CANTxInstance[0],*CANTxInstance[0].set_current[0],*CANTxInstance[0].set_current[1],*CANTxInstance[0].set_current[2],*CANTxInstance[0].set_current[3]);

		//��תĦ����
	//CANSendToMotor(&CANTxInstance[0],current_set,-current_set,*CANTxInstance[0].set_current[2],*CANTxInstance[0].set_current[3]);

	//CAN1 ID:5-8 ��̨��� yaw 5 pitch 6
	CANSendToMotor(&CANTxInstance[1],*CANTxInstance[1].set_current[0],*CANTxInstance[1].set_current[1],*CANTxInstance[1].set_current[2],*CANTxInstance[1].set_current[3]);

	//������Ϣ
	CANSendFpMessage(&CANTxInstance[3],&ChassisPointer()->chassis_x_speed_set,&ChassisPointer()->chassis_y_speed_set);//301


}

/**
  * @brief          ������Ϣ����
  * @param[in]      Null
  * @retval         Null
  */
void OthersSendTask()
{

}

/**
  * @brief          ������Ϣ����
  * @param[in]      Null
  * @retval         Null
  */
void YawSendTask()
{
}

/**
  * @brief          ��ָ̨����Ϣ����
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