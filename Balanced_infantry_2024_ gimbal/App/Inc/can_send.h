#ifndef __CAN_SEND_H
#define __CAN_SEND_H

#include "message_center.h"
#include "motor.h"
#ifdef __cplusplus
extern "C"{
#endif

//最多可交互信息数
#define MAX_CAN_COMMU_NUM 6
#define MAX_UINT8_GROUP 1
#define MAX_UINT16_GROUP 2
#define MAX_FP32_GROUP 2
	
#ifdef __cplusplus
enum CAN_CommuInfo_Type_e
{
	ALL_UINT8, ALL_FP32 , ALL_UINT16,
};

//CAN线接收数据类结构体 
typedef struct
{
	CAN_HandleTypeDef *can_handle;
	CAN_TxHeaderTypeDef tx_message_data;
	uint8_t *uint8_msg;
	uint16_t *uint16_msg;
	fp32 *fp32_msg;
}CANCommTxInstance_t;

typedef struct
{
	//发送和接收的stdid设置
	CAN_Rx_Instance_t commu_can;
}CANCommRxInstance_t;

void MotorSend();
void Uint8Send();
void Uint16Send();
void Float32Send();

#endif

void DataAddressInit(void);
void MotorSendTask(void);
void OthersSendTask(void);
void YawSendTask(void);
void GimbalToChassisTask(void);

#ifdef __cplusplus
}
#endif

#endif
