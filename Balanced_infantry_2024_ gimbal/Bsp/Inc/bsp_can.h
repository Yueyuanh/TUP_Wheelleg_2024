#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "can.h"
#include "struct_typedef.h"

#define MAX_CAN_NUM 2
#define MAX_REGISTER_NUM 8
#ifdef __cplusplus
extern "C"{
#endif

#ifdef __cplusplus
	
#endif
//CAN����������
typedef enum 
{
	ON_CAN1, ON_CAN2,
}CAN_NAME_e;

//CAN�߽�����ṹ��
typedef struct _
{
	CAN_NAME_e can_id;
	uint32_t rx_id;
	uint8_t rx_buff[8];     // ���ջ���,�����Ϣ����Ϊ8
	void (*CANMsgCallback)(struct _ *); // ���Ӷ������
	void *module_address;
}CAN_Rx_Instance_t;

//CAN�߽��յĳ�ʼ������
typedef void ModuleCallback(CAN_Rx_Instance_t *rx_instance);
void CANRxInitSet(CAN_Rx_Instance_t *input_init,CAN_NAME_e can_num,uint32_t module_rx_id,void *module_address,ModuleCallback *DecodeCallback);
CAN_Rx_Instance_t *CANRxRegister(CAN_Rx_Instance_t *input_rx);

//CAN�߷��͵Ľṹ��
typedef struct
{
	CAN_HandleTypeDef *can_handle;
	CAN_TxHeaderTypeDef tx_message_data;
	int16_t *set_current[4];
}CANTxInstance_t;

void CANTxRegister(CANTxInstance_t *input_tx,CAN_Rx_Instance_t *input_rx);
void CANFilterInit(void);

extern void (*CANMonitor)(uint8_t type);

//���ͺ���
void CANSendToMotor(CANTxInstance_t *tx_message,int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);
void CANSend16Message(CANTxInstance_t *tx_message,uint16_t message1,uint16_t message2,uint16_t message3,uint16_t message4);
void CANSendFpMessage(CANTxInstance_t *tx_message,fp32 *message1,fp32 *message2);
void CANSendU8Message(CANTxInstance_t *tx_message,uint8_t message[8]);

#ifdef __cplusplus
}
#endif

#endif
