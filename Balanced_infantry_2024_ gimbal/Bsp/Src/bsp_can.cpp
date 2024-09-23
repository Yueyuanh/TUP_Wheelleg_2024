/**
 ******************************************************************************
 * @file    bsp_can.cpp
 * @author  Xushuang
 * @version V1.0.0 Xushuang ������� 2023/8/25
 *          V2.0.0 Xushuang �ع����շ��� 2023/10/28
 * @date    2023/10/28
 * @brief		CAN��֧�ְ������������ã�CAN���ͺ�����CAN���ջص�����
 *					CAN���ջص��������岻���ڴ˴��޸ģ��������ļ���ֹ�޸�
 ******************************************************************************
 * @attention
 *	������µ��������CAN���ϵ��豸ʱ������Ҫ��������ʵ�֣������ڽ���������µ�case��
 *	��������չ�����豸����ʱ�������ڸ��豸�����н������²������ɣ�����
 *
 *	��һ��������Ҫ�Ľṹ������м���CAN_Rx_Instance_t*���CAN_Rx_Init_t�ࡣ
 * 	�ڶ���������CANRxInitSet()������CAN_Rx_Init_t����г�ʼ����
 *	������������CANRxRegister()��������ע��ǼǺͳ�ʼ�����á�
 *
 *	�ر�ע�⣺�趨��һ��static void Decode***(CAN_Rx_Instance_t *rx_instance)���͵�
 *	��Ϣ���뺯��������CANRxInitSet()����Ϊ�βδ���
 *
 *	������motor���У��轫���ע�ᵽCAN����
 *	1.class DJIMotorInstance
 *	{
 *		public:
 *			CAN_Rx_Instance_t *motor_can;				//���CANʵ��ָ��
 *			CAN_Rx_Init_t motor_can_init;				//���CANʵ����ʼ��
 *	}
 *	2.void DJIMotorInstance::DJIMotorInit()
 *	{
 *		CANRxInitSet(&motor_can_init,can_num,motor_rx_id,motor_address,DecodeDJIMotor);
 *		motor_can = CANRxRegister(&motor_can_init);
 *	}
 ******************************************************************************
 */
 //CAN����Ϣ���ջ���ͨ�ŵ�˵��δд
#include "bsp_can.h"
#include "string.h"

//CAN���߽���ָ�����飬���ڴ洢��������
CAN_Rx_Instance_t *rx_instance[MAX_CAN_NUM][MAX_REGISTER_NUM] = {NULL};
//��¼CAN1��CAN2�ϵĹ����豸����
uint8_t can1_register_id,can2_register_id;

/**
  * @brief          CAN�߹�������ʼ��
  * @param[in]      Null
  * @retval         Null
  */
void CANFilterInit(void)
{
		//�˲��ֺ��������Ż�
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}

/**
	* @brief          CAN�߷��͵�����ֵ
	* @param[in]      CANTxInstance_t *�����ͽṹ��
	* @param[in]      motor1��������Ϣ
	* @param[in]      motor2��������Ϣ
	* @param[in]      motor3��������Ϣ
	* @param[in]      motor4��������Ϣ
  * @retval         Null
  */
void CANSendToMotor(CANTxInstance_t *tx_message,int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4)
{
	uint32_t send_mail_box;
	uint8_t tx_buffer[8];
	
	tx_buffer[0] =	motor1 >> 8;
	tx_buffer[1] =	motor1 ;
	tx_buffer[2] =	motor2 >> 8;
	tx_buffer[3] =	motor2 ;
	tx_buffer[4] =	motor3 >> 8;
	tx_buffer[5] =	motor3 ;
	tx_buffer[6] =	motor4 >> 8;
	tx_buffer[7] =	motor4 ;
	
	HAL_CAN_AddTxMessage(tx_message->can_handle,&tx_message->tx_message_data,tx_buffer,&send_mail_box);
}

/**
	* @brief          CAN�߷���uint16�����ݽ���ͨ��
	* @param[in]      CANTxInstance_t *�����ͽṹ��
	* @param[in]      message1��������Ϣ
	* @param[in]      message2��������Ϣ
	* @param[in]      message3��������Ϣ
	* @param[in]      message4��������Ϣ
  * @retval         Null
  */
void CANSend16Message(CANTxInstance_t *tx_message,uint16_t message1,uint16_t message2,uint16_t message3,uint16_t message4)
{
	uint32_t send_mail_box;
	uint8_t tx_buffer[8];
	
	tx_buffer[0] =	message1 >> 8;
	tx_buffer[1] =	message1 ;
	tx_buffer[2] =	message2 >> 8;
	tx_buffer[3] =	message2 ;
	tx_buffer[4] =	message3 >> 8;
	tx_buffer[5] =	message3 ;
	tx_buffer[6] =	message4 >> 8;
	tx_buffer[7] =	message4 ;
	
	HAL_CAN_AddTxMessage(tx_message->can_handle,&tx_message->tx_message_data,tx_buffer,&send_mail_box);
}

/**
	* @brief          CAN�߷���float������ͨ��
	* @param[in]      CANTxInstance_t *�����ͽṹ��
	* @param[in]      *message1��������Ϣ
	* @param[in]      *message2��������Ϣ
  * @retval         Null
  */
void CANSendFpMessage(CANTxInstance_t *tx_message,fp32 *message1,fp32 *message2)
{
	uint32_t send_mail_box;
	uint16_t i;
	uint8_t tx_buffer[8];
	Un1 in_un1,in_un2; 
	
	in_un1.f = *message1;
	in_un2.f = *message2;
	for(i=0;i<4;i++)
	{
		tx_buffer[i] = 	in_un1.s[i];
		tx_buffer[i+4] = in_un2.s[i];
	}
	
	HAL_CAN_AddTxMessage(tx_message->can_handle,&tx_message->tx_message_data,tx_buffer,&send_mail_box);
}

/**
	* @brief          ����uint8_t��������
  * @param[in]      *message[8]������ָ��
  * @retval         Null
  */
void CANSendU8Message(CANTxInstance_t *tx_message,uint8_t message[8])//
{
	uint32_t send_mail_box;
	uint16_t i;
	uint8_t tx_buffer[8];
	
	for(i=0;i<8;i++)
	{
		tx_buffer[i] = message[i];
	}
	
	HAL_CAN_AddTxMessage(tx_message->can_handle,&tx_message->tx_message_data,tx_buffer,&send_mail_box);
}

/**
	* @brief          RxInit�ĳ�ʼ������
  * @param[in]      *input_init����Ҫ��ʼ����CAN_Rx_Init_t�ṹ��
	* @param[in]      can_num������CAN�ߺ�
	* @param[in]      module_rx_id�����õ�ID
  * @param[in]      *module_address��ģ���ַ
	* @param[in]      *DecodeCallback�����뺯��
  * @retval         CAN_Rx_Instance_t*���½���CAN����ʵ����ַ
  */
void CANRxInitSet(CAN_Rx_Instance_t *input_init,CAN_NAME_e can_num,uint32_t module_rx_id,void *module_address,ModuleCallback *DecodeCallback)
{
	input_init->can_id = can_num;
	input_init->rx_id = module_rx_id;
	input_init->module_address = module_address;
	input_init->CANMsgCallback = DecodeCallback;
}

/**
	* @brief          CAN���߽���ʵ���ǼǺ���
  * @param[in]      *input_rx��CAN���ճ�ʼ���ṹ��
  * @retval         CAN_Rx_Instance_t*���½���CAN����ʵ����ַ
  */
CAN_Rx_Instance_t *CANRxRegister(CAN_Rx_Instance_t *input_rx)
{
	//��ʵ��ָ����е�ַ��¼-�Ǽ�
	if(input_rx->can_id == ON_CAN1){
		rx_instance[0][can1_register_id++] = input_rx;
	}else if(input_rx->can_id == ON_CAN2){
		rx_instance[1][can2_register_id++] = input_rx;
	}
	return input_rx;
}

/**
  * @brief          CAN1���ջص�����
  * @param[in]      CAN_RxHeaderTypeDef*�����սṹ��
  * @retval         (*buffer_data)[8]���洢��������
  */
void CAN1Callback(CAN_RxHeaderTypeDef *buffer_rx_header,uint8_t (*buffer_data)[8])
{
	for (size_t i = 0; i < can1_register_id; ++i){ // �������˵������Ҫ�ҵ�ʵ��
		if (buffer_rx_header->StdId == rx_instance[0][i]->rx_id){
			if (rx_instance[0][i]->CANMsgCallback != NULL) {// �ص�������Ϊ�վ͵���
				memcpy(rx_instance[0][i]->rx_buff, buffer_data, 8); // ��Ϣ��������Ӧʵ��
                rx_instance[0][i]->CANMsgCallback(rx_instance[0][i]);     // �����ص��������ݽ����ʹ���
            }
			return;
        }
    }
}

/**
  * @brief          CAN2���ջص�����
  * @param[in]      CAN_RxHeaderTypeDef*�����սṹ��
  * @retval         (*buffer_data)[8]���洢��������
  */
void CAN2Callback(CAN_RxHeaderTypeDef *buffer_rx_header,uint8_t (*buffer_data)[8])
{
	for (size_t i = 0; i < can2_register_id; ++i){ // �������˵������Ҫ�ҵ�ʵ��
		if (buffer_rx_header->StdId == rx_instance[1][i]->rx_id){
			if (rx_instance[1][i]->CANMsgCallback != NULL) {// �ص�������Ϊ�վ͵���
				memcpy(rx_instance[1][i]->rx_buff, buffer_data, 8); // ��Ϣ��������Ӧʵ��
				rx_instance[1][i]->CANMsgCallback(rx_instance[1][i]);     // �����ص��������ݽ����ʹ���
            }
            return;
        }
    }
}

/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         Null
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data1[8];
	uint8_t rx_data2[8];
	
	if(hcan->Instance == CAN1){
			HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data1);
			CAN1Callback(&rx_header,&rx_data1);
	}else if(hcan->Instance == CAN2){
			HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data2);
			CAN2Callback(&rx_header,&rx_data2);
	}
}

