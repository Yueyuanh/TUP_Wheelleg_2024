/**
 ******************************************************************************
 * @file    message_center.cpp
 * @author  Xushuang
 * @version V1.0.0 �������
 * @date    2023/8/24
 * @brief		�˴�Ϊ��Ϣ���ģ������Ҫ����Ľṹ��ʱ����Message_Center_t�жཨ��
 *					һ����Ӧ�ṹ��ָ��,����Ӵ������ͣ��ڸ�����Ҫ���񴦽���ʵ��֮����
 *          �ڸ������ʼ���׶������ָ���ʼ��
 ******************************************************************************
 * @attention
 *������������һ������ui.cpp������������������Ϣ�������������������õ�ui.cpp��
 *��Ϣ�谴���²������
 *1.Msg_Type_eö��������´�����������
 *2.�½���ṹ��
 * typedef struct
 * {
 * }Ui_Pub_Msg_t;
 *3.Message_Center_t������Ӵ�����ָ��(Ui_Pub_Msg_t *ui_msg;)
 *4.void Message_Center_t::PointerInit(void *msg,Msg_Type_e type)���������case
 *5.�ڶ�Ӧ������ѭ��ǰ���г�ʼ��(CenterPointer().PointerInit(&ui_msg,UIPUB);)
 ******************************************************************************
 */
#include "message_center.h"
#include "stdio.h"

Message_Center_t message_center;

/**
  * @brief          ��Ӷ�Ӧ�ṹ��ָ��
  * @param[in]      *msg����ָ��ͬ���ͽṹ���ָ��
	* @param[in]      Msg_Type_e: ��Ϣ����
  * @retval         MessageErrorCode��������Ϣ��
  */
MessageErrorCode Message_Center_t::PointerInit(void *msg,Msg_Type_e type)
{
	if(msg == NULL)
		return NULL_POINTER;
	switch(type)
	{
		case SYSPUB:
			sys_msg = (Sys_Pub_Msg_t*)msg;
			break;
		case GIMBALPUB:
			gimbal_msg = (Gimbal_Pub_Msg_t*)msg;
			break;
		case CHASSISPUB:
			chassis_msg = (Chassis_Pub_Msg_t*)msg;
			break;
		case REVOLVERPUB:
			revolver_msg = (Revolver_Pub_Msg_t*)msg;
			break;
		case UIPUB:
			ui_msg = (UI_Pub_Msg_t*)msg;
			break;
		case MONITORPUB:
			monitor_msg = (Monitor_Pub_Msg_t*)msg;
			break;
		case TOTALNUM:
		default:			break;	

			return INVALID_MSG_TYPE;
	}
	return NO_MSG_ERROR;
}

/**
  * @brief          �����Ϣ����ָ���Ƿ�Ϊ��
  * @retval         ���ֵ
  */
bool Message_Center_t::CheckPointerEmpty()
{
	if(sys_msg == NULL || gimbal_msg == NULL|| 
		 chassis_msg == NULL || revolver_msg == NULL || 
	   monitor_msg == NULL){
		return false;
	}else{
		return true;
	}
}

/**
  * @brief          ����Ƿ��п�ָ��
  * @retval         ���ֵ
  */
bool CheckMessageCenter()
{
	return message_center.CheckPointerEmpty();
}

/**
  * @brief          ��Ϣ���ĵĽӿ�
  * @retval         Message_Center_t��ַ
  */
Message_Center_t *CenterPointer()
{
	return &message_center;
}





/**
  * @brief          ��Ϣ���ĵĽӿ�
  * @retval         Sys_Pub_Msg_t��ַ
  */
Sys_Pub_Msg_t *SysPointer()
{
	return message_center.sys_msg;
}

/**
  * @brief          ��Ϣ���ĵĽӿ�
  * @retval         Gimbal_Pub_Msg_t��ַ
  */
Gimbal_Pub_Msg_t *GimbalPointer()
{
	return message_center.gimbal_msg;
}

/**
  * @brief          ��Ϣ���ĵĽӿ�
  * @retval         Chassis_Pub_Msg_t��ַ
  */
Chassis_Pub_Msg_t *ChassisPointer()
{
	return message_center.chassis_msg;
}

/**
  * @brief          ��Ϣ���ĵĽӿ�
  * @retval         Revolver_Pub_Msg_t��ַ
  */
Revolver_Pub_Msg_t *RevolverPointer()
{
	return message_center.revolver_msg;
}

/**
  * @brief          ��Ϣ���ĵĽӿ�
  * @retval         UI_Pub_Msg_t��ַ
  */
UI_Pub_Msg_t *UiPointer()
{
	return message_center.ui_msg;
}

/**
  * @brief          ��Ϣ���ĵĽӿ�
  * @retval         Monitor_Pub_Msg_t��ַ
  */
Monitor_Pub_Msg_t *MonitorPointer()
{
	return message_center.monitor_msg;
}



///////////////����-����ģʽ
//������ͨ��Publisher_Create()����
//�������ö�Ӧ�ṹ���ָ���ȡ��ַ���洢����
//��������ֻ���ʼ��ʱ����һ�μ��ɣ��轨��һ����Ӧ�ṹ��ָ���ȡ��ַ
////��ַ��ȡָ�룬�����洢���ݣ�����������ʹ��
//Sys_Pub_Msg_t *sys_msg;
//Gimbal_Pub_Msg_t *gimbal_msg;

////�����ߴ���
////Publisher *sys_pub = Publisher_Create();
////Publisher *gimbal_pub = Publisher_Create();

////ϵͳ������ݴ���
//void SysHandle(void* data, size_t dataSize)
//{
//	sys_msg = (Sys_Pub_Msg_t*)data;
//}

////��̨������ݴ���
//void GimbalHandle(void* data, size_t dataSize)
//{
//	gimbal_msg = (Gimbal_Pub_Msg_t*)data;
//}

//Publisher *mid_sys;
//Publisher *mid_gimbal;

////��Ϣ���ĳ�ʼ��
//void MessageCenterInit()
//{
//	//�����ߴ���
//	Publisher *sys_pub = Publisher_Create();
////	mid_sys = sys_pub;
//	//��������Զ�ͣ��
////	Publisher_Subscribe(sys_pub, SysHandle);
//	//�����ߴ���
//	Publisher *gimbal_pub = Publisher_Create();
//	mid_gimbal = gimbal_pub;
////	Publisher_Subscribe(gimbal_pub, GimbalHandle);
//}

//void PublishMsg(void *data,Msg_Type_e type)
//{
//	switch(type)
//	{
//		case SYSPUB:
//			Publisher_Publish(mid_sys,data,sizeof(Sys_Pub_Msg_t));
//			break;
//		case GIMBALPUB:
//			Publisher_Publish(mid_gimbal,data,sizeof(Gimbal_Pub_Msg_t));
//			break;
//		case TOTAL:
//			break;
//	}
//}
