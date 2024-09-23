/**
 ******************************************************************************
 * @file    message_center.cpp
 * @author  Xushuang
 * @version V1.0.0 基本完成
 * @date    2023/8/24
 * @brief		此处为信息中心，多加需要共享的结构体时需在Message_Center_t中多建立
 *					一个对应结构体指针,并添加传递类型，在各个需要任务处建立实例之后，需
 *          在各任务初始化阶段需进行指针初始化
 ******************************************************************************
 * @attention
 *例：如需增加一个任务ui.cpp，其与其他任务有信息交互的需求，其他处需用到ui.cpp中
 *信息需按以下步骤操作
 *1.Msg_Type_e枚举中添加新传递数据名称
 *2.新建其结构体
 * typedef struct
 * {
 * }Ui_Pub_Msg_t;
 *3.Message_Center_t类中添加此类型指针(Ui_Pub_Msg_t *ui_msg;)
 *4.void Message_Center_t::PointerInit(void *msg,Msg_Type_e type)函数添加新case
 *5.在对应任务死循环前进行初始化(CenterPointer().PointerInit(&ui_msg,UIPUB);)
 ******************************************************************************
 */
#include "message_center.h"
#include "stdio.h"

Message_Center_t message_center;

/**
  * @brief          添加对应结构体指针
  * @param[in]      *msg：可指向不同类型结构体的指针
	* @param[in]      Msg_Type_e: 信息类型
  * @retval         MessageErrorCode：错误信息码
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
  * @brief          检查信息中心指针是否为空
  * @retval         真假值
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
  * @brief          检查是否有空指针
  * @retval         真假值
  */
bool CheckMessageCenter()
{
	return message_center.CheckPointerEmpty();
}

/**
  * @brief          信息中心的接口
  * @retval         Message_Center_t地址
  */
Message_Center_t *CenterPointer()
{
	return &message_center;
}





/**
  * @brief          信息中心的接口
  * @retval         Sys_Pub_Msg_t地址
  */
Sys_Pub_Msg_t *SysPointer()
{
	return message_center.sys_msg;
}

/**
  * @brief          信息中心的接口
  * @retval         Gimbal_Pub_Msg_t地址
  */
Gimbal_Pub_Msg_t *GimbalPointer()
{
	return message_center.gimbal_msg;
}

/**
  * @brief          信息中心的接口
  * @retval         Chassis_Pub_Msg_t地址
  */
Chassis_Pub_Msg_t *ChassisPointer()
{
	return message_center.chassis_msg;
}

/**
  * @brief          信息中心的接口
  * @retval         Revolver_Pub_Msg_t地址
  */
Revolver_Pub_Msg_t *RevolverPointer()
{
	return message_center.revolver_msg;
}

/**
  * @brief          信息中心的接口
  * @retval         UI_Pub_Msg_t地址
  */
UI_Pub_Msg_t *UiPointer()
{
	return message_center.ui_msg;
}

/**
  * @brief          信息中心的接口
  * @retval         Monitor_Pub_Msg_t地址
  */
Monitor_Pub_Msg_t *MonitorPointer()
{
	return message_center.monitor_msg;
}



///////////////发布-订阅模式
//发布者通过Publisher_Create()创建
//订阅者用对应结构体的指针读取地址来存储数据
//传输数据只需初始化时发送一次即可，需建立一个对应结构体指针读取地址
////地址获取指针，用来存储数据，其他处可以使用
//Sys_Pub_Msg_t *sys_msg;
//Gimbal_Pub_Msg_t *gimbal_msg;

////发布者创建
////Publisher *sys_pub = Publisher_Create();
////Publisher *gimbal_pub = Publisher_Create();

////系统句柄数据处理
//void SysHandle(void* data, size_t dataSize)
//{
//	sys_msg = (Sys_Pub_Msg_t*)data;
//}

////云台句柄数据处理
//void GimbalHandle(void* data, size_t dataSize)
//{
//	gimbal_msg = (Gimbal_Pub_Msg_t*)data;
//}

//Publisher *mid_sys;
//Publisher *mid_gimbal;

////消息中心初始化
//void MessageCenterInit()
//{
//	//订阅者创建
//	Publisher *sys_pub = Publisher_Create();
////	mid_sys = sys_pub;
//	//进仿真会自动停下
////	Publisher_Subscribe(sys_pub, SysHandle);
//	//订阅者创建
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
