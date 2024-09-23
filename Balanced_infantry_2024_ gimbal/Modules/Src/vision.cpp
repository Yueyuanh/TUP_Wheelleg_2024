#include "arm_math.h"
#include "usbd_cdc_if.h"
#include "vision.h"
#include "cmsis_os.h"
#include "cstring"

//虚拟串口USB需优化
extVisionSendHeader_t    VisionSendHeader;  //头
extVisionRecvData_t      VisionRecvData;    //视觉接收结构体
extVisionSendData_t      VisionSendData;    //视觉发送结构体
extCollect_shoot_info_t Send_shoot_info;

vision_info_t vision_info;

//uint8_t  Com6_Vision_Buffer[100] = {0};

//int Usart6_Clean_IDLE_Flag = 0;
////接收到的视觉数据暂存在这里
//extern  uint8_t  Com6_Vision_Buffer[ 100 ];
	
//格式转换联合体
typedef union
{
    uint8_t U[4];
    float F;
    int I;
		long L;
	  uint16_t U16_t;
}FormatTrans;


//#define NOW  0
//#define LAST 1

//uint32_t Vision_Time_Test[2] = {0};//前后两次事件
//uint16_t Vision_Ping = 0;//测试时间间隔
////视觉是否发了新数据,FALSE没有,TRUE发了新的
//uint8_t Vision_Get_New_Data = false;

////打符是否换装甲了
//uint8_t Vision_Armor = 0;

void VisionReadData(uint8_t *ReadFromUsart)
{
	//判断帧头数据是否为0xA5
	if(ReadFromUsart[0] == 0xA5)
	{
		//帧头CRC8校验
		if(Verify_CRC8_Check_Sum( ReadFromUsart, VISION_LEN_HEADER ) == true)
		{
			//帧尾CRC16校验
			if(Verify_CRC16_Check_Sum( ReadFromUsart, VISION_LEN_PACKED ) == true)
			{
				//接收数据拷贝
				memcpy(&vision_info.RxPacket, ReadFromUsart, VISION_LEN_PACKED);	
				if(vision_info.RxPacket.yaw_angle >=25 || vision_info.RxPacket.yaw_angle <=-25)
				{
					vision_info.State.rx_data_update = false;
				}
				//帧计算
				vision_info.State.rx_time_now  = xTaskGetTickCountFromISR();
				vision_info.State.rx_time_fps  = vision_info.State.rx_time_now - vision_info.State.rx_time_prev;
				vision_info.State.rx_time_prev = vision_info.State.rx_time_now;
				
				vision_info.State.rx_data_update = true;//标记视觉数据更新了
			}
		}
	}
	if(vision_info.RxPacket.yaw_angle == 99.99f)
	{
		memset(&vision_info.RxPacket, 0, 100);
	} 
}

uint8_t JudgeVisionState()
{
	if(vision_info.State.rx_time_now == vision_info.State.rx_time_prev)
		return false;
	else
		return true;
}

void Vision_Send_Data( uint8_t CmdID )
{
	uint8_t vision_send_pack[64] = {0};//大于22就行

	
	VisionSendHeader.SOF = VISION_SOF;
	VisionSendHeader.CmdID = CmdID;//对视觉来说最重要的数据
	
	//写入帧头
	memcpy( vision_send_pack, &VisionSendHeader, VISION_LEN_HEADER );
	
	//帧头CRC8校验协议
	Append_CRC8_Check_Sum( vision_send_pack, VISION_LEN_HEADER );
	
	//中间数据不用管,视觉用不到,用到了也是后面自瞄自动开火,用到角度补偿数据
	VisionSendData.INS_quat_send[0] = get_INS_quat_point()[0];
	VisionSendData.INS_quat_send[1] = get_INS_quat_point()[1];
	VisionSendData.INS_quat_send[2] = get_INS_quat_point()[2];
	VisionSendData.INS_quat_send[3] = get_INS_quat_point()[3];
	VisionSendData.INS_gyro_send[0] = get_gyro_data_point()[0];
	VisionSendData.INS_gyro_send[1] = get_gyro_data_point()[1];
	VisionSendData.INS_gyro_send[2] = get_gyro_data_point()[2];
	VisionSendData.INS_accel_send[0] = get_accel_data_point()[0];
	VisionSendData.INS_accel_send[1] = get_accel_data_point()[1];
	VisionSendData.INS_accel_send[2] = get_accel_data_point()[2];
	VisionSendData.shoot_delay[0] = 1;

	memcpy( vision_send_pack + 3, &VisionSendData, VISION_LEN_DATA);
	
	//帧尾CRC16校验协议
	Append_CRC16_Check_Sum( vision_send_pack, VISION_LEN_PACKED );
	
	//将打包好的数据发送
	CDC_Transmit_FS(vision_send_pack, VISION_LEN_PACKED);
	
	memset(vision_send_pack, 0, VISION_LEN_PACKED);
}

void vision_info_t::Collect_shoot_info(uint8_t CmdID)
{
	uint8_t send_pack[64] = {0};//大于22就行

	
	VisionSendHeader.SOF = VISION_SOF;
	VisionSendHeader.CmdID = CmdID;//对视觉来说最重要的数据
	
	//写入帧头
	memcpy( send_pack, &VisionSendHeader, VISION_LEN_HEADER );
	
	//帧头CRC8校验协议
	Append_CRC8_Check_Sum( send_pack, VISION_LEN_HEADER );
	
	//中间数据不用管,视觉用不到,用到了也是后面自瞄自动开火,用到角度补偿数据
	
//	Send_shoot_info.INS_quat_send[0] = INS_quat[0];
//	Send_shoot_info.INS_quat_send[1] = INS_quat[1];
//	Send_shoot_info.INS_quat_send[2] = INS_quat[2];
//	Send_shoot_info.INS_quat_send[3] = INS_quat[3];
//	Send_shoot_info.INS_gyro_send[0] = INS_gyro[0];
//	Send_shoot_info.INS_gyro_send[1] = INS_gyro[1];
//	Send_shoot_info.INS_gyro_send[2] = INS_gyro[2];
//	Send_shoot_info.INS_accel_send[0] = INS_accel[0];
//	Send_shoot_info.INS_accel_send[1] = INS_accel[1];
//	Send_shoot_info.INS_accel_send[2] = INS_accel[2];
//	Send_shoot_info.bullet_speed = get_bullet_speed();
//	Send_shoot_info.rotate_speed = get_set_speed();
//	Send_shoot_info.aver_tem = get_aver_tem();

	memcpy( send_pack + 3, &Send_shoot_info, VISION_LEN_DATA);
	
	//帧尾CRC16校验协议
	Append_CRC16_Check_Sum( send_pack, VISION_LEN_PACKED );
	
	//将打包好的数据发送
	
	CDC_Transmit_FS(send_pack, VISION_LEN_PACKED);
	
	memset(send_pack, 0, VISION_LEN_PACKED);
}
/**
  * @brief  获取yaw误差角度，自瞄专用
  * @param  误差指针
  * @retval void
  * @attention  左负右正
  */
void VisionErrorAngleYaw(float *error)
{
	//视觉左负右正,请根据云台控制角度选择正负(左加右减)
	*error = (1.0f*-vision_info.RxPacket.yaw_angle +0.f) *PI / 180.f ;
  //				* 8192.0f / 360.0f / 10.0f;//请根据自己对欧拉角的放大倍数来乘对应倍数
//	if(vision_info.RxPacket.yaw_angle == 0)//发零
//	{
//		*error = 0;
//	} 
}

/**
  * @brief  获取pitch误差角度，自瞄专用
  * @param  误差指针
  * @retval void
  * @attention  视觉上负下正,注意云台正负是抬头还是低头
  */
void VisionErrorAnglePit(float *error)
{	
	*error =(1.0f*vision_info.RxPacket.pitch_angle + 0.f) *PI / 180.f ;
	//深大此处数值为0，但是程序是有对距离进行补偿的代码的。
//	if(vision_info.RxPacket.pitch_angle == 0)
//	{
//		*error = 0;
//	}
}


/**
  * @brief  判断视觉数据更新了吗
  * @param  void
  * @retval TRUE更新了   FALSE没更新
  * @attention  为自瞄做准备,串口空闲中断每触发一次且通过校验,则Vision_Get_New_Data置TRUE
  */
bool VisionIfUpdate(void)
{
	return vision_info.State.rx_data_update;
}


/**
  * @brief  视觉数据更新标志位手动置0(false)
  * @param  void
  * @retval void
  * @attention  记得要清零,在哪清零自己选,调用这个函数就行
  */
void VisionCleanUpdateFlag(void)
{
	vision_info.State.rx_data_update = false;
}



/**
  * @brief  获取距离
	* @param  *distance：需要赋值的地址
  * @retval void
  * @attention  
  */
void VisionGetDistance(float *distance)
{
	*distance = vision_info.RxPacket.distance;
	if(vision_info.RxPacket.distance < 0)
	{
		*distance = 0;
	}
}

/**
  * @brief  得到三轴坐标
  * @param  void
  * @retval void
  * @attention  
  */
void VisionGetCoordinateSystem(float *x_axis,float *y_axis,float *z_axis)
{
	*x_axis = vision_info.RxPacket.x_info;
	*y_axis = vision_info.RxPacket.y_info;
	*z_axis = vision_info.RxPacket.z_info;
}

/**
  * @brief  得到预测三轴坐标
  * @param  void
  * @retval void
  * @attention  
  */
void VisionGetPredictCoordinate(float *predict_x,float *predict_y,float *predict_z)
{
	*predict_x = vision_info.RxPacket.predict_x_info;
	*predict_y = vision_info.RxPacket.predict_y_info;
	*predict_z = vision_info.RxPacket.predict_z_info;
}

uint8_t VisionGetIfTarget()
{
	return  vision_info.RxPacket.identify_target;
}
