#include "arm_math.h"
#include "usbd_cdc_if.h"
#include "vision.h"
#include "cmsis_os.h"
#include "cstring"

//���⴮��USB���Ż�
extVisionSendHeader_t    VisionSendHeader;  //ͷ
extVisionRecvData_t      VisionRecvData;    //�Ӿ����սṹ��
extVisionSendData_t      VisionSendData;    //�Ӿ����ͽṹ��
extCollect_shoot_info_t Send_shoot_info;

vision_info_t vision_info;

//uint8_t  Com6_Vision_Buffer[100] = {0};

//int Usart6_Clean_IDLE_Flag = 0;
////���յ����Ӿ������ݴ�������
//extern  uint8_t  Com6_Vision_Buffer[ 100 ];
	
//��ʽת��������
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

//uint32_t Vision_Time_Test[2] = {0};//ǰ�������¼�
//uint16_t Vision_Ping = 0;//����ʱ����
////�Ӿ��Ƿ���������,FALSEû��,TRUE�����µ�
//uint8_t Vision_Get_New_Data = false;

////����Ƿ�װ����
//uint8_t Vision_Armor = 0;

void VisionReadData(uint8_t *ReadFromUsart)
{
	//�ж�֡ͷ�����Ƿ�Ϊ0xA5
	if(ReadFromUsart[0] == 0xA5)
	{
		//֡ͷCRC8У��
		if(Verify_CRC8_Check_Sum( ReadFromUsart, VISION_LEN_HEADER ) == true)
		{
			//֡βCRC16У��
			if(Verify_CRC16_Check_Sum( ReadFromUsart, VISION_LEN_PACKED ) == true)
			{
				//�������ݿ���
				memcpy(&vision_info.RxPacket, ReadFromUsart, VISION_LEN_PACKED);	
				if(vision_info.RxPacket.yaw_angle >=25 || vision_info.RxPacket.yaw_angle <=-25)
				{
					vision_info.State.rx_data_update = false;
				}
				//֡����
				vision_info.State.rx_time_now  = xTaskGetTickCountFromISR();
				vision_info.State.rx_time_fps  = vision_info.State.rx_time_now - vision_info.State.rx_time_prev;
				vision_info.State.rx_time_prev = vision_info.State.rx_time_now;
				
				vision_info.State.rx_data_update = true;//����Ӿ����ݸ�����
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
	uint8_t vision_send_pack[64] = {0};//����22����

	
	VisionSendHeader.SOF = VISION_SOF;
	VisionSendHeader.CmdID = CmdID;//���Ӿ���˵����Ҫ������
	
	//д��֡ͷ
	memcpy( vision_send_pack, &VisionSendHeader, VISION_LEN_HEADER );
	
	//֡ͷCRC8У��Э��
	Append_CRC8_Check_Sum( vision_send_pack, VISION_LEN_HEADER );
	
	//�м����ݲ��ù�,�Ӿ��ò���,�õ���Ҳ�Ǻ��������Զ�����,�õ��ǶȲ�������
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
	
	//֡βCRC16У��Э��
	Append_CRC16_Check_Sum( vision_send_pack, VISION_LEN_PACKED );
	
	//������õ����ݷ���
	CDC_Transmit_FS(vision_send_pack, VISION_LEN_PACKED);
	
	memset(vision_send_pack, 0, VISION_LEN_PACKED);
}

void vision_info_t::Collect_shoot_info(uint8_t CmdID)
{
	uint8_t send_pack[64] = {0};//����22����

	
	VisionSendHeader.SOF = VISION_SOF;
	VisionSendHeader.CmdID = CmdID;//���Ӿ���˵����Ҫ������
	
	//д��֡ͷ
	memcpy( send_pack, &VisionSendHeader, VISION_LEN_HEADER );
	
	//֡ͷCRC8У��Э��
	Append_CRC8_Check_Sum( send_pack, VISION_LEN_HEADER );
	
	//�м����ݲ��ù�,�Ӿ��ò���,�õ���Ҳ�Ǻ��������Զ�����,�õ��ǶȲ�������
	
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
	
	//֡βCRC16У��Э��
	Append_CRC16_Check_Sum( send_pack, VISION_LEN_PACKED );
	
	//������õ����ݷ���
	
	CDC_Transmit_FS(send_pack, VISION_LEN_PACKED);
	
	memset(send_pack, 0, VISION_LEN_PACKED);
}
/**
  * @brief  ��ȡyaw���Ƕȣ�����ר��
  * @param  ���ָ��
  * @retval void
  * @attention  ������
  */
void VisionErrorAngleYaw(float *error)
{
	//�Ӿ�������,�������̨���ƽǶ�ѡ������(����Ҽ�)
	*error = (1.0f*-vision_info.RxPacket.yaw_angle +0.f) *PI / 180.f ;
  //				* 8192.0f / 360.0f / 10.0f;//������Լ���ŷ���ǵķŴ������˶�Ӧ����
//	if(vision_info.RxPacket.yaw_angle == 0)//����
//	{
//		*error = 0;
//	} 
}

/**
  * @brief  ��ȡpitch���Ƕȣ�����ר��
  * @param  ���ָ��
  * @retval void
  * @attention  �Ӿ��ϸ�����,ע����̨������̧ͷ���ǵ�ͷ
  */
void VisionErrorAnglePit(float *error)
{	
	*error =(1.0f*vision_info.RxPacket.pitch_angle + 0.f) *PI / 180.f ;
	//���˴���ֵΪ0�����ǳ������жԾ�����в����Ĵ���ġ�
//	if(vision_info.RxPacket.pitch_angle == 0)
//	{
//		*error = 0;
//	}
}


/**
  * @brief  �ж��Ӿ����ݸ�������
  * @param  void
  * @retval TRUE������   FALSEû����
  * @attention  Ϊ������׼��,���ڿ����ж�ÿ����һ����ͨ��У��,��Vision_Get_New_Data��TRUE
  */
bool VisionIfUpdate(void)
{
	return vision_info.State.rx_data_update;
}


/**
  * @brief  �Ӿ����ݸ��±�־λ�ֶ���0(false)
  * @param  void
  * @retval void
  * @attention  �ǵ�Ҫ����,���������Լ�ѡ,���������������
  */
void VisionCleanUpdateFlag(void)
{
	vision_info.State.rx_data_update = false;
}



/**
  * @brief  ��ȡ����
	* @param  *distance����Ҫ��ֵ�ĵ�ַ
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
  * @brief  �õ���������
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
  * @brief  �õ�Ԥ����������
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
