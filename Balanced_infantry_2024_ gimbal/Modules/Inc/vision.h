#ifndef __VISION_H
#define __VISION_H

#include "crc_check.h"
#include "imu.h"

/*--------------------------------�ݶ�Э��-------------------------------------*/

#define    VISION_LENGTH        22     		 //�ݶ�22�ֽ�,ͷ3�ֽ�,����17�ֽ�,β2�ֽ�

//��ʼ�ֽ�,Э��̶�Ϊ0xA5
#define    VISION_SOF         (0xA5)
#define    VISION_SOF2        (0xA5)

//���ȸ���Э�鶨��,���ݶγ���Ϊn��Ҫ����֡ͷ�ڶ��ֽ�����ȡ
#define    VISION_LEN_HEADER    3         //֡ͷ��
#define    VISION_LEN_DATA      59        //���ݶγ���,���Զ���
#define    VISION_LEN_TAIL      2	      //֡βCRC16
#define    VISION_LEN_PACKED    64        //���ݰ�����,���Զ���

#define    VISION_OFF         		(0x00)
#define    VISION_RED           	(0x01)
#define    VISION_BLUE          	(0x02)
#define    VISION_RBUFF_ANTI   	 	(0x03)//������
#define    VISION_BBUFF_ANTI   		(0x04)//������
#define    VISION_RBUFF_CLOCKWISE   (0x05)//��˳���
#define    VISION_BBUFF_CLOCKWISE   (0x06)//��˳���
#define    VISION_RBUFF_STAND   	(0x07)//��С��
#define    VISION_BBUFF_STAND   	(0x08)//��С��

#define NOW  0
#define LAST 1


#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
/* 	STM32 -> PC

	CmdID   0x00   �ر��Ӿ�
	CmdID   0x01   ʶ���ɫװ��
	CmdID   0x02   ʶ����ɫװ��
	CmdID   0x03   ���
	CmdID   0x04   ����
*/

/* 	PC -> STM32

	CmdID   0x00   �ر��Ӿ�
	CmdID   0x01   ʶ���ɫװ��
	CmdID   0x02   ʶ����ɫװ��
	CmdID   0x03   С��
	CmdID   0x04   ���
*/

//�������պͷ���ָ������бȽ�,���պͷ���ָ������ͬʱ,���ж�Ϊ���ݿ���

//֡ͷ��CRC8У��,��֤���͵�ָ������ȷ��

//PC�շ���STM32�շ��ɾ����ϵ,���½ṹ��������STM32,PC�������޸�

typedef __packed struct
{
	/* ͷ */
	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
	uint8_t   CmdID;		//ָ��
	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
	
}extVisionSendHeader_t;


//STM32����,ֱ�ӽ����ڽ��յ������ݿ������ṹ��
typedef __packed struct
{
	/* ͷ */
	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
	uint8_t   CmdID;		//ָ��
	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
	
	/* ���� */
	float     pitch_angle;
	float     yaw_angle;
	float     distance;			//����
	bool      target_Switch;		//�Ƿ��л�ʶ��Ŀ��
	uint8_t	  identify_target;	//��Ұ���Ƿ���Ŀ��/�Ƿ�ʶ����Ŀ��   0��  1��	
	uint8_t   is_spinning;	//���ʱ�Ƿ�ʶ����Ŀ�꣬1�ǣ�2ʶ���л���װ�ף�0ûʶ��
	bool      check_predict_state; //�ж��Ƿ���Ԥ�����
	bool      auto_firc;
	float     x_info;  //��������
	float     y_info;
	float     z_info;
	float     predict_x_info;  //Ԥ����������
	float     predict_y_info;
	float     predict_z_info;
	uint8_t	  auto_too_close;   //Ŀ�����̫��,�Ӿ���1������0
//	uint8_t	  blank_b;			//Ԥ��
//	uint8_t	  auto_too_close;   //Ŀ�����̫��,�Ӿ���1������0
	
	
	/* β */
	uint16_t  CRC16;       
	
}extVisionRecvData_t;	
	
//STM32����,ֱ�ӽ�����õ�����һ���ֽ�һ���ֽڵط��ͳ�ȥ
typedef struct
{
//	/* ͷ */
//	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
//	uint8_t   CmdID;		//ָ��
//	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
	
	/* ���� */
	float    INS_quat_send[4];
	float    INS_gyro_send[3];
	float    INS_accel_send[3];
	float    bullet_speed[1];
	float    shoot_delay[1];
	
//	uint8_t   lock_sentry;		//�Ƿ���̧ͷʶ���ڱ�
//	uint8_t   base;				//����
//	
//	uint8_t   blank_a;		//Ԥ��
//	uint8_t	  blank_b;
//	uint8_t	  blank_c;	
	
	/* β */
	uint16_t  CRC16;
	
}extVisionSendData_t;

typedef struct
{
//	/* ͷ */
//	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
//	uint8_t   CmdID;		//ָ��
//	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
	
	/* ���� */
	float    INS_quat_send[4];
	float    INS_gyro_send[3];
	float    INS_accel_send[3];
	float    bullet_speed;
	float    rotate_speed;
  float    aver_tem;
//	float send_data[3];
	/* β */
	uint16_t  CRC16;
	
}extCollect_shoot_info_t;
/**
 *	@brief	�Ӿ�ģʽ
 */
enum Vision_Mode_t
{
	VISION_MODE_MANUAL		= 0,	// �ֶ�ģʽ
	VISION_MODE_AUTO		= 1,	// ����ģʽ
	VISION_MODE_BIG_BUFF	= 2,	// ����ģʽ
	VISION_MODE_SMALL_BUFF	= 3,	// ��С��ģʽ
};
/* ������ʶ���� */
typedef struct
{
	uint8_t 		my_color;			// ��0/1��ʾ��ɫ
	Vision_Mode_t	mode;				// �Ӿ�ģʽ
	uint8_t  		rx_data_valid;		// �������ݵ���ȷ��
	uint16_t 		rx_err_cnt;			// �������ݵĴ���ͳ��
	uint32_t		rx_cnt;				// �������ݰ���ͳ��
	bool		    rx_data_update;		// ���������Ƿ����
	uint32_t 		rx_time_prev;		// �������ݵ�ǰһʱ��
	uint32_t 		rx_time_now;		// �������ݵĵ�ǰʱ��
	uint16_t 		rx_time_fps;		// ֡��
	bool            predict_state;       //�Ƿ���Ԥ��

} Vision_State_t;

class vision_info_t
{
	public:
		extVisionRecvData_t RxPacket;
	  extVisionSendData_t TxPacket;
	  Vision_State_t State;
	
//	
	  void Vision_Send_Data_Angle(uint8_t CmdID );
	  void Collect_shoot_info(uint8_t CmdID);

};	
	
extern vision_info_t vision_info;

void Vision_Send_Data( uint8_t CmdID );
void VisionErrorAngleYaw(float *error);
void VisionErrorAnglePit(float *error);
bool VisionIfUpdate(void);
void VisionCleanUpdateFlag(void);
void VisionGetDistance(float *distance);
void VisionGetCoordinateSystem(float *x_axis,float *y_axis,float *z_axis);
void VisionGetPredictCoordinate(float *predict_x,float *predict_y,float *predict_z);
uint8_t VisionGetIfTarget();
uint8_t JudgeVisionState();

#endif
void VisionReadData(uint8_t *ReadFromUsart);
#ifdef __cplusplus
}	
#endif

#endif
