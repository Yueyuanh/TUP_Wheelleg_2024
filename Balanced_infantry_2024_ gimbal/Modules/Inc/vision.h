#ifndef __VISION_H
#define __VISION_H

#include "crc_check.h"
#include "imu.h"

/*--------------------------------暂定协议-------------------------------------*/

#define    VISION_LENGTH        22     		 //暂定22字节,头3字节,数据17字节,尾2字节

//起始字节,协议固定为0xA5
#define    VISION_SOF         (0xA5)
#define    VISION_SOF2        (0xA5)

//长度根据协议定义,数据段长度为n需要根据帧头第二字节来获取
#define    VISION_LEN_HEADER    3         //帧头长
#define    VISION_LEN_DATA      59        //数据段长度,可自定义
#define    VISION_LEN_TAIL      2	      //帧尾CRC16
#define    VISION_LEN_PACKED    64        //数据包长度,可自定义

#define    VISION_OFF         		(0x00)
#define    VISION_RED           	(0x01)
#define    VISION_BLUE          	(0x02)
#define    VISION_RBUFF_ANTI   	 	(0x03)//红逆大符
#define    VISION_BBUFF_ANTI   		(0x04)//蓝逆大符
#define    VISION_RBUFF_CLOCKWISE   (0x05)//红顺大符
#define    VISION_BBUFF_CLOCKWISE   (0x06)//蓝顺大符
#define    VISION_RBUFF_STAND   	(0x07)//红小符
#define    VISION_BBUFF_STAND   	(0x08)//蓝小符

#define NOW  0
#define LAST 1


#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
/* 	STM32 -> PC

	CmdID   0x00   关闭视觉
	CmdID   0x01   识别红色装甲
	CmdID   0x02   识别蓝色装甲
	CmdID   0x03   红符
	CmdID   0x04   蓝符
*/

/* 	PC -> STM32

	CmdID   0x00   关闭视觉
	CmdID   0x01   识别红色装甲
	CmdID   0x02   识别蓝色装甲
	CmdID   0x03   小符
	CmdID   0x04   大符
*/

//可利用收和发的指令码进行比较,当收和发的指令码相同时,可判定为数据可用

//帧头加CRC8校验,保证发送的指令是正确的

//PC收发与STM32收发成镜像关系,以下结构体适用于STM32,PC需稍作修改

typedef __packed struct
{
	/* 头 */
	uint8_t   SOF;			//帧头起始位,暂定0xA5
	uint8_t   CmdID;		//指令
	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
}extVisionSendHeader_t;


//STM32接收,直接将串口接收到的数据拷贝进结构体
typedef __packed struct
{
	/* 头 */
	uint8_t   SOF;			//帧头起始位,暂定0xA5
	uint8_t   CmdID;		//指令
	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
	/* 数据 */
	float     pitch_angle;
	float     yaw_angle;
	float     distance;			//距离
	bool      target_Switch;		//是否切换识别目标
	uint8_t	  identify_target;	//视野内是否有目标/是否识别到了目标   0否  1是	
	uint8_t   is_spinning;	//打符时是否识别到了目标，1是，2识别到切换了装甲，0没识别到
	bool      check_predict_state; //判断是否开启预测跟随
	bool      auto_firc;
	float     x_info;  //三轴数据
	float     y_info;
	float     z_info;
	float     predict_x_info;  //预测三轴数据
	float     predict_y_info;
	float     predict_z_info;
	uint8_t	  auto_too_close;   //目标距离太近,视觉发1，否则发0
//	uint8_t	  blank_b;			//预留
//	uint8_t	  auto_too_close;   //目标距离太近,视觉发1，否则发0
	
	
	/* 尾 */
	uint16_t  CRC16;       
	
}extVisionRecvData_t;	
	
//STM32发送,直接将打包好的数据一个字节一个字节地发送出去
typedef struct
{
//	/* 头 */
//	uint8_t   SOF;			//帧头起始位,暂定0xA5
//	uint8_t   CmdID;		//指令
//	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
	/* 数据 */
	float    INS_quat_send[4];
	float    INS_gyro_send[3];
	float    INS_accel_send[3];
	float    bullet_speed[1];
	float    shoot_delay[1];
	
//	uint8_t   lock_sentry;		//是否在抬头识别哨兵
//	uint8_t   base;				//吊射
//	
//	uint8_t   blank_a;		//预留
//	uint8_t	  blank_b;
//	uint8_t	  blank_c;	
	
	/* 尾 */
	uint16_t  CRC16;
	
}extVisionSendData_t;

typedef struct
{
//	/* 头 */
//	uint8_t   SOF;			//帧头起始位,暂定0xA5
//	uint8_t   CmdID;		//指令
//	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
	/* 数据 */
	float    INS_quat_send[4];
	float    INS_gyro_send[3];
	float    INS_accel_send[3];
	float    bullet_speed;
	float    rotate_speed;
  float    aver_tem;
//	float send_data[3];
	/* 尾 */
	uint16_t  CRC16;
	
}extCollect_shoot_info_t;
/**
 *	@brief	视觉模式
 */
enum Vision_Mode_t
{
	VISION_MODE_MANUAL		= 0,	// 手动模式
	VISION_MODE_AUTO		= 1,	// 自瞄模式
	VISION_MODE_BIG_BUFF	= 2,	// 打大符模式
	VISION_MODE_SMALL_BUFF	= 3,	// 打小符模式
};
/* 辅助标识变量 */
typedef struct
{
	uint8_t 		my_color;			// 用0/1表示颜色
	Vision_Mode_t	mode;				// 视觉模式
	uint8_t  		rx_data_valid;		// 接收数据的正确性
	uint16_t 		rx_err_cnt;			// 接收数据的错误统计
	uint32_t		rx_cnt;				// 接收数据包的统计
	bool		    rx_data_update;		// 接收数据是否更新
	uint32_t 		rx_time_prev;		// 接收数据的前一时刻
	uint32_t 		rx_time_now;		// 接收数据的当前时刻
	uint16_t 		rx_time_fps;		// 帧率
	bool            predict_state;       //是否开启预测

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
