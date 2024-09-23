/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "main.h"
#include "string.h"
#include "math.h"

#ifdef __cplusplus
extern "C"{
#endif

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


//motor data read
#define get_motor_measure(ptr, data)                                    \
   {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];  																	\
				if (abs((ptr)->ecd - (ptr)->last_ecd) > 4096)										\
			{																																	\
				if ((ptr)->ecd  >= 6500 && (ptr)->last_ecd <= 1000)							\
				{																																\
						(ptr)->num += 1;																						\
				}																																\
				else	if((ptr)->ecd  <= 1000 && (ptr)->last_ecd >= 6500)				\
				{																																\
						(ptr)->num -= 1;																						\
				}																															  \
			}																																	\
    }
	 

#define motor_measure_MF9025(ptr, data)																	\
    {																																		\
				(ptr)->last_ecd = (ptr)->ecd;															\
        (ptr)->ecd = (uint16_t)((data)[7] << 8 | (data)[6]);						\
        (ptr)->speed_rpm = (int16_t)((data)[5] << 8 | (data)[4]);					\
        (ptr)->given_current = (int16_t)((data)[3] << 8 | (data)[2]);	\
        (ptr)->temperate = (data)[1];																		\
				if (abs((ptr)->ecd - (ptr)->last_ecd) > 32767)										\
			{																																	\
				if ((ptr)->ecd  >= 52000 && (ptr)->last_ecd <= 8000)							\
				{																																\
						(ptr)->num -= 1;																						\
				}																																\
				else	if((ptr)->ecd  <= 8000 && (ptr)->last_ecd >= 52000)				\
				{																																\
						(ptr)->num += 1;																						\
				}																															  \
			}																																	\
    }

		
#define motor_measure_MF9025_PI(ptr,data)																\
		{																																		\
			(ptr)->anglePidKp = (data)[2];																		\
			(ptr)->anglePidKi = (data)[3];																			\
			(ptr)->speedPidKp = (data)[4];																			\
			(ptr)->speedPidKi = (data)[5];																			\
			(ptr)->iqPidKp		= (data)[6];																			\
			(ptr)->iqPidKi		= (data)[7];																			\
		}																															

#define motor_measure_MF9025_error1(ptr,data)														\
		{																																		\
			(ptr)->temperature = (data)[1];																		\
			(ptr)->voltage = (uint16_t)((data)[3] << 8 | (data)[2])	;						\
			(ptr)->error_flag = (data)[7];																		\
		}




/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机*/
static motor_measure_t motor_chassis[7];
static motor_measure_t motor_mf9025_chassis[2];
//static mf3508_PI_feedback motor_mf9025_pid[2];
//static mf9025_error_read motor_mf9025_error1[2];

static Gimbal_order_t gimbal_order;

static CAN_TxHeaderTypeDef  sendToGimal_tx_message;

//底盘发给云台数据
static uint8_t              sendToGimbal_data[8];


static CAN_TxHeaderTypeDef  gimbal_tx_message;
static CAN_TxHeaderTypeDef  chassis_tx_message;


static CAN_TxHeaderTypeDef	chassis_mf9025_tx_message;

static uint8_t              mf9025_chassis_can_send_data[8];

static uint8_t              gimbal_can_send_data[8];
static uint8_t 							chassis_can_dip_Angle[8];
static uint8_t              chassis_can_send_data[8];


void get_gimbal_order(Gimbal_order_t* gimbal_order,uint8_t* rxdata);


void update_can_flag(void);


/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

		CAN_RxHeaderTypeDef rx_header_mf9025;//驱动轮数据
		static		uint8_t rx_data_mf9025[10];


    CAN_RxHeaderTypeDef rx_header_gimbal;//云台发来的数据
		static		uint8_t rx_data_gimbal[10];


	  if (hcan->Instance == CAN1)
		{
				HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header_mf9025, rx_data_mf9025);

				switch (rx_header_mf9025.StdId)
				{
						//MF9025
						case CAN_MF9025_1_ID:
						case CAN_MF9025_2_ID:
						{
						static uint8_t i = 0;
						
						// get motor id						
						i = rx_header_mf9025.StdId - CAN_MF9025_1_ID;




//					if(rx_data_mf9025[0] ==  0xA1)
////					{
								  motor_measure_MF9025(&motor_mf9025_chassis[i], rx_data_mf9025);
//					  }
//					if(rx_data_mf9025[0] ==  0x30)
//					  {
//								motor_measure_MF9025_PI(&motor_mf9025_pid[i],rx_data_mf9025);
//					  }
//					if(rx_data_mf9025[0] ==  0x9A)
//					  {
//						    motor_measure_MF9025_error1(&motor_mf9025_error1[i], rx_data_mf9025);
//					  }
					
								break;
							}
		}
			
			
	}
	   if (hcan->Instance == CAN2)//CAN2目前只有云台数据
    {
			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header_gimbal, rx_data_gimbal);
			switch (rx_header_gimbal.StdId)
			{
			 //云台数据
			 case 0x301:
			 case 0x302:
			 case 0x303:
			 case 0x304:
			 case 0x305:
			 case 0x306:
			 case 0x307:
			 Gimbal_order_decode(rx_header_gimbal.StdId,rx_data_gimbal);

			 break;
			 
        default:
        {
            break;
        }


			//电机接受云台数据


//			case CAN_GIMBAL_ORDER_ID:
//			{
//			memcpy((unsigned char*)&gimbal_order+16,(unsigned char*)rx_data,2*sizeof(int));
//			update_can_flag();
//			break;
//			}




			case	CAN_LOGO:
			{
					memcpy((unsigned char*)&gimbal_order+24,(unsigned char*)rx_data_gimbal,3*sizeof(bool_t));

					update_can_flag();
			}

			
    }
	
	}
}




/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          mf9025发送电机控制电流(0x141,0x142,0x143,0x144)（扭矩模式）
  * @param[in]      motor1: (0x141) 3508电机控制电流, 数值范围-2000~ 2000，对应实际转矩电流范围-32A~32A
  * @param[in]      motor2: (0x142) 3508电机控制电流, 数值范围-2000~ 2000，对应实际转矩电流范围-32A~32A
  * @param[in]      motor3: (0x143) 3508电机控制电流, 数值范围-2000~ 2000，对应实际转矩电流范围-32A~32A
  * @param[in]      motor4: (0x144) 3508电机控制电流, 数值范围-2000~ 2000，对应实际转矩电流范围-32A~32A
  * @retval         none
  */
void CAN_mf9025_cmd_chassis(int16_t motor1, int16_t motor2)
{
    uint32_t send_mail_box;
    chassis_mf9025_tx_message.StdId = 0x280;
    chassis_mf9025_tx_message.IDE = CAN_ID_STD;
    chassis_mf9025_tx_message.RTR = CAN_RTR_DATA;
    chassis_mf9025_tx_message.DLC = 0x08;
    mf9025_chassis_can_send_data[0] = motor1 ;
    mf9025_chassis_can_send_data[1] = motor1>> 8;
    mf9025_chassis_can_send_data[2] = motor2 ;
    mf9025_chassis_can_send_data[3] = motor2>> 8;
    mf9025_chassis_can_send_data[4] = 0x00;
    mf9025_chassis_can_send_data[5] = 0x00;
    mf9025_chassis_can_send_data[6] = 0x00;
    mf9025_chassis_can_send_data[7] = 0x00;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_mf9025_tx_message, mf9025_chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          读取mf9025,id为0x141电机内部PID
  * @retval         none
  */
void CAN_mf9025_cmd_chassis_L(void)
{
    uint32_t send_mail_box;
    chassis_mf9025_tx_message.StdId = 0x141;
    chassis_mf9025_tx_message.IDE = CAN_ID_STD;
    chassis_mf9025_tx_message.RTR = CAN_RTR_DATA;
    chassis_mf9025_tx_message.DLC = 0x08;
    mf9025_chassis_can_send_data[0] = 0x30;
    mf9025_chassis_can_send_data[1] = 0x00;
    mf9025_chassis_can_send_data[2] = 0x00;
    mf9025_chassis_can_send_data[3] = 0x00;
    mf9025_chassis_can_send_data[4] = 0x00;
    mf9025_chassis_can_send_data[5] = 0x00;
    mf9025_chassis_can_send_data[6] = 0x00;
    mf9025_chassis_can_send_data[7] = 0x00;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_mf9025_tx_message, mf9025_chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          读取mf9025,id为0x142电机内部PID
  * @retval         none
  */
void CAN_mf9025_cmd_chassis_R(void)
{
    uint32_t send_mail_box;
    chassis_mf9025_tx_message.StdId = 0x142;
    chassis_mf9025_tx_message.IDE = CAN_ID_STD;
    chassis_mf9025_tx_message.RTR = CAN_RTR_DATA;
    chassis_mf9025_tx_message.DLC = 0x08;
    mf9025_chassis_can_send_data[0] = 0x30;
    mf9025_chassis_can_send_data[1] = 0x00;
    mf9025_chassis_can_send_data[2] = 0x00;
    mf9025_chassis_can_send_data[3] = 0x00;
    mf9025_chassis_can_send_data[4] = 0x00;
    mf9025_chassis_can_send_data[5] = 0x00;
    mf9025_chassis_can_send_data[6] = 0x00;
    mf9025_chassis_can_send_data[7] = 0x00;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_mf9025_tx_message, mf9025_chassis_can_send_data, &send_mail_box);
}





/**
  * @brief          mf9025发送电机控制电流0x141（扭矩模式）
  * @param[in]      motor1: (0x141) 3508电机控制电流, ，数值范围-2048~ 2048，对应 MF 电机实际转矩电流范围-16.5A~16.5A
  * @retval         none
  */

void CAN_mf9025_cmd_chassis_Torque_control_1(int16_t motor1)
{
    uint32_t send_mail_box;
    chassis_mf9025_tx_message.StdId = 0x141;
    chassis_mf9025_tx_message.IDE = CAN_ID_STD;
    chassis_mf9025_tx_message.RTR = CAN_RTR_DATA;
    chassis_mf9025_tx_message.DLC = 0x08;
    mf9025_chassis_can_send_data[0] = 0xA1;
    mf9025_chassis_can_send_data[1] = 0x00;
    mf9025_chassis_can_send_data[2] = 0x00;
    mf9025_chassis_can_send_data[3] = 0x00;
    mf9025_chassis_can_send_data[4] = motor1 ;
    mf9025_chassis_can_send_data[5] = motor1>> 8;
    mf9025_chassis_can_send_data[6] = 0x00;
    mf9025_chassis_can_send_data[7] = 0x00;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_mf9025_tx_message, mf9025_chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          mf9025发送电机控制电流0x142（扭矩模式）
  * @param[in]      motor1: (0x141) 3508电机控制电流, ，数值范围-2048~ 2048，对应 MF 电机实际转矩电流范围-16.5A~16.5A
  * @retval         none
  */
void CAN_mf9025_cmd_chassis_Torque_control_2(int16_t motor1)
{
    uint32_t send_mail_box;
    chassis_mf9025_tx_message.StdId = 0x142;
    chassis_mf9025_tx_message.IDE = CAN_ID_STD;
    chassis_mf9025_tx_message.RTR = CAN_RTR_DATA;
    chassis_mf9025_tx_message.DLC = 0x08;
    mf9025_chassis_can_send_data[0] = 0xA1;
    mf9025_chassis_can_send_data[1] = 0x00;
    mf9025_chassis_can_send_data[2] = 0x00;
    mf9025_chassis_can_send_data[3] = 0x00;
    mf9025_chassis_can_send_data[4] = motor1   ;
    mf9025_chassis_can_send_data[5] = motor1 >> 8;
    mf9025_chassis_can_send_data[6] = 0x00;
    mf9025_chassis_can_send_data[7] = 0x00;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_mf9025_tx_message, mf9025_chassis_can_send_data, &send_mail_box);
}



/**
  * @brief          mf9025发送电机控制电流0x141（速度模式）
  * @param[in]      motor1: (0x141) 3508电机控制电流, ，数值范围-2048~ 2048，对应 MF 电机实际转矩电流范围-16.5A~16.5A
  * @retval         none
  */
void CAN_mf9025_cmd_chassis_Torque_control_L(int32_t motor1)
{
    uint32_t send_mail_box;
    chassis_mf9025_tx_message.StdId = 0x141;
    chassis_mf9025_tx_message.IDE = CAN_ID_STD;
    chassis_mf9025_tx_message.RTR = CAN_RTR_DATA;
    chassis_mf9025_tx_message.DLC = 0x08;
    mf9025_chassis_can_send_data[0] = 0xA2;
    mf9025_chassis_can_send_data[1] = 0x00;
    mf9025_chassis_can_send_data[2] = 0x00;
    mf9025_chassis_can_send_data[3] = 0x00;
    mf9025_chassis_can_send_data[4] = (uint8_t)motor1;
    mf9025_chassis_can_send_data[5] = (uint8_t)(motor1>>8);
    mf9025_chassis_can_send_data[6] = (uint8_t)(motor1 >> 16);
    mf9025_chassis_can_send_data[7] = (uint8_t)(motor1>> 24);

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_mf9025_tx_message, mf9025_chassis_can_send_data, &send_mail_box);
}



/**
  * @brief          mf9025发送电机控制电流0x142（速度模式）
  * @param[in]      motor1: (0x142) 3508电机控制电流, ，数值范围-2048~ 2048，对应 MF 电机实际转矩电流范围-16.5A~16.5A
  * @retval         none
  */
void CAN_mf9025_cmd_chassis_Torque_control_R(int32_t motor1)
{
    uint32_t send_mail_box;
    chassis_mf9025_tx_message.StdId = 0x142;
    chassis_mf9025_tx_message.IDE = CAN_ID_STD;
    chassis_mf9025_tx_message.RTR = CAN_RTR_DATA;
    chassis_mf9025_tx_message.DLC = 0x08;
    mf9025_chassis_can_send_data[0] = 0xA2;
    mf9025_chassis_can_send_data[1] = 0x00;
    mf9025_chassis_can_send_data[2] = 0x00;
    mf9025_chassis_can_send_data[3] = 0x00;
		mf9025_chassis_can_send_data[4] = (uint8_t)motor1;
    mf9025_chassis_can_send_data[5] = (uint8_t)(motor1>>8);
    mf9025_chassis_can_send_data[6] = (uint8_t)(motor1 >> 16);
    mf9025_chassis_can_send_data[7] = (uint8_t)(motor1>> 24);

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_mf9025_tx_message, mf9025_chassis_can_send_data, &send_mail_box);
}



void CAN_cmd_gimbal_command_0x714(float chassis_dip_Angle)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = 0x714;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x04;
    memcpy((unsigned char*)chassis_can_dip_Angle,(unsigned char*)&chassis_dip_Angle,2*sizeof(float));	
    HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, chassis_can_dip_Angle, &send_mail_box);
}

//void CAN_cmd_set_current_loop(void)
//{
//    uint32_t send_mail_box;
//    chassis_tx_message.StdId = 0x3ff;
//    chassis_tx_message.IDE = CAN_ID_STD;
//    chassis_tx_message.RTR = CAN_RTR_DATA;
//    chassis_tx_message.DLC = 0x08;
//    chassis_can_send_data[0] = 0x01;
//    chassis_can_send_data[1] = 0x01;
//    chassis_can_send_data[2] = 0x01;
//    chassis_can_send_data[3] = 0x01;
//    chassis_can_send_data[4] = 0x01;
//    chassis_can_send_data[5] = 0x01;
//    chassis_can_send_data[6] = 0x01;
//    chassis_can_send_data[7] = 0x01;

//    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
//}



//mf9025初始数据为100.，100，50，40，50，50数据顺序为接受顺序
/**
  * @brief         改变电机PID到ROM命令
  * @param[in]      motor1: (0x142) 3508电机控制电流, ，数值范围-2048~ 2048，对应 MF 电机实际转矩电流范围-16.5A~16.5A
  * @retval         none
  */
void CAN_mf9025_cmd_chassis_PID_readin_L(void)
{
    uint32_t send_mail_box;
    chassis_mf9025_tx_message.StdId = 0x141;
    chassis_mf9025_tx_message.IDE = CAN_ID_STD;
    chassis_mf9025_tx_message.RTR = CAN_RTR_DATA;
    chassis_mf9025_tx_message.DLC = 0x08;
    mf9025_chassis_can_send_data[0] = 0x32;
    mf9025_chassis_can_send_data[1] = 0x00;
    mf9025_chassis_can_send_data[2] = 100;
    mf9025_chassis_can_send_data[3] = 100;
		mf9025_chassis_can_send_data[4] = 50;
    mf9025_chassis_can_send_data[5] = 40;
    mf9025_chassis_can_send_data[6] = 50;
    mf9025_chassis_can_send_data[7] = 50;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_mf9025_tx_message, mf9025_chassis_can_send_data, &send_mail_box);
}
/**
  * @brief         改变电机PID到ROM命令
  * @param[in]      motor1: (0x142) 3508电机控制电流, ，数值范围-2048~ 2048，对应 MF 电机实际转矩电流范围-16.5A~16.5A
  * @retval         none
  */
void CAN_mf9025_cmd_chassis_PID_readin_R(void)
{
    uint32_t send_mail_box;
    chassis_mf9025_tx_message.StdId = 0x142;
    chassis_mf9025_tx_message.IDE = CAN_ID_STD;
    chassis_mf9025_tx_message.RTR = CAN_RTR_DATA;
    chassis_mf9025_tx_message.DLC = 0x08;
    mf9025_chassis_can_send_data[0] = 0x32;
    mf9025_chassis_can_send_data[1] = 0x00;
    mf9025_chassis_can_send_data[2] = 100;
    mf9025_chassis_can_send_data[3] = 100;
		mf9025_chassis_can_send_data[4] = 50;
    mf9025_chassis_can_send_data[5] = 40;
    mf9025_chassis_can_send_data[6] = 50;
    mf9025_chassis_can_send_data[7] = 50;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_mf9025_tx_message, mf9025_chassis_can_send_data, &send_mail_box);
}
/**
  * @brief        	读取电机状态
  * @retval         none
  */
void CAN_mf9025_read_motor_status1(void)
{
    uint32_t send_mail_box;
    chassis_mf9025_tx_message.StdId = 0x142;
    chassis_mf9025_tx_message.IDE = CAN_ID_STD;
    chassis_mf9025_tx_message.RTR = CAN_RTR_DATA;
    chassis_mf9025_tx_message.DLC = 0x08;
    mf9025_chassis_can_send_data[0] = 0x9C;
    mf9025_chassis_can_send_data[1] = 0x00;
    mf9025_chassis_can_send_data[2] = 0x00;
    mf9025_chassis_can_send_data[3] = 0x00;
		mf9025_chassis_can_send_data[4] = 0x00;
    mf9025_chassis_can_send_data[5] = 0x00;
    mf9025_chassis_can_send_data[6] = 0x00;
    mf9025_chassis_can_send_data[7] = 0x00;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_mf9025_tx_message, mf9025_chassis_can_send_data, &send_mail_box);
}
void CAN_mf9025_read_motor_status2(void)
{
    uint32_t send_mail_box;
    chassis_mf9025_tx_message.StdId = 0x141;
    chassis_mf9025_tx_message.IDE = CAN_ID_STD;
    chassis_mf9025_tx_message.RTR = CAN_RTR_DATA;
    chassis_mf9025_tx_message.DLC = 0x08;
    mf9025_chassis_can_send_data[0] = 0x9C;
    mf9025_chassis_can_send_data[1] = 0x00;
    mf9025_chassis_can_send_data[2] = 0x00;
    mf9025_chassis_can_send_data[3] = 0x00;
		mf9025_chassis_can_send_data[4] = 0x00;
    mf9025_chassis_can_send_data[5] = 0x00;
    mf9025_chassis_can_send_data[6] = 0x00;
    mf9025_chassis_can_send_data[7] = 0x00;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_mf9025_tx_message, mf9025_chassis_can_send_data, &send_mail_box);
}

void CAN_mf9025_read_motor_error(void)
{
    uint32_t send_mail_box;
    chassis_mf9025_tx_message.StdId = 0x141;
    chassis_mf9025_tx_message.IDE = CAN_ID_STD;
    chassis_mf9025_tx_message.RTR = CAN_RTR_DATA;
    chassis_mf9025_tx_message.DLC = 0x08;
    mf9025_chassis_can_send_data[0] = 0x9A;
    mf9025_chassis_can_send_data[1] = 0x00;
    mf9025_chassis_can_send_data[2] = 0x00;
    mf9025_chassis_can_send_data[3] = 0x00;
		mf9025_chassis_can_send_data[4] = 0x00;
    mf9025_chassis_can_send_data[5] = 0x00;
    mf9025_chassis_can_send_data[6] = 0x00;
    mf9025_chassis_can_send_data[7] = 0x00;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_mf9025_tx_message, mf9025_chassis_can_send_data, &send_mail_box);
}
void CAN_mf9025_read_motor_error1(void)
{
    uint32_t send_mail_box;
    chassis_mf9025_tx_message.StdId = 0x142;
    chassis_mf9025_tx_message.IDE = CAN_ID_STD;
    chassis_mf9025_tx_message.RTR = CAN_RTR_DATA;
    chassis_mf9025_tx_message.DLC = 0x08;
    mf9025_chassis_can_send_data[0] = 0x9A;
    mf9025_chassis_can_send_data[1] = 0x00;
    mf9025_chassis_can_send_data[2] = 0x00;
    mf9025_chassis_can_send_data[3] = 0x00;
		mf9025_chassis_can_send_data[4] = 0x00;
    mf9025_chassis_can_send_data[5] = 0x00;
    mf9025_chassis_can_send_data[6] = 0x00;
    mf9025_chassis_can_send_data[7] = 0x00;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_mf9025_tx_message, mf9025_chassis_can_send_data, &send_mail_box);
}


//开启电机
void CAN_mf9025_motor_on(void)
{
    uint32_t send_mail_box;
    chassis_mf9025_tx_message.StdId = 0x141;
    chassis_mf9025_tx_message.IDE = CAN_ID_STD;
    chassis_mf9025_tx_message.RTR = CAN_RTR_DATA;
    chassis_mf9025_tx_message.DLC = 0x08;
    mf9025_chassis_can_send_data[0] = 0x88;
    mf9025_chassis_can_send_data[1] = 0x00;
    mf9025_chassis_can_send_data[2] = 0x00;
    mf9025_chassis_can_send_data[3] = 0x00;
		mf9025_chassis_can_send_data[4] = 0x00;
    mf9025_chassis_can_send_data[5] = 0x00;
    mf9025_chassis_can_send_data[6] = 0x00;
    mf9025_chassis_can_send_data[7] = 0x00;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_mf9025_tx_message, mf9025_chassis_can_send_data, &send_mail_box);
}
void CAN_mf9025_motor_on1(void)
{
    uint32_t send_mail_box;
    chassis_mf9025_tx_message.StdId = 0x142;
    chassis_mf9025_tx_message.IDE = CAN_ID_STD;
    chassis_mf9025_tx_message.RTR = CAN_RTR_DATA;
    chassis_mf9025_tx_message.DLC = 0x08;
    mf9025_chassis_can_send_data[0] = 0x88;
    mf9025_chassis_can_send_data[1] = 0x00;
    mf9025_chassis_can_send_data[2] = 0x00;
    mf9025_chassis_can_send_data[3] = 0x00;
		mf9025_chassis_can_send_data[4] = 0x00;
    mf9025_chassis_can_send_data[5] = 0x00;
    mf9025_chassis_can_send_data[6] = 0x00;
    mf9025_chassis_can_send_data[7] = 0x00;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_mf9025_tx_message, mf9025_chassis_can_send_data, &send_mail_box);
}
/**
  * @brief          返回底盘电机 电机数据指针
  * @param[in]     
  * @retval         电机数据指针
  */

const motor_measure_t *get_chassis_motor_measure_point(int i)
{
    return &motor_chassis[i];
}





/**
  * @brief          返回mf9025底盘电机 电机数据指针
  * @param[in]     
  * @retval         电机数据指针
  */
const motor_measure_t *get_mf9025_chassis_motor_measure_point(int i)
{
    return &motor_mf9025_chassis[i];
}




//返回云台指令指针
const Gimbal_order_t *get_gimbal_order_point(void)
{
    return &gimbal_order;
}


void Gimbal_order_decode(uint32_t ID, uint8_t *can_rx_data )
{

		Un1 speed_x_set,speed_y_set,angle_z_set,yaw_sum,chassis_real_power,chassis_surplus_energy; 

		switch(ID)
		{
				//xyz bag
				case 0x301:
					
					for(uint8_t i=0;i < 4; i++)
					{
							speed_x_set.s[i] = can_rx_data[i];
							speed_y_set.s[i] = can_rx_data[i+4];
					}


					gimbal_order.chassis_x_speed_set=speed_x_set.f;
					gimbal_order.chassis_y_speed_set=speed_y_set.f;

					break;

				case 0x302:

					for(uint8_t i=0;i < 4; i++)
					{
							angle_z_set.s[i] = can_rx_data[i];
							yaw_sum.s[i] = can_rx_data[i+4];

					}

					gimbal_order.chassis_z_angle_set=angle_z_set.f;
					gimbal_order.chassis_yaw_sum=yaw_sum.f;
					

					break;


				//flag bag
				case 0x303:
					gimbal_order.chassis_init=can_rx_data[0];
					gimbal_order.leg_state=can_rx_data[1];
					gimbal_order.restart=can_rx_data[2];
					gimbal_order.lying_flag=can_rx_data[3];

					gimbal_order.sideway_flag=can_rx_data[4];
				  gimbal_order.spin_flag=can_rx_data[5];
					gimbal_order.jump_flag=can_rx_data[6];
					gimbal_order.chassis_power_limit=can_rx_data[7];
					break;

				//power bag
				case 0x304:

					for(uint8_t i=0;i < 4; i++)
					{
							chassis_real_power.s[i] = can_rx_data[i];
							chassis_surplus_energy.s[i] = can_rx_data[i+4];
					}


					gimbal_order.chassis_real_power=chassis_real_power.f;
					gimbal_order.chassis_surplus_energy=chassis_surplus_energy.f;
					break;



		}

}



//底盘发给云台函数
void CAN_SEND_TO_GIMBAL(int16_t phi1,int16_t phi4,int16_t data1,int16_t data2)
{
    uint32_t send_mail_box;
    sendToGimal_tx_message.StdId = 0x310;
    sendToGimal_tx_message.IDE = CAN_ID_STD;
    sendToGimal_tx_message.RTR = CAN_RTR_DATA;
    sendToGimal_tx_message.DLC = 0x08;
    sendToGimbal_data[0] = (phi1 >> 8);
    sendToGimbal_data[1] = phi1;
		sendToGimbal_data[0] = (phi4 >> 8);
    sendToGimbal_data[1] = phi4;
    sendToGimbal_data[0] = (data1 >> 8);
    sendToGimbal_data[1] = data1;
    sendToGimbal_data[0] = (data2 >> 8);
    sendToGimbal_data[1] = data2;
  
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &sendToGimal_tx_message, sendToGimbal_data, &send_mail_box);
}







//更新云台更新标志
void update_can_flag(void)
{
   gimbal_order.gimbal_update_flag=1;

}

//void chassis_protect(void)///每100ms运行一次用于检查can线是否更新，防止失控
//{
//	//如果无更新，则底盘失控
//	if(gimbal_order.gimbal_update_flag==0)
//	{
//	  gimbal_order.chassis_outof_control=1;
//	}
//	
//	//失控后有更新，则底盘回复控制
//	if(gimbal_order.gimbal_update_flag==1&&gimbal_order.chassis_outof_control==1)
//	{
//	
//	   gimbal_order.chassis_outof_control=0;
//	
//	}
//	//清除更新标志
//	if(gimbal_order.gimbal_update_flag==1)
//	{
//	    gimbal_order.gimbal_update_flag=0;
//	}
//	
//}

#ifdef __cplusplus
}        
#endif