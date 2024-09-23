#ifndef __MOTOR_H
#define __MOTOR_H

#include "struct_typedef.h"
#include "motor_def.h"
#include "buzzer.h"
#include "device_monitor.h"
#include "bsp_can.h"
#include "debug.h"
//�������ֵ����Լ���ֵ
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
//�������ֵת���ɽǶ�ֵ
#define MOTOR_ECD_TO_RAD 0.000766990394f   // 2*  PI  /8192
#define RPM_TO_RAD_S 0.10471976f  //r/m -> rad/s
#define MOTOR_ECD_TO_71_OUT_RAD  0.0000108f
#define MOTOR_ECD_TO_19_OUT_RAD  0.00004041700988f        //  2*  PI/8191/19
#define MOTOR_RPM_TO_19_OUT_RAD 0.005510526f   // r/m->rad/s �� 19
#define MOTOR_RPM_TO_71_OUT_RAD 0.0014749f  // r/m->rad/s �� 71
#define TO_71 0.0140845f

#ifdef __cplusplus
extern "C"{
#endif

#ifdef __cplusplus
	
//����������
enum Motor_Monitor_Type_e
{
	MOTOR_PARAM = 0,
	MOTOR_TEM = 1,
};

//�󽮵����Ϣ��
typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
    
	uint16_t last_ecd;
	int64_t difference_num; //��ֵ�������תȦ��
	
	int32_t output_encoder_num;
	uint16_t output_encoder_ecd;
	
	//�趨�˵�ת�ٱ�
	fp32 reduction_ratio;  //���ٱ�
}Dji_Motor_Measure_t;	

//����趨����
typedef struct
{
	const char *motor_name; //�������
	CAN_NAME_e can_id; //CAN��ID
	uint8_t set_id;		//�����ID��
	Motor_Type_e motor_type;		//�������
	Motor_Rotate_Direction_e direction; //�����ת����
	Reduction_Drive_Type_e reduction_type; //����������
	Control_Type_e control_type; //������ѡ������
	//ͨ������
	CAN_Rx_Instance_t motor_can;				//���CANʵ��
	//AM3508�������ݶ�ȡ
	CAN_Rx_Instance_t AMExtra_can;			//AM3508���CANʵ��
}Motor_Setting_t;

//Ϊÿ��ע�������ַ���ע
class DJIMotorInstance
{
	private:
		Motor_Watch_t motor_watch;	        //����۲���
		Motor_Setting_t *motor_settings;    //������ò���
	public:	

		Dji_Motor_Measure_t motor_measure;	//�������
		Motor_Controller_s controller;		  //������
	
		/**���캯��**/
		DJIMotorInstance();
		/**��ʼ������**/
		void DJIMotorInit(Motor_Setting_t *config);
		fp32 MotorEcdToAngle(uint16_t *ecd_angle,uint16_t offset_ecd,int8_t drive_radio);
		fp32 MotorWorkSpaceLimit(fp32 relative_angle_set,fp32 add,fp32 max_limit,fp32 min_limit);
		void ChangeControlMode(Control_Type_e change_type);
		/**���ƺ���**/
		void DJIMotorControl(fp32 *ref,fp32 *set,fp32 *motor_gyro,uint8_t filter_flag);
		void MotorZeroForce();
		fp32 PIDControl(fp32 *ref,fp32 *set,uint8_t filter_flag);
		/**���״̬��Ϣ�ӿں���**/
		fp32 CalcTotalecd();
		fp32 FilterSpeed(fp32 *fliter_num);
		fp32 GetRotorW(); //��ý��ٶ�
		fp32 GetOutputShaftW(); //����������ٶ�
		fp32 GetRotorRad();  //���ת�ӵĽǶ�
		fp32 GetOutputShaftRad(); //��������ĽǶ�
		int16_t GetRotorRpm();  //���ת��RPM
		int16_t GetGivenCurrent(); //��÷�������

		Dji_Motor_Measure_t *GetMotorMeasure();
		/**ָ���ַ����**/
		Motor_Setting_t *SetParamPointer();
		int16_t *SendCurrentPointer();
		/**CAN��Ϣ�ӿں���**/
		uint32_t GetRxID();
		CAN_NAME_e GetCANID();
		/**��⺯��**/
		void MonitorInitState();
		void MonitorTem();
		void MonitorOnlineState();
		/**��Ԫ����������Ϣ����**/
		friend void DecodeDJIMotor(CAN_Rx_Instance_t *rx_instance);
		friend void DecodeExtraAM3508Data(CAN_Rx_Instance_t *rx_instance);
};

uint8_t CheckSameID();
fp32 AbsoluteControlAddLimit(fp32 error_angle,fp32 add,fp32 relative_angle,fp32 max_limit,fp32 min_limit);
DJIMotorInstance *DJIMotorInstancePointer(uint8_t cnt);
void MonitorMotor(void);

extern void (*MotorMonitorDisplay)(Motor_Monitor_Type_e monitor_type);
#endif
	
#ifdef __cplusplus
}
#endif

#endif
