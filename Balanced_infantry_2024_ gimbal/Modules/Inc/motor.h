#ifndef __MOTOR_H
#define __MOTOR_H

#include "struct_typedef.h"
#include "motor_def.h"
#include "buzzer.h"
#include "device_monitor.h"
#include "bsp_can.h"
#include "debug.h"
//电机编码值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
//电机编码值转化成角度值
#define MOTOR_ECD_TO_RAD 0.000766990394f   // 2*  PI  /8192
#define RPM_TO_RAD_S 0.10471976f  //r/m -> rad/s
#define MOTOR_ECD_TO_71_OUT_RAD  0.0000108f
#define MOTOR_ECD_TO_19_OUT_RAD  0.00004041700988f        //  2*  PI/8191/19
#define MOTOR_RPM_TO_19_OUT_RAD 0.005510526f   // r/m->rad/s ÷ 19
#define MOTOR_RPM_TO_71_OUT_RAD 0.0014749f  // r/m->rad/s ÷ 71
#define TO_71 0.0140845f

#ifdef __cplusplus
extern "C"{
#endif

#ifdef __cplusplus
	
//电机监测类型
enum Motor_Monitor_Type_e
{
	MOTOR_PARAM = 0,
	MOTOR_TEM = 1,
};

//大疆电机信息包
typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
    
	uint16_t last_ecd;
	int64_t difference_num; //差值计算的旋转圈数
	
	int32_t output_encoder_num;
	uint16_t output_encoder_ecd;
	
	//设定乘的转速比
	fp32 reduction_ratio;  //减速比
}Dji_Motor_Measure_t;	

//电机设定参数
typedef struct
{
	const char *motor_name; //电机名称
	CAN_NAME_e can_id; //CAN线ID
	uint8_t set_id;		//电机的ID号
	Motor_Type_e motor_type;		//电机类型
	Motor_Rotate_Direction_e direction; //电机旋转方向
	Reduction_Drive_Type_e reduction_type; //减速箱类型
	Control_Type_e control_type; //控制器选择类型
	//通用数据
	CAN_Rx_Instance_t motor_can;				//电机CAN实例
	//AM3508额外数据读取
	CAN_Rx_Instance_t AMExtra_can;			//AM3508电机CAN实例
}Motor_Setting_t;

//为每个注册电机加字符标注
class DJIMotorInstance
{
	private:
		Motor_Watch_t motor_watch;	        //电机观测器
		Motor_Setting_t *motor_settings;    //电机设置参数
	public:	

		Dji_Motor_Measure_t motor_measure;	//电机数据
		Motor_Controller_s controller;		  //控制器
	
		/**构造函数**/
		DJIMotorInstance();
		/**初始化函数**/
		void DJIMotorInit(Motor_Setting_t *config);
		fp32 MotorEcdToAngle(uint16_t *ecd_angle,uint16_t offset_ecd,int8_t drive_radio);
		fp32 MotorWorkSpaceLimit(fp32 relative_angle_set,fp32 add,fp32 max_limit,fp32 min_limit);
		void ChangeControlMode(Control_Type_e change_type);
		/**控制函数**/
		void DJIMotorControl(fp32 *ref,fp32 *set,fp32 *motor_gyro,uint8_t filter_flag);
		void MotorZeroForce();
		fp32 PIDControl(fp32 *ref,fp32 *set,uint8_t filter_flag);
		/**电机状态信息接口函数**/
		fp32 CalcTotalecd();
		fp32 FilterSpeed(fp32 *fliter_num);
		fp32 GetRotorW(); //获得角速度
		fp32 GetOutputShaftW(); //获得输出轴角速度
		fp32 GetRotorRad();  //获得转子的角度
		fp32 GetOutputShaftRad(); //获得输出轴的角度
		int16_t GetRotorRpm();  //获得转子RPM
		int16_t GetGivenCurrent(); //获得反馈电流

		Dji_Motor_Measure_t *GetMotorMeasure();
		/**指针地址函数**/
		Motor_Setting_t *SetParamPointer();
		int16_t *SendCurrentPointer();
		/**CAN信息接口函数**/
		uint32_t GetRxID();
		CAN_NAME_e GetCANID();
		/**监测函数**/
		void MonitorInitState();
		void MonitorTem();
		void MonitorOnlineState();
		/**友元函数进行信息解码**/
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
