#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H


#include "struct_typedef.h"
#include "remote_control.h"
#include "pid.h"
#include "ins_task.h"
#include "LQR.h"
#include "motor_control.h"
#include "LQR_Wheelleg.h"
#include "filter.h"
#include "KalmanFilter.h"
#include "kalman_filter.h"
#include "can_receive.h"



//LPF
#define CHASSIS_BALANCE_ANGLE -7.0f
#define CHASSIS_MOTOR_LOWOUT  0.9f
#define CHASSIS_SPEED_LOWOUT  0.95f
#define CHASSIS_RC_LOWOUT     0.97f
#define CHASSIS_POSITION_LOWOUT 0.8f
#define CHASSIS_RC_POSITION_LOWOUT 0.8f

#define CHASSIS_TASK_INIT_TIME 357 //任务开始空闲一段时间

#define CHASSIS_WHEEL_HALF 0.1464f //驱动轮半径

#define MF9025_MOTOR_SPEED_PID_MAX_OUT    1900.0f
#define MF9025_MOTOR_SPEED_PID_MAX_IOUT   200.0f
#define CHASSIS_MOTOR_SPEED_PID_KP        10.0f
#define CHASSIS_MOTOR_SPEED_PID_KI        0.0f
#define CHASSIS_MOTOR_SPEED_PID_KD        0.0f
		
#define CHASSIS_BALANCE_MAX_OUT           2000.0f
#define CHASSIS_BALANCE_MAX_IOUT          200.0f
#define CHASSIS_BALANCE_PID_KP            70.0f
#define CHASSIS_BALANCE_PID_KI            0.0f
#define CHASSIS_BALANCE_PID_KD            0.0f
		
#define CHASSIS_SPEED_MAX_OUT             0.5f
#define CHASSIS_SPEED_MAX_IOUT            0.3f
#define CHASSIS_SPEED_PID_KP              0.0f
#define CHASSIS_SPEED_PID_KI              0.0f
#define CHASSIS_SPEED_PID_KD              0.0f
		
#define CHASSIS_TURN_MAX_OUT              500.0f
#define CHASSIS_TURN_MAX_IOUT             200.0f
#define CHASSIS_TURN_PID_KP               2.0f
#define CHASSIS_TURN_PID_KI               0.0f
#define CHASSIS_TURN_PID_KD               0.0f

#define CHASSIS_POSITION_MAX_OUT          22.0f
#define CHASSIS_POSITION_MAX_IOUT         10.0f
#define CHASSIS_POSITION_PID_KP           0.0f
#define CHASSIS_POSITION_PID_KI           0.0f
#define CHASSIS_POSITION_PID_KD           0.0f

#define CHASSIS_SIDEWAY_CHANNEL           0
#define CHASSIS_FORWARD_CHANNEL           1
#define CHASSIS_YAW_CHANNEL               2
#define CHASSIS_PITCH_CHANNEL							3
#define CHASSIS_ADDITION_CHANNEL          4

#define CHASSIS_VX_RC_SEN									0.0035f
#define CHASSIS_VY_RC_SEN                 0.003f
#define CHASSIS_WZ_RC_SEN									-0.0005f

#define RC_MODE_CHANGE         chassis_data.rc_ctrl->rc.s[0]==1
#define RC_CHASSIS_INIT        chassis_data.rc_ctrl->rc.s[0]==3
#define RC_ZERO_FORCE          chassis_data.rc_ctrl->rc.s[0]==2
#define RC_S_RIGHT						 chassis_data.rc_ctrl->rc.s[0]

#define RC_STAND_UP            chassis_data.rc_ctrl->rc.s[1]==1
#define RC_STAND               chassis_data.rc_ctrl->rc.s[1]==3
#define RC_STAND_DOWN          chassis_data.rc_ctrl->rc.s[1]==2
#define RC_S_LEFT							 chassis_data.rc_ctrl->rc.s[1]

#define RC_SIDEWAY						 chassis_data.rc_ctrl->rc.ch[0]
#define RC_FORWARD						 chassis_data.rc_ctrl->rc.ch[1]
#define RC_YAW						     chassis_data.rc_ctrl->rc.ch[2]
#define RC_PITCH						 	 chassis_data.rc_ctrl->rc.ch[3]
#define RC_ADDITION						 chassis_data.rc_ctrl->rc.ch[4]

#define CHASSIS_TIME_CONS      0.002f

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }	


#ifdef __cplusplus
extern "C"{
#endif



#ifdef __cplusplus



//chassis motor
typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
	fp32 speed_lpf;
	fp32 speed_lpf_last;
  fp32 speed_set;
	fp32 position;
	fp32 position_sum;
	fp32 differ_speed;
  fp32 power;
	fp32 current_set;
	fp32 current;
	fp32 current_scale;
	int16_t give_current;

}chassis_motor_t;

//ins data
typedef struct
{
	fp32 chassis_angle_yaw;
	fp32 chassis_angle_pitch;
	fp32 chassis_angle_roll;
	fp32 chassis_angle_yawsum;

	fp32 chassis_Gyro[3];  				// 角速度
  fp32 chassis_Accel[3]; 				// 加速度

}INS_data;


class leg_t
{
		public:
		//init
		float leg_angle_offset[4];
		//angle
		float leg_motor_angle[4];
		float leg_motor_angle_last[4];

};


typedef struct
{
		LPF_type_def gyro_picth_LPF;
		LPF_type_def gyro_leg_LPF[2];


		LPF_type_def wheel_speed_LPF[2];
		LPF_type_def wheel_position_LPF;

		LPF_type_def K_yaw_out;
		fp32 K_yaw_lpf;

		Kalman_fir_t World_accel_y;
		LPF_type_def World_accel_y_LPF;
		float TQ,TR,Accel_y_LPF;

		KalmanFilter_t Foot_distance_Kalman_R;
		KalmanFilter_t Foot_distance_Kalman_L;
		float Foot_Q[2];

	

		LPF_type_def foot_force_R,foot_force_L;

		LPF_type_def RC_forward_LPF;
		LPF_type_def RC_sideway_LPF;

		differ_type_def wheel_speed_differ[2];

}chassis_filter_t;

typedef struct
{
		int S_right_last;
		int S_left_last;
		int RC_sideway_last;
		int RC_forward_last;
		int RC_yaw_last;
		int RC_pitch_last;
		int RC_addition_last;

		fp32 chassis_forward_channel;
		fp32 chassis_sideway_chaneel;
		fp32 chassis_yaw_channel;

}chassis_RC_control;

//typedef struct
//{
//		//flag bag
//		uint8_t chassis_init;	//0：无力   1：初始化
//		uint8_t leg_state;		//0: 小板凳 1：正常 2：站立
//		uint8_t restart;      //0: 无效   1：C板重启
//		uint8_t lying_flag;	  //0: 无效		1：趴着前进标志位

//		uint8_t sideway_flag; //0: 正对   1：侧身
//		uint8_t spin_flag;		//0：正常   1：小陀螺		
//		uint8_t jump_flag;		//0：正常   1：跳跃
//		uint8_t chassis_power_limit;//功率上限

//		//fp bag
//		fp32 chassis_x_speed_set;		//底盘正向速度设定
//		fp32 chassis_y_speed_set;		//底盘侧向速度设定
//		//fp32 chassis_x_distance_set;//底盘正向移动距离设定
//		//fp32 chassis_y_distance_set;//底盘侧向移动距离设定
//		fp32 chassis_z_angle_set;		//底盘角度设定
//		fp32 chassis_yaw_sum;								//底盘相对圈数
//		//power bag
//		fp32 chassis_real_power;		//底盘实时功率
//		fp32 chassis_surplus_energy;//底盘剩余能量

//		uint8_t gimbal_update_flag; //云台更新标志
//		uint8_t spin_flag_last;

//			//拨轮标志位
//		bool_t rc_ctrl_down;
//		bool_t rc_ctrl_up;

//} Chassis_order_t;



//lqr
class chassis_data_t
{
		public:
	
		chassis_filter_t chassis_filter;
		chassis_RC_control RC;

		//硬件层
		const RC_ctrl_t *rc_ctrl;
		INS_data chassis_INS_data;

	  MOTOR_send cmd[4]; 
		MOTOR_recv data[4];
		chassis_motor_t chassis_9025_motor[2];

		// 观测器数据
    float yaw, pitch, roll,yaw_sum,yaw_error,yaw_set,yaw_add;                // 2 1 0
    float accel_x, accel_y, accel_z;       // 0 1 2
		float world_accel_x,world_accel_y,world_accel_z;
		float world_accel_y_kalman,world_accel_y_lpf;
    float gyro_yaw, gyro_pitch, gyro_roll; //

    float leg_angle[2];   // 腿相对于竖直方向的角度
    float leg_gyro[2];    //腿转的角加速度
    float leg_length[2];  // 腿的长度

    float leg_Torque[4];    // 关节电机反馈力矩
    float leg_position[4]; // 关节电机反馈角度
    float leg_speed[4];    // 关节电机反馈转速

    float wheel_Torque[2];         // 驱动轮电机反馈力矩
    float wheel_position[2];       // 驱动轮电机反馈角度
    float wheel_speed[2];          // 驱动轮电机速度
    float wheel_speed_lpf[2];      // 驱动轮电机速度低通
		float wheel_speed_kalman[2];   // 驱动轮电机速度卡尔曼
    float wheel_distance[2];       // 驱动轮位移
		float wheel_distance_kalman[2];// 驱动轮位移卡尔曼
		float wheel_position_offset[2];// 驱动轮初始值


    float foot_distance;
		float foot_distance_lpf;
		float foot_distance_kalman;
    float foot_speed;
    float foot_speed_lpf;
    float foot_speed_kalman;
		float foot_accel;
		float foot_roll_angle;
		float foot_T[2],foot_T_lpf[2],foot_force_lpf[2];

    float free_flag;             //是否悬空
    float leg_stand_F;           //跳跃推力

		float chassis_bias;					 //底盘角度偏转
		float chassis_bias_down;
		float chassis_bias_up;
		float chassis_wheel_distance;

    // 期望值设置
		const Gimbal_order_t *gimbal_order;
		Gimbal_order_t chassis_order;
		uint8_t spin_flag_last;

    float pitch_set, roll_set;       // 三轴角度设定
    float leg_base_length;                        // 基础腿长
    float leg_length_set[2];                  // 双腿长度设置
		float foot_speed_set;
    float foot_distance_set;                  // 前进距离设定
		float vx_set,vy_set;											//水平速度设置


		int chassis_spin_state;										//侧身或者小陀螺
    //LQR输出
    float K_balance_T[2];
    float K_coordinate_T;
    float K_stand_T[2];
    float K_roll_T[2];

    float Wheel_motor_T[2];

    // 控制器
		uint8_t control_dis_loop;//0:速度环 1：位置环
		uint8_t control_dis_loop_last;//0:速度环 1：位置环

    float F[2];
    float Tp[2];

    float T[4];       // 关节电机输出力矩
    float T_wheel[2]; // 轮毂电机输出力矩


		//jump
		int jump_flag;
		int JUMP_FEED;
		int jump_count[3];
		int jump_T[3];

		//RC flag
		int16_t rc_addition_last;
		int16_t rc_forward_last;
		int16_t rc_sideway_last;
		int16_t rc_yaw_last;
		int16_t rc_pitch_last;
		bool_t  rc_s_right_last;




		/*function*/
		chassis_data_t();										//底盘数据初始化
		void chassis_feedback(void);				//底盘数据反馈
		void chassis_control_state(void);		//底盘状态机
		void chassis_control_loop(void);		//底盘控制循环

		//state function
		void chassis_zero_control_set(void);//底盘无力控制
		void chassis_sideway_control_set(void); //底盘侧向模式
    void chassis_stand_down_mode(void);//小板凳模式
		void chassis_stand_mode(void);//正常操作模式
		void chassis_stand_up_mode(void);//站立模式
    void chassis_jump_mode(void);//跳跃模式
		void chassis_auto_length(void);//腿长自适应

		void chassis_getAbsoluteSpeedR(void);//绝对速度融合
		void chassis_getAbsoluteSpeedL(void);

		//player control
		void chassis_player_order(void);

		//go1_w_set
		void chassis_go1_W_set(float);

		//下板单独控制
		void Chassis_single_control();

};






#endif

void chassis_task(void const * argument);
void chassis_jump_task(void const * argument);

void LegInit(void);


void KalmanFilter_R_init();
void KalmanFilter_L_init();



extern void CAN_mf9025_cmd_chassis_Torque_control_2(int16_t motor1);
extern void CAN_mf9025_cmd_chassis_Torque_control_1(int16_t motor1);
extern const motor_measure_t *get_mf9025_chassis_motor_measure_point(int i);
extern void CAN_mf9025_cmd_chassis_Torque_control_R(int32_t motor1);
extern void CAN_mf9025_cmd_chassis_Torque_control_L(int32_t motor1);

extern void CAN_SEND_TO_GIMBAL(int16_t phi1,int16_t phi4,int16_t data1,int16_t data2);


extern float RAMP_float( float final, float now, float ramp );
extern HAL_StatusTypeDef SERVO_Send_recv(MOTOR_send *pData, MOTOR_recv *rData);


extern const RC_ctrl_t *get_remote_control_point(void);
extern const INS_t *get_ins_measure_point(void);
extern const Gimbal_order_t *get_gimbal_order_point(void);
const Gimbal_order_t *get_chassis_order_point(void);


#ifdef __cplusplus
}
#endif
#endif
