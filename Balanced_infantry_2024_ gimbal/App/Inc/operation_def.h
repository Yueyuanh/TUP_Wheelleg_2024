#ifndef __OPERATION_DEF_H
#define __OPERATION_DEF_H

//yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL   2
#define PITCH_CHANNEL 3
#define RIGTH_CHANNEL 0
#define LEFT_CHANNEL 1
//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0

//可改灵敏度
//鼠标
#define YAW_MOUSE_SEN   0.00005f
#define PITCH_MOUSE_SEN 0.00005f
//遥控器
#define YAW_RC_SEN    -0.000005f
#define PITCH_RC_SEN  0.00000222f //0.005

//可改灵敏度
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f

//YAW左右转控制按钮
#define GIMBAL_LEFT_KEY KEY_PRESSED_OFFSET_Q
#define GIMBAL_RIGHT_KEY KEY_PRESSED_OFFSET_E

//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND   0
//摇杆死区（底盘）
#define CHASSIS_RC_DEADLINE 10
//拨轮死区
#define THUMB_WHEEL_RANGE 600

////底盘运动过程最大前进速度
//#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
////底盘运动过程最大平移速度
//#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f

#endif
