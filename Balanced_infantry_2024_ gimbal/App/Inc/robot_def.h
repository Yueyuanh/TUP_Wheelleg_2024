#ifndef __ROBOT_DEF_H
#define __ROBOT_DEF_H

#include "board_def.h"

#define OFF_SET 0
#define ON_SET 1
/****************机器人设置*******************/
#define TASK_INIT_TIME 300  //所有任务初始化时间

/***发射弹丸类型***/
#define BULLET_17 0  //17mm弹丸
#define BULLET_42 1  //42mm弹丸
#define NO_BULLET 2  //无弹丸
#define ROBOT_SHOOT_TYPE BULLET_17

/***自瞄使用方式***/
#define HANDLE_LPF 0     //一节低通滤波
#define HANDLE_KALMAN 1  //卡尔曼滤波
#define Auto_Type HANDLE_LPF

/***轮组定义***/
#define All_Mecanum 0         //麦轮
#define All_Omnidirectional 1 //全向轮
#define CHASSIS_TYPE All_Mecanum

/***超级电容开启情况***/
#define POWER_CONTROL_TYPE OFF_SET

/****************云台设置*******************/
#define YAW_OFFSET 5452   //YAW轴设定中值
#define PIT_OFFSET 6050   //PIT轴设定中值

#define MOTOR_TO_YAW_RADIO 1 //电机与YAW轴角度转换比
#define MOTOR_TO_PIT_RADIO 1 //电机与PIT轴角度转换比
#define PIT_TRANSMISSION_RADIO 1  //PIT轴传动比

#define MAX_YAW_RELATIVE 3.141f  //YAW轴最大限位//MOTOR_ECD_TO_RAD*2048.0f
#define MIN_YAW_RELATIVE -3.141f //YAW轴最小限位//-MOTOR_ECD_TO_RAD*2048.0f

#define MAX_PIT_RELATIVE 0.45f   //PIT轴最大限位
#define MIN_PIT_RELATIVE -0.51f  //PIT轴最小限位
/****************底盘设置*******************/
#define NORMAL_MAX_CHASSIS_SPEED_X 600  //底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 600  //底盘运动过程最大平移速度

// 机器人底盘修改的参数,单位为mm(毫米)————————仍未使用
#define WHEEL_BASE 350              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 300             // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 60             // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

/****************发射设置*******************/
#define USE_SENSOR OFF_SET //拨盘处是否使用传感器

/***拨盘一整圈子弹个数***/
#define WHOLE_CIRCLE_BULLET_NUM 8 
/***连发射频设置***/

#define LOW_FREQUENCE 2
#define MID_FREQUENCE 5
#define HIGH_FREQUENCE 20
#define INIT_FREQUENCE HIGH_FREQUENCE

/***拨盘数量设置***/
#define REVOLVER_NUM 1  //数量
#define MAIN_REVOLVER 0 //主拨盘id
#define ASSIST_REVOLVER 1  //辅助拨盘id

/***摩擦轮组数设置***/
#define FIRC_STAGE 1  	//摩擦轮级数
#define FIRST_STAGE 0		//一级摩擦轮
#define SECONE_STAGE 1  //二级摩擦轮

/***摩擦轮转速设置***/
#define FIRST_STAGE_RPM_SET  8200 //一级转速设置
#define SECOND_STAGE_RPM_SET 2000 //二级摩擦轮设置

/***三等级弹速设定（已无用）***/
#define LOW_RPM_SET 8200   //低转速//5000
#define MID_RPM_SET 7000   //中转速
#define HIGH_RPM_SET 9000  //高转速
#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
//
	
	
#endif
	
#ifdef __cplusplus
}
#endif

#endif
