#ifndef __OPERATION_DEF_H
#define __OPERATION_DEF_H

//yaw,pitch����ͨ���Լ�״̬����ͨ��
#define YAW_CHANNEL   2
#define PITCH_CHANNEL 3
#define RIGTH_CHANNEL 0
#define LEFT_CHANNEL 1
//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0

//�ɸ�������
//���
#define YAW_MOUSE_SEN   0.00005f
#define PITCH_MOUSE_SEN 0.00005f
//ң����
#define YAW_RC_SEN    -0.000005f
#define PITCH_RC_SEN  0.00000222f //0.005

//�ɸ�������
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.006f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.005f

//YAW����ת���ư�ť
#define GIMBAL_LEFT_KEY KEY_PRESSED_OFFSET_Q
#define GIMBAL_RIGHT_KEY KEY_PRESSED_OFFSET_E

//����ǰ�����ҿ��ư���
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define RC_DEADBAND   0
//ҡ�����������̣�
#define CHASSIS_RC_DEADLINE 10
//��������
#define THUMB_WHEEL_RANGE 600

////�����˶��������ǰ���ٶ�
//#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
////�����˶��������ƽ���ٶ�
//#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f

#endif
