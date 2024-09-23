/**
 ******************************************************************************
 * @file    chassis.cpp
 * @author  Xushuang
 * @version V1.0.0 �������
 * @date    2023/9/20
 * @brief		�˴�Ϊ���̸�ģʽ����
 ******************************************************************************
 * @attention
 *	�����ĸ����˳���Ѿ��涨�ã�LF��LB��RB��RF˳��Ϊ0��1��2��3����chassis_motor
 *������0-3�ֱ�Ϊ��ǰ�������ǰ���Һ��˳��ÿ���ٶȵı������Ǵ�˳��
 *	����ֲ���µ���ʱ�����ڳ���ĵ��ʵ����ʼ�������ĵ���ÿ�����ӵ�ID���ɡ�
 ******************************************************************************
 */
#include "arm_math.h"
#include "chassis.h"
#include "referee_data.h"
#include "remote_control.h"
#include "tim.h"
//���ĺ�ΪX���������ҷ�ΪY���������Ϸ�ΪZ��������

//����ʵ������
chassis_t chassis;

/********�����ʼ������*********/
Motor_Setting_t chassis_config[4] = 
{
	{"chassis_lf",ON_CAN1,1,M3508,POSITIVE_DIRECT,RATIO_1_TO_19,SINGLE_LOOP},  //LF
	{"chassis_lb",ON_CAN1,2,M3508,POSITIVE_DIRECT,RATIO_1_TO_19,SINGLE_LOOP},	 //LB
	{"chassis_rb",ON_CAN1,3,M3508,POSITIVE_DIRECT,RATIO_1_TO_19,SINGLE_LOOP},	 //RB
	{"chassis_rf",ON_CAN1,4,M3508,POSITIVE_DIRECT,RATIO_1_TO_19,SINGLE_LOOP},	 //RF
};


/**
	* @brief          ������Ϣ���ݽ���
  * @param[in]      *rx_instance�����յ�ʵ��
  * @retval         Null
  */
static void Decodechassis(CAN_Rx_Instance_t *rx_instance)
{
	uint8_t *rxbuff = rx_instance->rx_buff;

	SysPointer()->chassis_phi1=(int16_t)((rxbuff[0] << 8) |(rxbuff[1]))/10000;
	SysPointer()->chassis_phi4=(int16_t)((rxbuff[2] << 8) |(rxbuff[3]))/10000;

}



/**
	* @brief          �����๹�캯��
  * @param[in]      NULL
  * @retval         NULL
  */
chassis_t::chassis_t()
{


}

/**
	* @brief          ���̳�ʼ��
  * @param[in]      NULL
  * @retval         NULL
  */
void ChassisInit()
{
	chassis.rc_ctrl=GetRemoteControlPoint();

  fp32 chassis_yaw_pid[3] = {CHASSIS_MOTOR_YAW_PID_KP,CHASSIS_MOTOR_YAW_PID_KI,CHASSIS_MOTOR_YAW_PID_KD};  
	chassis.chassis_angle_pid.Init(PID_POSITION,chassis_yaw_pid,CHASSIS_MOTOR_YAW_PID_MAX_OUT,CHASSIS_MOTOR_YAW_PID_MAX_IOUT);
	/******��Ϣ����ʵ������******/
	CenterPointer()->PointerInit(&chassis.chassis_msg,CHASSISPUB);

/*************************************************************������Ϣ��ʼ��********************************************************************/

	chassis.chassis_msg.chassis_init=0;
	chassis.chassis_msg.leg_state=0;
	chassis.chassis_msg.restart=0;
	chassis.chassis_msg.lying_flag=0;
	chassis.chassis_msg.sideway_flag=0;
	chassis.chassis_msg.spin_flag=0;
	chassis.chassis_msg.chassis_power_limit=55;


	chassis.chassis_msg.chassis_x_speed_set=0.00f;
	chassis.chassis_msg.chassis_y_speed_set=0.00f;
	chassis.chassis_msg.chassis_z_angle_set=0.00f;

	chassis.chassis_msg.chassis_real_power=30;
	chassis.chassis_msg.chassis_surplus_energy=55;

	//���̴�������
	CANRxInitSet(&chassis.chassis_can,ON_CAN2,0x310,&chassis,Decodechassis);
	CANRxRegister(&chassis.chassis_can);




}

/**
	* @brief          ����������
  * @param[in]      NULL
  * @retval         NULL
  */
void ChassisTask()
{

  chassis.chassis_feedback();
  chassis.chassis_player();
	chassis.chassis_control_state();
  chassis.chassis_control_loop();
//	chassis.SendMsgUpdate(); //������Ϣ

}

/**
	* @brief          ������Ϣ����
  * @param[in]      NULL
  * @retval         NULL
  */
void chassis_t::chassis_feedback()
{
		chassis_msg.chassis_z_angle_set=GimbalPointer()->yaw_relative_angle;
		chassis_msg.chassis_z_num=GimbalPointer()->yaw_relative_angle_sum;

		chassis_msg.chassis_real_power=JUDGE_fGetChassisPower();
		chassis_msg.chassis_surplus_energy=JUDGE_fGetChassisVolt();


		//UI ��Ϣ����
		SysPointer()->key_flag.spin_flag=chassis_msg.spin_flag;
		SysPointer()->key_flag.side_flag=chassis_msg.sideway_flag;


}

/**
	* @brief          ���̿���״̬����
  * @param[in]      NULL
  * @retval         NULL
  */
void chassis_t::chassis_control_state()
{
//		//�ȳ�ģʽ
//		if(RC_MODE_CHANGE) chassis_msg.leg_state=1;
//		else chassis_msg.leg_state=0;



		//����ģʽ
		if(SysPointer()->rc_restart)
		{
				chassis_msg.restart=1;
		}
		else
		{
				chassis_msg.restart=0;
		}
		


}

/**
	* @brief          ���̿���ѭ��
  * @param[in]      NULL
  * @retval         NULL
  */
void chassis_t::chassis_control_loop()
{
		if(RC_CHASSIS_INIT||RC_MODE_CHANGE)
		{

				//���̳�ʼ��
				chassis_msg.chassis_init=1;//1

				//����CTRL
				if(RC_ADDITION>600) chassis_msg.rc_ctrl_down=1;
				else chassis_msg.rc_ctrl_down=0;

				if(RC_ADDITION<-600) chassis_msg.rc_ctrl_up=1;
				else chassis_msg.rc_ctrl_up=0;

				//����
				if((chassis_msg.rc_ctrl_down && RC_SIDEWAY>300&&rc_sideway_last<=300) || SysPointer()->rc_sideway_flag)
				{
							chassis_msg.sideway_flag=!chassis_msg.sideway_flag;
				}

				//��߿���
				if(chassis_msg.rc_ctrl_down)
				{
						chassis_msg.chassis_x_speed_set=0;
						chassis_msg.chassis_y_speed_set=0;

						if(RC_FORWARD<-300&&rc_forward_last>=-300)//��
						{
								if(chassis_msg.leg_state-- <=0) chassis_msg.leg_state=0;

						}

						if(RC_FORWARD>300&&rc_forward_last<=300)//��
						{
								if(chassis_msg.leg_state++ >=2) chassis_msg.leg_state=2;
						}
				}

								


				//��������
				//��Ծ��־λ
				if((RC_ADDITION<-300)) 	chassis_msg.jump_flag=1;
				else  chassis_msg.jump_flag=0;
	

				//С���ݱ�־λ
				if(RC_MODE_CHANGE||SysPointer()->rc_spin_flag)
				{
						 chassis_msg.spin_flag=1;
				}
				else
				{
							chassis_msg.spin_flag=0;
					}

				if(rc_s_right_last==1&&RC_S_RIGHT==3)
				{
						chassis_msg.spin_flag=0;
				}




				//��������
				//��ֹС���ݺͲ����ͻ
				if(chassis_msg.spin_flag) chassis_msg.sideway_flag=0;
				
				//ſ���߱�־����
				chassis_msg.lying_flag=0;

		}
		else
		{	
				//���������µ�ʱ��
						

				chassis_msg.chassis_init=0;
				chassis_msg.leg_state=0;
				chassis_msg.sideway_flag=0;
				chassis_msg.spin_flag=0;


				if(RC_STAND) chassis_msg.lying_flag=1;//
				else chassis_msg.lying_flag=0;

	
		}
	
		
		rc_addition_last=rc_ctrl->rc.ch[4];
		rc_forward_last =RC_FORWARD;
		rc_sideway_last =RC_SIDEWAY;
		rc_yaw_last     =RC_YAW;
		rc_pitch_last   =RC_PITCH;
		rc_s_right_last = RC_S_RIGHT;
}


/**
	* @brief          RC����
  * @param[in]      NULL
  * @retval         NULL
  */
void chassis_t::chassis_player()
{

		chassis_msg.chassis_x_speed_set=RC_FORWARD-SysPointer()->vx_set;
		chassis_msg.chassis_y_speed_set=RC_SIDEWAY+SysPointer()->vy_set;

		if(chassis_msg.chassis_x_speed_set>660) chassis_msg.chassis_x_speed_set=660;
		else if(chassis_msg.chassis_x_speed_set<-660) chassis_msg.chassis_x_speed_set=-660;
		if(chassis_msg.chassis_y_speed_set>660) chassis_msg.chassis_y_speed_set=660;
		else if(chassis_msg.chassis_y_speed_set<-660) chassis_msg.chassis_y_speed_set=-660;

}


// ���˶�ѧ����
//Q�ǽǶȣ�S���ٶȣ�A�Ǽ��ٶ�
void Forward_kinematics_R(fp32 Q1, fp32 Q4, fp32 S1, fp32 S4, fp32 A1, fp32 A4)
{

    fp32 L0 = 0, Q0 = 0;
    fp32 xb, xd, yb, yd, Lbd, xc, yc;
    fp32 A0, B0, C0, Q2, Q3, S2;
    fp32 vxb, vxd, vyb, vyd, vxc, vyc;
    fp32 cos_Q1, cos_Q4, sin_Q1, sin_Q4;
    fp32 S0;
    // fp32 sin_Q2, S3,cos_Q2, sin_Q3, cos_Q3;
    // fp32 axb, ayb, axd, ayd, a2, axc;
    /******************************/
    //Q1 = pi + Q1;  //�����Ƕ���ʵ����ķ����Ƕȴ���
    cos_Q1 = arm_cos_f32(Q1);
    sin_Q1 = arm_sin_f32(Q1);
    cos_Q4 = arm_cos_f32(Q4);
    sin_Q4 = arm_sin_f32(Q4);
    xb = -LEG_0 / 2 + LEG_1 * cos_Q1;
    xd =  LEG_0 / 2 + LEG_1 * cos_Q4;
    yb =  LEG_1 * sin_Q1;
    yd =  LEG_1 * sin_Q4;

    // arm_sqrt_f32((xd-xb)*(xd-xb)+(yd-yb)*(yd-yb),&Lbd);
    Lbd = sqrt((xd - xb) * (xd - xb) + (yd - yb) * (yd - yb));
    A0 = 2 * LEG_2 * (xd - xb);
    B0 = 2 * LEG_2 * (yd - yb);
    C0 = LEG_2 * LEG_2 + Lbd * Lbd - LEG_2 * LEG_2;
    Q2 = 2 * atan((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)) / (A0 + C0));

    xc = xb + arm_cos_f32(Q2) * LEG_2;
    yc = yb + arm_sin_f32(Q2) * LEG_2;
    //	arm_sqrt_f32(xc*xc +yc*yc,&L0);
    L0 = sqrt(xc * xc + yc * yc);
    Q0 = atan(yc / xc);
    
    //����Ϊ�ٶ����
    vxb = -S1 * LEG_1 * sin_Q1;
    vyb =  S1 * LEG_1 * cos_Q1;
    vxd = -S4 * LEG_1 * sin_Q4;
    vyd =  S4 * LEG_1 * cos_Q4;
    Q3 = atan((yc - yd) / (xc - xd));
    S2 = ((vxd - vxb) * arm_cos_f32(Q3) + (vyd - vyb) * arm_sin_f32(Q3)) / (LEG_2 * arm_sin_f32(Q3 - Q2));
    //S3 = ((vxd - vxb) * cos(Q2) + (vyd - vyb) * sin(Q2)) / (LEG_2 * sin(Q3 - Q2));
    vxc = vxb - S2 * LEG_2 * arm_sin_f32(Q2);
    vyc = vyb + S2 * LEG_2 * arm_cos_f32(Q2);
    S0 = 3 * (-arm_sin_f32(abs(Q0)) * vxc - arm_cos_f32(Q0) * vyc);

    if(Q0<0) Q0+=pi;


    SysPointer()->L0 = L0;
    SysPointer()->chassis_phi0 = Q0;
    SysPointer()->chassis_phi2 = Q2;
    SysPointer()->chassis_phi3 = Q3;

}






