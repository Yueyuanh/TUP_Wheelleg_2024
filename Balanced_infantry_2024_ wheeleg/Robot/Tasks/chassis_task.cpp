#include "chassis_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "user_lib.h"
#include "bsp_usart.h"
#include "VMC.h"
#include "arm_math.h"

chassis_data_t chassis_data;
leg_t     leg_move;

extern VMC_t VMC_LEG_R,VMC_LEG_L;
extern LQR_Wheelleg LQR;

chassis_data_t* chassispoint(void)
{
	return &chassis_data;
}



void chassis_task(void const * argument)
{
//		usart1_init();
		//空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);


		while(1)
		{
				chassis_data.chassis_feedback();
				chassis_data.chassis_player_order();
				chassis_data.chassis_control_loop();

				CAN_mf9025_cmd_chassis_Torque_control_1((int16_t)chassis_data.chassis_9025_motor[0].give_current);
				CAN_mf9025_cmd_chassis_Torque_control_2(-(int16_t)chassis_data.chassis_9025_motor[1].give_current);

				SERVO_Send_recv(&chassis_data.cmd[0], &chassis_data.data[0]);	//将控制指令发送给电机，同时接收返回值
				SERVO_Send_recv(&chassis_data.cmd[1], &chassis_data.data[1]);	//

				SERVO_Send_recv(&chassis_data.cmd[2], &chassis_data.data[2]);	//
				SERVO_Send_recv(&chassis_data.cmd[3], &chassis_data.data[3]);	//

				vTaskDelay(2);
		}
}



/**
  * @brief          底盘初始化
  * @param[in]      none
  * @retval         none
  */
chassis_data_t::chassis_data_t()
{
		/*硬件初始化*/
		//rc init
		rc_ctrl=get_remote_control_point();

		//上下板
		//gimbal_order=get_gimbal_order_point();
		
		//下板
		gimbal_order=get_chassis_order_point();

	  //feedback init
		chassis_9025_motor[0].chassis_motor_measure=get_mf9025_chassis_motor_measure_point(0);
		chassis_9025_motor[1].chassis_motor_measure=get_mf9025_chassis_motor_measure_point(1);

		//motor init
		chassis_data.cmd[0].id=1; 			//给电机控制指令结构体赋值
		chassis_data.cmd[0].mode=1;
		chassis_data.cmd[0].T=0;
		chassis_data.cmd[0].W=0;
		chassis_data.cmd[0].Pos=0;
		chassis_data.cmd[0].K_P=0;
		chassis_data.cmd[0].K_W=0.002;//0.05

		chassis_data.cmd[1].id=2; 			//给电机控制指令结构体赋值
		chassis_data.cmd[1].mode=1;
		chassis_data.cmd[1].T=0;
		chassis_data.cmd[1].W=0;
		chassis_data.cmd[1].Pos=0;
		chassis_data.cmd[1].K_P=0;
		chassis_data.cmd[1].K_W=0.002;

		chassis_data.cmd[2].id=3; 			//给电机控制指令结构体赋值
		chassis_data.cmd[2].mode=1;
		chassis_data.cmd[2].T=0;
		chassis_data.cmd[2].W=0;
		chassis_data.cmd[2].Pos=0;
		chassis_data.cmd[2].K_P=0;
		chassis_data.cmd[2].K_W=0.002;

		chassis_data.cmd[3].id=4; 			//给电机控制指令结构体赋值
		chassis_data.cmd[3].mode=1;
		chassis_data.cmd[3].T=0;
		chassis_data.cmd[3].W=0;
		chassis_data.cmd[3].Pos=0;
		chassis_data.cmd[3].K_P=0;
		chassis_data.cmd[3].K_W=0.002;


		/*滤波器初始化*/
		//pitch角速度低通滤波
		LPF_init(&chassis_filter.gyro_picth_LPF,CHASSIS_TIME_CONS,0.8);
		LPF_init(&chassis_filter.gyro_leg_LPF[0],CHASSIS_TIME_CONS,0.2);
		LPF_init(&chassis_filter.gyro_leg_LPF[1],CHASSIS_TIME_CONS,0.2);


		//轮子速度微分器
	  LPF_init(&chassis_filter.wheel_speed_LPF[0],CHASSIS_TIME_CONS,CHASSIS_MOTOR_LOWOUT);
	  LPF_init(&chassis_filter.wheel_speed_LPF[1],CHASSIS_TIME_CONS,CHASSIS_MOTOR_LOWOUT);

	  LPF_init(&chassis_filter.K_yaw_out,CHASSIS_TIME_CONS,0.7);



		//轮子转速微分器
		differentiator_init(&chassis_filter.wheel_speed_differ[0]);
		differentiator_init(&chassis_filter.wheel_speed_differ[1]);
		
		

		//底盘卡尔曼滤波

		KalmanFilter_R_init();
		KalmanFilter_L_init();

		chassis_filter.TQ=1;
		chassis_filter.TR=1;
		Kalman_fir_init(&chassis_filter.World_accel_y,chassis_filter.TQ,chassis_filter.TR);
		chassis_filter.Accel_y_LPF=0.8;
		LPF_init(&chassis_filter.World_accel_y_LPF,CHASSIS_TIME_CONS,chassis_filter.Accel_y_LPF);

		//遥控器滤波
		LPF_init(&chassis_filter.RC_forward_LPF,CHASSIS_TIME_CONS,0.7);
		LPF_init(&chassis_filter.RC_sideway_LPF,CHASSIS_TIME_CONS,0.7);

		//足端压力
		foot_force_lpf[0]=0.99;
		foot_force_lpf[1]=0.99;
		LPF_init(&chassis_filter.foot_force_R,CHASSIS_TIME_CONS,foot_force_lpf[0]);
		LPF_init(&chassis_filter.foot_force_L,CHASSIS_TIME_CONS,foot_force_lpf[1]);


		/*控制器初始化*/


		control_dis_loop=1;
		
		//底盘偏转（暂时）
		chassis_bias_down=0.08;//0.06
		chassis_bias=0.07;//0.05
		chassis_bias_up=0.04;//0.04
	  chassis_wheel_distance=0.26f;


		chassis_spin_state=0;
		chassis_data.yaw_add=0.7f;


		//轮子初始化位置
		chassis_data.wheel_position_offset[0]=wheel_distance[0];
		chassis_data.wheel_position_offset[1]=wheel_distance[1];		
		
		//VMC
		VMC_init();

		//LQR
		LQR.init();

}



/**
  * @brief          底盘相关数据反馈
  * @param[in]      none
  * @retval         none
  */
void chassis_data_t::chassis_feedback()
{
		/*硬件数据更新*/
		//motor data feedback
		chassis_9025_motor[0].speed=chassis_9025_motor[0].chassis_motor_measure->speed_rpm;
		chassis_9025_motor[0].speed=chassis_9025_motor[0].speed*angle2rad*0.075f;
		chassis_9025_motor[1].speed=chassis_9025_motor[1].chassis_motor_measure->speed_rpm;
		chassis_9025_motor[1].speed=chassis_9025_motor[1].speed*angle2rad*0.075f;

		for(int i=0;i<2;i++)
		{

		//两个轮子位置 转化为弧度制
		chassis_9025_motor[i].position=chassis_data.chassis_9025_motor[i].chassis_motor_measure->ecd;
		chassis_9025_motor[i].position=pi*(chassis_9025_motor[i].position/65535);

		chassis_9025_motor[0].position_sum=(chassis_9025_motor[0].position)+chassis_9025_motor[0].chassis_motor_measure->num*pi;
		chassis_9025_motor[1].position_sum=(-chassis_9025_motor[1].position)-chassis_9025_motor[1].chassis_motor_measure->num*pi;
	
		//轮子速度<-位置微分
		chassis_9025_motor[i].differ_speed=differ_calc(&chassis_filter.wheel_speed_differ[i],chassis_9025_motor[i].position_sum,10,CHASSIS_TIME_CONS,1);

		//两个轮子移动距离
	  wheel_distance[i]=chassis_9025_motor[i].position_sum*CHASSIS_WHEEL_HALF-wheel_position_offset[i];
	
		//轮子速度低通滤波
		chassis_9025_motor[i].speed_lpf=LPF_calc(&chassis_filter.wheel_speed_LPF[i],chassis_9025_motor[i].speed);

		wheel_speed[i]=chassis_9025_motor[i].speed;

		}


		//车体速度需要融合传感器 两轮速度原始数据相反
		foot_speed_lpf=(chassis_9025_motor[0].speed_lpf-chassis_9025_motor[1].speed_lpf)/2;

		//foot_distance+=foot_speed_lpf*0.001f;
		foot_speed=(wheel_speed[0]+wheel_speed[1])/2;
		foot_distance=(wheel_distance[0]+wheel_distance[1])/2;
		

		//ins feedback
		chassis_INS_data.chassis_angle_pitch=get_ins_measure_point()->Pitch;
		chassis_INS_data.chassis_angle_yaw=get_ins_measure_point()->Yaw;
		chassis_INS_data.chassis_angle_roll=get_ins_measure_point()->Roll;
		chassis_INS_data.chassis_angle_yawsum=get_ins_measure_point()->YawTotalAngle;
		chassis_INS_data.chassis_Accel[0]=get_ins_measure_point()->Accel[0];
		chassis_INS_data.chassis_Accel[1]=get_ins_measure_point()->Accel[1];
		chassis_INS_data.chassis_Accel[2]=get_ins_measure_point()->Accel[2];
		chassis_INS_data.chassis_Gyro[0]=get_ins_measure_point()->Gyro[0];
		chassis_INS_data.chassis_Gyro[1]=get_ins_measure_point()->Gyro[1];
		chassis_INS_data.chassis_Gyro[2]=get_ins_measure_point()->Gyro[2];

		//角度转化为弧度
		chassis_data.yaw=chassis_INS_data.chassis_angle_yaw;
		chassis_data.yaw_sum=chassis_INS_data.chassis_angle_yawsum;


//		if(gimbal_order->spin_flag)
//		{
//				chassis_data.yaw_error = chassis_data.yaw_sum;
//		}
//		else
//		{
//				chassis_data.yaw_error=-chassis_data.gimbal_order->chassis_z_angle_set*rad2angle;//上板反馈
//		}
//		

				chassis_data.yaw_error = chassis_data.yaw_sum;



		chassis_data.pitch=chassis_INS_data.chassis_angle_pitch*angle2rad;
		chassis_data.roll=chassis_INS_data.chassis_angle_roll*angle2rad;

		//消除重力加速度
		chassis_data.accel_x=chassis_INS_data.chassis_Accel[X]+arm_sin_f32(roll)*9.81f;
		chassis_data.accel_y=chassis_INS_data.chassis_Accel[Y]-arm_sin_f32(pitch)*arm_cos_f32(roll)*9.81f;
		chassis_data.accel_z=chassis_INS_data.chassis_Accel[Z]-arm_cos_f32(roll)*arm_cos_f32(pitch)*9.81f;

		//求出世界坐标系加速度
		chassis_data.world_accel_x=accel_x*arm_cos_f32(roll)  + accel_z*arm_sin_f32(roll);
		chassis_data.world_accel_y=accel_x*arm_sin_f32(pitch) + accel_y*arm_cos_f32(pitch) - accel_z*arm_sin_f32(pitch)*arm_cos_f32(roll);
		chassis_data.world_accel_z=accel_x*arm_cos_f32(pitch)*arm_sin_f32(roll)          + accel_y*arm_sin_f32(pitch) + accel_z*arm_cos_f32(pitch)*arm_cos_f32(roll);

		chassis_data.world_accel_y_kalman=KalmanFilter_fir(&chassis_filter.World_accel_y,world_accel_y);
		chassis_data.world_accel_y_lpf=LPF_calc(&chassis_filter.World_accel_y_LPF,world_accel_y);

		//角速度
		chassis_data.gyro_yaw=chassis_INS_data.chassis_Gyro[Z];
		chassis_data.gyro_roll=chassis_INS_data.chassis_Gyro[Y];
		chassis_data.gyro_pitch=chassis_INS_data.chassis_Gyro[X];
		chassis_data.gyro_pitch=LPF_calc(&chassis_filter.gyro_picth_LPF,chassis_data.gyro_pitch);



		/*关节电机*/
		//leg motor pos  phi4_max=-25  phi1_max=205
		leg_move.leg_motor_angle[0]= (data[0].Pos-leg_move.leg_angle_offset[0])*1.75f-25;
		leg_move.leg_motor_angle[1]= (data[1].Pos-leg_move.leg_angle_offset[1])*1.75f+205;
		leg_move.leg_motor_angle[2]= (data[2].Pos-leg_move.leg_angle_offset[2])*1.75f-25;
		leg_move.leg_motor_angle[3]= (data[3].Pos-leg_move.leg_angle_offset[3])*1.75f+205;

		for(int i=0;i<4;i++)
		{
    		leg_Torque[i]=data[i].T;
				leg_position[i]=leg_move.leg_motor_angle[i];
				leg_speed[i]=data[i].W;
		}
		
		
		/*VMC计算*/
		VMC_LEG_L.Phi1=leg_move.leg_motor_angle[1]/rad2angle;
		VMC_LEG_L.Phi4=leg_move.leg_motor_angle[0]/rad2angle;
		VMC_LEG_R.Phi1=leg_move.leg_motor_angle[3]/rad2angle;
		VMC_LEG_R.Phi4=leg_move.leg_motor_angle[2]/rad2angle;

		//leg motor speed
		VMC_LEG_L.motor_speed_1=chassis_data.data[1].W;
		VMC_LEG_L.motor_speed_4=chassis_data.data[0].W;
		VMC_LEG_R.motor_speed_1=chassis_data.data[3].W;
		VMC_LEG_R.motor_speed_4=chassis_data.data[2].W;

		Forward_kinematics_L(VMC_LEG_L.Phi1,VMC_LEG_L.Phi4,VMC_LEG_L.motor_speed_1,VMC_LEG_L.motor_speed_4,0,0);
		Forward_kinematics_R(VMC_LEG_R.Phi1,VMC_LEG_R.Phi4,VMC_LEG_R.motor_speed_1,VMC_LEG_R.motor_speed_4,0,0);

		//Leg length
		leg_length[0]=VMC_LEG_R.L0;
		leg_length[1]=VMC_LEG_L.L0;
		leg_angle[0]=VMC_LEG_R.Phi0;
		leg_angle[1]=VMC_LEG_L.Phi0;
		leg_gyro[0]=VMC_LEG_R.Phi0_gyro-gyro_pitch;
		leg_gyro[1]=VMC_LEG_L.Phi0_gyro+gyro_pitch;
		
		leg_gyro[0]=LPF_calc(&chassis_filter.gyro_leg_LPF[0],leg_gyro[0]);
		leg_gyro[1]=LPF_calc(&chassis_filter.gyro_leg_LPF[1],leg_gyro[1]);
	

		foot_T[0]=data[2].T-data[3].T;
		foot_T[1]=data[0].T-data[1].T;
		foot_T_lpf[0]=LPF_calc(&chassis_filter.foot_force_R,foot_T[0]);
		foot_T_lpf[1]=LPF_calc(&chassis_filter.foot_force_L,foot_T[1]);

		if(foot_T_lpf[0]<0&&foot_T_lpf[1]<0) free_flag=1;
		else free_flag=0;
		
		//free_flag=0;

		/*机器人移动信息*/

		//底盘卡尔曼滤波计算

		
		//卡尔曼滤波传入数据
		chassis_filter.Foot_distance_Kalman_R.MeasuredVector[0]=wheel_distance[0];
		chassis_filter.Foot_distance_Kalman_R.MeasuredVector[1]=wheel_speed[0];
		chassis_filter.Foot_distance_Kalman_R.MeasuredVector[2]=world_accel_y_lpf;


		chassis_filter.Foot_distance_Kalman_L.MeasuredVector[0]=wheel_distance[1];
		chassis_filter.Foot_distance_Kalman_L.MeasuredVector[1]=wheel_speed[1];
		chassis_filter.Foot_distance_Kalman_L.MeasuredVector[2]=world_accel_y_lpf;


		//获取卡尔曼返回值
		float *kalman_value_R,*kalman_value_L;
		kalman_value_R=Kalman_Filter_Update(&chassis_filter.Foot_distance_Kalman_R);
		kalman_value_L=Kalman_Filter_Update(&chassis_filter.Foot_distance_Kalman_L);

		//返回值赋值
		wheel_distance_kalman[0]=kalman_value_R[0];
		wheel_distance_kalman[1]=kalman_value_L[0];
		wheel_speed_kalman[0]=kalman_value_R[1];
		wheel_speed_kalman[1]=kalman_value_L[1];


		//和速度/距离计算
		foot_distance_kalman=(kalman_value_R[0]+kalman_value_L[0])/2;
		foot_speed_kalman=(kalman_value_R[1]+kalman_value_L[1])/2;
		foot_accel=kalman_value_R[2]+kalman_value_L[2];

		//驱动轮电机加速度（与机体的加速度进行对比，并在一段时间内对加速度进行积分作为机体在这段时间内的位移）
		




		//send to gimbal

		//CAN_SEND_TO_GIMBAL(VMC_LEG_R.Phi1*10000,VMC_LEG_R.Phi4*10000,0,0);
}

/**
  * @brief          底盘状态设定(左拨杆（暂定）)
  * @param[in]      none
  * @retval         none
  */
void chassis_data_t::chassis_control_state()
{
		uint8_t leg_length_state;
		leg_length_state=chassis_data.gimbal_order->leg_state;

		if(leg_length_state==0)//小板凳模式
		{
				leg_base_length=0.18;

				chassis_stand_down_mode();

		}
		else if(leg_length_state==1)//正常模式
		{
				leg_base_length=0.25;

				chassis_stand_mode();

		}
		else if(leg_length_state==2)//站立模式
		{
				leg_base_length=0.32;

				chassis_stand_up_mode();
		}

		//基础腿长
		leg_length_set[0]=leg_base_length;
		leg_length_set[1]=leg_base_length;


		//关节电机阻尼设置
			if(chassis_data.jump_flag)
			{
				chassis_data.chassis_go1_W_set(0);
			}
			else
			{
				chassis_data.chassis_go1_W_set(0.002);
			}

}
/**
  * @brief          底盘控制量计算
  * @param[in]      none
  * @retval         none
  */

void chassis_data_t::chassis_control_loop()
{
		//底盘初始化
		if(gimbal_order->chassis_init)
		{
				//init yaw angle
				if(RC.S_right_last==0&&gimbal_order->chassis_init==1)
				{
							chassis_data.yaw_set=0;
							chassis_data.wheel_position_offset[0]=wheel_distance[0];
							chassis_data.wheel_position_offset[1]=wheel_distance[1];
							
							KalmanFilter_R_init();
							KalmanFilter_L_init();

							//复位
							LegInit();

							chassis_data.foot_distance_set=chassis_data.foot_distance;

				}
	

				//模式控制
				chassis_control_state();

				//腿长自适应
				chassis_auto_length();
			
				chassis_data.Tp[0]=chassis_data.K_balance_T[0]-chassis_data.K_coordinate_T;
				chassis_data.Tp[1]=chassis_data.K_balance_T[1]+chassis_data.K_coordinate_T;
				chassis_data.F[0] =chassis_data.K_stand_T[0]+chassis_data.K_roll_T[0];
				chassis_data.F[1] =chassis_data.K_stand_T[1]-chassis_data.K_roll_T[1];



				//VMC 
				VMC_calc(F[0],Tp[0],F[1],Tp[1]);



				//关节电机输出（限幅）
				LimitMax(VMC_LEG_L.T[0],10);
				LimitMax(VMC_LEG_L.T[1],10);
				LimitMax(VMC_LEG_R.T[0],10);
				LimitMax(VMC_LEG_R.T[1],10);

				cmd[1].T=VMC_LEG_L.T[0];
				cmd[0].T=VMC_LEG_L.T[1];
				cmd[3].T=VMC_LEG_R.T[0];
			  cmd[2].T=VMC_LEG_R.T[1];


				//驱动轮电机扭矩转化、输出限幅
				//chassis give current

				//由9025电机参数可知，转矩电流与电机输出扭矩存在转换系数
				//即：Iq=K*Tp (16匝)   K=195.3125f

//				chassis_9025_motor[0].give_current=chassis_data.Wheel_motor_T[0]*CHASSIS_MF9025_T_K;
//				chassis_9025_motor[1].give_current=chassis_data.Wheel_motor_T[1]*CHASSIS_MF9025_T_K;
//	
				chassis_9025_motor[0].give_current=chassis_data.Wheel_motor_T[0];
				chassis_9025_motor[1].give_current=chassis_data.Wheel_motor_T[1];
	
				LimitMax(chassis_9025_motor[0].give_current,1875);
				LimitMax(chassis_9025_motor[1].give_current,1875);
			

				//跳跃轮毂置零
				//if(chassis_data.jump_flag) chassis_jump_mode();



		}
		//无力模式
		else if(gimbal_order->chassis_init==0)  //疯车标志位
		{
				chassis_zero_control_set();
		}




		//rc_change 遥控器变化


		chassis_data.RC.S_right_last=gimbal_order->chassis_init;
		chassis_data.RC.RC_sideway_last=gimbal_order->chassis_y_speed_set;

		chassis_data.RC.S_left_last=RC_S_LEFT;

		chassis_data.RC.RC_forward_last=RC_FORWARD;
		chassis_data.RC.RC_yaw_last=RC_YAW;
		chassis_data.RC.RC_pitch_last=RC_PITCH;

}



/**
  * @brief          小板凳模式
  * @param[in]      none
  * @retval         none
  */
fp32 balance_LQR_sit_down[2][6]={{-10,-0.19,0,2.5,10,0.3},
												        {3000,5,0,75,10000,1500}};

fp32 balance_LQR_sit_down_dis[2][6]={{-10,-0.19,0,2.5,10,0.3},
												        {3000,5,0,75,10000,1500}};


fp32 balance_LQR_sit_down_free[2][6]={{-10,-0.19,0,0,0,0},
												              {0,0,0,0,0,0}};


fp32 balance_LQR_sit_down_spin[2][6]={{-10,-0.19,0,0,10,0.3},
												              {3000,5,0,75,10000,1500}};

fp32 balance_LQR_sit_down_lying[2][6]={{-10,-0.19,0,0,10,0.3},
												              {3000,5,0,75,10000,1500}};



void chassis_data_t::chassis_stand_down_mode()
{
		if(free_flag)
		{	
	      LQR.calc(balance_LQR_sit_down_free,chassis_bias_down);
		}
		else if(chassis_data.gimbal_order->sideway_flag)
		{
				LQR.calc(balance_LQR_sit_down,chassis_bias_down);
		}
		else if(chassis_data.gimbal_order->spin_flag)
		{
				LQR.calc(balance_LQR_sit_down_spin,chassis_bias_down);
		}
		else if(control_dis_loop)
		{
				LQR.calc(balance_LQR_sit_down_dis,chassis_bias_down);
		}
		else
		{
			  //LQR
		    LQR.calc(balance_LQR_sit_down,chassis_bias_down);
		}


	
			
}

/**
  * @brief          正常操作模式
  * @param[in]      none
  * @retval         none
  */
fp32 balance_LQR[2][6]={{-20,-0.1,0,2.7,10,0},
												{2000,0,0,200,7000,1500}};

fp32 balance_LQR_dis[2][6]={{-20,-0.1,0,2.7,10,0},
												    {2000,0,0,200,7000,1500}};


fp32 balance_LQR_free[2][6]={{-20,-0.12,0,0,0,0},
												{0,0,0,0,0,0}};

fp32 balance_LQR_spin[2][6]={{-20,-0.12,0,0,10,0.3},//小陀螺腿部对speed,pitc不响应
												     {2000,0,0,200,7000,1500}};


void chassis_data_t::chassis_stand_mode()
{
		if(free_flag)
		{
				LQR.calc(balance_LQR_free,chassis_bias);
		}
		else if(chassis_data.gimbal_order->sideway_flag)
		{
				LQR.calc(balance_LQR,chassis_bias);
		}
		else if(chassis_data.gimbal_order->spin_flag)
		{
				LQR.calc(balance_LQR_spin,chassis_bias);
		}
		else if(control_dis_loop)
		{
				LQR.calc(balance_LQR_dis,chassis_bias);
		}
		else
		{
				LQR.calc(balance_LQR,chassis_bias);
		}

}

/**
  * @brief          站立模式
  * @param[in]      none
  * @retval         none
  */
fp32 balance_LQR_stand_up[2][6]={{-12,-0.08,0,1,10,0.3},
												         {2500,5,0,75,8000,1500}};

fp32 balance_LQR_stand_up_dis[2][6]={{-12,-0.08,0,1,10,0.3},
												            {2500,5,0,75,8000,1500}};

fp32 balance_LQR_stand_up_free[2][6]={{-12,-0.08,0,0,0,0},
												         {0,0,0,0,0,0}};

fp32 balance_LQR_stand_up_spin[2][6]={{-12,-0.08,0,0,10,0.3},
												         {2500,5,0,75,7000,1500}};


void chassis_data_t::chassis_stand_up_mode()
{
		if(free_flag)
	  {
				LQR.calc(balance_LQR_stand_up_free,chassis_bias_up);	
	  }
		else if(chassis_data.gimbal_order->spin_flag)
		{
				LQR.calc(balance_LQR_stand_up_spin,chassis_bias_up);	
		}
		else if(control_dis_loop)
		{
				LQR.calc(balance_LQR_stand_up_dis,chassis_bias_up);
		}
	  else
		{
		  	LQR.calc(balance_LQR_stand_up,chassis_bias_up);	
		}

}

/**
  * @brief          底盘无力模式
  * @param[in]      none
  * @retval         none
  */
void chassis_data_t::chassis_zero_control_set()
{
		//轮毂电机电流置零
		chassis_9025_motor[0].give_current=0;
		chassis_9025_motor[1].give_current=0;

		//关节电机无力
		cmd[0].T=0;
		cmd[1].T=0;
		cmd[2].T=0;
		cmd[3].T=0;

		//jump_flag=0;
}



/**
  * @brief          跳跃模式
  * @param[in]      none
  * @retval         none
  */
void chassis_data_t::chassis_jump_mode()
{
		//leg angle

		//轮毂电机电流置零
		chassis_9025_motor[0].give_current=0;
		chassis_9025_motor[1].give_current=0;

}
/**
  * @brief          腿长自适应
  * @param[in]      none
  * @retval         none
  */
void chassis_data_t::chassis_auto_length()
{
		//机器人参数 两轮间距0.52  两侧腿长=基础腿长+-（1/2）* 0.52 * tan(roll)		
		//chassis_data.foot_roll_angle=roll+atan(2*(leg_length[0]-leg_length[1]));
		chassis_data.foot_roll_angle=roll;

		leg_length_set[1] = leg_base_length + chassis_wheel_distance * arm_sin_f32(foot_roll_angle)/arm_cos_f32(foot_roll_angle);
		leg_length_set[0] = leg_base_length - chassis_wheel_distance * arm_sin_f32(foot_roll_angle)/arm_cos_f32(foot_roll_angle);

}

/**
  * @brief          跳跃任务
  * @param[in]      none
  * @retval         none
  */
void chassis_jump_task(void const * argument)
{


		chassis_data.jump_count[0]=100;
		chassis_data.jump_count[1]=200;
		chassis_data.jump_count[2]=150;
		chassis_data.jump_T[0]=0;
		chassis_data.jump_T[1]=800;
		chassis_data.jump_T[2]=1000;

		while(1)
		{
			if(chassis_data.jump_flag)
			{

				chassis_data.chassis_go1_W_set(0);
				//先下蹲
//				chassis_data.JUMP_FEED=-chassis_data.jump_T[1];
//				osDelay(chassis_data.jump_count[1]);

				//蹬腿
				chassis_data.JUMP_FEED=chassis_data.jump_T[1];
				osDelay(chassis_data.jump_count[1]);

				//缩腿
				chassis_data.JUMP_FEED=-chassis_data.jump_T[2];
				osDelay(chassis_data.jump_count[2]);
				
        //跳跃完成
				chassis_data.jump_flag=0;
			}


		vTaskDelay(2);
		}
}

/**
  * @brief          操作控制
  * @param[in]      none
  * @retval         none
	* @addition
	*
  */
void chassis_data_t::chassis_player_order()
{
		//单板控制
		Chassis_single_control();


//		RC.chassis_forward_channel=RC_FORWARD*CHASSIS_VX_RC_SEN;
//		RC.chassis_sideway_chaneel=RC_SIDEWAY*CHASSIS_VY_RC_SEN;
//		RC.chassis_yaw_channel=RC_YAW*CHASSIS_WZ_RC_SEN;

		RC.chassis_forward_channel=chassis_data.gimbal_order->chassis_x_speed_set*CHASSIS_VX_RC_SEN;
		RC.chassis_sideway_chaneel=chassis_data.gimbal_order->chassis_y_speed_set*CHASSIS_VY_RC_SEN;

		vx_set=LPF_calc(&chassis_filter.RC_forward_LPF,RC.chassis_forward_channel);
		vy_set=LPF_calc(&chassis_filter.RC_sideway_LPF,RC.chassis_sideway_chaneel);
		
		//跳跃标志
		if(gimbal_order->jump_flag&&gimbal_order->leg_state!=2) chassis_data.jump_flag=1;

		//控制模式
		float min_speed=0.001;
		if(fabs(vx_set)>min_speed||fabs(vy_set)>min_speed)      control_dis_loop=0;//控速
		else if(fabs(vx_set)<min_speed&&fabs(vy_set)<min_speed) control_dis_loop=1;//控位移

		if(control_dis_loop_last==0&&control_dis_loop==1) foot_distance_set=foot_distance;

		control_dis_loop_last=control_dis_loop;





		if(gimbal_order->chassis_init)
		{

//				if(chassis_data.spin_flag_last==0&&gimbal_order->spin_flag==1)
//				{
//						yaw_set=yaw_error;
//				}

//				//侧身
//				if(chassis_data.gimbal_order->sideway_flag) chassis_data.yaw_set=-90;
//				//小陀螺
//				else if(chassis_data.gimbal_order->spin_flag)
//				{
//						//yaw_set=yaw_error;
//						chassis_data.yaw_set+=yaw_add;
//				}		
//				//正常状态
//				else 
//				{
//						chassis_data.yaw_set=0;
//				}

				//初始化
				if(gimbal_order->chassis_init_last) chassis_data.yaw_set=chassis_data.yaw_error;
				
				//小陀螺回正
				if(chassis_data.spin_flag_last==0&&gimbal_order->spin_flag==1)
				{
						yaw_set=yaw_error;
				}
				//小陀螺
				else if(chassis_data.gimbal_order->spin_flag)
				{
						//yaw_set=yaw_error;
						chassis_data.yaw_set+=yaw_add;
				}		
				//正常状态
				else 
				{
						chassis_data.yaw_set=gimbal_order->chassis_z_angle_set;
				}



				//侧身移动设置
				if(chassis_data.gimbal_order->sideway_flag)
				{
						  chassis_data.foot_speed_set=vy_set;
				}
				//小陀螺移动设置
				else if(chassis_data.gimbal_order->spin_flag)
				{
							chassis_data.foot_speed_set=
																				+vy_set*arm_cos_f32(chassis_data.gimbal_order->chassis_yaw_sum)*4+
																				-vx_set*arm_sin_f32(chassis_data.gimbal_order->chassis_yaw_sum)*4;
				}
				else  chassis_data.foot_speed_set=vx_set;

				
				chassis_data.spin_flag_last=gimbal_order->spin_flag;








				//yaw轴限幅
//				if(chassis_data.yaw_set>90) chassis_data.yaw_set=90;
//				else if(chassis_data.yaw_set<-90) chassis_data.yaw_set=-90;

		}
		else
		{
				//保护逻辑
				if(chassis_data.gimbal_order->restart)
				{
							HAL_NVIC_SystemReset();
				}
		}


}

/**
  * @brief          关节电机初始化
  * @param[in]      none
  * @retval         none
  */
void LegInit(void)
{

		float init_T=0.3f;//初始化扭矩

		chassis_data.cmd[0].T=-init_T;
		chassis_data.cmd[1].T= init_T;
		chassis_data.cmd[2].T=-init_T;
		chassis_data.cmd[3].T= init_T;

		osDelay(100);
		SERVO_Send_recv(&chassis_data.cmd[0], &chassis_data.data[0]);	//将控制指令发送给电机，同时接收返回值
		SERVO_Send_recv(&chassis_data.cmd[1], &chassis_data.data[1]);	//
		SERVO_Send_recv(&chassis_data.cmd[2], &chassis_data.data[2]);	//
		SERVO_Send_recv(&chassis_data.cmd[3], &chassis_data.data[3]);	//
		osDelay(2000);
		SERVO_Send_recv(&chassis_data.cmd[0], &chassis_data.data[0]);	//将控制指令发送给电机，同时接收返回值
		SERVO_Send_recv(&chassis_data.cmd[1], &chassis_data.data[1]);	//
		SERVO_Send_recv(&chassis_data.cmd[2], &chassis_data.data[2]);	//
		SERVO_Send_recv(&chassis_data.cmd[3], &chassis_data.data[3]);	//

		if(fabs(chassis_data.data[0].T-(-init_T))<0.02f)
		{
				leg_move.leg_angle_offset[0]=chassis_data.data[0].Pos;
		}
		if(fabs(chassis_data.data[1].T-(init_T))<0.02f)
		{
				leg_move.leg_angle_offset[1]=chassis_data.data[1].Pos;
		}
		if(fabs(chassis_data.data[2].T-(-init_T))<0.02f)
		{
				leg_move.leg_angle_offset[2]=chassis_data.data[2].Pos;
		}
		if(fabs(chassis_data.data[3].T-(init_T))<0.02f)
		{
				leg_move.leg_angle_offset[3]=chassis_data.data[3].Pos;
		}

}

void chassis_data_t::chassis_go1_W_set(float k_w_set)
{
        //关节电机阻尼设置
	    	chassis_data.cmd[0].K_W=k_w_set;
		    chassis_data.cmd[1].K_W=k_w_set;
				chassis_data.cmd[2].K_W=k_w_set;
				chassis_data.cmd[3].K_W=k_w_set;
}



/**
  * @brief          IMU融合加速度计获取绝对速度/位移
  * @param[in]      none
  * @retval         none
	* @addition
	*
  */

float mat_R_diagonal_elements[3] = {30, 25, 35};

void KalmanFilter_R_init()
{
	 static float dt=0.002f;

	 static float P_Init[9] =
     {
         10, 0, 0, 
         0, 30, 0, 
         0, 0, 10, 
     };
     static float F_Init[9] =
     {
         1, dt, 0.5f*dt*dt, 
         0, 1, dt, 
         0, 0, 1, 
     };
     static float Q_Init[9] =
     {
         0.25f*dt*dt*dt*dt, 0.5f*dt*dt*dt, 0.5f*dt*dt, 
         0.5f*dt*dt*dt,        dt*dt,         dt, 
         0.5f*dt*dt,              dt,         1, 
     };
 
     // 设置最小方差
     static float state_min_variance[3] = {0.03, 0.005, 0.1};
     
     // 开启自动调整
     chassis_data.chassis_filter.Foot_distance_Kalman_R.UseAutoAdjustment = 1;
 
     // 轮子位移 轮子速度 加速度计测得y轴运动加速度
     static uint8_t measurement_reference[3] = {1, 2, 3};

     static float measurement_degree[3] = {1, 1, 1};
     // 根据measurement_reference与measurement_degree生成H矩阵如下（在当前周期全部测量数据有效情况下）
     //  |1   0   0|
     //  |0   1   0|
     //  |0   0   1|
 
//     static float mat_R_diagonal_elements[3] = {30, 25, 35};
     //根据mat_R_diagonal_elements生成R矩阵如下（在当前周期全部测量数据有效情况下）
     //  |30   0   0|
     //  | 0  25   0|
     //  | 0   0  35|
 
     Kalman_Filter_Init(&chassis_data.chassis_filter.Foot_distance_Kalman_R, 3, 0, 3);
 
     // 设置矩阵值
     memcpy(chassis_data.chassis_filter.Foot_distance_Kalman_R.P_data, P_Init, sizeof(P_Init));
     memcpy(chassis_data.chassis_filter.Foot_distance_Kalman_R.F_data, F_Init, sizeof(F_Init));
     memcpy(chassis_data.chassis_filter.Foot_distance_Kalman_R.Q_data, Q_Init, sizeof(Q_Init));
     memcpy(chassis_data.chassis_filter.Foot_distance_Kalman_R.MeasurementMap, measurement_reference, sizeof(measurement_reference));
     memcpy(chassis_data.chassis_filter.Foot_distance_Kalman_R.MeasurementDegree, measurement_degree, sizeof(measurement_degree));
     memcpy(chassis_data.chassis_filter.Foot_distance_Kalman_R.MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
     memcpy(chassis_data.chassis_filter.Foot_distance_Kalman_R.StateMinVariance, state_min_variance, sizeof(state_min_variance));
}

void KalmanFilter_L_init()
{
	 static float dt=0.002f;

	 static float P_Init[9] =
     {
         10, 0, 0, 
         0, 30, 0, 
         0, 0, 10, 
     };
     static float F_Init[9] =
     {
         1, dt, 0.5f*dt*dt, 
         0, 1, dt, 
         0, 0, 1, 
     };
     static float Q_Init[9] =
     {
         0.25f*dt*dt*dt*dt, 0.5f*dt*dt*dt, 0.5f*dt*dt, 
         0.5f*dt*dt*dt,        dt*dt,         dt, 
         0.5f*dt*dt,              dt,         1, 
     };
 
     // 设置最小方差
     static float state_min_variance[3] = {0.03, 0.005, 0.1};
     
     // 开启自动调整
     chassis_data.chassis_filter.Foot_distance_Kalman_L.UseAutoAdjustment = 1;
 
     // 气压测得高度 GPS测得高度 加速度计测得z轴运动加速度
     static uint8_t measurement_reference[3] = {1, 1, 3};

     static float measurement_degree[3] = {1, 1, 1};
     // 根据measurement_reference与measurement_degree生成H矩阵如下（在当前周期全部测量数据有效情况下）
     //  |1   0   0|
     //  |0   1   0|
     //  |0   0   1|
 
    // static float mat_R_diagonal_elements[3] = {30, 25, 35};
     //根据mat_R_diagonal_elements生成R矩阵如下（在当前周期全部测量数据有效情况下）
     //  |30   0   0|
     //  | 0  25   0|
     //  | 0   0  35|
 
     Kalman_Filter_Init(&chassis_data.chassis_filter.Foot_distance_Kalman_L, 3, 0, 3);
 
     // 设置矩阵值
     memcpy(chassis_data.chassis_filter.Foot_distance_Kalman_L.P_data, P_Init, sizeof(P_Init));
     memcpy(chassis_data.chassis_filter.Foot_distance_Kalman_L.F_data, F_Init, sizeof(F_Init));
     memcpy(chassis_data.chassis_filter.Foot_distance_Kalman_L.Q_data, Q_Init, sizeof(Q_Init));
     memcpy(chassis_data.chassis_filter.Foot_distance_Kalman_L.MeasurementMap, measurement_reference, sizeof(measurement_reference));
     memcpy(chassis_data.chassis_filter.Foot_distance_Kalman_L.MeasurementDegree, measurement_degree, sizeof(measurement_degree));
     memcpy(chassis_data.chassis_filter.Foot_distance_Kalman_L.MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
     memcpy(chassis_data.chassis_filter.Foot_distance_Kalman_L.StateMinVariance, state_min_variance, sizeof(state_min_variance));
}


//仅下板控制，重写gimbal_order

void chassis_data_t::Chassis_single_control()
{
		chassis_order.chassis_x_speed_set=RC_FORWARD;
		chassis_order.chassis_y_speed_set=RC_SIDEWAY;

		if(chassis_order.chassis_x_speed_set>660) chassis_order.chassis_x_speed_set=660;
		else if(chassis_order.chassis_x_speed_set<-660) chassis_order.chassis_x_speed_set=-660;
		if(chassis_order.chassis_y_speed_set>660) chassis_order.chassis_y_speed_set=660;
		else if(chassis_order.chassis_y_speed_set<-660) chassis_order.chassis_y_speed_set=-660;


		chassis_order.chassis_z_angle_set+=RC_YAW*CHASSIS_WZ_RC_SEN;


	if(RC_CHASSIS_INIT||RC_MODE_CHANGE)
		{

				//底盘初始化
				chassis_order.chassis_init=1;//1

				//拨轮CTRL
				if(RC_ADDITION>600) chassis_order.rc_ctrl_down=1;
				else chassis_order.rc_ctrl_down=0;
				if(RC_ADDITION<-600) chassis_order.rc_ctrl_up=1;
				else chassis_order.rc_ctrl_up=0;


				//身高控制
				if(chassis_order.rc_ctrl_down)
				{
						chassis_order.chassis_x_speed_set=0;
						chassis_order.chassis_y_speed_set=0;

						if(RC_FORWARD<-300&&rc_forward_last>=-300)//下
						{
								if(chassis_order.leg_state-- <=0) chassis_order.leg_state=0;

						}

						if(RC_FORWARD>300&&rc_forward_last<=300)//上
						{
								if(chassis_order.leg_state++ >=2) chassis_order.leg_state=2;
						}
				}



//				//侧身
//				if((chassis_order->rc_ctrl_down && RC_SIDEWAY>300&&rc_sideway_last<=300) || SysPointer()->rc_sideway_flag)
//				{
//							chassis_order->sideway_flag=!chassis_msg.sideway_flag;
//				}
				 chassis_order.sideway_flag=0;


				//按键设置
				//跳跃标志位
				if((RC_ADDITION<-300)) 	chassis_order.jump_flag=1;
				else  chassis_order.jump_flag=0;
	

				//小陀螺标志位
				if(RC_MODE_CHANGE)
				{
						 chassis_order.spin_flag=1;
				}
				else
				{
							chassis_order.spin_flag=0;
					}

				if(rc_s_right_last==1&&RC_S_RIGHT==3)
				{
						chassis_order.spin_flag=0;
				}




				//按键保护
				//防止小陀螺和侧身冲突
				if(chassis_order.spin_flag) chassis_order.sideway_flag=0;
				
				//趴着走标志清零
				chassis_order.lying_flag=0;

		}
		else
		{	
				//当拨杆在下的时候
						

				chassis_order.chassis_init=0;
				chassis_order.leg_state=0;
				chassis_order.sideway_flag=0;
				chassis_order.spin_flag=0;


				if(RC_STAND) chassis_order.lying_flag=1;//
				else chassis_order.lying_flag=0;

	
		}
	
		chassis_order.chassis_init_last=chassis_order.chassis_init;
		rc_addition_last=rc_ctrl->rc.ch[4];
		rc_forward_last =RC_FORWARD;
		rc_sideway_last =RC_SIDEWAY;
		rc_yaw_last     =RC_YAW;
		rc_pitch_last   =RC_PITCH;
		rc_s_right_last = RC_S_RIGHT;
}

//返回云台指令指针
const Gimbal_order_t *get_chassis_order_point(void)
{
    return &chassis_data.chassis_order;
}
