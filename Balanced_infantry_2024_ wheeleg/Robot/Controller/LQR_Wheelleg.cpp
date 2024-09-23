#include "LQR_Wheelleg.h"
#include "chassis_task.h"
#include "cmath"

//对象实例化
LQR_Wheelleg LQR;
extern chassis_data_t chassis_data;


void LQR_Wheelleg::init()
{
		const float coordinate_pd[3] = {0, 0, 0};//0.7
    coordinate_PD.init(PID_POSITION, coordinate_pd, 20, 0);


    float stand_pid[3] = {40,0,5};
    stand_PID_R.init(PID_POSITION, stand_pid, 20, 10);
    stand_PID_L.init(PID_POSITION, stand_pid, 20, 10);
    LQR.stand_feed =1;               //前馈推力 等于 高度稳定状态下扭矩大小 130 15KG

    //这个roll是用来补偿转弯时的偏向力
    //float roll_PD[3] = {25, 0.3, 1.5};//75            
    float roll_PD[3] = {0, 0, 0};//75            
    roll_PID.init(PID_POSITION, roll_PD, 20, 10);

    //转向环，直接给到轮子的扭矩
    float yaw_pd[3] = {15, 0, 3500};//
    yaw_PD.init(PID_POSITION, yaw_pd, 1900, 0);


		//底盘初始化
    chassis_data.foot_distance_set = 0;

    chassis_data.pitch_set = 0;
    chassis_data.yaw_set = 0;
    chassis_data.roll_set = 0;
    chassis_data.leg_base_length = 0.22;
			
}
void LQR_Wheelleg::calc(const fp32 lqr[2][6],float bias)
{



/***********************************************************PID*********************************************************/

		//协调力矩->Tp
		chassis_data.K_coordinate_T=coordinate_PD.calc(chassis_data.leg_angle[0]-chassis_data.leg_angle[1],0);

		//站立力矩->F
		chassis_data.K_stand_T[0]=LQR.stand_feed+stand_PID_R.calc(chassis_data.leg_length[0],chassis_data.leg_length_set[0]);
		chassis_data.K_stand_T[1]=LQR.stand_feed+stand_PID_L.calc(chassis_data.leg_length[1],chassis_data.leg_length_set[1]);




		if(!chassis_data.free_flag)
		{
				//横滚补偿->F
				chassis_data.K_roll_T[0]=LQR.roll_PID.calc(chassis_data.roll,chassis_data.roll_set);
				chassis_data.K_roll_T[1]=LQR.roll_PID.calc(chassis_data.roll,chassis_data.roll_set);

				//yaw ->wheel_T
				LQR.K_yaw_out=yaw_PD.calc(chassis_data.yaw_error,chassis_data.yaw_set);
				LQR.K_yaw_out=LPF_calc(&chassis_data.chassis_filter.K_yaw_out,LQR.K_yaw_out);

		}
		else
		{
				//悬空状态下不跟yaw，不自适应

				chassis_data.K_roll_T[0]=0;
				chassis_data.K_roll_T[1]=0;
	
				LQR.K_yaw_out=0;
		}



/***********************************************************PID*********************************************************/

/*********************************************************状态变量******************************************************/

 // 关节电机平衡所用扭矩 Tp
 // 也就是说，这里的反馈只是定义了各个反馈量的正方向，而参数的正负已经定了，就是K阵的方向（取反）
    LQR.LQR_FEED_R[0][0] = (pi / 2 - chassis_data.leg_angle[0])+bias; //
    LQR.LQR_FEED_R[0][1] = (-chassis_data.leg_gyro[0]);          // 如何确定？单给轮的pitch，然后leg_angle给点，再确定leg_gyro的方向，这里改的并不是参数，而是物理系统的方向
    LQR.LQR_FEED_R[0][2] = (chassis_data.foot_distance_set - chassis_data.foot_distance);
    LQR.LQR_FEED_R[0][3] = (chassis_data.foot_speed_lpf-chassis_data.foot_speed_set); // 方向待定--这里又是轮和Tp的方向不一样，所以在Tp上做特化
    LQR.LQR_FEED_R[0][4] = (chassis_data.pitch+0.03);
    LQR.LQR_FEED_R[0][5] = (chassis_data.gyro_pitch);

    LQR.LQR_FEED_L[0][0] = (pi / 2 - chassis_data.leg_angle[1])-bias;
    LQR.LQR_FEED_L[0][1] = (-chassis_data.leg_gyro[1]);
    LQR.LQR_FEED_L[0][2] = (chassis_data.foot_distance_set - chassis_data.foot_distance);
    LQR.LQR_FEED_L[0][3] = (chassis_data.foot_speed_lpf-chassis_data.foot_speed_set);
    LQR.LQR_FEED_L[0][4] = (chassis_data.pitch+0.03);
    LQR.LQR_FEED_L[0][5] = (chassis_data.gyro_pitch);

/*********************************************************状态变量******************************************************/

/********************************************************腿输出计算******************************************************/
     LQR.LQR_OUT_R[0][0] = -lqr[0][0] *  LQR.LQR_FEED_R[0][0];
     LQR.LQR_OUT_R[0][1] = -lqr[0][1] *  LQR.LQR_FEED_R[0][1];
     LQR.LQR_OUT_R[0][2] = +lqr[0][2] * -LQR.LQR_FEED_R[0][2];
     LQR.LQR_OUT_R[0][3] = +lqr[0][3] * -LQR.LQR_FEED_R[0][3];
     LQR.LQR_OUT_R[0][4] = -lqr[0][4] * -LQR.LQR_FEED_R[0][4];
     LQR.LQR_OUT_R[0][5] = -lqr[0][5] * -LQR.LQR_FEED_R[0][5];
		
     LQR.LQR_OUT_L[0][0] = -lqr[0][0] * LQR.LQR_FEED_L[0][0];
     LQR.LQR_OUT_L[0][1] = -lqr[0][1] * LQR.LQR_FEED_L[0][1];
     LQR.LQR_OUT_L[0][2] = +lqr[0][2] * LQR.LQR_FEED_L[0][2];
     LQR.LQR_OUT_L[0][3] = +lqr[0][3] * LQR.LQR_FEED_L[0][3];
     LQR.LQR_OUT_L[0][4] = -lqr[0][4] * LQR.LQR_FEED_L[0][4];
     LQR.LQR_OUT_L[0][5] = -lqr[0][5] * LQR.LQR_FEED_L[0][5];

		 //累计输出
     chassis_data.K_balance_T[0] =
             + LQR.LQR_OUT_R[0][0] 
             + LQR.LQR_OUT_R[0][1] 
             + LQR.LQR_OUT_R[0][2] 
             + LQR.LQR_OUT_R[0][3] 
             + LQR.LQR_OUT_R[0][4] 
             + LQR.LQR_OUT_R[0][5];

     chassis_data.K_balance_T[1] =
             + LQR.LQR_OUT_L[0][0] 
						 + LQR.LQR_OUT_L[0][1] 
             + LQR.LQR_OUT_L[0][2] 
             + LQR.LQR_OUT_L[0][3] 
             + LQR.LQR_OUT_L[0][4] 
             + LQR.LQR_OUT_L[0][5];


/********************************************************腿输出计算******************************************************/

/********************************************************轮输出计算******************************************************/
// 这里，本身leg和wheel共用一个feedback
     LQR.LQR_OUT_R[1][0] = +lqr[1][0] * LQR.LQR_FEED_R[0][0]; // 这里轮和leg_angle是正反馈，但是参数是负反馈，所以需要反一下
     LQR.LQR_OUT_R[1][1] = +lqr[1][1] * LQR.LQR_FEED_R[0][1];
     LQR.LQR_OUT_R[1][2] = +lqr[1][2] * LQR.LQR_FEED_R[0][2];
     LQR.LQR_OUT_R[1][3] = -lqr[1][3] * LQR.LQR_FEED_R[0][3];
     LQR.LQR_OUT_R[1][4] = -lqr[1][4] * LQR.LQR_FEED_R[0][4];
     LQR.LQR_OUT_R[1][5] = -lqr[1][5] * LQR.LQR_FEED_R[0][5];

     chassis_data.Wheel_motor_T[0] =
             + LQR.LQR_OUT_R[1][0] 
						 + LQR.LQR_OUT_R[1][1] 
						 + LQR.LQR_OUT_R[1][2] 
						 + LQR.LQR_OUT_R[1][3] 
						 + LQR.LQR_OUT_R[1][4] 
						 + LQR.LQR_OUT_R[1][5] 
						 - LQR.K_yaw_out;

     LQR.LQR_OUT_L[1][0] = -lqr[1][0] * LQR.LQR_FEED_L[0][0];
     LQR.LQR_OUT_L[1][1] = -lqr[1][1] * LQR.LQR_FEED_L[0][1];
     LQR.LQR_OUT_L[1][2] = +lqr[1][2] * LQR.LQR_FEED_L[0][2];
     LQR.LQR_OUT_L[1][3] = -lqr[1][3] * LQR.LQR_FEED_L[0][3];
     LQR.LQR_OUT_L[1][4] = -lqr[1][4] * LQR.LQR_FEED_L[0][4];
     LQR.LQR_OUT_L[1][5] = -lqr[1][5] * LQR.LQR_FEED_L[0][5];

     chassis_data.Wheel_motor_T[1] =
             + LQR.LQR_OUT_L[1][0] 
						 + LQR.LQR_OUT_L[1][1] 
						 + LQR.LQR_OUT_L[1][2] 
						 + LQR.LQR_OUT_L[1][3] 
						 + LQR.LQR_OUT_L[1][4] 
						 + LQR.LQR_OUT_L[1][5] 
						 + LQR.K_yaw_out;












/********************************************************轮输出计算******************************************************/



}





