#ifndef __COMMUNICATE_TASK_H
#define __COMMUNICATE_TASK_H

#include "user_lib.h"
#include "vision.h"
#include "gimbal.h"

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
	
typedef struct
{
    float YawGet_KF;
    float YawTarget_KF;

    float PitchGet_KF;
    float PitchTarget_KF;

    float DistanceGet_KF;
    float DistanceTarget_KF;
} Visual_TypeDef;

class Vision_process_t
{
	public:
		QueueObj speed_queue;
    QueueObj accel_queue;
    QueueObj dis_queue;
    Visual_TypeDef  data_kal;
	  float predict_angle;
    float feedforwaurd_angle;
    float speed_get;
    float accel_get;
    float distend_get;
    float offset_yaw;
    float offset_pitch;
    bool  gyro_anti;
    bool  gyro_judge;	
	
	  float yaw_target_raw;
	  int attack_switch_ramp;
	  bool kalman_init_state;  
	
	  float yaw_2kal;
	  float yaw_1kal;
	
	  bool calc_v_state;
	  //UI
	  fp32 fx,fy;  //相机焦距参数
	  fp32 cx,cy;  //相机光心——图像中心坐标（960，540）
		fp32 f;
	  fp32 x_value,y_value,z_value;
	  fp32 predict_x_value,predict_y_value,predict_z_value;
	
	  Vision_process_t();
	  /*******视觉数据读取**********/
	  void Vision_Normal();
	  void Vision_get_info();
	  void Vision_kalman_update();
	  void Vision_info_infer();
		void Target_angle_update();
	  /********预测**********/
	  void Vision_Pridict();
		/******************/
		void Anti_Target();
		void AntiGyro();
		void AntiNormal();
		//UI
		void calc_x_y_coordinate();
	  void calc_pre_x_y_coordinate();
		void calc_follow_radius();
		void change_mode();
};
extern float YawTarget_now,PitchTarget_now;//实际视觉给定角度
extern float last_last_update_cloud_yaw,lastupdate_cloud_yaw,lastupdate_cloud_pitch,update_cloud_yaw,update_cloud_pitch ;
extern Vision_process_t Vision_process;

#endif

extern void communi_task(void const *pvParameters);
#ifdef __cplusplus
}	
#endif

#endif
