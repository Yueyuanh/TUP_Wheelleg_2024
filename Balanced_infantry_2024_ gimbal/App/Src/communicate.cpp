#include "arm_math.h"
#include "communicate.h"
#include "cmsis_os.h"
#include "system.h"

//��ʱδʹ�ã�����������

//�Ӿ����ݴ���ͳһ������
Vision_process_t Vision_process;

float YawTarget_now,PitchTarget_now;//ʵ���Ӿ������Ƕ�
float update_cloud_yaw = 0,update_cloud_pitch=0;	/*��¼�Ӿ���������ʱ����̨���ݣ����´ν�����*/
float lastupdate_cloud_yaw = 0,lastupdate_cloud_pitch=0; /*ǰ��֡������*/
float last_last_update_cloud_yaw =0;// ,last_last_update_cloud_pitch = 0;


///**
//  * @brief          �Ӿ���������
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void communi_task(void const *pvParameters)
//{
//	vTaskDelay(200);
//	uint32_t currentTime;
//	//�൱��vision_init
//	vision_info.RxPacket.is_spinning = 0;
//	while(1)
//	{
//		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
//		/*********�Ӿ���������*******/
////		vision_info.Collect_shoot_info(1);
//		vision_info.Vision_Send_Data(Vision_mode);
//		
//		//UI���ݼ���
//		Vision_process.Vision_get_info();
//		Vision_process.calc_x_y_coordinate();
//		Vision_process.calc_pre_x_y_coordinate();
//		Vision_process.calc_follow_radius();
//		
//		/*******2023����Ӣ��δʹ��******/
////		Vision_process.Vision_Normal();
//		
////		//����С���ݾ����տ�ʱʹ��
////		Vision_process.AntiNormal();
//	
//	  vTaskDelay(2);		//2ms
//	}
//	
//}

/**
  * @brief          ���캯����ʼ��
  * @param[in]      none
  * @param[in]      none
  * @retval         none
  */
Vision_process_t::Vision_process_t()
{
	speed_queue.queueLength = 60;
	accel_queue.queueLength = 60;
	dis_queue.queueLength = 60;
	attack_switch_ramp = 30;
	//ͼ���������
	fx = 883.3170;
	fy = 882.6257;
	f = sqrt(fx*fx + fy*fy);
	//ͼ����������
	cx = 959.8267;
	cy = 480;
}

/**
  * @brief          ͨ���Ӿ����ݶ�ȡ�ʹ���
  * @param[in]      none
  * @param[in]      none
  * @retval         none
  */
void Vision_process_t::Vision_Normal()
{
	/******�Ӿ����ݻ�ȡ*******/
	Vision_get_info();
	Target_angle_update();
	/******���Ӿ��Ƕ��������������˲�*******/
//	Vision_kalman_update();
//	/******�Ӿ���������*******/
//  Vision_info_infer();  	
//	vision_info.State.rx_data_update = false;
	
}

/**
  * @brief          ��ȡ�Ӿ�����
  * @param[in]      none
  * @param[in]      none
  * @retval         none
  */
void Vision_process_t::Vision_get_info()
{
//	vision_info.Vision_Error_Angle_Yaw(&(gimbal_point()->gimbal_kalman.Auto_Error_Yaw));
//  vision_info.Vision_Error_Angle_Pitch(&(gimbal_point()->gimbal_kalman.Auto_Error_Pitch));
	VisionGetCoordinateSystem(&x_value,&y_value,&z_value);
	VisionGetPredictCoordinate(&predict_x_value,&predict_y_value,&predict_z_value);
//	VisionGetIfTarget();
	
}

/**
  * @brief          ���㵱ǰ�����λ������
  * @param[in]      none
  * @param[in]      none
  * @retval         none
  */
void Vision_process_t::calc_x_y_coordinate()
{
//	x_coordinate = x_value*fx/z_value + cx;
//	y_coordinate = -y_value*fy/z_value + cy;
//	if(x_coordinate<400)
//		x_coordinate = 400;
//	else if(x_coordinate>1400)
//		x_coordinate = 1200;
}

/**
  * @brief          ��������Ԥ���λ������
  * @param[in]      none
  * @param[in]      none
  * @retval         none
  */
void Vision_process_t::calc_pre_x_y_coordinate()
{
//	pre_x_coordinate = predict_x_value*fx/predict_z_value + cx;
//	pre_y_coordinate = -predict_y_value*fy/predict_z_value + cy;
//	if(pre_x_coordinate<400)
//		pre_x_coordinate = 400;
//	else if(pre_x_coordinate>1400)
//		pre_x_coordinate = 1400;
}

///**
//  * @brief          ���㵱ǰ����ԲȦ���ʰ뾶
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void Vision_process_t::calc_follow_radius()
//{
//	follow_radius = 0.06f*f/vision_info.RxPacket.distance;
//}

///**
//  * @brief          ���Ӿ����ݽ��п������˲�
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void Vision_process_t::Vision_kalman_update()
//{
//	if(vision_info.RxPacket.identify_target == 1) //ʶ��װ�װ�  ������Ŀ���л���ʱ��
//	{
//		//�������˲� ������ƽ������
//		data_kal.YawGet_KF = gimbal_point()->gimbal_kalman.Yaw_Error_Vis_Kalman.KalmanFilter(gimbal_point()->gimbal_kalman.Auto_Error_Yaw); 	/*���Ӿ��Ƕ��������������˲�*/
//		data_kal.PitchGet_KF = gimbal_point()->gimbal_kalman.Pitch_Error_Vis_Kalman.KalmanFilter(gimbal_point()->gimbal_kalman.Auto_Error_Pitch);
//		data_kal.DistanceGet_KF =gimbal_point()->gimbal_kalman.Vision_Distance_Kalman.KalmanFilter(gimbal_point()->gimbal_kalman.Auto_Distance);

//		//��ʶ��װ�װ�ʱ�����ٶȶ��е����,�����´ν���ѭ��ʱ�����Ӿ���������
//		if(calc_v_state == false )//|| vision_info.RxPacket.target_Switch == 1)
//		{
//			Clear_Queue(&speed_queue);  //����ٶȶ���
//			Target_angle_update();      //�Ƕȸ���
//			speed_get = Get_Diff(3,&speed_queue,YawTarget_now);  //���Ƕ����ݼ������
//			//����ٶȵ�״̬����
//			gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.P_last = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.P_mid = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.P_now = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.X_last = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.X_mid = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.X_now = 0;
//		  calc_v_state = true ;   //����־λ��Ϊ1
//		}else{
//			Target_angle_update();    //�Ƕȸ���
//			Vision_info_infer();      //�Ӿ���������
//			//���׿������ںϼ���
//			gimbal_point()->gimbal_kalman.yaw_kf_result=kalman_filter_calc(&gimbal_point()->gimbal_kalman.yaw_kalman_filter,YawTarget_now,speed_get);
////			if((*gimbal_point()->gimbal_kalman.yaw_kf_result - YawTarget_now) > 0.05f)  //�˲���ɢʱ�ù۲���
////			{
////				*gimbal_point()->gimbal_kalman.yaw_kf_result = data_kal.YawGet_KF;
////			}
//		}
//	}else{
//		calc_v_state = false;
//		Target_angle_update();      //�Ƕȸ���
//	}
////	if(vision_info.RxPacket.target_Switch == 1)  //ʶ��Ŀ���ҽ���Ŀ���л�ʱ��Э����������³�ʼ��
////	{
////		kalman_init_state = false;
////		attack_switch_ramp = 30;
////		speed_get = 0;
//////			yaw_kalman_filter_para.P_data[0] = yaw_kalman_filter_para.P_data[3]= 0;//= pitch_kalman_filter_para.P_data[0] = pitch_kalman_filter_para.P_data[3] = 1;
//////			yaw_kalman_filter_para.P_data[1] = yaw_kalman_filter_para.P_data[2] = 0;//pitch_kalman_filter_para.P_data[1] = pitch_kalman_filter_para.P_data[2] = 1;
////		gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.P_last = gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.P_mid = gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.P_now = gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.X_last = gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.X_mid = gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.X_now =0;
////		gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.P_last= gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.P_mid=gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.P_now = gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.X_last = gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.X_mid = gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.X_now =0;
////		gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.P_last = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.P_mid = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.P_now = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.X_last = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.X_mid = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.X_now = 0;
////	}
//	
////	//�����ж��Ƿ����
////	if((data_kal.YawGet_KF > 0.035f && kalman_init_state == false) 
////	 ||(data_kal.YawGet_KF < 0.005f && kalman_init_state == false) 
////	 ||(vision_info.RxPacket.check_predict_state == false && gimbal_point()->target_change == 1))
//////	 ||(attack_switch_ramp != 0 && gimbal_point()->target_change == 1)) //�Ƕȴ���һ��ֵ��δʶ��Ŀ��ʱʱ����Ԥ��
////	{
////		vision_info.State.predict_state = false;
//////		if(gimbal_point()->target_change == 1)
//////		  attack_switch_ramp --;
//////		if(attack_switch_ramp == 0)
//////			gimbal_point()->target_change = 0;
////	}else{
//////		if(attack_switch_ramp == 0)
////	  vision_info.State.predict_state = true;
////		kalman_init_state = true;
////	}

//	
//	//�޿�����
////	data_kal.YawGet_KF = gimbal_point()->gimbal_kalman.Auto_Error_Yaw[NOW]; 	/*���Ӿ��Ƕ��������������˲�*/
////  data_kal.PitchGet_KF = gimbal_point()->gimbal_kalman.Auto_Error_Pitch[NOW];
////	data_kal.DistanceGet_KF =gimbal_point()->gimbal_kalman.Vision_Distance_Kalman.KalmanFilter(gimbal_point()->gimbal_kalman.Auto_Distance);
//	
////	//�Ƕȸ���
////	YawTarget_now = last_last_update_cloud_yaw + data_kal.YawGet_KF;
////  PitchTarget_now = lastupdate_cloud_pitch + data_kal.PitchGet_KF;

////	yaw_target_raw = YawTarget_now;
//////		YawTarget_now=update_cloud_yaw+Vision_process.data_kal.YawGet_KF;
//////    PitchTarget_now=update_cloud_pitch+Vision_process.data_kal.PitchGet_KF;

////	last_last_update_cloud_yaw = lastupdate_cloud_yaw;
////  lastupdate_cloud_yaw = update_cloud_yaw;
////  lastupdate_cloud_pitch = update_cloud_pitch;      /*��¼ǰ2֡������*/
////  update_cloud_yaw = gimbal_point()->yaw_motor.absolute_angle;/*�Ӿ����ݸ���ʱ����̨�Ƕ�*/
////  update_cloud_pitch =  gimbal_point()->pitch_motor.absolute_angle;
//}

///**
//  * @brief          �Ӿ���������
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void Vision_process_t::Vision_info_infer()
//{
//	/*************************�Ӿ���������*******************************************/
//		//�ٶȻ�ȡ
//		speed_get = Get_Diff(3,&speed_queue,YawTarget_now);//20
//		speed_get = 1 * (Vision_process.speed_get/vision_info.State.rx_time_fps); //ÿ����
//		speed_get = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.KalmanFilter(speed_get);
//		// speed_get = DeathZoom(speed_get,0,1);
//		speed_get = fp32_constrain(speed_get, -0.030, 0.030);
////  //���ٶȻ�ȡ
////  accel_get = Get_Diff(5,&accel_queue,speed_get);	 /*�°��ȡ���ٶ�10*/
////  accel_get = 15 * (Vision_process.accel_get/vision_info.State.rx_time_fps);//ÿ����
////  accel_get = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Accle_Kalman.KalmanFilter(Vision_process.accel_get);
////// accel_get = DeathZoom(accel_get,0,0.1);		/*�������� - �˳�0�㸽��������*/
////  accel_get = fp32_constrain(Vision_process.accel_get, -0.023, 0.023);
////  //�����ȡ
////  distend_get =  Get_Diff(5,&dis_queue,data_kal.DistanceGet_KF);
//	
//}

/**
  * @brief          Ŀ��Ƕȸ���
  * @param[in]      none
  * @param[in]      none
  * @retval         none
  */
void Vision_process_t::Target_angle_update()
{
	YawTarget_now = last_last_update_cloud_yaw + data_kal.YawGet_KF;
  PitchTarget_now = lastupdate_cloud_pitch + data_kal.PitchGet_KF;

//	yaw_target_raw = YawTarget_now;

	last_last_update_cloud_yaw = lastupdate_cloud_yaw;
  lastupdate_cloud_yaw = update_cloud_yaw;
  lastupdate_cloud_pitch = update_cloud_pitch;      /*��¼ǰ2֡������*/
//  update_cloud_yaw = gimbal_point()->yaw_motor.absolute_angle;/*�Ӿ����ݸ���ʱ����̨�Ƕ�*/
//  update_cloud_pitch =  gimbal_point()->pitch_motor.absolute_angle;
}

///**
//  * @brief          �Ӿ�Ԥ��
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void Vision_process_t::Vision_Pridict()
//{
//	static float acc_use = 1.0f;
//  static float predic_use = 0.6f;//  3 ,10
//  float dir_factor;
//  if( (speed_get * accel_get)>=0 )
//  {
//    dir_factor= 0.5f;//1  2
//  }
//  else
//  {
//    dir_factor= 1.0f;//1.5  4
//  }

//  feedforwaurd_angle = acc_use * accel_get;  	/*����ǰ����*/

//  predict_angle = predic_use * (1.1f * speed_get * data_kal.DistanceGet_KF
//												      + 1.5f * dir_factor * feedforwaurd_angle * data_kal.DistanceGet_KF);//+ chassis_move.motor_chassis.speed * 0.0001f ;
//  predict_angle = fp32_constrain(predict_angle, -0.092, 0.092);
//	
////--------------------------------------------------------------------------------------//	
////������ֹװ�װ�ɿ��Ǵ��ײ���
////	predict_angle = chassis_move.motor_chassis.speed * 1.0f ;
//}

///**
//  * @brief          ������̨���ƽǶ�ֵ����
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void Vision_process_t::Anti_Target()
//{
//	if(gyro_anti)
//    AntiGyro();  
//  else
//    AntiNormal();
//}

///**
//  * @brief          ����ʱ��������̨�Ƕ�ֵ
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void Vision_process_t::AntiNormal()
//{
//	  /*ֱ�Ӹ������.................................*/
////	    Vision_process.data_kal.YawTarget_KF=KalmanFilter(&gimbal_control.gimbal_kalman.Yaw_Set_Gim_Kalman,YawTarget_now);
////	    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&kalman_targetPitch,PitchTarget_now);
//    /*..................................................*/
////	if(vision_info.RxPacket.identify_target == 1)  //δʶ��װ�װ�ʱ�����ж��׿�����
////	{
////		gimbal_point()->gimbal_kalman.yaw_kf_result=kalman_filter_calc(&gimbal_point()->gimbal_kalman.yaw_kalman_filter,YawTarget_now,speed_get);
////		yaw_2kal = *gimbal_point()->gimbal_kalman.yaw_kf_result;
////  } 
//	yaw_2kal = *gimbal_point()->gimbal_kalman.yaw_kf_result;
//	yaw_1kal = gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.KalmanFilter(YawTarget_now);
//	/**************�Լ��ı�־λ**********/
//	//����Ԥ��ʱ�ö��׿������ں�
////	if(vision_info.State.predict_state == true){
////		data_kal.YawTarget_KF=yaw_2kal;
////	}else{
//////		attack_switch_ramp--;
////	  data_kal.YawTarget_KF=yaw_1kal;
////	}
//	  //ͨ���Ӿ�������־λ�������ж��Ƿ�Ҫ����Ԥ��
//	  if(vision_info.RxPacket.check_predict_state == true)
//			data_kal.YawTarget_KF=yaw_2kal; //�ö��׿�����
//		else
//			data_kal.YawTarget_KF=yaw_1kal; //��һ�׿�����
//		
//	 //��ֹ����nan����inf
//	  if(isnan(data_kal.YawTarget_KF) || isinf(data_kal.YawTarget_KF))
//		{
//			data_kal.YawTarget_KF  =  gimbal_point()->gimbal_kalman.Auto_Error_Yaw;
//		}
//		
//		data_kal.PitchTarget_KF= gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.KalmanFilter(PitchTarget_now);//����Ԥ��
//		//��ֹ����nan����inf
//		if(isnan(data_kal.PitchTarget_KF) || isinf(data_kal.PitchTarget_KF))
//			data_kal.PitchTarget_KF=gimbal_point()->gimbal_kalman.Auto_Error_Pitch;

////  data_kal.YawTarget_KF=YawTarget_now+Vision_process.predict_angle;
////  data_kal.YawTarget_KF=gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.KalmanFilter(data_kal.YawTarget_KF);
////  data_kal.PitchTarget_KF= gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.KalmanFilter(PitchTarget_now);//����Ԥ��

///*ֱ�Ӹ������.................................*/
////	Vision_process.data_kal.YawTarget_KF=KalmanFilter(&gimbal_control.gimbal_kalman.Yaw_Set_Gim_Kalman,YawTarget_now);
////    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&gimbal_control.gimbal_kalman.Pitch_Set_Gim_Kalman,PitchTarget_now);
//    /*..................................................*/
////    Vision_process.data_kal.YawTarget_KF=KalmanFilter(&gimbal_control.gimbal_kalman.Yaw_Set_Gim_Kalman,Vision_process.data_kal.YawTarget_KF);
////    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&gimbal_control.gimbal_kalman.Pitch_Set_Gim_Kalman,PitchTarget_now);//����Ԥ��
//}

///**
//  * @brief          ����δ�����з�ʱ��ס��̨
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void Vision_process_t::AntiGyro()
//{
//	/**********************************ֱ�Ӹ������***************************************/
//  data_kal.YawTarget_KF=gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.KalmanFilter(YawTarget_now);
//  data_kal.PitchTarget_KF=gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.KalmanFilter(PitchTarget_now);
//  /****************************************************************************************/
////  data_kal.YawTarget_KF=gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.KalmanFilter(data_kal.YawTarget_KF);
////  data_kal.PitchTarget_KF=gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.KalmanFilter(PitchTarget_now);//����Ԥ��
//}

//#include "communicate.h"
//#include "cmsis_os.h"
//#include "system_task.h"

////�Ӿ����ݴ���ͳһ������
//Vision_process_t Vision_process;

//float YawTarget_now,PitchTarget_now;//ʵ���Ӿ������Ƕ�
//float update_cloud_yaw = 0,update_cloud_pitch=0;	/*��¼�Ӿ���������ʱ����̨���ݣ����´ν�����*/
//float lastupdate_cloud_yaw = 0,lastupdate_cloud_pitch=0; /*ǰ��֡������*/
//float last_last_update_cloud_yaw =0;// ,last_last_update_cloud_pitch = 0;


///**
//  * @brief          �Ӿ���������
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void communi_task(void const *pvParameters)
//{
//	vTaskDelay(200);
//	uint32_t currentTime;
//	//�൱��vision_init
//	vision_info.RxPacket.is_spinning = 0;
//	while(1)
//	{
//		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
//		/*********�Ӿ���������*******/
////		vision_info.Collect_shoot_info(1);
//		vision_info.Vision_Send_Data(Vision_mode);
//		
//		//UI���ݼ���
//		Vision_process.Vision_get_info();
//		Vision_process.calc_x_y_coordinate();
//		Vision_process.calc_pre_x_y_coordinate();
//		Vision_process.calc_follow_radius();
//		
//		/*******2023����Ӣ��δʹ��******/
////		Vision_process.Vision_Normal();
//		
////		//����С���ݾ����տ�ʱʹ��
////		Vision_process.AntiNormal();
//		
//		CAN1_302_send_angle_info(yaw_relative_angle,pitch_angle);
//		CAN1_send_state_info(If_reset,If_firc,If_shift,If_auto,If_spin,rotate_speed);
//		CAN1_303_send_x_y(x_coordinate,y_coordinate);
//		CAN1_304_send_pre_x_y(pre_x_coordinate,pre_y_coordinate);
//		CAN1_305_send_r_info(follow_radius,Vision_mode);
//	  vTaskDelay(2);		//2ms
//	}
//	
//}

///**
//  * @brief          ���캯����ʼ��
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//Vision_process_t::Vision_process_t()
//{
//	speed_queue.queueLength = 60;
//	accel_queue.queueLength = 60;
//	dis_queue.queueLength = 60;
//	attack_switch_ramp = 30;
//	//ͼ���������
//	fx = 883.3170;
//	fy = 882.6257;
//	f = sqrt(fx*fx + fy*fy);
//	//ͼ����������
//	cx = 959.8267;
//	cy = 480;
//}

///**
//  * @brief          ͨ���Ӿ����ݶ�ȡ�ʹ���
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void Vision_process_t::Vision_Normal()
//{
//	/******�Ӿ����ݻ�ȡ*******/
//	Vision_get_info();
//	Target_angle_update();
//	/******���Ӿ��Ƕ��������������˲�*******/
////	Vision_kalman_update();
////	/******�Ӿ���������*******/
////  Vision_info_infer();  	
////	vision_info.State.rx_data_update = false;
//	
//}

///**
//  * @brief          ��ȡ�Ӿ�����
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void Vision_process_t::Vision_get_info()
//{
////	vision_info.Vision_Error_Angle_Yaw(&(gimbal_point()->gimbal_kalman.Auto_Error_Yaw));
////  vision_info.Vision_Error_Angle_Pitch(&(gimbal_point()->gimbal_kalman.Auto_Error_Pitch));
//	vision_info.Vision_Get_Distance(&(gimbal_point()->gimbal_kalman.Auto_Distance));
//	vision_info.Vision_Get_coordinate_system(&x_value,&y_value,&z_value);
//	vision_info.Vision_Get_predict_coordinate(&predict_x_value,&predict_y_value,&predict_z_value);
//	vision_info.Vision_Get_if_target(&target_flag);
//	
//	
//}

///**
//  * @brief          ���㵱ǰ�����λ������
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void Vision_process_t::calc_x_y_coordinate()
//{
//	x_coordinate = x_value*fx/z_value + cx;
//	y_coordinate = -y_value*fy/z_value + cy;
//	if(x_coordinate<400)
//		x_coordinate = 400;
//	else if(x_coordinate>1400)
//		x_coordinate = 1200;
//}

///**
//  * @brief          ��������Ԥ���λ������
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void Vision_process_t::calc_pre_x_y_coordinate()
//{
//	pre_x_coordinate = predict_x_value*fx/predict_z_value + cx;
//	pre_y_coordinate = -predict_y_value*fy/predict_z_value + cy;
//	if(pre_x_coordinate<400)
//		pre_x_coordinate = 400;
//	else if(pre_x_coordinate>1400)
//		pre_x_coordinate = 1400;
//}

///**
//  * @brief          ���㵱ǰ����ԲȦ���ʰ뾶
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void Vision_process_t::calc_follow_radius()
//{
//	follow_radius = 0.06f*f/vision_info.RxPacket.distance;
//}

///**
//  * @brief          ���Ӿ����ݽ��п������˲�
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void Vision_process_t::Vision_kalman_update()
//{
//	if(vision_info.RxPacket.identify_target == 1) //ʶ��װ�װ�  ������Ŀ���л���ʱ��
//	{
//		//�������˲� ������ƽ������
//		data_kal.YawGet_KF = gimbal_point()->gimbal_kalman.Yaw_Error_Vis_Kalman.KalmanFilter(gimbal_point()->gimbal_kalman.Auto_Error_Yaw); 	/*���Ӿ��Ƕ��������������˲�*/
//		data_kal.PitchGet_KF = gimbal_point()->gimbal_kalman.Pitch_Error_Vis_Kalman.KalmanFilter(gimbal_point()->gimbal_kalman.Auto_Error_Pitch);
//		data_kal.DistanceGet_KF =gimbal_point()->gimbal_kalman.Vision_Distance_Kalman.KalmanFilter(gimbal_point()->gimbal_kalman.Auto_Distance);

//		//��ʶ��װ�װ�ʱ�����ٶȶ��е����,�����´ν���ѭ��ʱ�����Ӿ���������
//		if(calc_v_state == false )//|| vision_info.RxPacket.target_Switch == 1)
//		{
//			Clear_Queue(&speed_queue);  //����ٶȶ���
//			Target_angle_update();      //�Ƕȸ���
//			speed_get = Get_Diff(3,&speed_queue,YawTarget_now);  //���Ƕ����ݼ������
//			//����ٶȵ�״̬����
//			gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.P_last = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.P_mid = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.P_now = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.X_last = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.X_mid = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.X_now = 0;
//		  calc_v_state = true ;   //����־λ��Ϊ1
//		}else{
//			Target_angle_update();    //�Ƕȸ���
//			Vision_info_infer();      //�Ӿ���������
//			//���׿������ںϼ���
//			gimbal_point()->gimbal_kalman.yaw_kf_result=kalman_filter_calc(&gimbal_point()->gimbal_kalman.yaw_kalman_filter,YawTarget_now,speed_get);
////			if((*gimbal_point()->gimbal_kalman.yaw_kf_result - YawTarget_now) > 0.05f)  //�˲���ɢʱ�ù۲���
////			{
////				*gimbal_point()->gimbal_kalman.yaw_kf_result = data_kal.YawGet_KF;
////			}
//		}
//	}else{
//		calc_v_state = false;
//		Target_angle_update();      //�Ƕȸ���
//	}
////	if(vision_info.RxPacket.target_Switch == 1)  //ʶ��Ŀ���ҽ���Ŀ���л�ʱ��Э����������³�ʼ��
////	{
////		kalman_init_state = false;
////		attack_switch_ramp = 30;
////		speed_get = 0;
//////			yaw_kalman_filter_para.P_data[0] = yaw_kalman_filter_para.P_data[3]= 0;//= pitch_kalman_filter_para.P_data[0] = pitch_kalman_filter_para.P_data[3] = 1;
//////			yaw_kalman_filter_para.P_data[1] = yaw_kalman_filter_para.P_data[2] = 0;//pitch_kalman_filter_para.P_data[1] = pitch_kalman_filter_para.P_data[2] = 1;
////		gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.P_last = gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.P_mid = gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.P_now = gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.X_last = gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.X_mid = gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.X_now =0;
////		gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.P_last= gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.P_mid=gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.P_now = gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.X_last = gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.X_mid = gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.X_now =0;
////		gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.P_last = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.P_mid = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.P_now = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.X_last = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.X_mid = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.X_now = 0;
////	}
//	
////	//�����ж��Ƿ����
////	if((data_kal.YawGet_KF > 0.035f && kalman_init_state == false) 
////	 ||(data_kal.YawGet_KF < 0.005f && kalman_init_state == false) 
////	 ||(vision_info.RxPacket.check_predict_state == false && gimbal_point()->target_change == 1))
//////	 ||(attack_switch_ramp != 0 && gimbal_point()->target_change == 1)) //�Ƕȴ���һ��ֵ��δʶ��Ŀ��ʱʱ����Ԥ��
////	{
////		vision_info.State.predict_state = false;
//////		if(gimbal_point()->target_change == 1)
//////		  attack_switch_ramp --;
//////		if(attack_switch_ramp == 0)
//////			gimbal_point()->target_change = 0;
////	}else{
//////		if(attack_switch_ramp == 0)
////	  vision_info.State.predict_state = true;
////		kalman_init_state = true;
////	}

//	
//	//�޿�����
////	data_kal.YawGet_KF = gimbal_point()->gimbal_kalman.Auto_Error_Yaw[NOW]; 	/*���Ӿ��Ƕ��������������˲�*/
////  data_kal.PitchGet_KF = gimbal_point()->gimbal_kalman.Auto_Error_Pitch[NOW];
////	data_kal.DistanceGet_KF =gimbal_point()->gimbal_kalman.Vision_Distance_Kalman.KalmanFilter(gimbal_point()->gimbal_kalman.Auto_Distance);
//	
////	//�Ƕȸ���
////	YawTarget_now = last_last_update_cloud_yaw + data_kal.YawGet_KF;
////  PitchTarget_now = lastupdate_cloud_pitch + data_kal.PitchGet_KF;

////	yaw_target_raw = YawTarget_now;
//////		YawTarget_now=update_cloud_yaw+Vision_process.data_kal.YawGet_KF;
//////    PitchTarget_now=update_cloud_pitch+Vision_process.data_kal.PitchGet_KF;

////	last_last_update_cloud_yaw = lastupdate_cloud_yaw;
////  lastupdate_cloud_yaw = update_cloud_yaw;
////  lastupdate_cloud_pitch = update_cloud_pitch;      /*��¼ǰ2֡������*/
////  update_cloud_yaw = gimbal_point()->yaw_motor.absolute_angle;/*�Ӿ����ݸ���ʱ����̨�Ƕ�*/
////  update_cloud_pitch =  gimbal_point()->pitch_motor.absolute_angle;
//}

///**
//  * @brief          �Ӿ���������
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void Vision_process_t::Vision_info_infer()
//{
//	/*************************�Ӿ���������*******************************************/
//		//�ٶȻ�ȡ
//		speed_get = Get_Diff(3,&speed_queue,YawTarget_now);//20
//		speed_get = 1 * (Vision_process.speed_get/vision_info.State.rx_time_fps); //ÿ����
//		speed_get = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.KalmanFilter(speed_get);
//		// speed_get = DeathZoom(speed_get,0,1);
//		speed_get = fp32_constrain(speed_get, -0.030, 0.030);
////  //���ٶȻ�ȡ
////  accel_get = Get_Diff(5,&accel_queue,speed_get);	 /*�°��ȡ���ٶ�10*/
////  accel_get = 15 * (Vision_process.accel_get/vision_info.State.rx_time_fps);//ÿ����
////  accel_get = gimbal_point()->gimbal_kalman.Gimbal_Yaw_Accle_Kalman.KalmanFilter(Vision_process.accel_get);
////// accel_get = DeathZoom(accel_get,0,0.1);		/*�������� - �˳�0�㸽��������*/
////  accel_get = fp32_constrain(Vision_process.accel_get, -0.023, 0.023);
////  //�����ȡ
////  distend_get =  Get_Diff(5,&dis_queue,data_kal.DistanceGet_KF);
//	
//}

///**
//  * @brief          Ŀ��Ƕȸ���
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void Vision_process_t::Target_angle_update()
//{
//	YawTarget_now = last_last_update_cloud_yaw + data_kal.YawGet_KF;
//  PitchTarget_now = lastupdate_cloud_pitch + data_kal.PitchGet_KF;

////	yaw_target_raw = YawTarget_now;

//	last_last_update_cloud_yaw = lastupdate_cloud_yaw;
//  lastupdate_cloud_yaw = update_cloud_yaw;
//  lastupdate_cloud_pitch = update_cloud_pitch;      /*��¼ǰ2֡������*/
//  update_cloud_yaw = gimbal_point()->yaw_motor.absolute_angle;/*�Ӿ����ݸ���ʱ����̨�Ƕ�*/
//  update_cloud_pitch =  gimbal_point()->pitch_motor.absolute_angle;
//}

///**
//  * @brief          �Ӿ�Ԥ��
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void Vision_process_t::Vision_Pridict()
//{
//	static float acc_use = 1.0f;
//  static float predic_use = 0.6f;//  3 ,10
//  float dir_factor;
//  if( (speed_get * accel_get)>=0 )
//  {
//    dir_factor= 0.5f;//1  2
//  }
//  else
//  {
//    dir_factor= 1.0f;//1.5  4
//  }

//  feedforwaurd_angle = acc_use * accel_get;  	/*����ǰ����*/

//  predict_angle = predic_use * (1.1f * speed_get * data_kal.DistanceGet_KF
//												      + 1.5f * dir_factor * feedforwaurd_angle * data_kal.DistanceGet_KF);//+ chassis_move.motor_chassis.speed * 0.0001f ;
//  predict_angle = fp32_constrain(predict_angle, -0.092, 0.092);
//	
////--------------------------------------------------------------------------------------//	
////������ֹװ�װ�ɿ��Ǵ��ײ���
////	predict_angle = chassis_move.motor_chassis.speed * 1.0f ;
//}

///**
//  * @brief          ������̨���ƽǶ�ֵ����
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void Vision_process_t::Anti_Target()
//{
//	if(gyro_anti)
//    AntiGyro();  
//  else
//    AntiNormal();
//}

///**
//  * @brief          ����ʱ��������̨�Ƕ�ֵ
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void Vision_process_t::AntiNormal()
//{
//	  /*ֱ�Ӹ������.................................*/
////	    Vision_process.data_kal.YawTarget_KF=KalmanFilter(&gimbal_control.gimbal_kalman.Yaw_Set_Gim_Kalman,YawTarget_now);
////	    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&kalman_targetPitch,PitchTarget_now);
//    /*..................................................*/
////	if(vision_info.RxPacket.identify_target == 1)  //δʶ��װ�װ�ʱ�����ж��׿�����
////	{
////		gimbal_point()->gimbal_kalman.yaw_kf_result=kalman_filter_calc(&gimbal_point()->gimbal_kalman.yaw_kalman_filter,YawTarget_now,speed_get);
////		yaw_2kal = *gimbal_point()->gimbal_kalman.yaw_kf_result;
////  } 
//	yaw_2kal = *gimbal_point()->gimbal_kalman.yaw_kf_result;
//	yaw_1kal = gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.KalmanFilter(YawTarget_now);
//	/**************�Լ��ı�־λ**********/
//	//����Ԥ��ʱ�ö��׿������ں�
////	if(vision_info.State.predict_state == true){
////		data_kal.YawTarget_KF=yaw_2kal;
////	}else{
//////		attack_switch_ramp--;
////	  data_kal.YawTarget_KF=yaw_1kal;
////	}
//	  //ͨ���Ӿ�������־λ�������ж��Ƿ�Ҫ����Ԥ��
//	  if(vision_info.RxPacket.check_predict_state == true)
//			data_kal.YawTarget_KF=yaw_2kal; //�ö��׿�����
//		else
//			data_kal.YawTarget_KF=yaw_1kal; //��һ�׿�����
//		
//	 //��ֹ����nan����inf
//	  if(isnan(data_kal.YawTarget_KF) || isinf(data_kal.YawTarget_KF))
//		{
//			data_kal.YawTarget_KF  =  gimbal_point()->gimbal_kalman.Auto_Error_Yaw;
//		}
//		
//		data_kal.PitchTarget_KF= gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.KalmanFilter(PitchTarget_now);//����Ԥ��
//		//��ֹ����nan����inf
//		if(isnan(data_kal.PitchTarget_KF) || isinf(data_kal.PitchTarget_KF))
//			data_kal.PitchTarget_KF=gimbal_point()->gimbal_kalman.Auto_Error_Pitch;

////  data_kal.YawTarget_KF=YawTarget_now+Vision_process.predict_angle;
////  data_kal.YawTarget_KF=gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.KalmanFilter(data_kal.YawTarget_KF);
////  data_kal.PitchTarget_KF= gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.KalmanFilter(PitchTarget_now);//����Ԥ��

///*ֱ�Ӹ������.................................*/
////	Vision_process.data_kal.YawTarget_KF=KalmanFilter(&gimbal_control.gimbal_kalman.Yaw_Set_Gim_Kalman,YawTarget_now);
////    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&gimbal_control.gimbal_kalman.Pitch_Set_Gim_Kalman,PitchTarget_now);
//    /*..................................................*/
////    Vision_process.data_kal.YawTarget_KF=KalmanFilter(&gimbal_control.gimbal_kalman.Yaw_Set_Gim_Kalman,Vision_process.data_kal.YawTarget_KF);
////    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&gimbal_control.gimbal_kalman.Pitch_Set_Gim_Kalman,PitchTarget_now);//����Ԥ��
//}

///**
//  * @brief          ����δ�����з�ʱ��ס��̨
//  * @param[in]      none
//  * @param[in]      none
//  * @retval         none
//  */
//void Vision_process_t::AntiGyro()
//{
//	/**********************************ֱ�Ӹ������***************************************/
//  data_kal.YawTarget_KF=gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.KalmanFilter(YawTarget_now);
//  data_kal.PitchTarget_KF=gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.KalmanFilter(PitchTarget_now);
//  /****************************************************************************************/
////  data_kal.YawTarget_KF=gimbal_point()->gimbal_kalman.Yaw_Set_Gim_Kalman.KalmanFilter(data_kal.YawTarget_KF);
////  data_kal.PitchTarget_KF=gimbal_point()->gimbal_kalman.Pitch_Set_Gim_Kalman.KalmanFilter(PitchTarget_now);//����Ԥ��
//}
