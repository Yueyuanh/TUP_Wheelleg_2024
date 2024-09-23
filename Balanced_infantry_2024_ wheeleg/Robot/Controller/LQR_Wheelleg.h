#ifndef LQR_WHEELLEG_H
#define LQR_WHEELLEG_H

#include "struct_typedef.h"
#include "pid.h"
#include "VMC.h"


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

class LQR_Wheelleg
{
		public:

		 /* data */

//    fp32 balance_LQR;
//    fp32 balance_LQR_chassis;
//    fp32 balance_LQR_sit;
//    fp32 balance_LQR_jump;

    PID_t roll_PID;
    PID_t stand_PID_R,stand_PID_L;
    float stand_feed;//Ç°À¡ÍÆÁ¦

    PID_t yaw_PD;
    float K_yaw_out;

    PID_t coordinate_PD;
    PID_t leg_length_roll_PID;

    float LQR_FEED_R[2][6];
    float LQR_FEED_L[2][6];
    
    float LQR_OUT_R[2][6];
    float LQR_OUT_L[2][6];


		/*function*/
		void init();
		void calc(const fp32 lqr[2][6],fp32 bias);


};




#endif	

#ifdef __cplusplus
}
#endif
#endif
