#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

#include "struct_typedef.h"
#include "device_monitor.h"
//串口长度设置
#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 50u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

#define RC_THUMB_USE_VALUE_MAX 			((uint16_t)600)
#define RC_THUMB_USE_VALUE_MIN  ((uint16_t)300)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

/* 检测遥控器开关状态 */
#define    IF_RC_SWR_UP      (rc_ctrl.rc.s[0] == RC_SW_UP)
#define    IF_RC_SWR_MID     (rc_ctrl.rc.s[0] == RC_SW_MID)
#define    IF_RC_SWR_DOWN    (rc_ctrl.rc.s[0] == RC_SW_DOWN)

#define    IF_RC_SWL_UP      (rc_ctrl.rc.s[1] == RC_SW_UP)
#define    IF_RC_SWL_MID     (rc_ctrl.rc.s[1] == RC_SW_MID)
#define    IF_RC_SWL_DOWN    (rc_ctrl.rc.s[1] == RC_SW_DOWN)
 
/*拨杆位置*/
#define    IF_RIGHT_ROCKER_RIGHT_TOP      (rc_ctrl.rc.ch[0] > 600 && rc_ctrl.rc.ch[1] > 600)
#define    IF_RIGHT_ROCKER_LEFT_TOP       (rc_ctrl.rc.ch[0] < -600 && rc_ctrl.rc.ch[1] > 600)
#define    IF_RIGHT_ROCKER_LEFT_BOTTOM    (rc_ctrl.rc.ch[0] < -600 && rc_ctrl.rc.ch[1] < -600)
#define    IF_RIGHT_ROCKER_RIGHT_BOTTOM   (rc_ctrl.rc.ch[0] > 600 && rc_ctrl.rc.ch[1] < -600)

#define    IF_LEFT_ROCKER_RIGHT_TOP       (rc_ctrl.rc.ch[2] > 600 && rc_ctrl.rc.ch[3] > 600)
#define    IF_LEFT_ROCKER_LEFT_TOP        (rc_ctrl.rc.ch[2] < -600 && rc_ctrl.rc.ch[3] > 600)
#define    IF_LEFT_ROCKER_LEFT_BOTTOM     (rc_ctrl.rc.ch[2] < -600 && rc_ctrl.rc.ch[3] < -600)
#define    IF_LEFT_ROCKER_RIGHT_BOTTOM    (rc_ctrl.rc.ch[2] > 600 && rc_ctrl.rc.ch[3] < -600)   

/*拨轮位置回中判断*/
#define 	 IF_THUMB_WHEEL_ZERO_STATE (abs(rc_ctrl.rc.ch[4]) < 50)

/* 获取鼠标三轴的移动速度 */
#define    MOUSE_X_MOVE_SPEED    (rc_ctrl.mouse.x)
#define    MOUSE_Y_MOVE_SPEED    (rc_ctrl.mouse.y)
#define    MOUSE_Z_MOVE_SPEED    (rc_ctrl.mouse.z)


/* 检测鼠标按键状态 
   按下为1，没按下为0*/
#define    IF_MOUSE_PRESSED_LEFT    (rc_ctrl.mouse.press_l == 1)
#define    IF_MOUSE_PRESSED_RIGH    (rc_ctrl.mouse.press_r == 1)

/* 检测键盘按键状态 
   若对应按键被按下，则逻辑表达式的值为1，否则为0 */
#define    IF_KEY_PRESSED         (  rc_ctrl.key.v  )
#define    IF_KEY_PRESSED_W       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL    ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT   ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT) != 0 )

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
#ifdef __cplusplus
extern "C"{
#endif

//#define NEGATIVE_DIRECTION -1
//#d
#ifdef __cplusplus
enum Thumb_direction_t
{
	ZERO, NEGATIVE_DIRECTION = -1, POSITIVE_DIRECTION = 1,
};	
	
//遥控器协议
typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;

} RC_ctrl_t;

enum Channel_t
{
	KEY_W,                     // 0
  KEY_S,                     // 1
  KEY_A,                     // 2
  KEY_D,                     // 3
  KEY_SHIFT,                 // 4
  KEY_CTRL,                  // 5
  KEY_Q,                     // 6
  KEY_E,                     // 7
  KEY_R,                     // 8
  KEY_F,                     // 9
  KEY_G,                     // 10
  KEY_Z,                     // 11
  KEY_X,                     // 12
  KEY_C,                     // 13
  KEY_V,                     // 14
  KEY_B,                     // 15
  MOUSE_L,                   // 16
  MOUSE_R,                   // 17
  RIGHT_ROCKER_RIGHT_TOP,    // 18
  RIGHT_ROCKER_LEFT_TOP,     // 19
  RIGHT_ROCKER_LEFT_BOTTOM,  // 20
  RIGHT_ROCKER_RIGHT_BOTTOM, // 21
  LEFT_ROCKER_RIGHT_TOP,     // 22
  LEFT_ROCKER_LEFT_TOP,      // 23
  LEFT_ROCKER_LEFT_BOTTOM,   // 24
  LEFT_ROCKER_RIGHT_BOTTOM,  // 25
	
	TOTAL_CHANNEL,   //总数
};

extern RC_ctrl_t rc_ctrl;
void ThumbWheelSet(Thumb_direction_t direction,uint8_t *control_flag);
void HalfThumbWheelSet(Thumb_direction_t direction,uint8_t *control_flag);
const RC_ctrl_t *GetRemoteControlPoint(void);	
bool_t RCHeldContinuousReturn(Channel_t channel, uint16_t time);
bool_t RCHeldSingleReturn(Channel_t channel, uint16_t time);
bool_t RCDoubleHeldSingleReturn(Channel_t channel_1, Channel_t channel_2, uint16_t time);

bool MonitorRc();
#endif
void RemoteControlInit(void);	
#ifdef __cplusplus
}
#endif

#endif
