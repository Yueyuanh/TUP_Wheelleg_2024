/**
 ******************************************************************************
 * @file    can_send.cpp
 * @author  Xushuang��YangMaoLin
 * @version V1.0.0 Xushuang �������
 *          V1.1.0 YangMaoLin ��ӷ�װ����
 * @date    2023/8/25
 * @brief		ң����ģ��
 *          ͨ��DT7����ͼ����·����Э��������Ӷ����п���
*						�������ļ���ֹ�޸�
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "remote_control.h"
#include "bsp_usart.h"
#include "bsp_dwt.h"
//remote control data 
//ң�������Ʊ���
RC_ctrl_t rc_ctrl;

//ң����������
DeviceStatus_t rc_watch; 
uint32_t rc_last_time;

DeviceHeartBag_t remote_heart;
static void SbusToRc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
/*****************����ʵ������**********************/
Usart_Instance_t remote_usart;

void DecodeRemoteData();
//ң������ʼ������
void RemoteControlInit(void)
{
	USARTInit(&remote_usart,&huart3,USART_RX_DMA,RC_FRAME_LENGTH,DecodeRemoteData); //��ʼ�����ô��ڽ���
	UsartRegister(&remote_usart);
	rc_watch.InitDeviceStatus(1000);
}
//���ݴ�����
void DecodeRemoteData()
{
	SbusToRc(remote_usart.rx_buff,&rc_ctrl);
	rc_watch.GetMsgT();
}

//ң������⺯��
bool MonitorRc()
{
	return rc_watch.UpdateDeviceStatus();
}

static void SbusToRc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
	 if (sbus_buf == NULL )
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
	
	//ͨ������
	for(int i = 0; i < 5; i++)
	{
		if(rc_ctrl->rc.ch[i] > 660 || rc_ctrl->rc.ch[i] < -660)
		{
			rc_ctrl->rc.ch[i] = 0;
		}
	}
}

/**
 * @brief       �жϰ����Ƿ񱻰��»�ҡ���Ƿ���ָ��λ��
 * @param[in]   channel��Ҫ�жϵ�ͨ��
 * @retval      ָ��ͨ���������1�����򷵻�0
 * @attention   ������ֻ��RCHeldContinuousReturn��������
 */
static bool_t IsChannelActivate(Channel_t channel)
{
	switch (channel)
	{
	case MOUSE_L:
		return IF_MOUSE_PRESSED_LEFT;
	case MOUSE_R:
		return IF_MOUSE_PRESSED_RIGH;
	case RIGHT_ROCKER_RIGHT_TOP:
		return IF_RIGHT_ROCKER_RIGHT_TOP;
	case RIGHT_ROCKER_LEFT_TOP:
		return IF_RIGHT_ROCKER_LEFT_TOP;
	case RIGHT_ROCKER_LEFT_BOTTOM:
		return IF_RIGHT_ROCKER_LEFT_BOTTOM;
	case RIGHT_ROCKER_RIGHT_BOTTOM:
		return IF_RIGHT_ROCKER_RIGHT_BOTTOM;
	case LEFT_ROCKER_RIGHT_TOP:
		return IF_LEFT_ROCKER_RIGHT_TOP;
	case LEFT_ROCKER_LEFT_TOP:
		return IF_LEFT_ROCKER_LEFT_TOP;
	case LEFT_ROCKER_LEFT_BOTTOM:
		return IF_LEFT_ROCKER_LEFT_BOTTOM;
	case LEFT_ROCKER_RIGHT_BOTTOM:
		return IF_LEFT_ROCKER_RIGHT_BOTTOM;
	default:
		return ((rc_ctrl.key.v & ((uint16_t)1 << channel)) != 0);
	}
}

/**
 * @brief       �жϰ������ְ��»�ҡ�˱�����ָ��λ�õ�ʱ���Ƿ񳬹���ֵ
 * @param[in]   channel��Ҫ����ͨ��
 * @param[in]   time�����ֵ�ʱ����ֵ
 * @retval      ���ͨ������������ָ����ʱ�䣬����1�����򷵻�0
 */
bool_t RCHeldContinuousReturn(Channel_t channel, uint16_t time)
{
	static uint16_t active_time[TOTAL_CHANNEL];

	if (IsChannelActivate(channel)){
		if (active_time[channel] < time){
			active_time[channel]++;
			return 0;
		}else{
			return 1;
		}
	}else{
		active_time[channel] = 0;
		return 0;
	}
}

/**
 * @brief       �жϰ������ְ��»�ҡ�˱�����ָ��λ�õ�ʱ���Ƿ񳬹���ֵ��������������ʱֻ����һ��1
 * @param[in]   channel��Ҫ����ͨ��
 * @param[in]   time�����ֵ�ʱ����ֵ
 * @retval      ���ͨ������������ָ����ʱ�䣬ֻ����һ��1�����򷵻�0
 */
bool_t RCHeldSingleReturn(Channel_t channel, uint16_t time)
{
    static bool_t has_returned_one[25];

    if (RCHeldContinuousReturn(channel, time)){
        if (has_returned_one[channel]){
            return 0;
        }else{
            has_returned_one[channel] = 1;
            return 1;
        }
    }else{
        has_returned_one[channel] = 0;
        return 0;
    }
}

/**
 * @brief       �ж�˫��ϼ����ְ��»�����ҡ��ͬʱ������ָ��λ�õ�ʱ���Ƿ񳬹���ֵ��������������ʱֻ����һ��1
 * @param[in]   channel_1��Ҫ���ĵ�һ��ͨ��
 * @param[in]   channel_2��Ҫ���ĵڶ���ͨ��
 * @param[in]   time�����ֵ�ʱ����ֵ
 * @retval      ���ͨ������������ָ����ʱ�䣬ֻ����һ��1�����򷵻�0
 */
bool_t RCDoubleHeldSingleReturn(Channel_t channel_1, Channel_t channel_2, uint16_t time)
{
	static bool_t has_returned_one[TOTAL_CHANNEL];

	if (RCHeldContinuousReturn(channel_1, time) && RCHeldContinuousReturn(channel_2, time)){
		if (has_returned_one[channel_1] || has_returned_one[channel_2]){
			return 0;
		}else{
			has_returned_one[channel_1] = 1;
			has_returned_one[channel_2] = 1;
			return 1;
	  }
	}else{
		has_returned_one[channel_1] = 0;
		has_returned_one[channel_2] = 0;
		return 0;
	}
}

/**
 * @brief       ȫ�������ã����͸���
 * @param[in]   direction������
 * @param[in]   control_flag���ı��־λ
 * @retval      NULL
 */
void ThumbWheelSet(Thumb_direction_t direction,uint8_t *control_flag)
{
	if(direction*rc_ctrl.rc.ch[4] > RC_THUMB_USE_VALUE_MAX)
		*control_flag = 1;
	else
		*control_flag = 0;
}

/**
 * @brief       �벦�����ã����͸���
 * @param[in]   direction������
 * @param[in]   control_flag���ı��־λ
 * @retval      NULL
 */
void HalfThumbWheelSet(Thumb_direction_t direction,uint8_t *control_flag)
{
	if(direction*rc_ctrl.rc.ch[4] > RC_THUMB_USE_VALUE_MIN &&
		 direction*rc_ctrl.rc.ch[4] < RC_THUMB_USE_VALUE_MAX)
		*control_flag = 1;
	else
		*control_flag = 0;
}

/**
  * @brief          ��ȡң��������ָ��
  * @param[in]      none
  * @retval         ң��������ָ��
  */
const RC_ctrl_t *GetRemoteControlPoint(void)
{
    return &rc_ctrl;
}

