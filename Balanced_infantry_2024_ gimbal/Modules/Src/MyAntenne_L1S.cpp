/**
 ******************************************************************************
 * @file    MyAntenna.cpp
 * @author  Xushuang
 * @version V1.0.0 基本完成
 * @date    2023/12/7
 * @brief		摩天激光测距文件
 *					正常此文件禁止修改
 ******************************************************************************
 * @attention
 *	卖家给了三种控制方式，此处仅使用了HEX通讯协议进行控制
 ******************************************************************************
 */
#include "MyAntenne_L1S.h"
#include "string.h"

void DecodeL1sData();

//建立实例
MyAntennaL1S_t L1S;

//初始化
void L1SInit()
{
	USARTInit(&L1S.module_usart,&huart1,USART_RX_DMA,L1S_BUFFER_LEN,DecodeL1sData); //初始化设置串口接收
	UsartRegister(&L1S.module_usart);  //登记注册
}

//信息解码
void DecodeL1sData()
{
	L1S.CalcDistance(L1S.module_usart.rx_buff);
}

MyAntennaL1S_t::MyAntennaL1S_t()
{

}

//计算距离
void MyAntennaL1S_t::CalcDistance(volatile uint8_t *info)
{
	distance = (uint32_t)(info[3] << 24 | info[4] << 16 | info[5] <<8 | info[6]);
}

/***********************************************************
* 函数名：HEX_Single_Meas_Cmd
* 参  数：void
* 返回值：void
* 描  述：发送单次测量命令
* 作者：pamala, 2020.2.15
************************************************************/
void HEX_Single_Meas_Cmd(void)             
{
    unsigned char cmd[5] = {0XA5, 0X5A, 0X02, 0X00, 0XFD};
		HAL_UART_Transmit_IT(L1S.module_usart.usart_handle, cmd, 5);
}

/***********************************************************
* 函数名：HEX_Conti_Meas_Cmd
* 参  数：void
* 返回值：void
* 描  述：发送连续测量命令
* 作者：pamala, 2020.2.15
************************************************************/
void HEX_Conti_Meas_Cmd(void)             
{
    unsigned char cmd[5] = {0XA5, 0X5A, 0X03, 0X00, 0XFC};
		HAL_UART_Transmit(L1S.module_usart.usart_handle, cmd, sizeof(cmd),200);
}

/***********************************************************
* 函数名：HEX_FastConti_Meas_Cmd
* 参  数：void
* 返回值：void
* 描  述：发送快速连续测量命令
* 作者：pamala, 2020.2.15
************************************************************/
void HEX_FastConti_Meas_Cmd(void)          
{
    unsigned char cmd[5] = {0XA5, 0X5A, 0X04, 0X00, 0XFB};
		HAL_UART_Transmit(L1S.module_usart.usart_handle, cmd, sizeof(cmd),500);
}

/***********************************************************
* 函数名：HEX_FastConti_Meas_Cmd
* 参  数：void
* 返回值：void
* 描  述：发送停止测量命令
* 作者：pamala, 2020.2.15
************************************************************/
void HEX_Stop_Meas_Cmd(void)             
{
    unsigned char cmd[5] = {0XA5, 0X5A, 0X05, 0X00, 0XFA};
		HAL_UART_Transmit(L1S.module_usart.usart_handle, cmd, sizeof(cmd),200);
}
