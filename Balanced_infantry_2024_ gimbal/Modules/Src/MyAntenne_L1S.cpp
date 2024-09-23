/**
 ******************************************************************************
 * @file    MyAntenna.cpp
 * @author  Xushuang
 * @version V1.0.0 �������
 * @date    2023/12/7
 * @brief		Ħ�켤�����ļ�
 *					�������ļ���ֹ�޸�
 ******************************************************************************
 * @attention
 *	���Ҹ������ֿ��Ʒ�ʽ���˴���ʹ����HEXͨѶЭ����п���
 ******************************************************************************
 */
#include "MyAntenne_L1S.h"
#include "string.h"

void DecodeL1sData();

//����ʵ��
MyAntennaL1S_t L1S;

//��ʼ��
void L1SInit()
{
	USARTInit(&L1S.module_usart,&huart1,USART_RX_DMA,L1S_BUFFER_LEN,DecodeL1sData); //��ʼ�����ô��ڽ���
	UsartRegister(&L1S.module_usart);  //�Ǽ�ע��
}

//��Ϣ����
void DecodeL1sData()
{
	L1S.CalcDistance(L1S.module_usart.rx_buff);
}

MyAntennaL1S_t::MyAntennaL1S_t()
{

}

//�������
void MyAntennaL1S_t::CalcDistance(volatile uint8_t *info)
{
	distance = (uint32_t)(info[3] << 24 | info[4] << 16 | info[5] <<8 | info[6]);
}

/***********************************************************
* ��������HEX_Single_Meas_Cmd
* ��  ����void
* ����ֵ��void
* ��  �������͵��β�������
* ���ߣ�pamala, 2020.2.15
************************************************************/
void HEX_Single_Meas_Cmd(void)             
{
    unsigned char cmd[5] = {0XA5, 0X5A, 0X02, 0X00, 0XFD};
		HAL_UART_Transmit_IT(L1S.module_usart.usart_handle, cmd, 5);
}

/***********************************************************
* ��������HEX_Conti_Meas_Cmd
* ��  ����void
* ����ֵ��void
* ��  ��������������������
* ���ߣ�pamala, 2020.2.15
************************************************************/
void HEX_Conti_Meas_Cmd(void)             
{
    unsigned char cmd[5] = {0XA5, 0X5A, 0X03, 0X00, 0XFC};
		HAL_UART_Transmit(L1S.module_usart.usart_handle, cmd, sizeof(cmd),200);
}

/***********************************************************
* ��������HEX_FastConti_Meas_Cmd
* ��  ����void
* ����ֵ��void
* ��  �������Ϳ���������������
* ���ߣ�pamala, 2020.2.15
************************************************************/
void HEX_FastConti_Meas_Cmd(void)          
{
    unsigned char cmd[5] = {0XA5, 0X5A, 0X04, 0X00, 0XFB};
		HAL_UART_Transmit(L1S.module_usart.usart_handle, cmd, sizeof(cmd),500);
}

/***********************************************************
* ��������HEX_FastConti_Meas_Cmd
* ��  ����void
* ����ֵ��void
* ��  ��������ֹͣ��������
* ���ߣ�pamala, 2020.2.15
************************************************************/
void HEX_Stop_Meas_Cmd(void)             
{
    unsigned char cmd[5] = {0XA5, 0X5A, 0X05, 0X00, 0XFA};
		HAL_UART_Transmit(L1S.module_usart.usart_handle, cmd, sizeof(cmd),200);
}
