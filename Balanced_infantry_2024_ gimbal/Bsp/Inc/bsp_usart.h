#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "usart.h"
#include "board_def.h"
//����ͳһ����
#define DEVICE_USART_CNT 3     // C���������3������
#define MAX_USART_RXBUF 256  //�����Ϣ����

#ifdef __cplusplus
extern "C"{
#endif

#ifdef __cplusplus
// ģ��ص�����,���ڽ���Э��
typedef void (*UsartMsgCallback)();
	
enum USART_TRANSFER_MODE_e
{
  USART_TRANSFER_NONE=0,
  USART_TRANSFER_BLOCKING,
  USART_TRANSFER_IT,
  USART_TRANSFER_DMA,
};

enum USART_Rx_MODE_e
{
	USART_RX_IT,
	USART_RX_DMA,
};

typedef struct
{
	UART_HandleTypeDef            *usart_handle;             //���
	USART_Rx_MODE_e               rx_mode;                   //���շ�ʽ
	uint8_t                       rx_buf_size;               //��С
	uint8_t                       rx_buff[MAX_USART_RXBUF];  //����
	UsartMsgCallback              callback_func;             //�ص�����
}Usart_Instance_t;

void USARTInit(Usart_Instance_t *init_module,UART_HandleTypeDef *init_handle,USART_Rx_MODE_e init_mode,uint8_t init_size,UsartMsgCallback init_func);
Usart_Instance_t *UsartRegister(Usart_Instance_t *init_usart);
void USARTSend(UART_HandleTypeDef *huart,uint8_t *send_buff,uint16_t send_size,USART_TRANSFER_MODE_e mode);
#endif

void USARTUnable(UART_HandleTypeDef *huart);
void USARTRestart(UART_HandleTypeDef *huart,DMA_HandleTypeDef *dma_huart);
	
#ifdef __cplusplus
}
#endif

#endif
