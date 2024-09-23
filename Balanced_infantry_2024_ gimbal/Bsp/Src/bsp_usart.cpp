/**
 ******************************************************************************
 * @file    bsp_usart.cpp
 * @author  Xushuang
 * @version V1.0.0 �������
 *          
 * @date    2023/8/25
 * @brief	����֧�ְ������Ը���board_def.h���������ͽ��д�����;������
 *			�������ļ���ֹ�޸�
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "arm_math.h"
#include "bsp_usart.h"

/* usart service instance, modules' info would be recoreded here using USARTRegister() */
/* usart����ʵ��,����ע����usart��ģ����Ϣ�ᱻ���������� */
static uint8_t idx;
Usart_Instance_t *usart_instance[DEVICE_USART_CNT] = {NULL};

//�����Ƿ���Ժϲ�Ϊͬһ���������ֿ����ܿ������̣�
void USARTInit(Usart_Instance_t *init_module,UART_HandleTypeDef *init_handle,USART_Rx_MODE_e init_mode,uint8_t init_size,UsartMsgCallback init_func)
{
	init_module->usart_handle = init_handle;
	init_module->rx_mode = init_mode;
	init_module->rx_buf_size = init_size;
	init_module->callback_func = init_func;
}
//ע�ắ��
Usart_Instance_t *UsartRegister(Usart_Instance_t *init_usart)
{
	if(init_usart == NULL)
		return NULL;
	
	HAL_UARTEx_ReceiveToIdle_DMA(init_usart->usart_handle, init_usart->rx_buff, init_usart->rx_buf_size);
    // �ر�dma half transfer�жϷ�ֹ���ν���HAL_UARTEx_RxEventCallback()
    // ����HAL���һ�����ʧ��,����DMA�������/������Լ�����IDLE�ж϶��ᴥ��HAL_UARTEx_RxEventCallback()
    // ����ֻϣ�������һ�ֺ͵��������,���ֱ�ӹر�DMA�봫���ж�
    __HAL_DMA_DISABLE_IT(init_usart->usart_handle->hdmarx, DMA_IT_HT);
	
	__HAL_UART_DISABLE_IT(init_usart->usart_handle, UART_IT_ERR);
	__HAL_UART_DISABLE_IT(init_usart->usart_handle, UART_IT_PE);
	
	usart_instance[idx++] = init_usart;
	return init_usart;
}


/**
  * @brief          �����жϻص������������жϣ�
  * @param[in]      *huart�����ں�
  * @param[in]      size�����ݴ�С
  * @retval         NULL
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < idx; ++i)
    { // find the instance which is being handled
        if (huart == usart_instance[i]->usart_handle)
        { // call the callback function if it is not NULL
            if (usart_instance[i]->callback_func != NULL)
            {
                usart_instance[i]->callback_func();
                memset(usart_instance[i]->rx_buff, 0, Size); // ���ս��������buffer,���ڱ䳤�����Ǳ�Ҫ��
            }
			if(usart_instance[i]->rx_mode == USART_RX_DMA)
				HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->rx_buff, usart_instance[i]->rx_buf_size);
			else if(usart_instance[i]->rx_mode == USART_RX_IT)
				HAL_UARTEx_ReceiveToIdle_IT(usart_instance[i]->usart_handle, usart_instance[i]->rx_buff, usart_instance[i]->rx_buf_size);
						
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
						__HAL_UART_DISABLE_IT(usart_instance[i]->usart_handle, UART_IT_ERR);
						__HAL_UART_DISABLE_IT(usart_instance[i]->usart_handle, UART_IT_PE);

            return; // break the loop
        }
    }
}

/**
 * @brief �����ڷ���/���ճ��ִ���ʱ,����ô˺���,��ʱ�������Ҫ���ľ���������������
 *
 * @note  ����Ĵ���:��żУ��/���/֡����
 *
 * @param huart ��������Ĵ���
 */
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//    for (uint8_t i = 0; i < idx; ++i)
//    {
//        if (huart == usart_instance[i]->usart_handle)
//        {
//            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->rx_buff, usart_instance[i]->rx_buf_size);
//            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
////            LOGWARNING("[bsp_usart] USART error callback triggered, instance idx [%d]", i);
//            return;
//        }
//    }
//}

extern "C"
{
//�������Ĵ����ж�
void USART3_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart3);
}

////����һ�Ĵ����ж�
//void USART1_IRQHandler(void)
//{
//	HAL_UART_IRQHandler(&huart1);
//}
}
//���ڷ���
void USARTSend(UART_HandleTypeDef *huart,uint8_t *send_buff,uint16_t send_size,USART_TRANSFER_MODE_e mode)
{
	switch(mode)
	{
		case USART_TRANSFER_BLOCKING:
			HAL_UART_Transmit(huart,send_buff,send_size,200);
			break;
		case USART_TRANSFER_IT:
			HAL_UART_Transmit_IT(huart,send_buff,send_size);
			break;
		case USART_TRANSFER_DMA:
			HAL_UART_Transmit_DMA(huart,send_buff,send_size);
			break;
		case USART_TRANSFER_NONE:
			break;
	}
}

//���ڷ����ַ�
void Usart_SendString(UART_HandleTypeDef *huart,uint8_t *str)
{
	unsigned int k=0;
	do {
		HAL_UART_Transmit( huart,(uint8_t *)(str + k) ,1,1000);
		k++;
	} while (*(str + k)!='\0');
}

//���ô���ͨ�Žӿ�
void USARTUnable(UART_HandleTypeDef *huart)
{
    __HAL_UART_DISABLE(huart);
}

//������������ͨ�Žӿ�
void USARTRestart(UART_HandleTypeDef *huart,DMA_HandleTypeDef *dma_huart)
{
//    __HAL_UART_DISABLE(huart);
//    __HAL_DMA_DISABLE(dma_huart);

//    dma_huart->Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(dma_huart);
    __HAL_UART_ENABLE(huart);
}

