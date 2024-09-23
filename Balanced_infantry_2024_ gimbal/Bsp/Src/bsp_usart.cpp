/**
 ******************************************************************************
 * @file    bsp_usart.cpp
 * @author  Xushuang
 * @version V1.0.0 基本完成
 *          
 * @date    2023/8/25
 * @brief	串口支持包，可以根据board_def.h开发板类型进行串口用途的设置
 *			正常此文件禁止修改
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "arm_math.h"
#include "bsp_usart.h"

/* usart service instance, modules' info would be recoreded here using USARTRegister() */
/* usart服务实例,所有注册了usart的模块信息会被保存在这里 */
static uint8_t idx;
Usart_Instance_t *usart_instance[DEVICE_USART_CNT] = {NULL};

//尝试是否可以合并为同一个函数（分开更能看出过程）
void USARTInit(Usart_Instance_t *init_module,UART_HandleTypeDef *init_handle,USART_Rx_MODE_e init_mode,uint8_t init_size,UsartMsgCallback init_func)
{
	init_module->usart_handle = init_handle;
	init_module->rx_mode = init_mode;
	init_module->rx_buf_size = init_size;
	init_module->callback_func = init_func;
}
//注册函数
Usart_Instance_t *UsartRegister(Usart_Instance_t *init_usart)
{
	if(init_usart == NULL)
		return NULL;
	
	HAL_UARTEx_ReceiveToIdle_DMA(init_usart->usart_handle, init_usart->rx_buff, init_usart->rx_buf_size);
    // 关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
    // 这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
    // 我们只希望处理第一种和第三种情况,因此直接关闭DMA半传输中断
    __HAL_DMA_DISABLE_IT(init_usart->usart_handle->hdmarx, DMA_IT_HT);
	
	__HAL_UART_DISABLE_IT(init_usart->usart_handle, UART_IT_ERR);
	__HAL_UART_DISABLE_IT(init_usart->usart_handle, UART_IT_PE);
	
	usart_instance[idx++] = init_usart;
	return init_usart;
}


/**
  * @brief          串口中断回调函数（空闲中断）
  * @param[in]      *huart：串口号
  * @param[in]      size：数据大小
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
                memset(usart_instance[i]->rx_buff, 0, Size); // 接收结束后清空buffer,对于变长数据是必要的
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
 * @brief 当串口发送/接收出现错误时,会调用此函数,此时这个函数要做的就是重新启动接收
 *
 * @note  最常见的错误:奇偶校验/溢出/帧错误
 *
 * @param huart 发生错误的串口
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
//串口三的串口中断
void USART3_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart3);
}

////串口一的串口中断
//void USART1_IRQHandler(void)
//{
//	HAL_UART_IRQHandler(&huart1);
//}
}
//串口发送
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

//串口发送字符
void Usart_SendString(UART_HandleTypeDef *huart,uint8_t *str)
{
	unsigned int k=0;
	do {
		HAL_UART_Transmit( huart,(uint8_t *)(str + k) ,1,1000);
		k++;
	} while (*(str + k)!='\0');
}

//禁用串口通信接口
void USARTUnable(UART_HandleTypeDef *huart)
{
    __HAL_UART_DISABLE(huart);
}

//重新启动串口通信接口
void USARTRestart(UART_HandleTypeDef *huart,DMA_HandleTypeDef *dma_huart)
{
//    __HAL_UART_DISABLE(huart);
//    __HAL_DMA_DISABLE(dma_huart);

//    dma_huart->Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(dma_huart);
    __HAL_UART_ENABLE(huart);
}

