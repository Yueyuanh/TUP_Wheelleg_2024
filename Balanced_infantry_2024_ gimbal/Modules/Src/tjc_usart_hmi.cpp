#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "bsp_usart.h"
#include "tjc_usart_hmi.h"
#define STR_LENGTH 100

//�������޸ģ�δ�޸Ľ���
//Copy��δ�޸�
typedef struct
{
    uint16_t Head;
    uint16_t Tail;
    uint16_t Lenght;
    uint8_t  Ring_data[RINGBUFF_LEN];
}RingBuff_t;

RingBuff_t ringBuff;	//����һ��ringBuff�Ļ�����
uint8_t RxBuff[1];



/********************************************************
��������  	TJCPrintf
���ߣ�    	wwd
���ڣ�    	2022.10.08
���ܣ�    	�򴮿ڴ�ӡ����,ʹ��ǰ�뽫USART_SCREEN_write���������Ϊ��ĵ�Ƭ���Ĵ��ڷ��͵��ֽں���
���������		�ο�printf
����ֵ�� 		��ӡ�����ڵ�����
�޸ļ�¼��
**********************************************************/
//void USART1_printf (char *fmt, ...){ 
//	char buffer[STR_LENGTH+1];  // ���ݳ���
//	u8 i = 0;	
//	va_list arg_ptr;
//	va_start(arg_ptr, fmt);  
//	vsnprintf(buffer, STR_LENGTH+1, fmt, arg_ptr);
//	while ((i < STR_LENGTH) && (i < strlen(buffer))){
//        USART_SendData(USART1, (u8) buffer[i++]);
//        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); 
//	}
//	va_end(arg_ptr);
//	
//}

uint8_t frame_tail[] = {0xff,0xff,0xff};
void TJCPrintf (const char *str, ...){ 
	uint8_t buffer[STR_LENGTH+1];  // ���ݳ���
	uint8_t i = 0;	
	va_list arg_ptr;
	va_start(arg_ptr, str);  
	vsnprintf((char *)buffer, STR_LENGTH+1, str, arg_ptr);
	va_end(arg_ptr);
	while ((i < STR_LENGTH) && (i < strlen((char *)buffer)))
	{
        USARTSend(&huart1,&buffer[i++],1,USART_TRANSFER_BLOCKING);
        while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET); 
	}
	USARTSend(&huart1,frame_tail,3,USART_TRANSFER_BLOCKING);
	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET); 
}




/********************************************************
��������  	USART1_IRQHandler
���ߣ�
���ڣ�    	2022.10.08
���ܣ�    	���ڽ����ж�,�����յ�������д�뻷�λ�����
���������
����ֵ�� 		void
�޸ļ�¼��
**********************************************************/
//void USART1_IRQHandler(void)
//{
//	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
//	{
//		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
//		RxBuff[0] = USART_ReceiveData(USART1);
//		writeRingBuff(RxBuff[0]);
//		
//		
//		
//		
//	}
//}



/********************************************************
��������  	initRingBuff
���ߣ�
���ڣ�    	2022.10.08
���ܣ�    	��ʼ�����λ�����
���������
����ֵ�� 		void
�޸ļ�¼��
**********************************************************/
void initRingBuff(void)
{
  //��ʼ�������Ϣ
  ringBuff.Head = 0;
  ringBuff.Tail = 0;
  ringBuff.Lenght = 0;
}



/********************************************************
��������  	writeRingBuff
���ߣ�
���ڣ�    	2022.10.08
���ܣ�    	�����λ�����д������
���������
����ֵ�� 		void
�޸ļ�¼��
**********************************************************/
void writeRingBuff(uint8_t data)
{
  if(ringBuff.Lenght >= RINGBUFF_LEN) //�жϻ������Ƿ�����
  {
    return ;
  }
  ringBuff.Ring_data[ringBuff.Tail]=data;
  ringBuff.Tail = (ringBuff.Tail+1)%RINGBUFF_LEN;//��ֹԽ��Ƿ�����
  ringBuff.Lenght++;

}




/********************************************************
��������  	deleteRingBuff
���ߣ�
���ڣ�    	2022.10.08
���ܣ�    	ɾ�����ڻ���������Ӧ���ȵ�����
���������		Ҫɾ���ĳ���
����ֵ�� 		void
�޸ļ�¼��
**********************************************************/
void deleteRingBuff(uint16_t size)
{
	if(size >= ringBuff.Lenght)
	{
	    initRingBuff();
	    return;
	}
	for(int i = 0; i < size; i++)
	{

		if(ringBuff.Lenght == 0)//�жϷǿ�
		{
		initRingBuff();
		return;
		}
		ringBuff.Head = (ringBuff.Head+1)%RINGBUFF_LEN;//��ֹԽ��Ƿ�����
		ringBuff.Lenght--;

	}

}



/********************************************************
��������  	read1BFromRingBuff
���ߣ�
���ڣ�    	2022.10.08
���ܣ�    	�Ӵ��ڻ�������ȡ1�ֽ�����
���������		position:��ȡ��λ��
����ֵ�� 		����λ�õ�����(1�ֽ�)
�޸ļ�¼��
**********************************************************/
uint8_t read1BFromRingBuff(uint16_t position)
{
	uint16_t realPosition = (ringBuff.Head + position) % RINGBUFF_LEN;

	return ringBuff.Ring_data[realPosition];
}




/********************************************************
��������  	getRingBuffLenght
���ߣ�
���ڣ�    	2022.10.08
���ܣ�    	��ȡ���ڻ���������������
���������
����ֵ�� 		���ڻ���������������
�޸ļ�¼��
**********************************************************/
uint16_t getRingBuffLenght()
{
	return ringBuff.Lenght;
}


/********************************************************
��������  	isRingBuffOverflow
���ߣ�
���ڣ�    	2022.10.08
���ܣ�    	�жϻ��λ������Ƿ�����
���������
����ֵ�� 		1:���λ��������� , 2:���λ�����δ��
�޸ļ�¼��
**********************************************************/
uint8_t isRingBuffOverflow()
{
	return ringBuff.Lenght == RINGBUFF_LEN;
}



////��ʼ��IO ����1 
////bound:������
//void USART1_Init(uint32_t bound){ 
//	
//	//����1��ʼ��������
//	//GPIO�˿�����
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;	 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
//	 //USART1_TX   PA.9
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
//	GPIO_Init(GPIOA, &GPIO_InitStructure);  
//	//USART1_RX	  PA.10
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
//	GPIO_Init(GPIOA, &GPIO_InitStructure); 
//	//Usart1 NVIC ����
//	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ��� 
//	//USART ��ʼ������
//	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
//	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
//	USART_Init(USART1, &USART_InitStructure); //��ʼ������
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//����ENABLE/�ر�DISABLE�ж�
//	USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ��� 
//}


