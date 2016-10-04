#include "usart2.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "usart.h"
#include "misc.h"
//#include "stdio.h"
//#include "string.h"

 u8 mode_data[8];
 u8 six_data_stop[3]={0X59,0X59,0X59};  //ֹͣ��������
 u8 six_data_start[3]={0X58,0X58,0X58};  //������������


/**************************************************************************
�������ܣ�����2��ʼ��
��ڲ�����pclk2:PCLK2 ʱ��Ƶ��(Mhz)    bound:������
����  ֵ����
**************************************************************************/
void uart2_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //ʹ��PORTA��ʱ��  
	RCC->APB1ENR|=1<<17;  //ʹ�ܴ���ʱ�� 
	GPIOA->CRL&=0XFFFF00FF; 
	GPIOA->CRL|=0X00008B00;//IO״̬����
	GPIOA->ODR|=1<<2;	  
	RCC->APB1RSTR|=1<<17;   //��λ����1
	RCC->APB1RSTR&=~(1<<17);//ֹͣ��λ	   	   
	//����������
 	USART2->BRR=mantissa; // ����������	 
	USART2->CR1|=0X200C;  //1λֹͣ,��У��λ.
	//ʹ�ܽ����ж�
	USART2->CR1|=1<<8;    //PE�ж�ʹ��
	USART2->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(3,3,USART2_IRQn,2);//��2��������ȼ� 

    // -YJ- �򿪿����ж� 
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);    
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);  // ���������һ֡��ʱ��ᴥ���жϣ�������������Ǽ����У����������??
}



/*
  USART����GPIO����
*/
void USART2_Configuration(u32 baud_rate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* ��GPIOAʱ�ӡ�AFIOʱ�ӣ�USART2ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	/* USART2 TX PA2 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* USART2 RX PA3 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
											
	/* USART ���� */
	USART_DeInit(USART2);
	USART_InitStructure.USART_BaudRate = baud_rate;//115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	/* ʹ��USART2�շ��ж� */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);		   

	// -YJ- �򿪿����ж� 
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);  // ���������һ֡��ʱ��ᴥ���жϣ�������������Ǽ����У����������??

	/* ʹ��USART2 */
	USART_Cmd(USART2, ENABLE);
	/* ���������ɱ�־ */
	USART_ClearFlag(USART2, USART_FLAG_TC);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);		   
  
    /* Enable the USART2 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			 //�����ж�����
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}



/**************************************************************************
//�������ܣ�����2�����ж�
//��ڲ�������
//����  ֵ����
//**************************************************************************/
//void USART2_IRQHandler(void)
//{	
//if(USART2->SR & (1<<5))//���յ�����
//	{	  
//	 static	int uart_receive=0;//����������ر���
//	 uart_receive=USART2->DR; 
//	 mode_data[0]=uart_receive;
//			if(mode_data[0]==six_data_start[0]			&&mode_data[1]==six_data_start[1]			&&mode_data[2]==six_data_start[2]			)
//		{	
//			Flag_Stop=0;   //3�����ٵ� С���رյ��
//			mode_data[0]=0;	mode_data[1]=0;	mode_data[2]=0;	
//		}
//			if(mode_data[0]==six_data_stop[0]			&&mode_data[1]==six_data_stop[1]			&&mode_data[2]==six_data_stop[2]			)
//		{	
//			Flag_Stop=1;   //3�����ٵ� С���������
//			mode_data[0]=0;	mode_data[1]=0;	mode_data[2]=0;	
//		}
//		if(uart_receive==0x59)  Flag_sudu=2;  //���ٵ���Ĭ��ֵ��
//		if(uart_receive==0x58)  Flag_sudu=1;  //���ٵ�
//		
//	  if(uart_receive>10)  //Ĭ��ʹ��appΪ��MiniBalanceV3.5 ��ΪMiniBalanceV3.5��ң��ָ��ΪA~H ��HEX������10
//    {			
//			if(uart_receive==0x5A)	Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ɲ��
//			else if(uart_receive==0x41)	Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ǰ
//			else if(uart_receive==0x45)	Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;//////////////��
//			else if(uart_receive==0x42||uart_receive==0x43||uart_receive==0x44)	
//														Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;  //��
//			else if(uart_receive==0x46||uart_receive==0x47||uart_receive==0x48)	    //��
//														Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;
//			else Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ɲ��
//  	}
//		if(uart_receive<10)     //����appΪ��MiniBalanceV1.0  ��ΪMiniBalanceV1.0��ң��ָ��ΪA~H ��HEX��С��10
//		{			

//			if(uart_receive==0x00)	Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ɲ��
//			else if(uart_receive==0x01)	Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ǰ
//			else if(uart_receive==0x05)	Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;//////////////��
//			else if(uart_receive==0x02||uart_receive==0x03||uart_receive==0x04)	
//														Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;  //��
//			else if(uart_receive==0x06||uart_receive==0x07||uart_receive==0x08)	    //��
//														Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;
//			else Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ɲ��
//  	}	
//		mode_data[2]=mode_data[1];
//		mode_data[1]=mode_data[0];
//	}  											 				 
//} 


/*
  USART2�жϷ������
*/				  
void USART2_IRQHandler(void)
{
    u8 Clear = Clear;		// ����������������"û���õ�"����
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
		RxBuffer2_tmp[RxCounter2++] = USART_ReceiveData(USART2);	//�������� ����ֽڽ���											   				 

    }
	else if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)	// ������յ�1֡����
	{
		Clear = USART2->SR;		// ��SR�Ĵ���
		Clear = USART2->DR;		// ��DR�Ĵ���(�ȶ�SR�ٶ�DR������Ϊ�����IDLE�ж�)
		ReceiveState2 = 1;			// ��ǽ��յ���1֡����

        FLAG_stop_uart2_debug_msg = 1; // ���յ�����Ϣʱ��1:��ֹ��ӡ�����������յ�����

        memset(RxBuffer2, 0, 1000);
        memcpy(RxBuffer2, RxBuffer2_tmp, RxCounter2);
        memset(RxBuffer2_tmp, 0, 1000);

        RxCounter2_frame = RxCounter2;
        RxCounter2 = 0;            
	}
	

    /*��ֹ�����ж� */
    if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)                    
    { 
        USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    }
}





/*
  ���ڷ��ͺ���
*/
void USART_Send(USART_TypeDef* USARTx, uint8_t *Dat, uint16_t len)
{
  uint16_t i;
  for(i=0;i<len;i++)
  {
  	USART_SendData(USARTx, Dat[i]);
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);     
  }
}

/*
  ���ڷ����ַ���
*/
void USART_STR(USART_TypeDef* USARTx, char *str)
{
    if( FLAG_stop_uart2_debug_msg == 0) // ���յ�����Ϣʱ��1:��ֹ��ӡ�����������յ�����)
    {
        uint8_t len,i;
    	len=strlen(str);
    	for(i=0;i<len;i++)
    	{
    	  	USART_SendData(USARTx, str[i]);
    		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET); 	
    	}
    }
}


static void Char2Str(char *Datout,char *Datin,unsigned char len)
{
    unsigned char j;
    
    for(j=0;j<len;j++)
    {
        sprintf(Datout,",%02X",Datin[j]);
        Datout+=3;	
    }	
}

//*****************************
//   TODO 09.11 ��ӡ�������ݸ�ʽ��������
///*****************************/
void Comm_Send_msg_str(USART_TypeDef* USARTx, char *data_name, int16_t *data, uint16_t len)
{
    char Buf[60];
    uint16_t i;

    USART_STR(USARTx, data_name);
    
    for(i=0; i<len; i++)
    {
        memset(Buf,0x00,60);  // ���
        sprintf(Buf, ": %d", data);
    	USART_STR(USARTx, Buf);	        
    }   
    
	USART_STR(USARTx,"\r\n");				
}



