#include "usart2.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "usart.h"
#include "misc.h"
//#include "stdio.h"
//#include "string.h"

 u8 mode_data[8];
 u8 six_data_stop[3]={0X59,0X59,0X59};  //停止数据样本
 u8 six_data_start[3]={0X58,0X58,0X58};  //启动数据样本


/**************************************************************************
函数功能：串口2初始化
入口参数：pclk2:PCLK2 时钟频率(Mhz)    bound:波特率
返回  值：无
**************************************************************************/
void uart2_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB1ENR|=1<<17;  //使能串口时钟 
	GPIOA->CRL&=0XFFFF00FF; 
	GPIOA->CRL|=0X00008B00;//IO状态设置
	GPIOA->ODR|=1<<2;	  
	RCC->APB1RSTR|=1<<17;   //复位串口1
	RCC->APB1RSTR&=~(1<<17);//停止复位	   	   
	//波特率设置
 	USART2->BRR=mantissa; // 波特率设置	 
	USART2->CR1|=0X200C;  //1位停止,无校验位.
	//使能接收中断
	USART2->CR1|=1<<8;    //PE中断使能
	USART2->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	MY_NVIC_Init(3,3,USART2_IRQn,2);//组2，最低优先级 

    // -YJ- 打开空闲中断 
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);    
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);  // 当完成收完一帧的时候会触发中断，但是这个本质是检测空闲，如果连续发??
}



/*
  USART配置GPIO配置
*/
void USART2_Configuration(u32 baud_rate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 打开GPIOA时钟、AFIO时钟，USART2时钟 */
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
											
	/* USART 配置 */
	USART_DeInit(USART2);
	USART_InitStructure.USART_BaudRate = baud_rate;//115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	/* 使能USART2收发中断 */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);		   

	// -YJ- 打开空闲中断 
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);  // 当完成收完一帧的时候会触发中断，但是这个本质是检测空闲，如果连续发??

	/* 使能USART2 */
	USART_Cmd(USART2, ENABLE);
	/* 清除发送完成标志 */
	USART_ClearFlag(USART2, USART_FLAG_TC);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);		   
  
    /* Enable the USART2 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			 //串口中断设置
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}



/**************************************************************************
//函数功能：串口2接收中断
//入口参数：无
//返回  值：无
//**************************************************************************/
//void USART2_IRQHandler(void)
//{	
//if(USART2->SR & (1<<5))//接收到数据
//	{	  
//	 static	int uart_receive=0;//蓝牙接收相关变量
//	 uart_receive=USART2->DR; 
//	 mode_data[0]=uart_receive;
//			if(mode_data[0]==six_data_start[0]			&&mode_data[1]==six_data_start[1]			&&mode_data[2]==six_data_start[2]			)
//		{	
//			Flag_Stop=0;   //3击低速挡 小车关闭电机
//			mode_data[0]=0;	mode_data[1]=0;	mode_data[2]=0;	
//		}
//			if(mode_data[0]==six_data_stop[0]			&&mode_data[1]==six_data_stop[1]			&&mode_data[2]==six_data_stop[2]			)
//		{	
//			Flag_Stop=1;   //3击高速挡 小车启动电机
//			mode_data[0]=0;	mode_data[1]=0;	mode_data[2]=0;	
//		}
//		if(uart_receive==0x59)  Flag_sudu=2;  //低速挡（默认值）
//		if(uart_receive==0x58)  Flag_sudu=1;  //高速档
//		
//	  if(uart_receive>10)  //默认使用app为：MiniBalanceV3.5 因为MiniBalanceV3.5的遥控指令为A~H 其HEX都大于10
//    {			
//			if(uart_receive==0x5A)	Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////刹车
//			else if(uart_receive==0x41)	Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////前
//			else if(uart_receive==0x45)	Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;//////////////后
//			else if(uart_receive==0x42||uart_receive==0x43||uart_receive==0x44)	
//														Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;  //左
//			else if(uart_receive==0x46||uart_receive==0x47||uart_receive==0x48)	    //右
//														Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;
//			else Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////刹车
//  	}
//		if(uart_receive<10)     //备用app为：MiniBalanceV1.0  因为MiniBalanceV1.0的遥控指令为A~H 其HEX都小于10
//		{			

//			if(uart_receive==0x00)	Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////刹车
//			else if(uart_receive==0x01)	Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////前
//			else if(uart_receive==0x05)	Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;//////////////后
//			else if(uart_receive==0x02||uart_receive==0x03||uart_receive==0x04)	
//														Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;  //左
//			else if(uart_receive==0x06||uart_receive==0x07||uart_receive==0x08)	    //右
//														Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;
//			else Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////刹车
//  	}	
//		mode_data[2]=mode_data[1];
//		mode_data[1]=mode_data[0];
//	}  											 				 
//} 


/*
  USART2中断服务程序
*/				  
void USART2_IRQHandler(void)
{
    u8 Clear = Clear;		// 用来消除编译器的"没有用到"提醒
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
		RxBuffer2_tmp[RxCounter2++] = USART_ReceiveData(USART2);	//接收数据 逐个字节接收											   				 

    }
	else if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)	// 如果接收到1帧数据
	{
		Clear = USART2->SR;		// 读SR寄存器
		Clear = USART2->DR;		// 读DR寄存器(先读SR再读DR，就是为了清除IDLE中断)
		ReceiveState2 = 1;			// 标记接收到了1帧数据

        FLAG_stop_uart2_debug_msg = 1; // 接收调参信息时，1:禁止打印，除非正常收到数据

        memset(RxBuffer2, 0, 1000);
        memcpy(RxBuffer2, RxBuffer2_tmp, RxCounter2);
        memset(RxBuffer2_tmp, 0, 1000);

        RxCounter2_frame = RxCounter2;
        RxCounter2 = 0;            
	}
	

    /*禁止发送中断 */
    if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)                    
    { 
        USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    }
}





/*
  串口发送函数
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
  串口发送字符串
*/
void USART_STR(USART_TypeDef* USARTx, char *str)
{
    if( FLAG_stop_uart2_debug_msg == 0) // 接收调参信息时，1:禁止打印，除非正常收到数据)
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
//   TODO 09.11 打印出的数据格式还有问题
///*****************************/
void Comm_Send_msg_str(USART_TypeDef* USARTx, char *data_name, int16_t *data, uint16_t len)
{
    char Buf[60];
    uint16_t i;

    USART_STR(USARTx, data_name);
    
    for(i=0; i<len; i++)
    {
        memset(Buf,0x00,60);  // 清空
        sprintf(Buf, ": %d", data);
    	USART_STR(USARTx, Buf);	        
    }   
    
	USART_STR(USARTx,"\r\n");				
}



