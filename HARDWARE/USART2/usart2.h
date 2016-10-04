#ifndef __USRAT_H
#define __USRAT_H 
#include "sys.h"	  	
extern u8 Usart3_Receive;
void uart2_init(u32 pclk2,u32 bound);
void USART2_IRQHandler(void);

void USART_Send(USART_TypeDef* USARTx, uint8_t *Dat,uint16_t len);
void USART_STR(USART_TypeDef* USARTx,char *str);
void Comm_Send_msg_str(USART_TypeDef* USARTx, char *data_name, int16_t *data, uint16_t len);
void USART2_Configuration(u32 baud_rate);


#endif

