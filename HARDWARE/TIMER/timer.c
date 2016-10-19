#include "timer.h"
#include "led.h"

u16 TIM4CH4_CAPTURE_STA, TIM4CH4_CAPTURE_STA;

/**************************************************************************
函数功能：定时中断初始化
入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
**************************************************************************/
void Timer1_Init(u16 arr,u16 psc)  
{  
	RCC->APB2ENR|=1<<11;//TIM1时钟使能    
 	TIM1->ARR=arr;      //设定计数器自动重装值   
	TIM1->PSC=psc;      //预分频器7200,得到10Khz的计数时钟
	TIM1->DIER|=1<<0;   //允许更新中断				
	TIM1->DIER|=1<<6;   //允许触发中断	   
	TIM1->CR1|=0x01;    //使能定时器
	MY_NVIC_Init(1, 3, TIM1_UP_IRQn, 2);
}  

/**************************************************************************
函数功能：定时器2通道4输入捕获  2,3已经被encoder使用了
入口参数：入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
**************************************************************************/
void TIM2_Cap_Init(u16 arr,u16 psc)	
{	 
	RCC->APB1ENR|=1<<0;   	//TIM2 时钟使能 
	RCC->APB2ENR|=1<<2;    	//使能PORTA时钟   	 
	GPIOA->CRL&=0XFFFF00FF; 
	GPIOA->CRL|=0X00008200;//Pa.2 推挽输出   	Pa.3 输入 
	
    TIM2->ARR=arr;  		//设定计数器自动重装值   
	TIM2->PSC=psc;  		//预分频器 
	TIM2->CCMR2|=1<<8;		//CC1S=01 	选择输入端 IC1映射到TI1上
 	TIM2->CCMR2|=0<<12; 		//IC1F=0000 配置输入滤波器 不滤波
 	TIM2->CCMR2|=0<<10; 	//IC2PS=00 	配置输入分频,不分频 

	TIM2->CCER|=0<<13; 		//CC1P=0	上升沿捕获
	TIM2->CCER|=1<<12; 		//CC1E=1 	允许捕获计数器的值到捕获寄存器中

	TIM2->DIER|=1<<4;   	//允许捕获中断				
	TIM2->DIER|=1<<0;   	//允许更新中断	
	TIM2->CR1|=0x01;    	//使能定时器2
	MY_NVIC_Init(1,3,TIM2_IRQn,1);
}


/**************************************************************************
函数功能：定时器4通道3输入捕获初始化
入口参数：入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
**************************************************************************/
void TIM4_Cap_Init(u16 arr,u16 psc)	
{	 
    RCC->APB1ENR|=1<<2;     //TIM4时钟使能     
    RCC->APB2ENR|=1<<3;    	//使能PORTB时钟   	 
    GPIOB->CRL&=0XFFFFFF00; 
    GPIOB->CRL|=0X00000028;//  PB.0 输入 PB.1输出

    TIM4->ARR=arr;  	//设定计数器自动重装值   
    TIM4->PSC=psc;  	//预分频器 
    TIM4->CCMR2|=1<<0;	//选择输入端 
    TIM4->CCMR2|=0<<4; 	// 配置输入滤波器 不滤波
    TIM4->CCMR2|=0<<2; 	//配置输入分频,不分频 

    TIM4->CCER|=0<<9; 	//上升沿捕获
    TIM4->CCER|=1<<8; 	//允许捕获计数器的值到捕获寄存器中

    TIM4->DIER|=1<<3;   //允许捕获中断				
    TIM4->DIER|=1<<0;   //允许更新中断	
    TIM4->CR1|=0x01;    //使能定时器4
    MY_NVIC_Init(1,3,TIM4_IRQn,1);
}


/**************************************************************************
函数功能：超声波回波脉宽读取中断
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void TIM4_IRQHandler(void)
{ 		    		  			    
    u16 tsr;
    tsr=TIM3->SR;
    if((TIM4CH4_CAPTURE_STA&0X80)==0)//还未成功捕获	
    {
        if(tsr&0X01)//溢出
        {	    
            if(TIM4CH4_CAPTURE_STA&0X40)//已经捕获到高电平了
            {
                if((TIM4CH4_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
                {
                    TIM4CH4_CAPTURE_STA|=0X80;//标记成功捕获了一次
                    TIM4CH4_CAPTURE_STA=0XFFFF;
                }else{
                    TIM4CH4_CAPTURE_STA++;
                }
            }	 
        }
        
        if(tsr&0x08)//捕获3发生捕获事件
        {	
            if(TIM4CH4_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
            {	  			
                TIM4CH4_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
                TIM4CH4_CAPTURE_STA=TIM3->CCR3;	//获取当前的捕获值.
                TIM3->CCER&=~(1<<9);			//CC1P=0 设置为上升沿捕获
            }else  								//还未开始,第一次捕获上升沿
            {
                TIM4CH4_CAPTURE_STA=0;			//清空
                TIM4CH4_CAPTURE_STA=0;
                TIM4CH4_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
                TIM3->CNT=0;					//计数器清空
                TIM3->CCER|=1<<9; 				//CC1P=1 设置为下降沿捕获
            }		    
        }			     	    					   
    }
    TIM3->SR=0;//清除中断标志位 	     
}


/**************************************************************************
函数功能：超声波接收回波函数
入口参数：无
返回  值：无
**************************************************************************/
void Read_Distane(void)
{   
    PBout(1)=1;
    delay_us(15);  
    PBout(1)=0;	
    if(TIM4CH4_CAPTURE_STA&0X80)//成功捕获到了一次高电平
    {
        g_distance=TIM4CH4_CAPTURE_STA&0X3F;
        g_distance*=65536;					        //溢出时间总和
        g_distance+=TIM4CH4_CAPTURE_STA;		//得到总的高电平时间
        g_distance=g_distance*170/1000;
        //	printf("%d \r\n",g_distance);
        TIM4CH4_CAPTURE_STA=0;			//开启下一次捕获
    }				
}


