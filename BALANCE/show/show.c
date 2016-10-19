#include "show.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
unsigned char i;          //��������
unsigned char Send_Count; //������Ҫ���͵����ݸ���
/**************************************************************************
�������ܣ�OLED��ʾ
��ڲ�������
����  ֵ����
**************************************************************************/

void oled_show(void)
{
    OLED_Display_On();  //��ʾ����
    //=============��ʾ�˲���=======================//	
    OLED_ShowString(00,0,"WAY-");
    OLED_ShowNumber(30,0, Way_Angle,1,12);
    if(Way_Angle==1)	
        OLED_ShowString(45,0,"DMP");
    else if(Way_Angle==2)	
        OLED_ShowString(45,0,"Kalman");
    else if(Way_Angle==3)	
        OLED_ShowString(45,0,"Hubu");
    
    //=============��ʾ�¶�=======================//	
    OLED_ShowString(00,10,"T:");
    OLED_ShowNumber(45,10,Temperature/10,2,12);
    OLED_ShowNumber(68,10,Temperature%10,1,12);
    OLED_ShowString(58,10,".");
    OLED_ShowString(80,10,"`C");
    
    //=============��ʾ������1=======================//	
    OLED_ShowString(00,20,"EncoL:");
    if( Encoder_Left<0)		
        OLED_ShowString(60,20,"-"),
        OLED_ShowNumber(65,20,-Encoder_Left,5,12);
    else                 	
        OLED_ShowString(60,20,"+"),
        OLED_ShowNumber(65,20, Encoder_Left,5,12);
    
    //=============��ʾ������2=======================//		
    OLED_ShowString(00,30,"EncoR:");
    if(Encoder_Right<0)		
        OLED_ShowString(60,30,"-"),
        OLED_ShowNumber(65,30,-Encoder_Right,5,12);
    else               		
        OLED_ShowString(60,30,"+"),
        OLED_ShowNumber(65,30,Encoder_Right,5,12);	
    
    //=============��ʾ��ѹ=======================//
    OLED_ShowString(00,40,"Volta");
    OLED_ShowString(58,40,".");
    OLED_ShowString(80,40,"V");
    OLED_ShowNumber(45,40,Voltage/100,2,12);
    OLED_ShowNumber(68,40,Voltage%100,2,12);
    if(Voltage%100<10) 	OLED_ShowNumber(62,40,0,2,12);
    
    //=============��ʾ�Ƕ�=======================//
    OLED_ShowString(0,50,"Angle");
    if(Angle_Balance<0)		
        OLED_ShowNumber(45,50,Angle_Balance+360,3,12);
    else
        OLED_ShowNumber(45,50,Angle_Balance,3,12);
    
    //=============ˢ��=======================//
    OLED_Refresh_Gram();	
	}
/**************************************************************************
�������ܣ���APP��������
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void APP_Show(void)
{   
    char Buf[60];
    int app_2,app_3,app_4;
    app_4=(Voltage-1110)*2/3;		if(app_4<0)app_4=0;if(app_4>100)app_4=100;
    app_3=Encoder_Right*1.1; if(app_3<0)app_3=-app_3;			
    app_2=Encoder_Left*1.1;  if(app_2<0)app_2=-app_2;
//    printf("Z%d:%d:%d:%dL$",(u8)app_2,(u8)app_3,(u8)app_4,(int)Angle_Balance);
//    sprintf(Buf,"Z%d:%d:%d:%dL$",(u8)app_2,(u8)app_3,(u8)app_4,(int)Angle_Balance);
    sprintf(Buf,"Z%d:%d:%d:%dL$",(u8)Flag_Qian,(u8)Flag_Hou,(u8)app_4,(int)Angle_Balance);    

    USART_STR(USART2,Buf);
}

/**************************************************************************
�������ܣ�����ʾ��������λ���������� �ر���ʾ��
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void DataScope(void)
{   
//    Vol=(float)Voltage/100;
		DataScope_Get_Channel_Data( Angle_Balance, 1 );       //��ʾ�Ƕ� ��λ���ȣ��㣩
//		DataScope_Get_Channel_Data( Distance/10, 2 );         //��ʾ�����������ľ��� ��λ��CM 
//		DataScope_Get_Channel_Data( Vol, 3 );                 //��ʾ��ص�ѹ ��λ��V
//		DataScope_Get_Channel_Data( 0 , 4 );   
//		DataScope_Get_Channel_Data(0, 5 ); //����Ҫ��ʾ�������滻0������
//		DataScope_Get_Channel_Data(0 , 6 );//����Ҫ��ʾ�������滻0������
//		DataScope_Get_Channel_Data(0, 7 );
//		DataScope_Get_Channel_Data( 0, 8 ); 
//		DataScope_Get_Channel_Data(0, 9 );  
//		DataScope_Get_Channel_Data( 0 , 10);
		Send_Count = DataScope_Data_Generate(3);
		for( i = 0 ; i < Send_Count; i++) 
		{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[i]; 
		}
}

