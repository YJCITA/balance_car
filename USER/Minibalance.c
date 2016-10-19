#include "sys.h"

#include "stdio.h"
#include "string.h"

  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
u8 Way_Angle=2;                             //��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲� Ĭ�ϴ��ؿ������˲�
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; //����ң����صı���
u8 Flag_Stop=0,Flag_Show=0;                 //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
int Encoder_Left,Encoder_Right;             //���ұ��������������
int Moto1,Moto2;                            //���PWM���� Ӧ��Motor�� ��Moto�¾�	
int Temperature;                            //��ʾ�¶�
int Voltage;                                //��ص�ѹ������صı���
float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ����� ƽ�������� ת��������
float Show_Data_Mb;                     //ȫ����ʾ������������ʾ��Ҫ�鿴������
u32 g_distance = 0;                               //���������

// usart2 	
u8 RxBuffer2[1000] = {0x00};	 	
u8 RxBuffer2_tmp[1000] = {0x00};
u16 RxCounter2 = 0;	// ���ռ�������¼��ǰ֡��Ч���ݸ�����������
u16 RxCounter2_frame = 0;	// ������һ��֮֡����ַ�����
u8 ReceiveState2 = 0;	
float g_att_kp = 300, g_att_kd = 0.6;
//float g_vel_kp = 0.6, g_vel_ki = 0.0045;
float g_vel_kp = 500, g_vel_ki = 0; // ����

//float g_vel_kp = 0.65, g_vel_ki = 0.0035;

u8 FLAG_stop_uart2_debug_msg = 0; // ���յ�����Ϣʱ��1:��ֹ��ӡ�����������յ�����

void get_tuned_parameter();
void init();

int main(void)
{ 
    init();
    while(1)
    {
        Temperature = Read_Temperature();  //===��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ�	
        if(Flag_Show==0)          //ʹ��MiniBalanceV3.5 APP��OLED��ʾ��
        {
            APP_Show();	
            oled_show();          //===��ʾ����
        }
        else                      //ʹ��MiniBalanceV3.5��λ�� ��λ��ʹ�õ�ʱ����Ҫ�ϸ��ʱ�򣬹ʴ�ʱ�ر�app��ز��ֺ�OLED��ʾ��
        {
            DataScope();          //����MiniBalanceV3.5��λ��
        }	       
//        get_tuned_parameter();
        delay_ms(10);	//��ʱ�������ݴ���Ƶ�ʣ�ȷ��ͨ�ŵ��ȶ�

    } 
}

// ϵͳ��س�ʼ��
void init()
{
    Stm32_Clock_Init(9);            //ϵͳʱ������
    delay_init(72);                 //��ʱ��ʼ��
    JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
    JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
    LED_Init();                     //��ʼ���� LED ���ӵ�Ӳ���ӿ�
    KEY_Init();                     //������ʼ��
    OLED_Init();                    //OLED��ʼ��
    uart_init(72,115200);           //��ʼ������1
    uart2_init(36,57600);            //����2��ʼ��  2016.10.02 origin: ���滻
//    USART2_Configuration(57600);    
    MiniBalance_PWM_Init(7199,0);   //=====��ʼ��PWM 10KHZ������������� 
    Encoder_Init_TIM2();            //=====�������ӿ�
    Encoder_Init_TIM3();            //��ʼ��������2 
// -YJ- 10.01
//    Adc_Init();                     //=====adc��ʼ��    
    IIC_Init();                     //ģ��IIC��ʼ��
    MPU6050_initialize();           //=====MPU6050��ʼ��	
    DMP_Init();                     //��ʼ��DMP     
    Timer1_Init(49,7199);           //=====5MS��һ���жϷ�����; 7199: 10khz
//    Timer1_Init(99,7199);           //=====10MS��һ���жϷ�����; 7199: 10khz
}


// ͨ������2��ȡ��������
void get_tuned_parameter()
{    
    u8 i;
    u8 str_data[20] = {0x00};
    int16_t A_rec[2];
    int16_t V_rec[2];
    char Buf[60];
    // ����һ�����⣬�����ǽ����˽��յ�һ֡���ݵ��жϣ����������Ǵ�ģ������ӡ�յ������Ļ���
    // �������жϣ���ֹͣ��ӡ��Ϣ�����ǵ������ݱ������յ�
    if(ReceiveState2 == 1)//������յ�1֡����
    {
        if(RxBuffer2[0] == 'A')// ��������ģʽ
		{
//		    scanf(&RxBuffer2[4], "%f", &tt);
            sscanf(&RxBuffer2[2], "%hd", &A_rec[0]);
            sscanf(&RxBuffer2[7], "%hd", &A_rec[1]);

            if(RxCounter2_frame == 10)
            {
                if(A_rec[0]>0 && A_rec[1]>=0)
                {
                    FLAG_stop_uart2_debug_msg = 0; // ���յ�����Ϣʱ��1:��ֹ��ӡ�����������յ�����
                    g_att_kp = A_rec[0]/10.0;
                    g_att_kd = A_rec[1]/10.0;

                    USART_STR(USART2,"A_rec: ");	
                    sprintf(Buf,"%d %d", (s16)A_rec[0], (s16)A_rec[1]);
                    USART_STR(USART2,Buf);
                    USART_STR(USART2,"\r\n");	
                }
            }

        }

        if(RxBuffer2[0] == 'V')// ��������ģʽ
		{
//		    scanf(&RxBuffer2[4], "%f", &tt);
            sscanf(&RxBuffer2[2], "%hd", &V_rec[0]);
            sscanf(&RxBuffer2[4], "%hd", &V_rec[1]);
            if(RxCounter2_frame == 7)
            {
                if(V_rec[0]>0 && V_rec[1]>=0)
                {
                    FLAG_stop_uart2_debug_msg = 0; // ���յ�����Ϣʱ��1:��ֹ��ӡ�����������յ�����
                    g_vel_kp = V_rec[0]/100.0;
                    g_vel_ki = V_rec[1]/1000.0;

                    I_vel_error = 0;

                    USART_STR(USART2,"V_rec: ");	
                    sprintf(Buf,"%d %d", (s16)V_rec[0], (s16)V_rec[1]);
                    USART_STR(USART2,Buf);
                    USART_STR(USART2,"\r\n");
                }
            }
        }
        ReceiveState2 = 0;
        RxCounter2_frame = 0;
        
    }

}
