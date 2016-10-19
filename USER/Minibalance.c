#include "sys.h"

#include "stdio.h"
#include "string.h"

  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
u8 Way_Angle=2;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 默认搭载卡尔曼滤波
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; //蓝牙遥控相关的变量
u8 Flag_Stop=0,Flag_Show=0;                 //停止标志位和 显示标志位 默认停止 显示打开
int Encoder_Left,Encoder_Right;             //左右编码器的脉冲计数
int Moto1,Moto2;                            //电机PWM变量 应是Motor的 向Moto致敬	
int Temperature;                            //显示温度
int Voltage;                                //电池电压采样相关的变量
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
float Show_Data_Mb;                     //全局显示变量，用于显示需要查看的数据
u32 g_distance = 0;                               //超声波测距

// usart2 	
u8 RxBuffer2[1000] = {0x00};	 	
u8 RxBuffer2_tmp[1000] = {0x00};
u16 RxCounter2 = 0;	// 接收计数，记录当前帧有效数据个数，不清零
u16 RxCounter2_frame = 0;	// 接收完一整帧之后的字符数量
u8 ReceiveState2 = 0;	
float g_att_kp = 300, g_att_kd = 0.6;
//float g_vel_kp = 0.6, g_vel_ki = 0.0045;
float g_vel_kp = 500, g_vel_ki = 0; // 三轮

//float g_vel_kp = 0.65, g_vel_ki = 0.0035;

u8 FLAG_stop_uart2_debug_msg = 0; // 接收调参信息时，1:禁止打印，除非正常收到数据

void get_tuned_parameter();
void init();

int main(void)
{ 
    init();
    while(1)
    {
        Temperature = Read_Temperature();  //===读取MPU6050内置温度传感器数据，近似表示主板温度。	
        if(Flag_Show==0)          //使用MiniBalanceV3.5 APP和OLED显示屏
        {
            APP_Show();	
            oled_show();          //===显示屏打开
        }
        else                      //使用MiniBalanceV3.5上位机 上位机使用的时候需要严格的时序，故此时关闭app监控部分和OLED显示屏
        {
            DataScope();          //开启MiniBalanceV3.5上位机
        }	       
//        get_tuned_parameter();
        delay_ms(10);	//延时减缓数据传输频率，确保通信的稳定

    } 
}

// 系统相关初始化
void init()
{
    Stm32_Clock_Init(9);            //系统时钟设置
    delay_init(72);                 //延时初始化
    JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
    JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试
    LED_Init();                     //初始化与 LED 连接的硬件接口
    KEY_Init();                     //按键初始化
    OLED_Init();                    //OLED初始化
    uart_init(72,115200);           //初始化串口1
    uart2_init(36,57600);            //串口2初始化  2016.10.02 origin: 被替换
//    USART2_Configuration(57600);    
    MiniBalance_PWM_Init(7199,0);   //=====初始化PWM 10KHZ，用于驱动电机 
    Encoder_Init_TIM2();            //=====编码器接口
    Encoder_Init_TIM3();            //初始化编码器2 
// -YJ- 10.01
//    Adc_Init();                     //=====adc初始化    
    IIC_Init();                     //模拟IIC初始化
    MPU6050_initialize();           //=====MPU6050初始化	
    DMP_Init();                     //初始化DMP     
    Timer1_Init(49,7199);           //=====5MS进一次中断服务函数; 7199: 10khz
//    Timer1_Init(99,7199);           //=====10MS进一次中断服务函数; 7199: 10khz
}


// 通过串口2获取调参数据
void get_tuned_parameter()
{    
    u8 i;
    u8 str_data[20] = {0x00};
    int16_t A_rec[2];
    int16_t V_rec[2];
    char Buf[60];
    // 纯在一个问题，可能是进入了接收到一帧数据的中断，但是数据是错的，引入打印收到参数的机制
    // 当进入中断，就停止打印消息，除非调参数据被正常收到
    if(ReceiveState2 == 1)//如果接收到1帧数据
    {
        if(RxBuffer2[0] == 'A')// 进入设置模式
		{
//		    scanf(&RxBuffer2[4], "%f", &tt);
            sscanf(&RxBuffer2[2], "%hd", &A_rec[0]);
            sscanf(&RxBuffer2[7], "%hd", &A_rec[1]);

            if(RxCounter2_frame == 10)
            {
                if(A_rec[0]>0 && A_rec[1]>=0)
                {
                    FLAG_stop_uart2_debug_msg = 0; // 接收调参信息时，1:禁止打印，除非正常收到数据
                    g_att_kp = A_rec[0]/10.0;
                    g_att_kd = A_rec[1]/10.0;

                    USART_STR(USART2,"A_rec: ");	
                    sprintf(Buf,"%d %d", (s16)A_rec[0], (s16)A_rec[1]);
                    USART_STR(USART2,Buf);
                    USART_STR(USART2,"\r\n");	
                }
            }

        }

        if(RxBuffer2[0] == 'V')// 进入设置模式
		{
//		    scanf(&RxBuffer2[4], "%f", &tt);
            sscanf(&RxBuffer2[2], "%hd", &V_rec[0]);
            sscanf(&RxBuffer2[4], "%hd", &V_rec[1]);
            if(RxCounter2_frame == 7)
            {
                if(V_rec[0]>0 && V_rec[1]>=0)
                {
                    FLAG_stop_uart2_debug_msg = 0; // 接收调参信息时，1:禁止打印，除非正常收到数据
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
