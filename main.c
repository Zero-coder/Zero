

//头文件
#include "stm32f10x.h"
#include "main.h"
#include "GPIOLIKE51.h"
#include "Miscellaneous.h"
#include "SysTick.h"
#include "usart.h"
#include "string.h"
#include "encoder.h"
#include "timer.h"
#include "motor.h"
#include "control.h"
u8 Flag_Stop=0,Flag_Show=0;                 //停止标志位和 显示标志位 默认停止 显示打开
float a[3],w[3],angle[3],T;//对于acceleration w=rad/s angle 在三轴的数据，以及temperature   绕着X轴是pitch  绕着Y轴是roll 绕着Z轴是
extern unsigned char Re_buf[11],temp_buf[11],counter;//实际在stm32f10x_it.c里面定义的
extern unsigned char sign;
int Encoder_Left,Encoder_Right;//左右编码器的脉冲计数
int Moto1,Moto2;                            //电机PWM变量 应是Motor的 向Moto致敬	
float Angle_Balance,Gyro_Balance;           //平衡倾角 平衡陀螺仪 转向陀螺仪
unsigned char i=0;
unsigned char Temp[11];//接收6050用的数组
void Get_Angle();
u16 flag_switch=0;
int main(void)
{

		SysTick_Init(72);
	  LED_config();	
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组 分2组
		USART3_Init(115200);//MPU6050与MCU 通信 没有用USART2的原因是OLED源码用到了PA3orPA2管脚 所以把通信换成了usart3
		USART1_Init(115200);//MCU与上位机PC 通信
		OLED_Init();
		MiniBalance_PWM_Init(7199,0);   //=====初始化PWM 10KHZ，用于驱动电机 如需初始化电调接口 
	  Encoder_Init_TIM2();            //=====编码器接口
    Encoder_Init_TIM4();            //=====初始化编码器2
	 TIM3_Int_Init(499,7199);
		delay_ms(200);
//	AIN1=1;AIN2=0;
//	BIN1=1;BIN2=0;

while(1)
{
		LED1=0;
	
	if(flag_switch%5==0)
	{
		flag_switch=0;
	LED1=1;
	delay_ms(75);
	}
	flag_switch++;
	delay_ms(15);
	

	
}
}
		
//// AIN1=BIN1=1;
//// AIN2=BIN2=0;
////	PWMA=PWMB=6900;
////		oled_show(); //===显示屏打开
////		if(i%500==0)
////		{
//////		LED1=~LED1;//LED1 PCout(13)电源旁边的灯
//////		LED2=~LED2;//LED2 PAout(12)指示灯
////			i=0;
////			AIN1=~AIN1;
////			AIN2=~AIN2;
////			BIN1=~BIN1;
////			BIN2=~BIN2;
////		}
////		i++;
////PWMA=7100;
////PWMB=7100;		
//		
//		 
////			printf("                      Reference\r\n");
////			printf("X角度：%.2f  Y角度：%.2f  Z角度：%.2f \r\n",angle[0],angle[1],angle[2]);
////			printf("X速度：%.2f  Y速度：%.2f  Z速度：%.2f\r\n",w[0],w[1],w[2]);
////			printf("X加速度：%.2f  Y加速度：%.2f  Z加速度：%.2f\r\n",a[0],a[1], a[2]);
////			printf("Current temperature:%.2f °C\r\n",T);
//			Get_Angle();//得到角度
//			Balance_Pwm =balance(angle[0],w[0]);                   //===平衡PID控制	
//	  Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);                  //===速度环PID控制	 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
// 	    Moto1=Balance_Pwm-Velocity_Pwm;                                    //===计算左轮电机最终PWM
// 	  	Moto2=Balance_Pwm-Velocity_Pwm;   
//// 	    Moto1=-Velocity_Pwm;                                    //===计算左轮电机最终PWM
//// 	  	Moto2=-Velocity_Pwm;   //===计算右轮电机最终PWM
////			if(Pick_Up(a[2],angle[0],Encoder_Left,Encoder_Right))//===检查是否小车被那起
////			Flag_Stop=1;	                                                      //===如果被拿起就关闭电机
////			if(Put_Down(angle[0],Encoder_Left,Encoder_Right))              //===检查是否小车被放下
////			Flag_Stop=0;	                                                      //===如果被放下就启动电机

//   		Xianfu_Pwm();                                                       //===PWM限幅
//      if(Turn_Off(angle[0])==0)                                      //===如果不存在异常
// 			Set_Pwm(Moto1,Moto2);                                               //===赋值给PWM寄存器  
//      delay_ms(5);
//         }
//         
//      } //if end
//			
//	printf("左编码器计数: ");	printf("%d\r\n",Encoder_Left);
//	printf("右编码器计数: ");	printf("%d\r\n",Encoder_Right);
			
			// 直立环 与 速度环




//	  MiniBalance_PWM_Init(7199,0);   //=====初始化PWM 10KHZ，用于驱动电机 如需初始化电调接口 
//	  Encoder_Init_TIM2();            //=====编码器接口
//    Encoder_Init_TIM4();            //=====初始化编码器2
//	  TIM3_Int_Init(499,7199);//72Mhz /500/7200=20hz 等效于 1/20= 0.05s 定时器中断取出编码值显示



//		if(i%100==0)
//		{
//		LED1=~LED1;//LED1 PCout(13)电源旁边的灯
////		LED2=~LED2;//LED2 PAout(12)指示灯
//			i=0;
////			AIN1=~AIN1;
////			AIN2=~AIN2;
//		}
//		i++;

//		if(Key==0)            //通过按键进行PWM占空比的变化
//		{
//			LED2=~LED2;
//			PWMA+=1000;
//			PWMB+=1000;
//			if(PWMA>=7100)
//			{PWMA=7100;}
//			if(PWMB>=7100)
//			{PWMB=7100;}
//		}



//		 delay_ms(1500);

//打印串口显示
//		printf("左编码器计数: ");	printf("%d\r\n",Encoder_Left);
//		printf("右编码器计数: ");	printf("%d\r\n",Encoder_Right);
//		
//void Get_Angle()
//{
//	if(sign)
//      {  
//         memcpy(Temp,Re_buf,11);//将数组Re_buf[11] Copy 给Temp[]数组以便于后面计算时候只改变Temp[]数组的值 而不改变组Re_buf[]的原值
//         if(Re_buf[0]==0x55)       //检查帧头
//         {  
//            switch(Re_buf[1])
//            {
//               case 0x51: //标识这个包是加速度包
//                  a[0] = ((short)(Temp[3]<<8 | Temp[2]))/32768.0*16;      //X轴加速度
//                  a[1] = ((short)(Temp[5]<<8 | Temp[4]))/32768.0*16;      //Y轴加速度
//                  a[2] = ((short)(Temp[7]<<8 | Temp[6]))/32768.0*16;      //Z轴加速度
//                  T    = ((short)(Temp[9]<<8 | Temp[8]))/340.0+36.25;      //温度
//                  break;
//               case 0x52: //标识这个包是角速度包
//                  w[0] = ((short)(Temp[3]<<8| Temp[2]))/32768.0*2000;      //X轴角速度
//                  w[1] = ((short)(Temp[5]<<8| Temp[4]))/32768.0*2000;      //Y轴角速度
//                  w[2] = ((short)(Temp[7]<<8| Temp[6]))/32768.0*2000;      //Z轴角速度
//                  T    = ((short)(Temp[9]<<8| Temp[8]))/340.0+36.25;      //温度
//                  break;
//               case 0x53: //标识这个包是角度包
//                  angle[0] = ((short)(Temp[3]<<8| Temp[2]))/32768.0*180;   //X轴滚转角（x 轴）
//                  angle[1] = ((short)(Temp[5]<<8| Temp[4]))/32768.0*180;   //Y轴俯仰角（y 轴）
//                  angle[2] = ((short)(Temp[7]<<8| Temp[6]))/32768.0*180;   //Z轴偏航角（z 轴）
//                  T        = ((short)(Temp[9]<<8| Temp[8]))/340.0+36.25;   //温度

//                  //printf("X轴角度：%.2f   Y轴角度：%.2f   Z轴角度：%.2f\r\n",angle[0],angle[1],angle[2]);
//                  break;
//               default:  break;
//            }
//    }
// }
//			printf("%.2f,%.2f,%.2f\n",angle[0],angle[1],angle[2]);
//			//			printf("                      Reference\r\n");
//			//printf("X角度：%.2f  Y角度：%.2f  Z角度：%.2f \r\n",angle[0],angle[1],angle[2]);
////			printf("X速度：%.2f  Y速度：%.2f  Z速度：%.2f\r\n",w[0],w[1],w[2]);
////			printf("X加速度：%.2f  Y加速度：%.2f  Z加速度：%.2f\r\n",a[0],a[1], a[2]);
////			printf("Current temperature:%.2f °C\r\n",T);
//			}