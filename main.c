

//ͷ�ļ�
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
u8 Flag_Stop=0,Flag_Show=0;                 //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
float a[3],w[3],angle[3],T;//����acceleration w=rad/s angle ����������ݣ��Լ�temperature   ����X����pitch  ����Y����roll ����Z����
extern unsigned char Re_buf[11],temp_buf[11],counter;//ʵ����stm32f10x_it.c���涨���
extern unsigned char sign;
int Encoder_Left,Encoder_Right;//���ұ��������������
int Moto1,Moto2;                            //���PWM���� Ӧ��Motor�� ��Moto�¾�	
float Angle_Balance,Gyro_Balance;           //ƽ����� ƽ�������� ת��������
unsigned char i=0;
unsigned char Temp[11];//����6050�õ�����
void Get_Angle();
u16 flag_switch=0;
int main(void)
{

		SysTick_Init(72);
	  LED_config();	
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //�ж����ȼ����� ��2��
		USART3_Init(115200);//MPU6050��MCU ͨ�� û����USART2��ԭ����OLEDԴ���õ���PA3orPA2�ܽ� ���԰�ͨ�Ż�����usart3
		USART1_Init(115200);//MCU����λ��PC ͨ��
		OLED_Init();
		MiniBalance_PWM_Init(7199,0);   //=====��ʼ��PWM 10KHZ������������� �����ʼ������ӿ� 
	  Encoder_Init_TIM2();            //=====�������ӿ�
    Encoder_Init_TIM4();            //=====��ʼ��������2
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
////		oled_show(); //===��ʾ����
////		if(i%500==0)
////		{
//////		LED1=~LED1;//LED1 PCout(13)��Դ�Աߵĵ�
//////		LED2=~LED2;//LED2 PAout(12)ָʾ��
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
////			printf("X�Ƕȣ�%.2f  Y�Ƕȣ�%.2f  Z�Ƕȣ�%.2f \r\n",angle[0],angle[1],angle[2]);
////			printf("X�ٶȣ�%.2f  Y�ٶȣ�%.2f  Z�ٶȣ�%.2f\r\n",w[0],w[1],w[2]);
////			printf("X���ٶȣ�%.2f  Y���ٶȣ�%.2f  Z���ٶȣ�%.2f\r\n",a[0],a[1], a[2]);
////			printf("Current temperature:%.2f ��C\r\n",T);
//			Get_Angle();//�õ��Ƕ�
//			Balance_Pwm =balance(angle[0],w[0]);                   //===ƽ��PID����	
//	  Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);                  //===�ٶȻ�PID����	 ��ס���ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��
// 	    Moto1=Balance_Pwm-Velocity_Pwm;                                    //===�������ֵ������PWM
// 	  	Moto2=Balance_Pwm-Velocity_Pwm;   
//// 	    Moto1=-Velocity_Pwm;                                    //===�������ֵ������PWM
//// 	  	Moto2=-Velocity_Pwm;   //===�������ֵ������PWM
////			if(Pick_Up(a[2],angle[0],Encoder_Left,Encoder_Right))//===����Ƿ�С��������
////			Flag_Stop=1;	                                                      //===���������͹رյ��
////			if(Put_Down(angle[0],Encoder_Left,Encoder_Right))              //===����Ƿ�С��������
////			Flag_Stop=0;	                                                      //===��������¾��������

//   		Xianfu_Pwm();                                                       //===PWM�޷�
//      if(Turn_Off(angle[0])==0)                                      //===����������쳣
// 			Set_Pwm(Moto1,Moto2);                                               //===��ֵ��PWM�Ĵ���  
//      delay_ms(5);
//         }
//         
//      } //if end
//			
//	printf("�����������: ");	printf("%d\r\n",Encoder_Left);
//	printf("�ұ���������: ");	printf("%d\r\n",Encoder_Right);
			
			// ֱ���� �� �ٶȻ�




//	  MiniBalance_PWM_Init(7199,0);   //=====��ʼ��PWM 10KHZ������������� �����ʼ������ӿ� 
//	  Encoder_Init_TIM2();            //=====�������ӿ�
//    Encoder_Init_TIM4();            //=====��ʼ��������2
//	  TIM3_Int_Init(499,7199);//72Mhz /500/7200=20hz ��Ч�� 1/20= 0.05s ��ʱ���ж�ȡ������ֵ��ʾ



//		if(i%100==0)
//		{
//		LED1=~LED1;//LED1 PCout(13)��Դ�Աߵĵ�
////		LED2=~LED2;//LED2 PAout(12)ָʾ��
//			i=0;
////			AIN1=~AIN1;
////			AIN2=~AIN2;
//		}
//		i++;

//		if(Key==0)            //ͨ����������PWMռ�ձȵı仯
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

//��ӡ������ʾ
//		printf("�����������: ");	printf("%d\r\n",Encoder_Left);
//		printf("�ұ���������: ");	printf("%d\r\n",Encoder_Right);
//		
//void Get_Angle()
//{
//	if(sign)
//      {  
//         memcpy(Temp,Re_buf,11);//������Re_buf[11] Copy ��Temp[]�����Ա��ں������ʱ��ֻ�ı�Temp[]�����ֵ �����ı���Re_buf[]��ԭֵ
//         if(Re_buf[0]==0x55)       //���֡ͷ
//         {  
//            switch(Re_buf[1])
//            {
//               case 0x51: //��ʶ������Ǽ��ٶȰ�
//                  a[0] = ((short)(Temp[3]<<8 | Temp[2]))/32768.0*16;      //X����ٶ�
//                  a[1] = ((short)(Temp[5]<<8 | Temp[4]))/32768.0*16;      //Y����ٶ�
//                  a[2] = ((short)(Temp[7]<<8 | Temp[6]))/32768.0*16;      //Z����ٶ�
//                  T    = ((short)(Temp[9]<<8 | Temp[8]))/340.0+36.25;      //�¶�
//                  break;
//               case 0x52: //��ʶ������ǽ��ٶȰ�
//                  w[0] = ((short)(Temp[3]<<8| Temp[2]))/32768.0*2000;      //X����ٶ�
//                  w[1] = ((short)(Temp[5]<<8| Temp[4]))/32768.0*2000;      //Y����ٶ�
//                  w[2] = ((short)(Temp[7]<<8| Temp[6]))/32768.0*2000;      //Z����ٶ�
//                  T    = ((short)(Temp[9]<<8| Temp[8]))/340.0+36.25;      //�¶�
//                  break;
//               case 0x53: //��ʶ������ǽǶȰ�
//                  angle[0] = ((short)(Temp[3]<<8| Temp[2]))/32768.0*180;   //X���ת�ǣ�x �ᣩ
//                  angle[1] = ((short)(Temp[5]<<8| Temp[4]))/32768.0*180;   //Y�ḩ���ǣ�y �ᣩ
//                  angle[2] = ((short)(Temp[7]<<8| Temp[6]))/32768.0*180;   //Z��ƫ���ǣ�z �ᣩ
//                  T        = ((short)(Temp[9]<<8| Temp[8]))/340.0+36.25;   //�¶�

//                  //printf("X��Ƕȣ�%.2f   Y��Ƕȣ�%.2f   Z��Ƕȣ�%.2f\r\n",angle[0],angle[1],angle[2]);
//                  break;
//               default:  break;
//            }
//    }
// }
//			printf("%.2f,%.2f,%.2f\n",angle[0],angle[1],angle[2]);
//			//			printf("                      Reference\r\n");
//			//printf("X�Ƕȣ�%.2f  Y�Ƕȣ�%.2f  Z�Ƕȣ�%.2f \r\n",angle[0],angle[1],angle[2]);
////			printf("X�ٶȣ�%.2f  Y�ٶȣ�%.2f  Z�ٶȣ�%.2f\r\n",w[0],w[1],w[2]);
////			printf("X���ٶȣ�%.2f  Y���ٶȣ�%.2f  Z���ٶȣ�%.2f\r\n",a[0],a[1], a[2]);
////			printf("Current temperature:%.2f ��C\r\n",T);
//			}