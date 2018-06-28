#include "myPWM.h"
void Motot1_Init(void)
{
  gpio_init(PORTA,13,GPO,HIGH);//DIR
  FTM_PWM_init(MOTOR_1,10000,0);//设置频率为10k
}
void Motot2_Init(void)
{
  gpio_init(PORTA,12,GPO,HIGH);//DIR
  FTM_PWM_init(MOTOR_2,10000,0);//设置频率为10k
}

//void Steer_Init(void)
//{
//  FTM_PWM_init(STEER_,50,(u16)(1.5/20*65536));//初始0.5ms高电平旋转0度
// // Time_Steer=1.5;
//}

void PWM_DISENABLE(FTMn ftmn, CHn ch)
{
    FTM_PWM_Duty(ftmn,ch,0);//关闭PWM
}

