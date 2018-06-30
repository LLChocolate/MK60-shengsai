#include "ISR_FUN.h"
#include "include.h"
int Diff_PID_ave = 0;

void Speed_Control(void)
{
  static unsigned char speed_Period=0;//速度控制周期变量
  static int R_ALL[Speed_filter_Period]={0};//100ms内的速度值
//  static float speed_Delta=0;
//  static float Tmp_Speed_P =0; 
//  float SpeedRate = 0;
  //float SpeedRate;  
  //更新速度（前20ms的累加值）
  if(speed_Period >= (Speed_filter_Period))   
  speed_Period = 0;    
  Speed_R_sum-=R_ALL[speed_Period];
  R_ALL[speed_Period]=Motor2.Speed;
  Speed_R_sum+=R_ALL[speed_Period];
  speed_Period++;
}

void Speed_output(void)
{
  //角速度差速环

  //速度反馈环均值滤波
//  if(TimeCnt_Start_Reduct_Flag==1)
//  {
    Motor2_PID.feedback = Motor2.Speed;
//  }
//  else 
//  {
//    Motor2_PID.feedback = Speed_R_sum;//Motor2.Speed;
//  }
  Speed_goal2=Speed_stand;
  Motor2_PID.target = Speed_goal2;
  PID_process(&Motor2_PID);
  Duty_Motor2 = Motor2_PID.result;// + Diff_PID.result;

  if(Motor2_PID.result>=65500)Motor2_PID.result = 65500;
  else if(Motor2_PID.result<=50)Motor2_PID.result = 50;

  Duty_Motor2 = 0xffff - Motor2_PID.result;//pwm越大速度越慢
  FTM_PWM_Duty(MOTOR_2,Duty_Motor2);
}


void AD_new(void)
{
  static u8  AD_Period = 0;
  static u16 L_AD[speed_Period_Constant],
              R_AD[speed_Period_Constant];
  if(AD_Period > (speed_Period_Constant - 1))
    AD_Period = 0;
  
  L_AD_Ave -= L_AD[AD_Period];
  R_AD_Ave -= R_AD[AD_Period];
  L_AD[AD_Period] = ad_once(MYADC_2,ADC_12bit);
  R_AD[AD_Period] = ad_once(MYADC_3,ADC_12bit);
  L_AD_Ave += L_AD[AD_Period];
  R_AD_Ave += R_AD[AD_Period];
  AD_Period++;  
}

void Servo_Diff_PID(void)
{
  Servo_PID.feedback = Diff_error;
  if(Island.State==Right_Island_pre)
  {
    Servo_PID.P = stand_p*1.5;
    Servo_PID.D = stand_d*1.5;
  }
  else
  {
    Servo_PID.P = stand_p;
    Servo_PID.D = stand_d;
  }
  Diff_PID_Process(&Servo_PID);
  cmt_pwm_duty(Servo_PID.result+SERVO_MIDDLE);
}