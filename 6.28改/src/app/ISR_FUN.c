#include "ISR_FUN.h"
#include "include.h"
int Diff_PID_ave = 0;

void Speed_Control(void)
{
  static unsigned char speed_Period=0;//速度控制周期变量
  static int L_ALL[Speed_filter_Period]={0},
              R_ALL[Speed_filter_Period]={0};//100ms内的速度值
//  static float speed_Delta=0;
//  static float Tmp_Speed_P =0; 
//  float SpeedRate = 0;
  //float SpeedRate;  
  //更新速度（前20ms的累加值）
  if(speed_Period >= (Speed_filter_Period))   
  speed_Period = 0;    
  Speed_L_sum-=L_ALL[speed_Period];
  Speed_R_sum-=R_ALL[speed_Period];
  L_ALL[speed_Period]=Motor1.Speed;
  R_ALL[speed_Period]=Motor2.Speed;
  Speed_L_sum+=L_ALL[speed_Period];
  Speed_R_sum+=R_ALL[speed_Period];
  speed_Period++;
}

void Speed_output(void)
{
  static u8 Diff_PID_period = 0;
  static int Diff_PID_old[Diff_error_Period_Constant] = {0};
  static float PWM_result = 0;
  //角速度差速环
  Diff_PID.feedback = Diff_error;
  
  if(Diff_PID_period > (Diff_error_Period_Constant - 1))
  Diff_PID_period = 0;
  
  Diff_PID_ave -= Diff_PID_old[Diff_PID_period];
  Diff_PID_old[Diff_PID_period] = Diff_PID.feedback;
  Diff_PID_ave += Diff_PID_old[Diff_PID_period];
  Diff_PID_period++;  
  
  Diff_PID.UP_Limit  =  Speed_stand;
  Diff_PID.LOW_Limit = -Speed_stand;
//  Gyro_PID.feedback  = mpu6050_Gyro_x_ave;
//  Gyro_PID.target    = Cur_error;
//  Gyro_PID.UP_Limit  = Motor1_PID.result;
//  Gyro_PID.LOW_Limit = -Motor1_PID.result;
  if(DIFF_PID_CHANGE_FLAG==1)
  {
    Diff_PID.P=stand_p*1.3;
    Diff_PID.D=stand_d*1.3;
  }
  else
  {
    Diff_PID.P=stand_p;
    Diff_PID.D=stand_d;
    Master_Speed=Speed_stand;//+Speed_max_to_min_diff;
  }

  Diff_PID_Process(&Diff_PID);
  //速度反馈环均值滤波
  if(TimeCnt_Start_Reduct_Flag==1)
  {
    Motor1_PID.feedback = Motor1.Speed*Speed_filter_Period;
    Motor2_PID.feedback = Motor2.Speed*Speed_filter_Period;
  }
  else 
  {
    Motor1_PID.feedback = Speed_L_sum;//Motor1.Speed;//+Motor2.Speed)/2;//(Speed_L_sum+Speed_R_sum)*1.0/(Speed_filter_Period*2);
    Motor2_PID.feedback = Speed_R_sum;//Motor2.Speed;
  }
  Speed_goal1=Master_Speed-Diff_PID.result;
  Speed_goal2=Master_Speed+Diff_PID.result;
  Motor1_PID.target = Speed_goal1;
  Motor2_PID.target = Speed_goal2;
  PID_process(&Motor1_PID);//速度PID
  PID_process(&Motor2_PID);
  
    Duty_Motor1 = Motor1_PID.result;// - Diff_PID.result;
    Duty_Motor2 = Motor2_PID.result;// + Diff_PID.result;
  if(Duty_Motor1>=65000)Duty_Motor1 = 65000;
  else if(Duty_Motor1<=-65000)Duty_Motor1 = -65000;
  if(Duty_Motor2>=65000)Duty_Motor2 = 65000;
  else if(Duty_Motor2<=-65000)Duty_Motor2 = -65000;
  if(Duty_Motor1<0)
  {
    MOTOR1_DIR = 1;//如果result<0，电机反向
    Duty_Motor1 = 0xffff+Duty_Motor1;
    FTM_PWM_Duty(MOTOR_1,Duty_Motor1);
  }
  else
  {
    MOTOR1_DIR = 0;
    Duty_Motor1 = Duty_Motor1;
    FTM_PWM_Duty(MOTOR_1,Duty_Motor1);
  }
  if(Duty_Motor2<0)
  {
    MOTOR2_DIR = 1;//如果result<0，电机反向
    Duty_Motor2 = 0xffff+Duty_Motor2;
    FTM_PWM_Duty(MOTOR_2,Duty_Motor2);
  }
  else
  {
    MOTOR2_DIR = 0;
    Duty_Motor2 = Duty_Motor2;
    FTM_PWM_Duty(MOTOR_2,Duty_Motor2);
  }
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