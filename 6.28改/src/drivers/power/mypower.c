#include "mypower.h"

u8 myPower_Init(void)
{
  adc_init(MYADC_3);
}

u8 myPower_Test(void)
{
  if(ad_mid(MYADC_3,ADC_12bit)>Run_.Run_Volts*2/7.6)
    return 1;
  else 
    return 0;
}
