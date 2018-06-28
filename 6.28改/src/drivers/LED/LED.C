#include "led.h"

void myLED_Init(void)
{
  gpio_init(PORTD,5,GPO,HIGH);
  gpio_init(PORTD,3,GPO,HIGH);
}


void LED_Open_once(void)
{
  LED1=1;
  delayms(10);
  LED1=0;
}
