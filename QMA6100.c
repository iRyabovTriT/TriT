#include "QMA6100.h"
#include "apm32f4xx_gpio.h"
#include "apm32f4xx.h"

void PowerChangeQMA6100(Status_power_t value)
{
   switch(value)
   {
      case DISABLE_QMA6100 :
         PowerOff();
         break;
      case ENABLE_QMA6100 :
         PowerOn();
         break;
   }
}

void PowerOn(void)
{
   GPIO_SetBit(GPIOD, GPIO_PIN_15);
}

void PowerOff(void)
{
   GPIO_ResetBit(GPIOD, GPIO_PIN_15);
}
