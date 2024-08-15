#ifndef __DEBUG_LED_H
#define __DEBUG_LED_H

#include "apm32f4xx.h"
#include "apm32f4xx_gpio.h"
#include "apm32f4xx_rcm.h"
#include "apm32f4xx_can.h"

void Delay(uint32_t time_value);
void Led_Init();

#endif /* __DEBUG_LED_H */