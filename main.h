#ifndef __MAIN_H
#define __MAIN_H

#include "apm32f4xx.h"
#include "apm32f4xx_gpio.h"
#include "apm32f4xx_rcm.h"
#include "apm32f4xx_can.h"
#include "apm32f4xx_fmc.h"

void Delay(uint32_t time_value);
void Led_Init();
void CAN_Init();
void RCM_Init();
// variables
static int ErrorValue = 0;


//Test
static int StatusCAN = 0;
#endif /* __MAIN_H */