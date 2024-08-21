#ifndef __MAIN_H
#define __MAIN_H

#include "apm32f4xx.h"
#include "apm32f4xx_gpio.h"
#include "apm32f4xx_rcm.h"
#include "apm32f4xx_can.h"
#include "apm32f4xx_fmc.h"
#include "apm32f4xx_usart.h"
#include "apm32f4xx_eint.h"
#include "apm32f4xx_misc.h"
#include "apm32f4xx_i2c.h"
#include "i2c.h"


#define CAN1_TX_IRQn 19
#define CAN1_RX0_IRQn 20

#define CANMode_DEBUG CAN_MODE_SILENT_LOOPBACK
#define CANMode_Normal CAN_MODE_NORMAL

#define ENABLE_IRQ 1
#define DISABLE_IRQ 0

void Delay(uint32_t time_value);
void GPIO_Init(void);
void CAN_Init(void);
void RCM_Init(void);
void IRQ_Init(void);
void I2C_Setting(void);
// variables
static int ErrorValue = 0;


//Test
static int StatusCAN = 0;
static int StatucCANMessage = 0;
static int StatucCANMode = 0;
#endif /* __MAIN_H */