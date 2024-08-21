#include "main.h"
#include "system_apm32f4xx.h"

#define apm32f40x 1

#define I2C_Request_Write 0x12
#define I2C_Request_Read 0x13

// Config CAN
#define CANMode CANMode_DEBUG   // CANMode_Normal OR CANMode_DEBUG
#define CAN_IRQ_STATUS DISABLE_IRQ   // ENABLE_IRQ or DISABLE_IRQ

#define ADDRESS_ACCEL 0x12

void GPIO_Init()
{
   GPIO_Config_T configStruct;
   
   RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOD);
   
   configStruct.pin = GPIO_PIN_13 | GPIO_PIN_12;
   configStruct.mode = GPIO_MODE_OUT;
   configStruct.otype = GPIO_OTYPE_PP;
   configStruct.speed = GPIO_SPEED_50MHz;
   GPIO_Config(GPIOD, &configStruct);
   GPIOD->BSCH = GPIO_PIN_13 | GPIO_PIN_12;
}

void CAN_Init()
{
   RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOC);
   RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOB);
   
   GPIO_ConfigPinAF(GPIOB, GPIO_PIN_SOURCE_8, GPIO_AF_CAN1);
   GPIO_ConfigPinAF(GPIOB, GPIO_PIN_SOURCE_9, GPIO_AF_CAN1);
   
   GPIO_Config_T GPIOConfig;
   
   GPIOConfig.pin = GPIO_PIN_8; // CAN RX
   GPIOConfig.speed = GPIO_SPEED_50MHz;
   GPIOConfig.mode = GPIO_MODE_IN;
   GPIOConfig.pupd = GPIO_PUPD_UP;
   GPIO_Config(GPIOB, &GPIOConfig);
   
   GPIOConfig.pin = GPIO_PIN_9;  // CAN TX
   GPIOConfig.speed = GPIO_SPEED_50MHz;
   GPIOConfig.mode = GPIO_MODE_AF;
   GPIOConfig.otype = GPIO_OTYPE_PP;
   GPIO_Config(GPIOB, &GPIOConfig);
   
   GPIOConfig.pin = GPIO_PIN_2; //  CAN STB
   GPIOConfig.speed = GPIO_SPEED_50MHz;
   GPIOConfig.mode = GPIO_MODE_OUT;
   GPIO_Config(GPIOC, &GPIOConfig);
   GPIO_ResetBit(GPIOC, GPIO_PIN_2);
   
   RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_CAN1);
   Delay(0xfff);
   CAN_DisableDBGFreeze(CAN1);
   
   CAN_Config_T configCAN;
   
   configCAN.mode = CANMode; 
   configCAN.syncJumpWidth = CAN_SJW_1;
   configCAN.prescaler = 21;
   configCAN.timeSegment1 = CAN_TIME_SEGMENT1_6;
   configCAN.timeSegment2 = CAN_TIME_SEGMENT2_1;
   configCAN.autoBusOffManage = DISABLE;
   configCAN.autoWakeUpMode = DISABLE;
   configCAN.nonAutoRetran = DISABLE;
   configCAN.rxFIFOLockMode = DISABLE;
   configCAN.txFIFOPriority = DISABLE;
   
   CAN_FilterConfig_T CAN_Filter;
   CAN_Filter.filterNumber = 0;
   CAN_Filter.filterMode = CAN_FILTER_MODE_IDMASK;
   CAN_Filter.filterScale = CAN_FILTER_SCALE_32BIT;
   CAN_Filter.filterIdHigh = 0x0000;
   CAN_Filter.filterIdLow = 0x0000;
   CAN_Filter.filterMaskIdHigh = 0x0000;
   CAN_Filter.filterMaskIdLow = 0x0000;
   CAN_Filter.filterFIFO = CAN_FILTER_FIFO_0;
   CAN_Filter.filterActivation = ENABLE;
   CAN_ConfigFilter(CAN1, &CAN_Filter);
   
   
   CAN_TxMessage_T TxMessage;
   TxMessage.stdID = 0x13;
   TxMessage.extID = 0x00;
   TxMessage.typeID = CAN_TYPEID_STD;
   TxMessage.remoteTxReq = CAN_RTXR_DATA;
   TxMessage.dataLengthCode = 0x8;
   TxMessage.data[0] = 0xf1;
   //StatucCANMessage = CAN_TxMessage(CAN1, &TxMessage);
   
   StatusCAN = CAN_Config(CAN1, &configCAN);
   StatucCANMode = CAN_OperatingMode(CAN1, CAN_OPERATING_MODE_NORMAL);
}

void CAN1_TX_IRQHandler() // 
{
   //GPIO_ToggleBit(GPIOD, GPIO_PIN_13);
   Delay(0xffffff);
}

void CAN1_RX0_IRQHandler() // Прерывание по приходу сообщения CAN
{
   CAN_RxMessage_T MessageCAN;
   CAN_RxMessage(CAN1, CAN_RX_FIFO_0, &MessageCAN);
   CAN_ReleaseFIFO(CAN1, CAN_RX_FIFO_0);
   GPIO_ToggleBit(GPIOD, GPIO_PIN_13);
   Delay(0xffffff);
}

void CAN1_RX1_IRQHandler()
{
   GPIO_ToggleBit(GPIOD, GPIO_PIN_13);
   Delay(0xfffff);
}

void Delay(uint32_t time_value)
{
   while(time_value--);
}

void RCM_Init()
{
   RCM_ConfigHSE(RCM_HSE_OPEN);
   while(RCM_WaitHSEReady() != SUCCESS) {;};
   
   RCM_ConfigPLL1(RCM_PLLSEL_HSE, 8, 168, RCM_PLL_SYS_DIV_2, 4); 
   RCM_ConfigPLL2(384, 2);
   RCM_EnablePLL1();
   RCM_EnablePLL2();
   RCM_ConfigSYSCLK(RCM_SYSCLK_SEL_PLL);
   RCM->CFG_B.I2SSEL = 0x0;
   while(RCM->CFG_B.SCLKSWSTS != 0x2) // Ждем запуска PLL
   {}
   RCM_DisableHSI();
   RCM_ConfigAHB(RCM_AHB_DIV_1);
   RCM_ConfigAPB1(RCM_APB_DIV_2);   
   RCM_ConfigAPB2(RCM_APB_DIV_1); 
      
   RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
   SystemCoreClockUpdate();
}

int main_Init()
{
   RCM_Init();
   GPIO_Init();
   CAN_Init();  
   IRQ_Init();
   I2C_Setting();
   Delay(0x1fff);
   
}

void I2C_Setting()
{
   rcc_I2C_init();
   i2c_setup_gpio();
   i2c_init();
}

int main()
{
   //RCM->PLL1CFG = 0x2400301T0;
   main_Init();
  
   
   // Test I2C
   i2c_write(ADDRESS_ACCEL, 0x00, 0x22);
   
   Delay(0xffffff);
   
   uint8_t data = i2c_read(ADDRESS_ACCEL, 0x00);
   
   
   while(1)
   {
      if(1)//!ErrorValue)
      {
         Delay(0xffffff);
         GPIO_ToggleBit(GPIOD, GPIO_PIN_12);
         
         Delay(0xffffff);
      }
      
   }
}

void IRQ_Init()
{
   __enable_irq ();

   #if CAN_IRQ_STATUS
   //NVIC_EnableIRQ(CAN1_TX_IRQn);
   NVIC_EnableIRQ(CAN1_RX0_IRQn);
  // CAN_EnableInterrupt(CAN1, CAN_INT_TXME);
   CAN_EnableInterrupt(CAN1, CAN_INT_F0MP);
   CAN_EnableInterrupt(CAN1, CAN_INT_WUP);
   //NVIC_EnableIRQRequest(19, 0, 0);
   #endif
}


