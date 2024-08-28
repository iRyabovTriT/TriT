#include "main.h"
#include "system_apm32f4xx.h"
#include "i2c_accel.h"
#include "DMA.h"

#define apm32f40x 1

#define I2C_Request_Write 0x12
#define I2C_Request_Read 0x13

// Config CAN
#define CANMode CANMode_DEBUG   // CANMode_Normal OR CANMode_DEBUG

#define CAN_IRQ_STATUS DISABLE_IRQ   // ENABLE_IRQ or DISABLE_IRQ
#define I2C_IRQ_STATUS DISABLE_IRQ

#define ADDRESS_ACCEL 0x12



void GPIO_Init(void)
{
   GPIO_Config_T configStruct;
   
   RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOD);
   
   configStruct.pin = GPIO_PIN_13 | GPIO_PIN_12;
   configStruct.mode = GPIO_MODE_OUT;
   configStruct.otype = GPIO_OTYPE_PP;
   configStruct.speed = GPIO_SPEED_50MHz;
   GPIO_Config(GPIOD, &configStruct);
   GPIOD->BSCH = GPIO_PIN_13 | GPIO_PIN_12; 
   
   //QMA6100 Power
   configStruct.pin = GPIO_PIN_15;
   configStruct.mode = GPIO_MODE_OUT;
   configStruct.otype = GPIO_OTYPE_PP;
   configStruct.speed = GPIO_SPEED_50MHz;
   GPIO_Config(GPIOD, &configStruct);
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
   GPIOConfig.pupd = GPIO_PUPD_UP;
   GPIO_Config(GPIOB, &GPIOConfig);
   
   GPIOConfig.pin = GPIO_PIN_2; //  CAN STB
   GPIOConfig.speed = GPIO_SPEED_50MHz;
   GPIOConfig.mode = GPIO_MODE_OUT;
   GPIO_Config(GPIOC, &GPIOConfig);
   GPIO_ResetBit(GPIOC, GPIO_PIN_2);
   
   RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_CAN1);
   Delay(0xfffff);
   
   CAN_Reset(CAN1);
   
   CAN_DisableDBGFreeze(CAN1);
   
   CAN_Config_T configCAN;
   
   CAN_ConfigStructInit(&configCAN);
   
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
   
   //StatusCAN = CAN_Config(CAN1, &configCAN);
   
   CAN1->MCTRL_B.INITREQ = BIT_RESET;
   
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
   StatusCANMessage = CAN_TxMessage(CAN1, &TxMessage);
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
   CAN_Init();  
   IRQ_Init();
}

void ACL_I2C_DeInit(void)
{
  ACL_I2C_LowLevel_DeInit();
}


void ACL_I2C_Init(void)
{
   ACL_I2C_LowLevel_Init();
   
   I2C_Config_T i2c_config_v;
   
   I2C_Reset(I2C3);
   
   i2c_config_v.mode = I2C_MODE_I2C;
   i2c_config_v.dutyCycle = I2C_DUTYCYCLE_2;
   i2c_config_v.clockSpeed = 100000;
   i2c_config_v.ack = I2C_ACK_ENABLE;
   i2c_config_v.ackAddress = I2C_ACK_ADDRESS_7BIT;
   i2c_config_v.ownAddress1 = 0x0;
   
   I2C_Config(I2C3, &i2c_config_v);
   
   I2C_Enable(I2C3);
   
   I2C_EnableDMA(I2C3);
}

void pause(uint32_t time)
{
  uint32_t i;
    for (i=0; i<time; i++); //reset_wdt;
}

int main()
{
   RCM_Init();
   RCM_ClearStatusFlag();
   
   pause(4000);
   
   GPIO_Init();
      
   ACL_I2C_Init();
   
   main_Init();
   
   pause(3000);
   
   PowerChangeQMA6100(ENABLE_QMA6100);
   
   I2C_ClearErrorFlag();
   
   Delay(0xffffff);
   // Test I2C
   
   static uint8_t dataI2C = 0;
   static uint8_t LockI2C = 0;
   
   //dataI2C = i2c_read(ADDRESS_ACCEL, 0x00);
   
   //Delay(0xffffff);
   
   uint32_t HCLKFreq = RCM_ReadHCLKFreq();
   uint32_t SYSCLKFreq = RCM_ReadSYSCLKFreq();
   
   sfe_qma6100p_pm_t sfe_qma6100p_pm;
   sfe_qma6100p_pm.mode_bit = 1;
   sfe_qma6100p_pm.mclk_sel = 5;
   
   uint8_t settingQMA = (sfe_qma6100p_pm.mode_bit << 7) | sfe_qma6100p_pm.mclk_sel;
   //uint8_t data = i2c_read(ADDRESS_ACCEL, 0x00);
   //i2c_write(ADDRESS_ACCEL, 0x11, 1 << 7);
   Delay(0xffffff);
   uint8_t count = 0;
   
   while(1)
   {
      Delay(0xffffff);
      GPIO_ToggleBit(GPIOD, GPIO_PIN_12);
      ACL_BufferTX[0] = 0x85;
      uint8_t res = ACL_I2C_WriteReg(0x11, 1);
      Delay(0xffffff);
   }
}



void IRQ_Init()
{
   __enable_irq ();

   #if CAN_IRQ_STATUS
   NVIC_EnableIRQ(CAN1_TX_IRQn);
   NVIC_EnableIRQ(CAN1_RX0_IRQn);
  // CAN_EnableInterrupt(CAN1, CAN_INT_TXME);
   CAN_EnableInterrupt(CAN1, CAN_INT_F0MP);
   CAN_EnableInterrupt(CAN1, CAN_INT_WUP);
   //NVIC_EnableIRQRequest(19, 0, 0);
   #endif
}

void I2C3_EV_IRQHandler(void)
{
   GPIO_ToggleBit(GPIOD, GPIO_PIN_13);
}

void I2C3_ER_IRQHandler()
{
   //I2C3_Reset();
   ErrorI2C = 1;
   //I2C3->STS1_B.BERRFLG = 0;
   GPIO_ToggleBit(GPIOD, GPIO_PIN_13);
}