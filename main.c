#include "main.h"
#include "system_apm32f4xx.h"

#define apm32f40x 1

#define I2C_Request_Write 0x12
#define I2C_Request_Read 0x13

// Config CAN
#define CANMode CANMode_DEBUG   // CANMode_Normal OR CANMode_DEBUG

#define CAN_IRQ_STATUS DISABLE_IRQ   // ENABLE_IRQ or DISABLE_IRQ
#define I2C_IRQ_STATUS DISABLE_IRQ

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
   RCM_Init();
   GPIO_Init();
   CAN_Init();  
   IRQ_Init();
   I2C_Setting();
   //Test_I2C_Init();
   //Delay(0x1fff);
}

/*
   
      Test I2C Start!!!!!

*/
void Test_I2C_Init(void)
{
   i2c_setup_gpio();
   
   I2C_Config_T I2C_Config_v;
   I2C_Config_v.mode = I2C_MODE_I2C;
   I2C_Config_v.dutyCycle = I2C_DUTYCYCLE_2;
   I2C_Config_v.ownAddress1 = 0;
   I2C_Config_v.ack = I2C_ACK_ENABLE;
   I2C_Config_v.clockSpeed = 100000;
   
   I2C_Config(I2C3, &I2C_Config_v);
   
   I2C_DisableGeneralCall(I2C3);
   I2C_DisableDualAddress(I2C3);
}

void Test_I2C_Read(uint8_t address, uint8_t registry, uint8_t * data)
{
   //I2C_EnableAcknowledge(I2C3);
   
   //I2C_DisableGenerateStop(I2C3);
   
   I2C_ClearErrorFlag();
   
   while(I2C_ReadStatusFlag(I2C3, I2C_FLAG_BUSBSY));
   
   I2C_EnableGenerateStart(I2C3);
   
   while(!I2C_ReadStatusFlag(I2C3, I2C_FLAG_START));
   
   //I2C_DisableGenerateStart(I2C3);
   
   (void) I2C3->STS1;
   
   I2C_TxData(I2C3, address << 1);
   
   while(!I2C_ReadStatusFlag(I2C3, I2C_FLAG_ADDR));
   
   (void) I2C3->STS1;
   (void) I2C3->STS2;
   
   //while(!I2C_ReadStatusFlag(I2C3, I2C_FLAG_TXBE));
   
   I2C_TxData(I2C3, registry);
   
   while(!I2C_ReadStatusFlag(I2C3, I2C_FLAG_TXBE));
   
   //I2C_EnableGenerateStop(I2C3);
   
   //while(I2C_ReadStatusFlag(I2C3, I2C_FLAG_BUSBSY));
   
   //I2C_DisableGenerateStop(I2C3);
   
   I2C_EnableGenerateStart(I2C3);
   
   while(!I2C_ReadStatusFlag(I2C3, I2C_FLAG_START));
   
   (void) I2C3->STS1;
   //I2C_DisableGenerateStart(I2C3);
   
   I2C_TxData(I2C3, (address << 1) | 1);
   
   while(!I2C_ReadStatusFlag(I2C3, I2C_FLAG_ADDR));
   
   (void) I2C3->STS1;
   (void) I2C3->STS2;
   
   I2C3->CTRL1_B.ACKEN = 0x0;
   
   I2C_EnableGenerateStop(I2C3);
   
   (void) I2C3->STS2;
   
   while(!I2C_ReadStatusFlag(I2C3, I2C_FLAG_RXBNE));
   
   *data = I2C_RxData(I2C3);
   
   //while(I2C_ReadStatusFlag(I2C3, I2C_FLAG_AE));
   
   //I2C_EnableGenerateStop(I2C3);
   //I2C_Tx7BitAddress();
   
   I2C_ClearErrorFlag();
}

/*
   
      Test I2C Stop!!!!!

*/

void I2C_Setting()
{
   rcc_I2C_init();
   i2c_setup_gpio();
      
   I2C_Reset(I2C3);
   
   I2C_Config_T I2C_Config_v;
   I2C_Config_v.mode = I2C_MODE_I2C;
   I2C_Config_v.dutyCycle = I2C_DUTYCYCLE_2;
   I2C_Config_v.ownAddress1 = 0;
   I2C_Config_v.ack = I2C_ACK_ENABLE;
   I2C_Config_v.ackAddress = I2C_ACK_ADDRESS_7BIT;
   I2C_Config_v.clockSpeed = 100000;
   
   I2C_Config(I2C3, &I2C_Config_v);
   
   I2C_Enable(I2C3);
   
   #if I2C_IRQ_STATUS
   NVIC_EnableIRQ(I2C3_EV_IRQn);
   NVIC_EnableIRQ(I2C3_ER_IRQn);
  // CAN_EnableInterrupt(CAN1, CAN_INT_TXME);
   I2C_EnableInterrupt(I2C3, I2C_INT_BUF);
   I2C_EnableInterrupt(I2C3, I2C_INT_ERR);
   #endif
   I2C_ClearErrorFlag();
   //i2c_init();
   //I2C_EnablePEC(I2C3);
}

void I2C3_Reset(void)
{
   I2C3->CTRL1_B.SWRST = 1;
   Delay(0xff);
   I2C3->CTRL1_B.SWRST = 0;
   
   I2C_Config_T I2C_Config_v;
   I2C_Config_v.mode = I2C_MODE_I2C;
   I2C_Config_v.dutyCycle = I2C_DUTYCYCLE_2;
   I2C_Config_v.ownAddress1 = 0;
   I2C_Config_v.ack = I2C_ACK_ENABLE;
   I2C_Config_v.clockSpeed = 100000;
   
   I2C_Config(I2C3, &I2C_Config_v);
}

uint8_t dataAcc[2] = {0};


int main()
{
   //RCM->PLL1CFG = 0x2400301T0;
   main_Init();
  
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
   //i2c_write(ADDRESS_ACCEL, SFE_QMA6100P_PM, settingQMA);
   
   while(1)
   {
      if(1)//!ErrorValue)
      {
         //I2C_ClearErrorFlag();
         //Delay(0xffffff);
         GPIO_ToggleBit(GPIOD, GPIO_PIN_12);
         //Delay(0xffffff);
         if(1)//count > 5)
         {
            //LockI2C = 1;
            //uint8_t data = (sfe_qma6100p_pm.mode_bit << 7) | sfe_qma6100p_pm.mclk_sel;
            
            
            //uint8_t dataTest;
            
            //Test_I2C_Read(ADDRESS_ACCEL, 0x13, &dataTest);
            //i2c_write(ADDRESS_ACCEL, 0x13, settingQMA);
            Delay(0xffffff);
            dataI2C = i2c_read(ADDRESS_ACCEL, 0x00);
            Delay(0xffffff);
            //dataI2C = i2c_read(ADDRESS_ACCEL, SFE_QMA6100P_PM);
            count = 0;
         }
            
         //i2c_read_many(ADDRESS_ACCEL, 0x01, &dataAcc, 2);
         //Delay(0xffffff);
         count++;
      }
      
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
   I2C3_Reset();
   ErrorI2C = 1;
   //I2C3->STS1_B.BERRFLG = 0;
   GPIO_ToggleBit(GPIOD, GPIO_PIN_13);
}