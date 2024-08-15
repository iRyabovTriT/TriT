#include "main.h"

void Led_Init()
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
   CAN_Config_T configCAN;
   
   // Натсройка GPIO для CAN
   RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOA);
   
   GPIO_Config_T configStruct;
   
   configStruct.pin |= GPIO_PIN_11;
   configStruct.mode = GPIO_MODE_IN;
   configStruct.otype = GPIO_OTYPE_PP;
   configStruct.speed = GPIO_SPEED_50MHz;
   GPIO_Config(GPIOA, &configStruct);
   GPIOD->BSCH = GPIO_PIN_11;
   GPIO_ConfigPinAF(GPIOA, GPIO_PIN_SOURCE_11, GPIO_AF_CAN1);
   
   configStruct.pin |= GPIO_PIN_12;
   configStruct.mode = GPIO_MODE_OUT;
   configStruct.otype = GPIO_OTYPE_PP;
   configStruct.speed = GPIO_SPEED_50MHz;
   GPIO_Config(GPIOA, &configStruct);
   GPIOD->BSCH = GPIO_PIN_12;
   GPIO_ConfigPinAF(GPIOA, GPIO_PIN_SOURCE_12, GPIO_AF_CAN1);
   
   //CAN Config
   RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_CAN1);
   
   /*
   CAN1->MCTRL_B.SLEEPREQ = BIT_RESET;
   CAN1->MCTRL_B.INITREQ = 0x1;
   Delay(0xffffff);
   CAN1->MCTRL_B.INITREQ = 0x0;
   Delay(0xffffff);
   */
   configCAN.txFIFOPriority = DISABLE;
   configCAN.rxFIFOLockMode = DISABLE;
   configCAN.nonAutoRetran = DISABLE;
   configCAN.autoWakeUpMode = DISABLE;
   configCAN.autoBusOffManage = DISABLE;
   
   configCAN.mode = CAN_MODE_NORMAL; 
   configCAN.prescaler = 21;
   configCAN.timeSegment1 = CAN_TIME_SEGMENT1_6;
   configCAN.timeSegment2 = CAN_TIME_SEGMENT2_1;
   
   
   CAN_DisableDBGFreeze(CAN1);
   
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
   //CAN_SlaveStartBank(CAN1, 14);
   
   CAN_ConfigFilter(CAN1, &CAN_Filter);
   int StatusCAN = CAN_Config(CAN1, &configCAN);
}



void Delay(uint32_t time_value)
{
   while(time_value--);
}



void RCM_Init()
{
   RCM_ConfigHSE(RCM_HSE_OPEN);
   while(RCM_WaitHSEReady() != SUCCESS) {;};
   
   RCM_ConfigPLL1(RCM_PLLSEL_HSE, 2, 84, RCM_PLL_SYS_DIV_2, 2); //50 -> 84
   RCM_EnablePLL1();   
   RCM_ConfigSYSCLK(RCM_SYSCLK_SEL_PLL);
   while(RCM->CFG_B.SCLKSWSTS != 0x2) // Ждем запуска PLL
   {}
   RCM_DisableHSI();
   RCM_ConfigAHB(RCM_AHB_DIV_1);
   RCM_ConfigAPB1(RCM_APB_DIV_4);   
   RCM_ConfigAPB2(RCM_APB_DIV_2); 
      
   RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
   SystemCoreClockUpdate();
}

int main_Init()
{
   RCM_Init();
   //ErrorValue = RCM_Init();
   Led_Init();
   CAN_Init();
}

int main()
{
   //RCM->PLL1CFG = 0x24003010;
   main_Init();
   
   
   while(1)
   {
      if(1)//!ErrorValue)
      {
         Delay(0x111fff);
         GPIO_ToggleBit(GPIOD, GPIO_PIN_13 | GPIO_PIN_12);
      }
      
   }
}

