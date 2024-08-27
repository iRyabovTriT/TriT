#include "apm32f4xx.h"
#include "apm32f4xx_gpio.h"
#include "apm32f4xx_rcm.h"
#include "apm32f4xx_i2c.h"
#include "i2c.h"


extern uint8_t ErrorI2C;

void fault_i2c(void)
{
   I2C3->CTRL1_B.STOP = 0x1;
   I2C3->CTRL1_B.START = 0x0;
   ErrorI2C = 0;
   I2C3->CTRL1_B.STOP = 0x0;
   return;
}



void i2c_setup_gpio(void)
{
   RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOA);
   RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOC);
   
   GPIOA->MODE_B.MODE8 = 0x2;
   GPIOC->MODE_B.MODE9 = 0x2;
   
   GPIOA->OMODE_B.OMODE8 = 0x1;
   GPIOC->OMODE_B.OMODE9 = 0x1;
   
   GPIOA->OSSEL_B.OSSEL8 = 0x3;
   GPIOC->OSSEL_B.OSSEL9 = 0x3;
   
   GPIOA->PUPD_B.PUPD8 = 0x1;
   GPIOC->PUPD_B.PUPD9 = 0x1;
   
   GPIO_ConfigPinAF(GPIOA, GPIO_PIN_SOURCE_8, GPIO_AF_I2C3);
   GPIO_ConfigPinAF(GPIOC, GPIO_PIN_SOURCE_9, GPIO_AF_I2C3);
   
   RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_I2C3);
}

void rcc_I2C_init(void)
{
   
}

void i2c_init(void)
{
   uint32_t Clock = 16000000UL; // Частота тактирования модуля
   uint32_t Speed = 100000UL;   // 100 кГц
   
   I2C3->CTRL1_B.SWRST = 1;
   I2C3->CTRL1 = 0x0;
   
   I2C3->CTRL2_B.CLKFCFG = 0x0; //Reset Fq I2C
   I2C3->CTRL2_B.CLKFCFG = Clock / 1000000UL;
   
   uint16_t Value = (uint16_t)(Clock / (Speed * 2));
   
   if(Value < 4) Value = 4;
   
   I2C3->CLKCTRL |= 80;
   
   I2C3->RISETMAX = (Clock / 1000000UL) + 1;
   
   I2C3->CTRL1_B.I2CEN = 0x1;
   I2C3->CTRL1_B.ACKEN = 0x1;
}

uint8_t i2c_read(uint8_t address, uint8_t registry)
{
   uint8_t count = 0;
   
   while(I2C3->STS2_B.BUSBSYFLG){if(ErrorI2C) { fault_i2c(); return 0;}};
   
   int a = 5;
   while(a--);
   
   I2C3->CTRL1_B.START = 0x1;                // Generated Start

   while(!(I2C3->STS1_B.STARTFLG)){if(ErrorI2C) {fault_i2c(); return 0;}};
   (void) I2C3->STS2;;
   I2C3->DATA = address << 1;
   
   (void) I2C3->STS1;
   
   while(!(I2C3->STS1_B.ADDRFLG)){if(ErrorI2C) { fault_i2c(); return 0;}};
   (void) I2C3->STS2;
   
   while(!(I2C3->STS1_B.TXBEFLG));
   I2C3->DATA = registry;
   
  // I2C3->CTRL1_B.STOP = 0x1;
   
   //a = 5;
  // while(a--);
   
   I2C3->CTRL1_B.START = 0x1;
   
   while(!(I2C3->STS1_B.STARTFLG)){if(ErrorI2C){ fault_i2c(); return 0;}};
   I2C3->DATA = (address << 1) | 1;
   
   while(!(I2C3->STS1_B.ADDRFLG)){if(ErrorI2C) { fault_i2c(); return 0;}};
   I2C3->CTRL1_B.ACKEN = 0x0;
   
   (void) I2C3->STS2;
   
   while(!(I2C3->STS1_B.RXBNEFLG)){if(ErrorI2C) { fault_i2c(); return 0;}};
   uint8_t value = (uint8_t)I2C3->DATA;
   
   I2C3->CTRL1_B.STOP = 0x1;
   
   I2C3->CTRL1_B.STOP = 0x0;
   return value;
}

void i2c_read_many(uint8_t address, uint8_t registry, uint8_t * result, uint8_t length)
{
   while(I2C3->STS2_B.BUSBSYFLG);
   
   I2C3->CTRL1_B.START = 0x1;                // Generated Start

   while(!(I2C3->STS1_B.STARTFLG));
   I2C3->DATA = address << 1;
   
   while(!(I2C3->STS1_B.ADDRFLG));
   (void) I2C3->STS2;
   
   while(!(I2C3->STS1_B.TXBEFLG));
   I2C3->DATA = registry;
   
   I2C3->CTRL1_B.STOP = 0x1;
   
   I2C3->CTRL1_B.START = 0x1;
   
   while(!(I2C3->STS1_B.STARTFLG));
   I2C3->DATA = (address << 1) | 1;
   
   if(length == 2)
   {
      while(!(I2C3->STS1_B.ADDRFLG));
      
      I2C3->CTRL1_B.ACKEN = 0x0;
      I2C3->CTRL1_B.ACKPOS = 0x1;
      
      (void) I2C3->STS2;
      
      while(!(I2C3->STS1_B.BTCFLG));
      I2C3->CTRL1_B.STOP = 0x1;
      
      *result++ = (uint8_t)I2C3->DATA;
      *result++ = (uint8_t)I2C3->DATA;
   }
   
   if(length > 2)
   {
      while(!(I2C3->STS1_B.ADDRFLG));
         
      (void) I2C3->STS2;
      
      length--;
      while(length--)
      {
         while(!(I2C3->STS1_B.BTCFLG));
         *result++ = (uint8_t)I2C3->DATA;
         
         if(length == 1)
         {
            I2C3->CTRL1_B.ACKEN = 0x0;
            I2C3->CTRL1_B.STOP = 0x1;
         }
      }
      
      *result++ = (uint8_t)I2C3->DATA;
   }
   
   
}

void i2c_write_many(uint8_t address, uint8_t registry, uint8_t* buf, uint32_t length)
{
   I2C3->CTRL1_B.START = 0x1;                // Generated Start
   
   while(!(I2C3->STS1_B.STARTFLG));
   I2C3->DATA = address << 1;
   
   while(!(I2C3->STS1_B.ADDRFLG)){if(ErrorI2C) fault_i2c();};
   (void) I2C3->STS2;
   
   while(!(I2C3->STS1_B.TXBEFLG));
   I2C3->DATA = registry;
   
   while(--length)
   {
      while(!(I2C3->STS1_B.BTCFLG)){if(ErrorI2C) fault_i2c();};
      I2C3->DATA = *buf++;
   }
   
   while(!(I2C3->STS1_B.BTCFLG));
   I2C3->CTRL1_B.STOP = 0x1;
}

void i2c_write(uint8_t address, uint8_t registry, uint8_t data)
{
   int a = 5;
   while(a--)
   
   I2C3->CTRL1_B.START = 0x1;                // Generated Start
   
   while(!(I2C3->STS1_B.STARTFLG)){if(ErrorI2C) {fault_i2c(); return;}};
   (void) I2C3->STS2;
   I2C3->DATA_B.DATA = address << 1;                // Write device address
   
   while(!(I2C3->STS1_B.ADDRFLG)){if(ErrorI2C) {fault_i2c(); return;}};
   (void) I2C3->STS2;
   
   //int a = 5;
   //while(a--)
   
   while(!(I2C3->STS1_B.TXBEFLG)){if(ErrorI2C) {fault_i2c(); return;}};
   I2C3->DATA = registry;
   
   while(!(I2C3->STS1_B.BTCFLG)){if(ErrorI2C) {fault_i2c(); return;}};
   I2C3->DATA = data;
   
   I2C3->CTRL1_B.STOP = 0x1;
   
   //I2C3->CTRL1_B.STOP = 0x0;
}

void I2C_ClearErrorFlag(void)
{
   
   I2C3->STS1_B.SMBALTFLG = BIT_RESET;
        
   I2C3->STS1_B.TTEFLG = BIT_RESET;
        
   I2C3->STS1_B.PECEFLG = BIT_RESET;
        
   I2C3->STS1_B.OVRURFLG = BIT_RESET;
        
   I2C3->STS1_B.AEFLG = BIT_RESET;
        
   I2C3->STS1_B.ALFLG = BIT_RESET;
        
   I2C3->STS1_B.BERRFLG = BIT_RESET;
}
