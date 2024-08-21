#include "apm32f4xx.h"
#include "i2c.h"
#include "apm32f4xx_gpio.h"
#include "apm32f4xx_rcm.h"

void i2c_setup_gpio(void)
{
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
}

void rcc_I2C_init(void)
{
   RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOA);
   RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOC);
   RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_I2C3);
}

void i2c_init(void)
{
   I2C3->CTRL2_B.CLKFCFG = 0x0; //Reset Fq I2C
   I2C3->CTRL2_B.CLKFCFG = 0x2;
   
   I2C3->CLKCTRL |= 80;
   
   I2C3->RISETMAX = 3;
   
   I2C3->CTRL1_B.I2CEN = 0x1;
   I2C3->CTRL1_B.ACKEN = 0x1;
}

uint8_t i2c_read(uint8_t address, uint8_t registry)
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
   
   while(!(I2C3->STS1_B.ADDRFLG));
   I2C3->CTRL1_B.ACKEN = 0x0;
   
   (void) I2C3->STS2;
   
   while(!(I2C3->STS1_B.RXBNEFLG));
   uint8_t value = (uint8_t)I2C3->DATA;
   
   I2C3->CTRL1_B.STOP = 0x1;
   
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
   
   while(!(I2C3->STS1_B.ADDRFLG));
   (void) I2C3->STS2;
   
   while(!(I2C3->STS1_B.TXBEFLG));
   I2C3->DATA = registry;
   
   while(--length)
   {
      while(!(I2C3->STS1_B.BTCFLG));
      I2C3->DATA = *buf++;
   }
   
   while(!(I2C3->STS1_B.BTCFLG));
   I2C3->CTRL1_B.STOP = 0x1;
}

void i2c_write(uint8_t address, uint8_t registry, uint8_t data)
{
   I2C3->CTRL1_B.START = 0x1;                // Generated Start
   
   while(!(I2C3->STS1_B.STARTFLG));
   I2C3->DATA = address << 1;                // Write device address
   
   while(!(I2C3->STS1_B.ADDRFLG));
   (void) I2C3->STS2;
   
   while(!(I2C3->STS1_B.TXBEFLG));
   I2C3->DATA = registry;
   
   while(!(I2C3->STS1_B.BTCFLG));
   I2C3->DATA = data;
   
   I2C3->CTRL1_B.STOP = 0x1;
}
