#include <stdint.h>
#include <stdio.h>
#include "nrf24le1.h"
#include "hal_clk.h"
#include "hal_uart.h"
#include "hal_spi.h"
#include "hal_delay.h"
// Cusomization of low level stdio function. Used by for example printf().
#ifdef __ICC8051__
int putchar(int c)
#else /*presume C51 or other accepting compilator*/
char putchar(char c)
#endif
{
  hal_uart_putchar(c);
  return c;
}

// Cusomization of low level stdio function. Used by for example gets().
#ifdef __ICC8051__
int getchar(void)
#else /*presume C51 or other accepting compilator*/
char getchar(void)
#endif
{
  return hal_uart_getchar();
}

// Repeated putchar to print a string
void putstring(char *s)
{
  while(*s != 0)
    putchar(*s++);
}
void hal_spi_slave_readn(char* rbuf)
{
uint32_t RXCnt=0;
uint8_t in1=0;
/* while (!hal_spi_slave_csn_high())
   {  
   rbuf[RXCnt]=SPISDAT;
   RXCnt=RXCnt+1;
   in1=1;
   }		  */
   rbuf[0] =	hal_spi_slave_read();
   rbuf[1] = hal_spi_slave_read();
   rbuf[2] = hal_spi_slave_read();

   if (1==1) 
   {
   printf("data is: %d %d %d \r\n", (uint16_t)rbuf[0],(uint16_t)rbuf[1],(uint16_t)rbuf[2]);
   printf("sdata is: %d \r\n", (uint16_t)SPISDAT);
   printf("s2data is: %d \r\n", (uint16_t)rbuf[1]);
   printf("sPIF is: %d \r\n", (uint16_t)SPIF);
   printf("count is: %d \r\n", (uint16_t)RXCnt);
   in1=0;
   }
//   printf("Exit From Function \r\n");
}

uint16_t rcev=0;
bool inter,read = false;
char buf[3];
void main() 
{
uint8_t i=0;

 
  P01 =1;
  P0DIR = 0xD7;
  P1DIR = 0xFE;

   // Initializes the UART
  hal_uart_init(UART_BAUD_9K6);

  // Wait for XOSC to start to ensure proper UART baudrate
  while(hal_clk_get_16m_source() != HAL_CLK_XOSC16M)
  {}   
  EA=1;
  hal_spi_slave_init(HAL_SPI_MODE_0,HAL_SPI_MSB_LSB );
  SPI=1U;
 
 while (1) {
     
 if (inter){
   printf("data is: %02X %02X %02X \r\n", (uint16_t)buf[0],(uint16_t)buf[1],(uint16_t)buf[2]);
   printf("sdata is: %d \r\n", (uint16_t)SPISDAT);
   printf("s2data is: %d \r\n", (uint16_t)buf[1]);
   printf("sPIF is: %d \r\n", (uint16_t)SPI);
 // hal_spi_slave_readn(buf);
  printf("interupt hooking!");
  inter = false;
 
  }
 //if (read)  printf("SPI READ!");
 //  rcev=hal_spi_slave_read();
//   printf("data1 is: %d %d %d %d \r\n",(uint16_t)buf[1],(uint16_t)buf[0],(uint16_t)buf[2],(uint16_t)buf[3]);
 //  printf("data is: %d \r\n",(uint16_t)rcev);
 if (SPISSTAT >0) printf("data is: %02X \r\n",SPISSTAT);

 //delay_s(2);
 
 
 }
}
SER_ISR() 
{
   inter=true;
   buf[0] =	hal_spi_slave_rw(0xFF);
   buf[1] =	hal_spi_slave_rw(buf[0]);
   buf[2] =	hal_spi_slave_rw(buf[1]);
}
