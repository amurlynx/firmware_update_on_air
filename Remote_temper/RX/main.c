#include <stdio.h>
#include "nrf24le1.h"
#include "hal_uart.h"
#include "hal_clk.h"
#include "hal_nrf.h"
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


// Global variables
static bool volatile radio_busy,get_pack,irq_get;
uint8_t payload[2];
	int t; 
void main(void)
{
 

  // Configure TXD pin as output.
  // P0.5, P0.3 and P1.0 are configured as outputs to make the example run on
  // either 24-pin, 32-pin or 48-pin nRF24LE1 variants.
  P01 =1;
  P0DIR = 0xD7;
  P1DIR = 0xFE;

   // Initializes the UART
  hal_uart_init(UART_BAUD_9K6);

  // Wait for XOSC to start to ensure proper UART baudrate
  while(hal_clk_get_16m_source() != HAL_CLK_XOSC16M)
  {}

  // Enable global interrupts
  EA = 1;
   // Enable the radio clock
  RFCKEN = 1U;

  // Enable RF interrupt
  RF = 1U;
    // Configure radio as primary receiver (PTX)
  hal_nrf_set_operation_mode(HAL_NRF_PRX);

  // Set payload width to 3 bytes
  hal_nrf_set_rx_payload_width((int)HAL_NRF_PIPE0, sizeof(int));
  
  
  // Power up radio
  hal_nrf_set_power_mode(HAL_NRF_PWR_UP);
  irq_get =false;
   
    CE_HIGH();
  
  // Print "Hello World" at start-up
  putstring("\r\nHello World!\r\n");
  get_pack=false;
  for(;;)
  {
   delay_s(2);
   //putstring("\r\nHello World!\r\n");
	if (irq_get) putstring("\r\nIRQ catched!\r\n");
    if (get_pack) {
//	putstring((char *)printf("Payload IS %d \r\n", payload));
//   	putstring((char *)printf("Payload IS %H \r\n", payload));
    	putstring("\r\n Get Packet \r\n");
/*	    t = payload[1];
        t <<=8;
        t |= payload[0];
        if  (payload[1]>0x7f) {
                t = (t ^0xffff)+1;
                t=0-t;
}
*/	 putstring((char *)printf("Temper:%03d.%04d C\r\n",t>>4,(t%16)*625));
	//RF=1U;
//	if (payload>0) putstring((char *)printf("Temper Is:%03d.%04d C\r\n",payload>>4,(payload%16)*625));
	}
	 else 
	 {
	 putstring("\r\nNot packet!\r\n");
	 }
// 	delay_s(4);
    // If any characters received
//   if( hal_uart_chars_available() )
//    {
//			P3 = 0x11;		
      // Echo received characters
 //     putchar(getchar());
 //   }
//    irq_get = false;
//      get_pack = false;
  }
}

// Radio interrupt
// Radio interrupt
NRF_ISR()
{
  uint8_t irq_flags;

  // Read and clear IRQ flags from radio
  irq_flags = hal_nrf_get_clear_irq_flags();
  irq_get=!irq_get;
  // If data received
  if((irq_flags & (1<<(uint8_t)HAL_NRF_RX_DR)) > 0)
  {
    // Read payload
    while(!hal_nrf_rx_fifo_empty())
    {
      hal_nrf_read_rx_payload((const uint8_t *) &t);
  	  get_pack=true;
    }

    // Write received payload[0] to port 0

//    P0 = payload[0];
//  RF = 0U;	
  }
}
/** @} */
