#include <stdio.h>
#include "nrf24le1.h"
#include "hal_uart.h"
#include "hal_clk.h"
#include "hal_nrf.h"
#include "hal_delay.h"
#define DSPIN P10
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
void OneWireReset()
{
     DSPIN=0;
	 P1DIR = 0xFE;
//	 gpio_pin_configure(DSPIN, GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT);
     delay_us(500);
	 P1DIR =0x01;
//gpio_pin_configure(DSPIN, GPIO_PIN_CONFIG_OPTION_DIR_INPUT);

     delay_us(500);
}
void OneWireOutByte(uint8_t d)
{
   uint8_t n;
   EA=0;
   for(n=8; n!=0; n--)
   {
      if ((d & 0x01) == 1)
      {
	  	 DSPIN=0;
   		 P1DIR = 0xFE;
 //        gpio_pin_val_clear(DSPIN);
 //        gpio_pin_configure(DSPIN, GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT);
         delay_us(2); //5
		 P1DIR = 0x01;
 //        gpio_pin_configure(DSPIN, GPIO_PIN_CONFIG_OPTION_DIR_INPUT);
         delay_us(60);
      }
      else
      {
	  	 DSPIN=0;
   		 P1DIR = 0xFE;
//         gpio_pin_val_clear(DSPIN);
//         gpio_pin_configure(DSPIN, GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT);
         delay_us(60);
		 P1DIR = 0x01;
 //        gpio_pin_configure(DSPIN, GPIO_PIN_CONFIG_OPTION_DIR_INPUT);      
      }
      d=d>>1;
   }
   EA=1;
   //interrupt_control_global_enable();
}

uint8_t OneWireInByte()
{
    uint8_t d=0, n,b=0;
  EA=0;//interrupt_control_global_disable();

    for (n=0; n<8; n++)
    {
	  	 DSPIN=0;
   		 P1DIR = 0xFE;
//        gpio_pin_val_clear(DSPIN);
//        gpio_pin_configure(DSPIN, GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT);
        delay_us(2); // 5
        P1DIR=0x01;//gpio_pin_configure(DSPIN, GPIO_PIN_CONFIG_OPTION_DIR_INPUT);
        delay_us(2); // 5
        b = DSPIN; //gpio_pin_val_read(DSPIN);
        delay_us(50);
        d = (d >> 1) | (b<<7);
    }
    EA=1;//interrupt_control_global_enable();
    return(d);
}

int Read_Temper()
{
uint8_t DSdata[2];
int16_t t_S;
     OneWireReset();
     OneWireOutByte(0xcc);
     OneWireOutByte(0x44);

     OneWireReset();
     OneWireOutByte(0xcc);
     OneWireOutByte(0xbe);

     DSdata[0] = OneWireInByte();
     DSdata[1] = OneWireInByte();

     OneWireReset();
     
        t_S = DSdata[1];
        t_S <<=8;
        t_S |= DSdata[0];
        if  (DSdata[1]>0x7f) {
                t_S = (t_S ^0xffff)+1;
                t_S=0-t_S;
}
return t_S; 
}


// Global variables
static bool volatile radio_busy;

void main(void)
{
	int t;  
	uint8_t test;
	uint8_t DSdata[2];
     OneWireReset();
     OneWireOutByte(0xcc);
     OneWireOutByte(0x44);

     OneWireReset();
     OneWireOutByte(0xcc);
     OneWireOutByte(0xbe);

     DSdata[0] = OneWireInByte();
     DSdata[1] = OneWireInByte();

     OneWireReset();

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
  // Power up radio
  hal_nrf_set_power_mode(HAL_NRF_PWR_UP);

  // Print "Hello World" at start-up
  putstring("\r\nHello World!\r\n");
  printf("\r\nData%d",DSdata[0]);
  printf("\r\nData%d",DSdata[1]);
  printf("\r\n SizeOf int: %d",sizeof(int));
  for(;;)
  {
    delay_s(10);
	t= Read_Temper();
	test = 0x39;
	printf("Temper Is:%03d.%04d C\n",t>>4,(t%16)*625);
//	hal_nrf_write_tx_payload((const uint8_t *)&t, 16U);
	hal_nrf_write_tx_payload((const uint8_t *)&t, sizeof(t));
//	hal_nrf_write_tx_payload(DSdata,2U);
    // Toggle radio CE signal to start transmission
	CE_PULSE();

    radio_busy = true;
    // Wait for radio operation to finish
    while (radio_busy)
    {
    }
//    putstring((char *)printf("Temper Is:%03d.%04d C\n",t>>4,(t%16)*625));
// 	delay_s(4);
    // If any characters received
//   if( hal_uart_chars_available() )
//    {
//			P3 = 0x11;		
      // Echo received characters
 //     putchar(getchar());
 //   }
  }
}

// Radio interrupt
NRF_ISR()
{
  uint8_t irq_flags;

  // Read and clear IRQ flags from radio
  irq_flags = hal_nrf_get_clear_irq_flags();

  switch(irq_flags)
  {
    // Transmission success
    case (1 << (uint8_t)HAL_NRF_TX_DS):
      radio_busy = false;
      // Data has been sent
      break;
    // Transmission failed (maximum re-transmits)
    case (1 << (uint8_t)HAL_NRF_MAX_RT):
      // When a MAX_RT interrupt occurs the TX payload will not be removed from the TX FIFO.
      // If the packet is to be discarded this must be done manually by flushing the TX FIFO.
      // Alternatively, CE_PULSE() can be called re-starting transmission of the payload.
      // (Will only be possible after the radio irq flags are cleared)
      hal_nrf_flush_tx();
      radio_busy = false;
      break;
    default:
      break;
  }
}