
#include "libs.h"
#include "nRFLE.c"
#include "main.h"
#define LCD_E  GPIO_PIN_ID_P1_4
#define LCD_RS GPIO_PIN_ID_P1_1
#define LCD_D4 GPIO_PIN_ID_P1_0
#define LCD_D5 GPIO_PIN_ID_P0_7
#define LCD_D6 GPIO_PIN_ID_P0_5
#define LCD_D7 GPIO_PIN_ID_P0_4
#define LCD_CMD true
#define LCD_CHAR false
#define LCD_WIDTH 16

void lcd_init()
{
  gpio_pin_configure(LCD_E,GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT);
  gpio_pin_configure(LCD_RS,GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT);
  gpio_pin_configure(LCD_D4,GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT);
  gpio_pin_configure(LCD_D5,GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT);
  gpio_pin_configure(LCD_D6,GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT);
  gpio_pin_configure(LCD_D7,GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT);
  delay_us(20);
  lcd_byte(0x33,LCD_CMD);
  delay_us(5);
  lcd_byte(0x32,LCD_CMD);
  delay_ms(1);
  lcd_byte(0x28,LCD_CMD);
  lcd_byte(0x0C,LCD_CMD);
  lcd_byte(0x06,LCD_CMD);
  lcd_byte(0x01,LCD_CMD);
}
void lcd_string(char *s)
{
//  Send string to display
  while (*s !=0)
//  message = message.ljust(LCD_WIDTH," ")
//  for i in range(LCD_WIDTH):
    lcd_byte((int)*s++,LCD_CHAR);
}
void lcd_byte(unsigned char bits, bool mode)
{
//   Send byte to data pins
//   bits = data
//   mode = True  for character
//          False for command
  if (mode) gpio_pin_val_set(LCD_RS);
  else gpio_pin_val_clear(LCD_RS);
//  GPIO.output(LCD_RS, mode) # RS
//   High bits
  gpio_pin_val_clear(LCD_D4);
  gpio_pin_val_clear(LCD_D5);
  gpio_pin_val_clear(LCD_D6);
  gpio_pin_val_clear(LCD_D7);
//  GPIO.output(LCD_D4, False)
//  GPIO.output(LCD_D5, False)
//  GPIO.output(LCD_D6, False)
//  GPIO.output(LCD_D7, False)
  if (bits&0x10==0x10) gpio_pin_val_set(LCD_D4);
  if (bits&0x20==0x20) gpio_pin_val_set(LCD_D5);
  if (bits&0x40==0x40) gpio_pin_val_set(LCD_D6);
  if (bits&0x80==0x80) gpio_pin_val_set(LCD_D7);
//   Toggle 'Enable' pin
  delay_us(5);
  gpio_pin_val_set(LCD_E);
  delay_us(5);
  gpio_pin_val_clear(LCD_E);
  delay_us(5);

//   Low bits
  gpio_pin_val_clear(LCD_D4);
  gpio_pin_val_clear(LCD_D5);
  gpio_pin_val_clear(LCD_D6);
  gpio_pin_val_clear(LCD_D7);
//  GPIO.output(LCD_D4, False)
//  GPIO.output(LCD_D5, False)
//  GPIO.output(LCD_D6, False)
//  GPIO.output(LCD_D7, False)
  if (bits&0x01==0x01) gpio_pin_val_set(LCD_D4);
  if (bits&0x02==0x02) gpio_pin_val_set(LCD_D5);
  if (bits&0x04==0x04) gpio_pin_val_set(LCD_D6);
  if (bits&0x08==0x08) gpio_pin_val_set(LCD_D7);
//  # Toggle 'Enable' pin
  delay_ms(5);
  gpio_pin_val_set(LCD_E);
  delay_us(5);
  gpio_pin_val_clear(LCD_E);
  delay_us(5);
 }


void main()
{
lcd_init();
while (1) {

//lcd_byte(0x80,LCD_CMD);
//lcd_byte(0x48,LCD_CHAR);
lcd_string("Hel");
}
}
