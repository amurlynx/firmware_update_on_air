// описание библиотек для работы с модулем nRF24LE1
// http://homes-smart.ru/index.php/oborudovanie/bez-provodov-2-4-ggts/opisanie-radiomodulya-nrf24l01



#include <stdio.h>
#include <stdint.h>

#include "reg24le1.h"
//#include "rf.h"
#include "delay.h"

#include "wiringLE.c"


// gpio
#include "src/gpio/src/gpio_pin_configure.c"
#include "src/gpio/src/gpio_pin_val_read.c"
#include "src/gpio/src/gpio_pin_val_clear.c"
#include "src/gpio/src/gpio_pin_val_set.c"
#include "src/gpio/src/gpio_pin_val_write.c"

// delay
#include "src/delay/src/delay_us.c"
#include "src/delay/src/delay_s.c"
#include "src/delay/src/delay_ms.c"

#include "src/rtc2/src/rtc2_configure.c"
