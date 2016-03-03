/*
 * ADS1115 driver example
 *
 * March 2015 John Whittington http://www.jbrengineering.co.uk @j_whittington
 *
*==============================================================*/

#include <wiringPi.h>
#include <ads1115.h>
#include <stdio.h>
#include <stdint.h>

int main(int argc, char *argv[]) {
  uint16_t value;
  double voltage;
  wiringPiSetup();

  ads1115Setup(100,0x48);

  for (;;) {
    value = (uint16_t) analogRead(100);
    voltage = value * (4.096 / 32768);

    printf("ADS1115 Reading: %u\n\r",value);
    printf("ADS1115 Voltage: %g\n\r",voltage);
    delay(10);
  }

  return 0;
}
