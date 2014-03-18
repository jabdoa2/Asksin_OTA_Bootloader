#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <util/delay.h>
#include "uart/uart.h"
#include "cc.h"


int main(void)
{
  DDRB = 0x01;	/* set pin 0 as output */
  for(;;) {
    PORTB |= 0x01; /* switch pin 0 on */
    _delay_ms(250); /* wait */
    _delay_ms(250);
    PORTB &= ~0x01; /* switch pin 0 off */
    _delay_ms(250); /* wait */
    _delay_ms(250);
  } /* do it again */
}

