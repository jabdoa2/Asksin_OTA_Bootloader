#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <util/delay.h>
#include "uart/uart.h"
#include "cc.h"
 
#define BOOT_UART_BAUD_RATE     9600     /* Baudrate */
#define XON                     17       /* XON Zeichen */
#define XOFF                    19       /* XOFF Zeichen */

uint8_t data[60];
uint8_t hasData = 0;

char pHexChar(const uint8_t val) {
	const char hexDigits[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
	uart_putc(hexDigits[val >> 4]);
	uart_putc(hexDigits[val & 0xF]);
	return 0;
}

char pHex(const uint8_t *buf, uint8_t len) {
	for (uint8_t i=0; i<len; i++) {
		pHexChar(buf[i]);
		if(i+1 < len) uart_putc(' ');
	}
	return 0;
}

void hm_enc(uint8_t *buffer) {

	buffer[1] = (~buffer[1]) ^ 0x89;
	uint8_t buf2 = buffer[2];
	uint8_t prev = buffer[1];

	uint8_t i;
	for (i=2; i<buffer[0]; i++) {
		prev = (prev + 0xdc) ^ buffer[i];
		buffer[i] = prev;
	}

	buffer[i] ^= buf2;
}

int main()
{
    unsigned int 	c=0;               /* Empfangenes Zeichen + Statuscode */
    unsigned char	temp,              /* Variable */
                        flag=1,            /* Flag zum steuern der Endlosschleife */
			p_mode=0;	   /* Flag zum steuern des Programmiermodus */
    void (*start)( void ) = 0x0000;        /* Funktionspointer auf 0x0000 */
 
    DDRB = 0x01;  /* set pin 0 as output */
    PORTB |= 0x01; /* switch pin 0 on */
    _delay_ms(250); /* wait */
    _delay_ms(250);
    PORTB &= ~0x01; /* switch pin 0 off */



    /* Interrupt Vektoren verbiegen */
 
/*
    char sregtemp = SREG;
    cli();
    temp = MCUCR;
    MCUCR = temp | (1<<IVCE);
    MCUCR = temp | (1<<IVSEL);
    SREG = sregtemp;
 */


    EIMSK = 1<<INT0;					// Enable INT0
    EICRA = 1<<ISC01 | 0<<ISC00;			// falling edge
    /* Einstellen der Baudrate und aktivieren der Interrupts */
    uart_init( UART_BAUD_SELECT(BOOT_UART_BAUD_RATE,F_CPU) ); 
 
    _delay_ms(1000);


    init(0);
    sei();
    uart_puts("Hallo hier ist der Bootloader\n\r");

cli();
	// Send this message: 14 00 00 10 23 25 B7 00 00 00 00 4B 45 51 30 37 33 34 31 31
	uint8_t msg[21] = {
//		0x14, 0x00, 0x00, 0x10, 0xAB, 0xCD, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x43, 0x55, 0x53, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36
		0x14, 0x00, 0x00, 0x10, 0x23, 0x25, 0xB7, 0x00, 0x00, 0x00, 0x00, 0x4B, 0x45, 0x51, 0x30, 0x37, 0x33, 0x34, 0x31, 0x31, 0x37
	};
	hm_enc(msg);
	sendData(msg, 0);
sei();

    do
    {
        c = uart_getc();
        if( !(c & UART_NO_DATA) )
        {
            switch((unsigned char)c)
            {
                 case 'q': 
		     flag=0;
                     uart_puts("Verlasse den Bootloader!\n\r");
                     break;
                 case '0': 
                     uart_puts("Going to 10k Mode\n\r");
			cli();
                     init(0);
			sei();
                     break;
                 case '1': 
                     uart_puts("Going to 100k Mode\n\r");
			cli();
                     init(1);
			sei();
                     break;


                  default:
                     uart_puts("Du hast folgendes Zeichen gesendet: ");
                     uart_putc((unsigned char)c);
                     uart_puts("\n\r");
                     break;
            }
        }
	if (hasData) {
		uint8_t prev = data[1];
		data[1] = (~data[1]) ^ 0x89;

		uint8_t i, t;
		for (i=2; i<data[0]; i++) {
			t = data[i];
			data[i] = (prev + 0xdc) ^ data[i];
			prev = t;
		}

		data[i] ^= data[2];
                uart_puts("Got data: ");
		pHex(data, data[0]+1);
                uart_puts("\n\r");
		hasData = 0;
	}
    }
    while(flag);
 
    uart_puts("Springe zur Adresse 0x0000!\n\r");
    _delay_ms(1000);
 
    /* vor Rücksprung eventuell benutzte Hardware deaktivieren
       und Interrupts global deaktivieren, da kein "echter" Reset erfolgt */
 
    /* Interrupt Vektoren wieder gerade biegen */
    cli();
    temp = MCUCR;
    MCUCR = temp | (1<<IVCE);
    MCUCR = temp & ~(1<<IVSEL);
 
    /* Rücksprung zur Adresse 0x0000 */
    start(); 
    return 0;
}


ISR(INT0_vect) {
	cli();
	if (receiveData(data)) {
		hasData = 1;
	}
	sei();
}
