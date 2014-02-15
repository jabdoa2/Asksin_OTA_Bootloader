#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <util/delay.h>
#include "uart/uart.h"
 
#define BOOT_UART_BAUD_RATE     9600     /* Baudrate */
#define XON                     17       /* XON Zeichen */
#define XOFF                    19       /* XOFF Zeichen */
 
int main()
{
    unsigned int 	c=0;               /* Empfangenes Zeichen + Statuscode */
    unsigned char	temp,              /* Variable */
                        flag=1,            /* Flag zum steuern der Endlosschleife */
			p_mode=0;	   /* Flag zum steuern des Programmiermodus */
    void (*start)( void ) = 0x0000;        /* Funktionspointer auf 0x0000 */
 
    /* Interrupt Vektoren verbiegen */
 
    char sregtemp = SREG;
    cli();
    temp = MCUCR;
    MCUCR = temp | (1<<IVCE);
    MCUCR = temp | (1<<IVSEL);
    SREG = sregtemp;
 
    /* Einstellen der Baudrate und aktivieren der Interrupts */
//    uart_init( UART_BAUD_SELECT(BOOT_UART_BAUD_RATE,F_CPU) ); 
    sei();
 
//    uart_puts("Hallo hier ist der Bootloader\n\r");
    _delay_ms(1000);
 /*
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
                  default:
                     uart_puts("Du hast folgendes Zeichen gesendet: ");
                     uart_putc((unsigned char)c);
                     uart_puts("\n\r");
                     break;
            }
        }
    }
    while(flag);
 
    uart_puts("Springe zur Adresse 0x0000!\n\r");
    _delay_ms(1000);
 */
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
