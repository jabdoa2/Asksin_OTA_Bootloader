#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <util/delay.h>
#include "uart/uart.h"
#include "cc.h"
#include <string.h>

#define BOOT_UART_BAUD_RATE     57600     /* Baudrate */
#define XON                     17       /* XON Zeichen */
#define XOFF                    19       /* XOFF Zeichen */

uint8_t data[60];
uint8_t hasData = 0;
uint8_t hmid[3] = {0xAB, 0xCD, 0xEF};
uint8_t flasher_hmid[3];
uint16_t timeoutCounter = 0;

char pHexChar(const uint8_t val) {
	const char hexDigits[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
	uart_putc(hexDigits[val >> 4]);
	uart_putc(hexDigits[val & 0xF]);
	return 0;
}

char pHex(const uint8_t *buf, uint16_t len) {
	for (uint16_t i=0; i<len; i++) {
		pHexChar(buf[i]);
		if(i+1 < len) uart_putc(' ');
	}
	return 0;
}

void program_page (uint32_t page, uint8_t *buf)
{
	uint16_t i;
	uint8_t sreg;

	/* Disable interrupts */
	sreg = SREG;
	cli();

	eeprom_busy_wait ();

	boot_page_erase (page);
	boot_spm_busy_wait ();      /* Wait until the memory is erased. */

	for (i=0; i<SPM_PAGESIZE; i+=2)
	{
		/* Set up little-endian word. */
		uint16_t w = *buf++;
		w += (*buf++) << 8;

		boot_page_fill (page + i, w);
	}

	boot_page_write (page);     /* Store buffer in flash page.		*/
	boot_spm_busy_wait();       /* Wait until the memory is written.*/

	/* Reenable RWW-section again. We need this if we want to jump back */
	/* to the application after bootloading. */
	boot_rww_enable ();

	/* Re-enable interrupts (if they were ever enabled). */
	SREG = sreg;
	sei();
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

void hm_dec(uint8_t *buffer) {

        uint8_t prev = buffer[1];
        buffer[1] = (~buffer[1]) ^ 0x89;

        uint8_t i, t;
        for (i=2; i<buffer[0]; i++) {
                t = buffer[i];
                buffer[i] = (prev + 0xdc) ^ buffer[i];
                prev = t;
        }

        buffer[i] ^= buffer[2];
}

void send_hm_data(uint8_t *msg)
{
	hm_enc(msg);
	cli();
	sendData(msg, 0);
	sei();
}

void send_ack(uint8_t *receiver, uint8_t messageId) {
	uint8_t ack_msg[11] = { 10, messageId, 0x80, 0x02, hmid[0], hmid[1], hmid[2], receiver[0], receiver[1], receiver[2], 0x00};
	send_hm_data(ack_msg);	
}

void send_nack_to_msg(uint8_t *msg) {
	uint8_t nack_msg[11] = { 10, msg[1], 0x80, 0x02, hmid[0], hmid[1], hmid[2], msg[4], msg[5], msg[6], 0x80};
	send_hm_data(nack_msg);	
}

void send_ack_if_requested(uint8_t* msg) {
	if (! (msg[2] & (1 << 5)))
	{
		// no ack requested
		return;
	}
	uart_puts("Sending ack\n\r");
	
	// send ack to sender of msg
	uint8_t ack_hmid[3];
	ack_hmid[0] = msg[4];
	ack_hmid[1] = msg[5];
	ack_hmid[2] = msg[6];
	send_ack(ack_hmid, msg[1]);
}

void startApplication() {
	void (*start)( void ) = 0x0000;        /* Funktionspointer auf 0x0000 */
	unsigned char	temp;
	/* vor Rücksprung eventuell benutzte Hardware deaktivieren
	und Interrupts global deaktivieren, da kein "echter" Reset erfolgt */

	/* Interrupt Vektoren wieder gerade biegen */
	cli();
	temp = MCUCR;
	MCUCR = temp | (1<<IVCE);
	MCUCR = temp & ~(1<<IVSEL);

	/* Rücksprung zur Adresse 0x0000 */
	start(); 
}

void setup_interrupts_for_bootloader() {
	unsigned char	temp;              /* Variable */
	/* Interrupt Vektoren verbiegen */
	char sregtemp = SREG;
	cli();
	temp = MCUCR;
	MCUCR = temp | (1<<IVCE);
	MCUCR = temp | (1<<IVSEL);
	SREG = sregtemp;
}

void startApplicationOnTimeout()
{
	if (timeoutCounter > 30000) { // wait about 10s
		uart_puts("Timeout reached. Starting application!\n\r");
		_delay_ms(250);
		startApplication();
	}
}

void send_bootloader_sequence()
{
	uart_puts("Sending bootloader sequence\n\r");

	// Send this message: 14 00 00 10 23 25 B7 00 00 00 00 4B 45 51 30 37 33 34 31 31
	// NAME: KEQ0123456
	uint8_t msg[21] = {
		0x14, 0x00, 0x00, 0x10, 0xAB, 0xCD, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x4B, 0x45, 0x51, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36
	};
	send_hm_data(msg);
}

void wait_for_CB_msg()
{
	// reset timeout
	timeoutCounter = 0;
	while(1) {
	        uart_getc();

		startApplicationOnTimeout();

		// Wait for data
		if(! hasData) {
			continue;
		}
		
		hm_dec(data);

		hasData = 0;
		if (data[7] != hmid[0] || data[8] != hmid[1] || data[9] != hmid[2]) {
                	uart_puts("Got data but not for us\n\r");
			continue;
		}

		/*
		 * Wait for: 0F 01 00 CB 1A B1 50 AB CD EF 10 5B 11 F8 15 47
		 */
		if (data[3] == 0xCB) {
                	uart_puts("Got message to start config\n\r");
			flasher_hmid[0] = data[4];
			flasher_hmid[1] = data[5];
			flasher_hmid[2] = data[6];
			send_ack_if_requested(data);
			break;
		}
		send_nack_to_msg(data);
	}

}

void switch_radio_to_100k_mode()
{
	uart_puts("Switching to 100k mode\n\r");
	cli();
	init(1);
	sei();
}

void switch_radio_to_10k_mode()
{
	uart_puts("Switching to 10k mode\n\r");
	cli();
	init(0);
	sei();
}

void flash_from_rf()
{
	timeoutCounter = 0;

	uart_puts("Start to receive firmware\n\r");

	uint8_t state = 0; // 0 = block has not started yet, 1 = block started
	uint8_t blockData[SPM_PAGESIZE];
	uint16_t blockLen = 0;
	uint16_t blockPos = 0;
	uint32_t pageCnt = 0;
	uint16_t expectedMsgId = data[1] + 1;
	while (1) {
	        uart_getc();

		startApplicationOnTimeout();

		// Wait for data
		if(! hasData) {
			continue;
		}
		
		hm_dec(data);

		hasData = 0;
		if (data[7] != hmid[0] || data[8] != hmid[1] || data[9] != hmid[2]) {
                	uart_puts("Got data but not for us\n\r");
			continue;
		}
		if (data[3] != 0xCA) {
                	uart_puts("Got other message type\n\r");
			continue;
		}

		if (data[1] != expectedMsgId) {
			if (data[1] == expectedMsgId + 1 && pageCnt > 0) {
				// The other side may have missed our ack. It will resend the last block
				expectedMsgId--;
				pageCnt--;
				state = 0;
				uart_puts("Retransmit. Will reflash!\n\r");
			} else {
				// We are in an invalid state now
				state = 0;
				uart_puts("FATAL: Wrong msgId detected!\n\r");
			}
		}

		if (state == 0) {
			// Read len and check memory
			blockLen = (data[10] << 8);
			blockLen += data[11];
			if (blockLen != SPM_PAGESIZE) {
				uart_puts("Block is not page size\n\r");
				state = 0;
				continue;
			}
			if (data[0]-11 > SPM_PAGESIZE) {
				uart_puts("Block to big\n\r");
				state = 0;
				continue;
			}
			state = 1;
			blockPos = data[0]-11;
			memcpy(&blockData, &data[12], data[0]-11);
		} else {
			if (blockPos + data[0]-9 > blockLen) {
				uart_puts("Got more data than blocklen\n\r");
				state = 0;
				continue;
			}
			memcpy(&blockData[blockPos], &data[10], data[0]-9);
			blockPos += data[0]-9;
		}
		
		if (data[2] == 0x20) {
			if (blockPos != blockLen) {
				uart_puts("blockLen and blockPos do not match\n\r");
				state = 0;
				continue;
			} else {
				uart_puts("Got complete block!\n\r");
//				_delay_ms(100);
				program_page(pageCnt * SPM_PAGESIZE, blockData);
				pageCnt++;
				expectedMsgId++;
				timeoutCounter = 0;
			}
			//pHex(blockData, blockLen);
			send_ack(flasher_hmid, data[1]);
			state = 0;
		}
	}

}

void setup_timer() {
	// Timer 0 konfigurieren
	TCCR0B |= (1<<CS01) | (!(1<<CS00)) | (!(1<<CS02));	//PRESCALER 8
	TCNT0 = 0;
	TIMSK0 |= (1<<TOIE0);
}

void setup_cc1100_interrupts() {
	EIMSK = 1<<INT0;					// Enable INT0
	EICRA = 1<<ISC01 | 0<<ISC00;			// falling edge
}

int main()
{
	// Blink LED
	DDRB = 0x01;  /* set pin 0 as output */
	PORTB |= 0x01; /* switch pin 0 on */
	_delay_ms(250); /* wait */
	_delay_ms(250);
	PORTB &= ~0x01; /* switch pin 0 off */

	// map to correct interrupt table for bootloader
	setup_interrupts_for_bootloader();

	// setup timer for timeout counter
	setup_timer();

	// setup interrupts for cc1100
	setup_cc1100_interrupts();

	// init uart
	uart_init( UART_BAUD_SELECT(BOOT_UART_BAUD_RATE,F_CPU) ); 

	// Activate interrupts
	sei();
 
	// go to standard 10k mode
	switch_radio_to_10k_mode();

	// send broadcast to allow windows tool or flash_ota to discover device
	send_bootloader_sequence();

	// wait for msg in 10k mode to change to 100k mode
	wait_for_CB_msg();

	// switch to 100k mode
	switch_radio_to_100k_mode();

	// this is needed for windows tool
	wait_for_CB_msg();

	// run the actual flashing
	flash_from_rf();
}


ISR(INT0_vect) {
	cli();
	if (receiveData(data)) {
		hasData = 1;
	}
	sei();
}

//INTERUPT
ISR(TIMER0_OVF_vect) 
{
	TCNT0 = 0;
	timeoutCounter++;
}
