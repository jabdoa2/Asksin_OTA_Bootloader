#include "bootloader.h"
#include "config.h"

#if DEBUG == 1
//	#define VERSION_STRING       "\nAskSin OTA Bootloader V0.5.1 \n\n"			// version number for debug info
	#define VERSION_STRING       "\nV0.5.1\n\n"									// version number for debug info
	#define BOOT_UART_BAUD_RATE  57600											// Baudrate
#endif

/*****************************************
 *        Address data section           *
 *           See Makefile                *
 *****************************************/
#define ADDRESS_SECTION __attribute__ ((section (".addressData")))
const uint8_t hm_Type[2]        ADDRESS_SECTION = {HM_TYPE};					// 2 bytes device type
const uint8_t hm_serial[10]     ADDRESS_SECTION = {HM_SERIAL};					// 10 bytes serial number
const uint8_t hm_id[3]          ADDRESS_SECTION = {HM_ID};						// 3 bytes device address

#if DEBUG == 1
	void pHexChar(const uint8_t val) {
		const char hexDigits[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
		uart_putc(hexDigits[val >> 4]);
		uart_putc(hexDigits[val & 0xF]);
	}

	void pHex(const uint8_t *buf, uint8_t len) {
		for (uint16_t i=0; i < len; i++) {
			pHexChar(buf[i]);
		}
		uart_putc(' ');
	}
#endif

void program_page (uint32_t page, uint8_t *buf) {
	uint16_t i;
	uint8_t sreg;

	/* Disable interrupts */
	sreg = SREG;
	cli();

	eeprom_busy_wait ();

	boot_page_erase (page);
	boot_spm_busy_wait ();														// Wait until the memory is erased.

	for (i=0; i < SPM_PAGESIZE; i+=2) {
		uint16_t w = *buf++;													// Set up little-endian word.
		w += (*buf++) << 8;

		boot_page_fill (page + i, w);
	}

	boot_page_write (page);														// Store buffer in flash page.
	boot_spm_busy_wait();														// Wait until the memory is written.

	/*
	 * Re-enable RWW-section again. We need this if we want to jump back
	 * to the application after bootloading.
	 */
	boot_rww_enable ();

	SREG = sreg;																// Re-enable interrupts (if they were ever enabled).
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
	for (i = 2; i < buffer[0]; i++) {
		t = buffer[i];
		buffer[i] = (prev + 0xdc) ^ buffer[i];
		prev = t;
	}

	buffer[i] ^= buffer[2];
}

void send_hm_data(uint8_t *msg) {
	hm_enc(msg);
	cli();
	sendData(msg, 0);
	sei();
}

void send_ack(uint8_t *receiver, uint8_t messageId) {
	uint8_t ack_msg[11] = { 10, messageId, 0x80, 0x02, hmID[0], hmID[1], hmID[2], receiver[0], receiver[1], receiver[2], 0x00};
	send_hm_data(ack_msg);	
}

void send_nack_to_msg(uint8_t *msg) {
	uint8_t nack_msg[11] = { 10, msg[1], 0x80, 0x02, hmID[0], hmID[1], hmID[2], msg[4], msg[5], msg[6], 0x80};
	send_hm_data(nack_msg);	
}

void send_ack_if_requested(uint8_t* msg) {
	if (! (msg[2] & (1 << 5))) {
		// no ack requested
		return;
	}
	
	#if DEBUG == 1
		uart_puts_P("Send ack\n");
	#endif

	// send ack to sender of msg
	uint8_t ack_hmid[3];
	ack_hmid[0] = msg[4];
	ack_hmid[1] = msg[5];
	ack_hmid[2] = msg[6];
	send_ack(ack_hmid, msg[1]);
}

#if CRC_FLASH == 1
	/*
	 * function to update CRC with additional byte. based on
	 * http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=127852&start=0
	 */
	static uint16_t updcrc(uint8_t c, uint16_t crc) {
		uint8_t flag;
		for (uint8_t i = 0; i < 8; ++i) {
			flag = !!(crc & 0x8000);
			crc <<= 1;
			if (c & 0x80) {
				crc |= 1;
			}
			if (flag) {
				crc ^= 0x1021;
			}
			c <<= 1;
		}
		return crc;
	}

	/*
	 * Read through program memory for defined CODE_LEN and calculate CRC.
	 * Then compare with CRC stored at the end of CODE_LEN.
	 */
	uint8_t crc_app_ok(void) {
		uint16_t crc = 0xFFFF;
		for (uint16_t i=0; i < CODE_LEN; i++) {
			crc = updcrc(pgm_read_byte(i), crc);
		}
		// augment
		crc = updcrc(0, updcrc(0, crc));
		return (pgm_read_word(CODE_LEN) == crc);
	}

	/*
	 * Do a reset if CRC check fails, so that bootloader is ready to receive new firmware.
	 */
	void resetOnCRCFail(){
		if(crc_app_ok()){
			#if DEBUG == 1
				uart_puts_P("CRC OK\r\n");
				_delay_ms(250);
			#endif

			return;
		}
		wdt_reset();

		#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
			bitSet(PORT_STATUSLED, PIN_STATUSLED);
			_delay_ms(2000);
			bitClear(PORT_STATUSLED, PIN_STATUSLED);
		#endif

		#if DEBUG == 1
			uart_puts_P("CRC fail: Reboot\r\n");
		#endif

		wdt_enable(WDTO_1S);
		while(1);																// wait for Watchdog to generate reset
	}
#endif 

void startApplication() {
	void (*start)( void ) = 0x0000;												// Funktionspointer auf 0x0000
	unsigned char	temp;

	/*
	 * Vor Rücksprung eventuell benutzte Hardware deaktivieren
	 * und Interrupts global deaktivieren, da kein "echter" Reset erfolgt
	 */

	/* Interrupt Vektoren wieder gerade biegen */
	cli();
	temp = MCUCR;
	MCUCR = temp | (1 << IVCE);
	MCUCR = temp & ~(1 << IVSEL);

	start();																	// Rücksprung zur Adresse 0x0000
}

void setup_interrupts_for_bootloader() {
	unsigned char	temp;
	/* Interrupt Vektoren verbiegen */
	char sregtemp = SREG;
	cli();
	temp = MCUCR;
	MCUCR = temp | (1 << IVCE);
	MCUCR = temp | (1 << IVSEL);
	SREG = sregtemp;
}

void startApplicationOnTimeout() {
	if (timeoutCounter > 30000) {												// wait about 10s at 8Mhz
		#if DEBUG == 1
			uart_puts_P("Timeout. Start program!\n");
			_delay_ms(250);
		#endif

		#if CRC_FLASH == 1
			/*
			 * if CRC-Check is enabled, check CRC checksum before application start
			 */
			resetOnCRCFail();
		#endif

		startApplication();
	}
}

void send_bootloader_sequence() {
	#if DEBUG == 1
		uart_puts_P("Send bootloader sequence\n");
	#endif

	/*
	 *                                 HM_ID                         HM_SERIAL
	 *                               |--------|           |-----------------------------|
	 * Send this message: 14 00 00 10 23 25 B7 00 00 00 00 41 42 43 44 45 46 47 48 49 50
	 */
	uint8_t msg[21] = {
		0x14, 0x00, 0x00, 0x10, hmID[0], hmID[1], hmID[2], 0x00, 0x00, 0x00, 0x00,
		hmSerial[0], hmSerial[1], hmSerial[2], hmSerial[3], hmSerial[4],
		hmSerial[5], hmSerial[6], hmSerial[7], hmSerial[8], hmSerial[9]
	};

	send_hm_data(msg);
}

void wait_for_CB_msg() {
	#if DEBUG == 1
		uart_puts_P("Wait for CB Msg\n");
	#endif

	timeoutCounter = 0;															// reset timeout
	while(1) {
		startApplicationOnTimeout();

		if(! hasData) {															// Wait for data
			continue;
		}
		
		hm_dec(data);

		hasData = 0;
		if (data[7] != hmID[0] || data[8] != hmID[1] || data[9] != hmID[2]) {
			#if DEBUG == 1
				uart_puts_P("Got data but not for us\n");
			#endif

			continue;
		}

		/*
		 * Wait for: 0F 01 00 CB 1A B1 50 AB CD EF 10 5B 11 F8 15 47
		 */
		if (data[3] == 0xCB) {
			#if DEBUG == 1
				uart_puts_P("Got message to start config\n");
			#endif

			flasher_hmid[0] = data[4];
			flasher_hmid[1] = data[5];
			flasher_hmid[2] = data[6];
			send_ack_if_requested(data);
			break;
		}

		send_nack_to_msg(data);
	}
}

void switch_radio_to_100k_mode() {
	#if DEBUG == 1
		uart_puts_P("Switch to 100k\n");
	#endif

	cli();
	init(1);
	sei();
}

void switch_radio_to_10k_mode() {
	#if DEBUG == 1
		uart_puts_P("Switch to 10k\n");
	#endif

	cli();
	init(0);
	sei();
}

void flash_from_rf() {
	timeoutCounter = 0;

	#if DEBUG == 1
		uart_puts_P("Start receive firmware\n");
	#endif

	uint8_t state = 0;															// 0 = block has not started yet, 1 = block started
	uint8_t blockData[SPM_PAGESIZE];
	uint16_t blockLen = 0;
	uint16_t blockPos = 0;
	uint32_t pageCnt = 0;
	uint16_t expectedMsgId = data[1] + 1;

	while (1) {
		startApplicationOnTimeout();

		if(! hasData) {															// Wait for data
			continue;
		}
		
		hm_dec(data);

		hasData = 0;
		if (data[7] != hmID[0] || data[8] != hmID[1] || data[9] != hmID[2]) {
			#if DEBUG == 1
				uart_puts_P("Got data but not for us\n");
			#endif

			continue;
		}

		if (data[3] != 0xCA) {
			#if DEBUG == 1
				uart_puts_P("Got other msg type\n");
			#endif

			continue;
		}

		if (data[1] != expectedMsgId) {
			if (data[1] == expectedMsgId + 1 && pageCnt > 0) {

				/*
				 * The other side may have missed our ack. It will resend the last block
				 */
				expectedMsgId--;
				pageCnt--;
				state = 0;

				#if DEBUG == 1
					uart_puts_P("Retransmit. Will reflash!\n");
				#endif
			} else {
				state = 0;

				#if DEBUG == 1
					uart_puts_P("FATAL: Wrong msgId detected!\n");
				#endif
			}
		}

		if (state == 0) {
			blockLen = (data[10] << 8);											// Read len and check memory
			blockLen += data[11];
			if (blockLen != SPM_PAGESIZE) {
				#if DEBUG == 1
					uart_puts_P("Block length differ with page size\n");
				#endif

				state = 0;
				continue;
			}
			if (data[0]-11 > SPM_PAGESIZE) {
				#if DEBUG == 1
					uart_puts_P("Block to big\n");
				#endif

				state = 0;
				continue;
			}
			state = 1;
			blockPos = data[0]-11;
			memcpy(&blockData, &data[12], data[0]-11);

		} else {
			if (blockPos + data[0]-9 > blockLen) {
				#if DEBUG == 1
					uart_puts_P("Got more data than block length\n");
				#endif

				state = 0;
				continue;
			}
			memcpy(&blockData[blockPos], &data[10], data[0]-9);
			blockPos += data[0]-9;
		}
		
		if (data[2] == 0x20) {
			if (blockPos != blockLen) {
				#if DEBUG == 1
					uart_puts_P("Block length and block position do not match\n");
				#endif

				state = 0;
				continue;
			} else {
				#if DEBUG == 1
					uart_puts_P("Block complete!\n");
				#endif

				#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
					bitSet(PORT_STATUSLED, PIN_STATUSLED);						// Status-LED on
				#endif

				program_page(pageCnt * SPM_PAGESIZE, blockData);
				pageCnt++;
				expectedMsgId++;
				timeoutCounter = 0;

				#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
					bitClear(PORT_STATUSLED, PIN_STATUSLED);					// Status-LED off, we blinking
				#endif
			}

			#if DEBUG == 1
				//pHex(blockData, blockLen);
			#endif

			send_ack(flasher_hmid, data[1]);
			state = 0;
		}
	}
}

/**
 * Setup timer 0
 */
void setup_timer() {
	TCCR0B |= (1 << CS01) | (!(1 << CS00)) | (!(1 << CS02));					//PRESCALER 8
	TCNT0 = 0;
	TIMSK0 |= (1 << TOIE0);
}

/*
 * Setup interrupt
 * INT0 at GDO0 on CC1101, falling edge
 */
void setup_cc1100_interrupts() {
	EIMSK = 1 << INT0;															// Enable INT0
	EICRA = 1 << ISC01 | 0 << ISC00;											// falling edge
}

#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
	/*
	 * Let the led blinks 1 time
	 */
	void blinkLED() {
		bitSet(PORT_STATUSLED, PIN_STATUSLED);									// Status-LED on
		_delay_ms(25);
		bitClear(PORT_STATUSLED, PIN_STATUSLED);									// Status-LED on
		_delay_ms(200);
	}
#endif

int main() {
	MCUSR=0;																	// disable watchdog (used for software reset the device)
	wdt_reset();
	wdt_disable();

	#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
		bitSet(DDR_STATUSLED, PIN_STATUSLED);									// Set pin for LED as output

		blinkLED();																// Blink status led indicating bootloader
	#endif

	setup_interrupts_for_bootloader();											// map to correct interrupt table for bootloader
	setup_timer();																// setup timer for timeout counter
	setup_cc1100_interrupts();													// setup interrupts for cc1100

	#if DEBUG == 1
		// init uart
		uart_init( UART_BAUD_SELECT(BOOT_UART_BAUD_RATE,F_CPU) );
		uart_puts_P(VERSION_STRING);
	#endif

	// we must copy the adress data from program space first
	memcpy_P(&hmID, &hm_id[0], 3);
	memcpy_P(&hmSerial, &hm_serial[0], 10);

	switch_radio_to_10k_mode();													// go to standard 10k mode

	blinkLED();																	// Blink status led again after init done

	send_bootloader_sequence();													// send broadcast to allow windows tool or flash_ota to discover device

	wait_for_CB_msg();															// wait for msg in 10k mode to change to 100k mode

	switch_radio_to_100k_mode();												// switch to 100k mode

	wait_for_CB_msg();															// this is needed for windows tool

	flash_from_rf();															// run the actual flashing
}

/*
 * ISR for INT0 Interrupt
 */
ISR(INT0_vect) {
	cli();
	if (receiveData(data)) {
		hasData = 1;
	}
	sei();
}

/*
 * ISR for TIMER0 Interrupt
 */
ISR(TIMER0_OVF_vect) {
	TCNT0 = 0;
	timeoutCounter++;
}
