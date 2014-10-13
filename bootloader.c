#include "bootloader.h"
#include "config.h"

#if DEBUG > 0
	#define VERSION_STRING       "\nAskSin OTA Bootloader V0.6.2\n\n"			// version number for debug info
#endif

#if DEBUG > 1
	#define BOOT_UART_BAUD_RATE  115200			// Baudrate
#else
	#define BOOT_UART_BAUD_RATE  57600			// Baudrate
#endif

#ifndef WAIT_FOR_CONFIG
	#define WAIT_FOR_CONFIG = 10
#endif

const char dataNotForUs[] = "Data not for us\n";

/*****************************************
 *        Address data section           *
 *           See Makefile                *
 *****************************************/
#define ADDRESS_SECTION_TYPE   __attribute__ ((section (".addressDataType")))
#define ADDRESS_SECTION_SERIAL __attribute__ ((section (".addressDataSerial")))
#define ADDRESS_SECTION_ID     __attribute__ ((section (".addressDataId")))
const uint8_t hm_Type[2]        ADDRESS_SECTION_TYPE   = {HM_TYPE};				// 2 bytes device type
const uint8_t hm_serial[10]     ADDRESS_SECTION_SERIAL = {HM_SERIAL};			// 10 bytes serial number
const uint8_t hm_id[3]          ADDRESS_SECTION_ID     = {HM_ID};				// 3 bytes device address

uint8_t expectedMsgIdOffset;

#if DEBUG > 1
	void pHex(const uint8_t *buf, uint8_t len) {
		const char hexDigits[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

		for (uint8_t i=0; i < len; i++) {
			uart_putc(hexDigits[buf[i] >> 4]);
			uart_putc(hexDigits[buf[i] & 0xF]);
			uart_puts_P(" ");
		}
	}

	void debugData(const uint8_t *buf, uint8_t tx) {
		if (tx) {
			uart_puts("TX: ");
		} else {
			uart_puts("RX: ");
		}
		pHex(buf, buf[0]);
		uart_puts_P("\n");
	}
#endif

/**
 * The bootloader main methode.
 * Here we initialize all required stuff.
 */
int main() {
	#if defined(PORT_CONFIG_BTN) && defined(DDR_CONFIG_BTN) && defined (INPUT_CONFIG_BTN) && defined(PIN_CONFIG_BTN)
		uint8_t watchdogReset = 0;
		watchdogReset = bitRead(MCUSR, WDRF);									// is reset triggert from watchdog?
	#endif
	
	MCUSR=0;																	// disable watchdog (used for software reset the device)
	wdt_reset();
	wdt_disable();

	#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
		bitSet(DDR_STATUSLED, PIN_STATUSLED);									// Set pin for LED as output
		blinkLED(25, 200);														// Blink status led indicating bootloader
	#endif

	setup_interrupts();

	#if DEBUG > 0
		uart_puts_P("Sw to 10k\n");
	#endif
	cc1101Init(CC1101_MODE_10k);												// Initialize cc1101 and go to standard 10k mode.

	#if DEBUG > 0
		uart_init( UART_BAUD_SELECT(BOOT_UART_BAUD_RATE,F_CPU) );				// init uart
		uart_puts_P(VERSION_STRING);
	#endif

	#if defined(PORT_CONFIG_BTN) && defined(DDR_CONFIG_BTN) && defined (INPUT_CONFIG_BTN) && defined(PIN_CONFIG_BTN)
		bitClear(DDR_CONFIG_BTN, PIN_CONFIG_BTN);								// Set pin for config button as input
		bitSet(PORT_CONFIG_BTN, PIN_CONFIG_BTN);								// set pullup
		_delay_us(10);

		uint8_t crcOk = 1;
		#if CRC_FLASH == 1
			crcOk = crc_app_ok();
		#endif

		/**
		 * Check if config button pressed after power on reset or a watchdog reset was triggert, then start bootloader. Else start application.
		 */
		if( bitRead(INPUT_CONFIG_BTN, PIN_CONFIG_BTN) && !watchdogReset) {		// check if button not pressed (button must be at high level)

			#if CRC_FLASH == 1
				if (crcOk) {
					startApplication();											// then start Application
				}
			#else
				startApplication();												// then start Application
			#endif
		}
	#endif

	#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
		blinkLED(25, 200);														// Blink status led again after init done
	#endif

	// we must copy the address data from program space first
	memcpy_P(&hmID, &hm_id[0], 3);
	memcpy_P(&hmSerial, &hm_serial[0], 10);

	send_bootloader_sequence();													// send broadcast to allow the ccu2, windows tool or flash_ota to discover device
	wait_for_CB_msg();															// wait for message in 10k mode to change to 100k mode

	#if DEBUG > 0
		uart_puts_P("Sw to 100k\n");
	#endif
	cc1101Init(CC1101_MODE_100k);												// Initialize cc1101 again and switch to 100k mode

	wait_for_CB_msg();															// wait again for CB message
	flash_from_rf();															// run the actual flashing
}

/**
 * Initialize all needed interrupts
 */
void setup_interrupts() {
	/**
	 * Setup interrupts for bootloder
	 * map to correct interrupt table for bootloader
	 * (Interrupt Vektoren auf Bootloader-Bereich verbiegen)
	 */
	char sregtemp = SREG;

	cli();
	uint8_t temp = MCUCR;
	MCUCR = temp | (1<<IVCE);													// Enable change of Interrupt Vectors
	MCUCR = temp | (1<<IVSEL);													// Move interrupts to bootloader section

	SREG = sregtemp;

	/**
	 * Setup timer 0 for timeout counter
	 */
	TCCR0B |= (1 << CS01) | (!(1 << CS00)) | (!(1 << CS02));					//PRESCALER 8
	TCNT0 = 0;
	TIMSK0 |= (1 << TOIE0);

	/**
	 * Setup interrupt for INT0 at GDO0 on CC1101, falling edge.
	 */
	EIMSK = 1 << INT0;															// Enable INT0
	EICRA = 1 << ISC01 | 0 << ISC00;											// falling edge
}

/**
 * Program a page of the program memory
 */
void program_page (uint32_t page, uint8_t *buf) {
	uint16_t i;

	cli();																		// disable interrupts

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

	sei();																		// re-enable interrupts
}

/**
 * Encode data for sending
 */
void hmEncode(uint8_t *buffer) {
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

/*
 * Decode data after receiving.
 */
void hmDecode(uint8_t *buffer) {
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

/**
 * Do encode and trigger send data
 */
void hmSendData(uint8_t *msg) {
	hmEncode(msg);
	cli();
	sendData(msg, 0);
	sei();
}

void send_ack(uint8_t *receiver, uint8_t messageId) {
	uint8_t ack_msg[11] = { 10, messageId, 0x80, 0x02, hmID[0], hmID[1], hmID[2], receiver[0], receiver[1], receiver[2], 0x00};
	hmSendData(ack_msg);
}

void send_nack_to_msg(uint8_t *msg) {
	uint8_t nack_msg[11] = { 10, msg[1], 0x80, 0x02, hmID[0], hmID[1], hmID[2], msg[4], msg[5], msg[6], 0x80};
	hmSendData(nack_msg);
}

void send_ack_if_requested(uint8_t* msg) {
	if (! (msg[2] & (1 << 5))) {
		// no ack requested
		return;
	}
	
	#if DEBUG > 0
		uart_puts_P("TX: ACK\n");
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
			#if DEBUG > 0
				uart_puts_P("CRC OK\n");
			#endif

			return;
		}

		// At this point we have a CRC failure. We reboot the device
		#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
			blinkLED(2000, 1);													// blink led
		#endif

		wdt_reset();

		#if DEBUG > 0
			uart_puts_P("CRC fail: Boot\n");
		#endif

		wdt_enable(WDTO_1S);
		while(1);																// wait for Watchdog to generate reset
	}
#endif 

/**
 * Start the main application
 */
void startApplication() {
	void (*start)( void ) = 0x0000;												// Funktionspointer auf 0x0000

	#if DEBUG > 0
		uart_puts_P("Start App\n");
		_delay_ms(250);
	#endif

	/*
	 * deactivate used hardware and global deactivate interrupts. Because, this is no real reset.
	 */
	#if defined(PORT_CONFIG_BTN) && defined(DDR_CONFIG_BTN) && defined(INPUT_CONFIG_BTN) && (PIN_CONFIG_BTN)
		bitClear(PORT_CONFIG_BTN, PIN_CONFIG_BTN);								// reset pullup
	#endif


	// Restore interrupt vectors
	cli();
	uint8_t temp = MCUCR;
	MCUCR = temp | (1 << IVCE);													// Enable change of Interrupt Vectors
	MCUCR = temp & ~(1 << IVSEL);												// Move interrupts to application section

	start();																	// Rücksprung zur Adresse 0x0000
}

/**
 * Check for timeout.
 * If timeout reached, start the main application
 */
void startApplicationOnTimeout() {
	if (timeoutCounter > 30000) {												// wait about 10s at 8Mhz
		#if DEBUG > 0
			uart_puts_P("Timeout\n");
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

/**
 * Send the bootloader sequence to broadcast to inform a waiting CCU or flash application.
 */
void send_bootloader_sequence() {
	#if DEBUG > 0
		uart_puts_P("TX: bootloader sequence\n");
	#endif

	/*
	 *                                     SourceId TargetId            HM_SERIAL
	 *                                    |--------|--------|  |-----------------------------|
	 * Send this message like: 14 00 00 10 AB CD EF 00 00 00 00 41 42 43 44 45 46 47 48 49 50
	 */
	uint8_t msg[21] = {
		0x14, 0x00, 0x00, 0x10, hmID[0], hmID[1], hmID[2], 0x00, 0x00, 0x00, 0x00,
		hmSerial[0], hmSerial[1], hmSerial[2], hmSerial[3], hmSerial[4],
		hmSerial[5], hmSerial[6], hmSerial[7], hmSerial[8], hmSerial[9]
	};

	#if DEBUG > 1
		debugData(msg, 1);
	#endif

	send_hm_data(msg);
}

/*
 * Wait for a CB message
 */
void wait_for_CB_msg() {
	#if DEBUG > 0
		uart_puts_P("Wait for CB Msg\n");
	#endif

	timeoutCounter = 0;															// reset timeout
	while(1) {
		startApplicationOnTimeout();

		if( !hasData ) {														// Wait for data
			continue;
		}
		

		hmDecode(data);
		#if DEBUG > 1
			debugData(data, 0);
		#endif

		hasData = 0;
		if (data[7] != hmID[0] || data[8] != hmID[1] || data[9] != hmID[2]) {
			#if DEBUG > 0
				uart_puts_P("Got data but not for us\n");
			#endif

			continue;
		}

		// Wait for a CB message like: 0F 01 00 CB 11 22 33 AB CD EF 10 5B 11 F8 15 47
		if (data[3] == 0xCB) {
			#if DEBUG > 0
				uart_puts_P("Got msg to start config\n");
			#endif

			if (data[0] >= 17) {
				// spechial for update with ccu
				expectedMsgIdOffset = data[17];
			}

			flasher_hmid[0] = data[4];
			flasher_hmid[1] = data[5];
			flasher_hmid[2] = data[6];
			send_ack_if_requested(data);
			break;
		}

		send_nack_to_msg(data);
	}
}

/**
 * Here we retrieve the firmware data and flash it into the flash memory
 */
void flash_from_rf() {
	uint8_t state = 0;															// 0 = block has not started, 1 = block started
	uint8_t blockData[SPM_PAGESIZE];											// buffer to store the data of a whole memory page
	uint16_t blockLen = 0;
	uint16_t blockPos = 0;
	uint32_t pageCnt = 0;

	timeoutCounter = 0;
	uint8_t expectedMsgId = data[1] + expectedMsgIdOffset + 1;

	#if DEBUG > 0
		uart_puts_P("Start receive firmware\n");
	#endif

	while (1) {
		startApplicationOnTimeout();

		if(! hasData ) {														// Wait for data
			continue;
		}
		
		hmDecode(data);

		hasData = 0;
		if (data[7] != hmID[0] || data[8] != hmID[1] || data[9] != hmID[2]) {
			#if DEBUG > 0
				uart_puts_p(dataNotForUs);
			#endif

			continue;
		}

		if (data[3] != 0xCA) {
			#if DEBUG > 0
				uart_puts_P("Got other message type\n");
			#endif

			continue;
		}

		#if DEBUG > 1
			debugData(data, 0);
		#endif

		if (data[1] != expectedMsgId) {
			if (data[1] == expectedMsgId + 1 && pageCnt > 0) {
				/*
				 * The other side may have missed our ACK. It will re send the last block
				 */
				expectedMsgId--;
				pageCnt--;
				state = 0;

				#if DEBUG > 0
					uart_puts_P("Retransmit. We will re flash!\n");
				#endif

			} else {
				state = 0;

				#if DEBUG > 0
					uart_puts_P("FATAL: Wrong msgId detected\n");
				#endif
			}
		}

		if (state == 0) {
			blockLen = (data[10] << 8);											// Read block length and check memory
			blockLen += data[11];
			if (blockLen != SPM_PAGESIZE) {
				#if DEBUG > 0
					uart_puts_P("blockLen != pageSize\n");
				#endif

				state = 0;
				continue;
			}
			if (data[0]-11 > SPM_PAGESIZE) {
				#if DEBUG > 0
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
				#if DEBUG > 0
					uart_puts_P("Data size > blockLen\n");
				#endif

				state = 0;
				continue;
			}

			memcpy(&blockData[blockPos], &data[10], data[0]-9);
			blockPos += data[0]-9;
		}
		
		if (data[2] == 0x20) {													// last data for current block
			if (blockPos != blockLen) {
				#if DEBUG > 0
					uart_puts_P("blockLen and blockPos not match\n");
				#endif

				state = 0;
				continue;

			} else {
				#if DEBUG > 0
					uart_puts_P(".");											// Block complete
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

			//#if DEBUG > 1
			//	pHex(blockData, blockLen);
			//#endif

			send_ack(flasher_hmid, data[1]);

			state = 0;
		}

		if (state == 0) {
			expectedMsgId = data[1] + expectedMsgIdOffset + 1;
			
			/**
			 * Spechial for update with CCU
			 * The message counter overflows at 0x80 if the CCU deliver the update
			 */
			if (expectedMsgIdOffset > 1 && expectedMsgId >= 0x80) {
				expectedMsgId = expectedMsgId ^ 0x80;
			}
		}
	}
}

// Status-LED relatedt functions
#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
	/*
	 * Let the led blinks 1 time
	 */
	void blinkLED(uint16_t onTime, uint16_t offTime) {
		bitSet(PORT_STATUSLED, PIN_STATUSLED);									// Status-LED on
		while(onTime > 0) {_delay_ms(1); onTime--; }

		bitClear(PORT_STATUSLED, PIN_STATUSLED);								// Status-LED off
		while(offTime > 0) {_delay_ms(1); offTime--; }
	}
#endif

/*
 * ISR for INT0 Interrupt
 */
ISR(INT0_vect) {
	cli();
	
	if (receiveData(recData)) {
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
