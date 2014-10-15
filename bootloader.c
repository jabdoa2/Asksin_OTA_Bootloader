#include "bootloader.h"
#include "config.h"

#define VERSION_MAJOR     0
#define VERSION_MINOR     7
#define VERSION_PATCH     0

#define LED_VERION_CODE_1 150
#define LED_VERION_CODE_0 300

#if DEBUG > 0
	#define STR(x) #x
	#define VERSION_STRING_FUNC(v1, v2, v3)  "\nAskSin OTA Bootloader V" STR(v1) "." STR(v2) "." STR(v3) "\n\n"
	#define VERSION_STRING VERSION_STRING_FUNC (VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH)
#endif

#define BOOT_UART_BAUD_RATE  57600												// Baudrate for debugging

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
		pHex(buf, buf[0]+1);
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
		watchdogReset = bitRead(MCUSR, WDRF);									// is reset was triggered from watchdog?
	#endif
	
	MCUSR=0;																	// disable watchdog (used for software reset the device)
	wdt_reset();
	wdt_disable();

	#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
		bitSet(DDR_STATUSLED, PIN_STATUSLED);									// Set pin for LED as output

		#if SHOW_VERSION_AT_LED													// show blink sequence for bootloader version
			blinkLED(LED_VERION_CODE_0, LED_VERION_CODE_0, VERSION_MAJOR);
			_delay_ms(500);
			blinkLED(LED_VERION_CODE_1, LED_VERION_CODE_1, VERSION_MINOR);
			_delay_ms(500);
			blinkLED(LED_VERION_CODE_0, LED_VERION_CODE_0, VERSION_PATCH);
			_delay_ms(1000);
		#else
			blinkLED(25, 200, 1);												// Blink status led one time to show bootloader started
		#endif
	#endif

	setupInterrupts();

	#if DEBUG > 0
		uart_puts_P("Switch to 10k mode\n");
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

		/**
		 * Check if config button pressed after power on reset or a watchdog reset was triggered, then start bootloader. Else start application.
		 */
		if( bitRead(INPUT_CONFIG_BTN, PIN_CONFIG_BTN) && !watchdogReset) {		// check if button not pressed (button must be at high level)
			if (crc_app_ok()) {
				startApplication();												// then start Application
			}
		}
	#endif

	#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
		blinkLED(25, 200, 1);													// Blink status led again after init done
	#endif

	// we must copy the address data from program space first
	memcpy_P(&hmID, &hm_id[0], 3);
	memcpy_P(&hmSerial, &hm_serial[0], 10);

	sendBootloaderSequence();													// send broadcast to allow the ccu2, windows tool or flash_ota to discover device
	waitForCbMsg();																// wait for message in 10k mode to change to 100k mode

	#if DEBUG > 0
		uart_puts_P("Switch to 100k mode\n");
	#endif
	cc1101Init(CC1101_MODE_100k);												// Initialize cc1101 again and switch to 100k mode

	waitForCbMsg();																// wait again for CB message
	flashFromRF();															// run the actual flashing
}

/**
 * Initialize all needed interrupts
 */
void setupInterrupts() {
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
void programPage (uint32_t pageAddr, uint8_t *buf) {
	if (pageAddr > CODE_END+1 - SPM_PAGESIZE) {
		#if DEBUG > 0
			uart_puts_P("pageAddr exceeds flash memory\n");
		#endif

		return;
	}

	cli();																		// disable interrupts

	eeprom_busy_wait ();

	boot_page_erase (pageAddr);													// we must erase the page before
	boot_spm_busy_wait ();														// Wait until the memory is erased.

	for (uint16_t i = 0; i < SPM_PAGESIZE; i+= 2) {
		uint16_t word = *buf++;													// Set up little-endian word.
		word += (*buf++) << 8;

		boot_page_fill(pageAddr + i, word);
	}

	boot_page_write (pageAddr);													// Store buffer in flash page.
	boot_spm_busy_wait();														// Wait until the memory is written.

	/*
	 * Re-enable RWW-section again. We need this if we want to jump back
	 * to the application after bootloading.
	 */
	boot_rww_enable ();

	sei();																		// re-enable interrupts
}

/*
 * Check for if data was received and decode them.
 */
uint8_t hmCheckAndDecodeData() {
	if ( !hasData ) {
		// no data received jet
		return 0;
	}

	cli();

	uint8_t areDataForUs = 0;
	hasData = 0;

	data[0] = recData[0];
	data[1] = (~recData[1]) ^ 0x89;

	uint8_t i;
	for (i = 2; i < recData[0]; i++) {
		data[i] = (recData[i-1] + 0xDC) ^ recData[i];

		// Check if data for us
		if (i >= 7 && i <= 9) {
			areDataForUs = (data[i] == hmID[i-7]);
			if (!areDataForUs) {
				break;
			}
		}
	}
	data[i] = recData[i] ^ data[2];

	sei();

	if (areDataForUs) {
		#if DEBUG > 1
			debugData(data, 0);
		#endif

		return 1;

	} else {
		#if DEBUG > 1
			uart_puts_P("Data not for us\n");
		#endif

		// Data not for us
		return 0;
	}
}

/**
 * Encode data for sending and call sendData
 */
void hmEncodeAndSendData(uint8_t *msg) {
	msg[1] = (~msg[1]) ^ 0x89;
	uint8_t buf2 = msg[2];
	uint8_t prev = msg[1];

	uint8_t i;
	for (i=2; i<msg[0]; i++) {
		prev = (prev + 0xdc) ^ msg[i];
		msg[i] = prev;
	}

	msg[i] ^= buf2;

	cli();
	sendData(msg, 0);
	sei();
}

// CRC check related functions
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
 * Read through program memory for defined CODE_END-1 and calculate CRC.
 * Then compare with CRC stored at the end of CODE_END-1.
 */
uint8_t crc_app_ok(void) {
	uint16_t crc = 0xFFFF;
	for (uint16_t i=0; i < CODE_END-1; i++) {
		crc = updcrc(pgm_read_byte(i), crc);
	}
	// augment
	crc = updcrc(0, updcrc(0, crc));

	return (pgm_read_word(CODE_END-1) == crc);
}

/*
 * Check if CRC was ok.
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
		blinkLED(2000, 1, 1);													// blink led
	#endif

	#if DEBUG > 0
		uart_puts_P("CRC fail, Reboot\n");
	#endif

	wdt_reset();

	wdt_enable(WDTO_1S);
	while(1);																// wait for watchdog to generate reset
}

/**
 * Send response (ACK or NACK)
 * We will send ACK only if requested
 */
void sendResponse(uint8_t *msg, uint8_t type) {

	if (type == MSG_RESPONSE_TYPE_ACK) {
		if ( !(msg[2] & 0x20) ) {
			// no ACK required
			return;
		}

		#if DEBUG > 1
			uart_puts_P("TX ACK\n");
		#endif
	}

	uint8_t responseMsg[11] = { 10, msg[1], 0x80, 0x02, hmID[0], hmID[1], hmID[2], msg[4], msg[5], msg[6], type };
	hmEncodeAndSendData(responseMsg);
}

/**
 * Start the main application
 */
void startApplication() {
	void (*start)( void ) = 0x0000;												// Funktionspointer auf 0x0000

	/**
	 * Check if new Bootloader was flashed via OTA and is ready to be transfered into BL area.
	 */
	if(pgm_read_word(BOOTLOADER_START - 4) == MAGIC_WORD){
		#if DEBUG > 0
			uart_puts_P("Bootloader selfUpdate!\n");
			_delay_us(32000);
		#endif

		#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
			// blink led 10 times indicating bootloader update
			blinkLED(50, 50, 10);
		#endif

		updateBootloaderFromRWW();
	}

	#if DEBUG > 0
		uart_puts_P("Start App\n");
		_delay_us(32000);
	#endif

	/*
	 * deactivate used hardware and global deactivate interrupts. Because, this is no real reset.
	 */
	#if defined(PORT_CONFIG_BTN) && defined(DDR_CONFIG_BTN) && defined(INPUT_CONFIG_BTN) && (PIN_CONFIG_BTN)
		bitClear(PORT_CONFIG_BTN, PIN_CONFIG_BTN);								// reset pullup
	#endif

	#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
		bitClear(PORT_STATUSLED, PIN_STATUSLED);								// Status-LED off
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

		resetOnCRCFail();

		startApplication();
	}
}

/**
 * Send the bootloader sequence to broadcast to inform a waiting CCU or flash application.
 */
void sendBootloaderSequence() {
	#if DEBUG > 0
		uart_puts_P("TX bootloader sequence\n");
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

	hmEncodeAndSendData(msg);
}

/*
 * Wait for a CB message
 */
void waitForCbMsg() {
	#if DEBUG > 0
		uart_puts_P("Wait for CB msg\n");
	#endif

	timeoutCounter = 0;															// reset timeout
	while(1) {
		startApplicationOnTimeout();

		if ( !hmCheckAndDecodeData() ) {										// Wait for data and decode it
			continue;
		}

		// Wait for a CB message like: 0F 01 00 CB 11 22 33 AB CD EF 10 5B 11 F8 15 47
		if (data[3] == 0xCB) {
			#if DEBUG > 0
				uart_puts_P("Got CB msg\n");
			#endif

			sendResponse(data, MSG_RESPONSE_TYPE_ACK);
			break;

		} else {
			sendResponse(data, MSG_RESPONSE_TYPE_NACK);
		}
	}
}

/**
 * Here we retrieve the firmware data and flash it into the flash memory
 */
void flashFromRF() {
	uint8_t state = FLASH_STATE_BLOCK_NOT_STARTED;								// 0 = block has not started, 1 = block started
	uint8_t blockData[SPM_PAGESIZE];											// buffer to store the data of a whole memory page

	uint16_t blockPos  = 0;
	uint32_t pageCount = 0;

	uint8_t  previousMsgId = data[1];											// last message id

	timeoutCounter = 0;

	#if DEBUG > 0
		uart_puts_P("Receive firmware\n");
	#endif

	while (1) {
		startApplicationOnTimeout();

		if ( !hmCheckAndDecodeData() ) {										// Wait for data and decode it
			continue;
		}

		if (data[3] != 0xCA) {
			#if DEBUG > 0
				uart_puts_P("Got other msgType\n");
			#endif

			continue;
		}

		// here we check if current msgId > last msgId
		if (data[1] == previousMsgId && state == FLASH_STATE_BLOCK_NOT_STARTED) {
			if (pageCount > 0) {
				// The other side may have missed our ACK. It will re send the last block
				pageCount--;

				#if DEBUG > 0
					uart_puts_P("Retransmit, reflash!\n");
				#endif
			}
		}

		// current msgId sould > previousMsgId. Exception: previousMsgId 70-80 or previousMsgId > 245
		if ( (data[1] < previousMsgId && previousMsgId < 70) || (data[1] < previousMsgId && (previousMsgId > 80 && previousMsgId < 245)) ) {
			#if DEBUG > 0
				uart_puts_P("Wrong MsgId!\n");
			#endif

			state = FLASH_STATE_BLOCK_NOT_STARTED;
			continue;
		}

		if (state == FLASH_STATE_BLOCK_NOT_STARTED) {
			if ( ((data[10] << 8) + data[11]) != SPM_PAGESIZE) {				// check block size again SPM_PAGESIZE
				#if DEBUG > 0
					uart_puts_P("blockLen differ pageSize\n");
				#endif

				state = FLASH_STATE_BLOCK_NOT_STARTED;
				continue;
			}

			state = FLASH_STATE_BLOCK_STARTED;

			blockPos = data[0]-11;
			memcpy(&blockData, &data[12], blockPos);

		} else {
			if (blockPos + data[0]-9 > SPM_PAGESIZE) {
				#if DEBUG > 0
					uart_puts_P("To many data for pageSize\n");
				#endif

				state = FLASH_STATE_BLOCK_NOT_STARTED;
				continue;
			}

			memcpy(&blockData[blockPos], &data[10], data[0]-9);
			blockPos += data[0]-9;
		}

		if (state == FLASH_STATE_BLOCK_STARTED && data[2] == 0x20) {			// last data for current block
			if (blockPos != SPM_PAGESIZE) {
				#if DEBUG > 0
					uart_puts_P("pageSize and blockPos differ\n");
				#endif

				state = FLASH_STATE_BLOCK_NOT_STARTED;
				continue;

			} else {
				#if DEBUG > 0
					uart_puts_P(".");											// Block complete
				#endif

				#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
					bitSet(PORT_STATUSLED, PIN_STATUSLED);						// Status-LED on
				#endif

				// here we flash the page into memory
				programPage(pageCount * SPM_PAGESIZE, blockData);
				pageCount++;
				timeoutCounter = 0;

				#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
					bitClear(PORT_STATUSLED, PIN_STATUSLED);					// Status-LED off, we blinking
				#endif

//				#if DEBUG > 1
//					pHex(blockData, SPM_PAGESIZE);
//				#endif

				previousMsgId = data[1];
				sendResponse(data, MSG_RESPONSE_TYPE_ACK);

				state = FLASH_STATE_BLOCK_NOT_STARTED;
			}
		}
	}
}

// Status-LED relatedt functions
#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
	/*
	 * Let the led blinks count times
	 */
	void blinkLED(uint16_t onTime, uint16_t offTime, uint8_t count) {
		for (uint8_t i=0; i < count; i++){
			bitSet(PORT_STATUSLED, PIN_STATUSLED);								// Status-LED on
			for (uint16_t j=0; j < onTime; j++) {_delay_ms(1); }

			bitClear(PORT_STATUSLED, PIN_STATUSLED);							// Status-LED off
			for (uint16_t j=0; j < offTime; j++) {_delay_ms(1); }
		}
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


/**
 * The updateBootloaderFromRWW function is placed in the topmost page and cannot be changed via OTA Update
 * This must not be touched for OTA update otherwise you may brick the bootloader and need and ISP update
 */
void updateBootloaderFromRWW(void) __attribute__ ((section (".bootloaderUpdate")));

/**
 * Copy bootloader image from RWW section into NRWW section (except top page with this function)
 */
void updateBootloaderFromRWW(){
	cli();																		// make shure all interrups are off

	#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
		bitSet(PORT_STATUSLED, PIN_STATUSLED);									// Status-LED on for indicating bootloader update
	#endif

	for (uint8_t i=0; i < BOOTLOADER_PAGES; i++){
		uint32_t pageAddr = BOOTLOADER_START + (i * SPM_PAGESIZE);				// address of page to flash
		boot_page_erase (pageAddr);												// we must erase the page before
		boot_spm_busy_wait();													// Wait until the memory is erased.

		for(uint16_t j=0; j < SPM_PAGESIZE; j+=2){
			boot_page_fill(pageAddr+j, pgm_read_word((i * SPM_PAGESIZE) + j));	//wordbuf=pgm_read_word((i*SPM_PAGESIZE)+j);
		}

		boot_page_write(pageAddr);												// write new page
		boot_spm_busy_wait();													// Wait until the memory is written.
	}

	// Bootloader update complete. We delete the MAGIC_WORD at end of program section
	boot_page_erase(BOOTLOADER_START - SPM_PAGESIZE);
	boot_spm_busy_wait();														// Wait until the memory is erased.

	/*
	 * Re-enable RWW-section again. We need this if we want to jump back
	 * to the application after bootloading.
	 */
	boot_rww_enable();

	#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
		bitClear(PORT_STATUSLED, PIN_STATUSLED);								// Status-LED off
	#endif

	wdt_enable(WDTO_1S);
	while(1);																	// wait for watchdog to generate reset
}
