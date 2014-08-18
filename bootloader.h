#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <util/delay.h>
#include "uart/uart.h"
#include <string.h>
#include "cc.h"
#include "config.h"

uint8_t hmID[3];
uint8_t hmSerial[10];

uint8_t data[60];
uint8_t hasData = 0;
uint8_t flasher_hmid[3];
uint16_t timeoutCounter = 0;

int main();
void setup_interrupts();
void program_page (uint32_t page, uint8_t *buf);
void hm_enc(uint8_t *buffer);
void hm_dec(uint8_t *buffer);
void send_hm_data(uint8_t *msg);
void send_ack(uint8_t *receiver, uint8_t messageId);
void send_nack_to_msg(uint8_t *msg);
void send_ack_if_requested(uint8_t* msg);
void startApplication();
void startApplicationOnTimeout();
void send_bootloader_sequence();
void wait_for_CB_msg();
void switch_radio_to_100k_mode();
void switch_radio_to_10k_mode();
void flash_from_rf();

ISR(INT0_vect);
ISR(TIMER0_OVF_vect);

#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
	void blinkLED(uint16_t onTime, uint16_t offTime);
#endif

#if DEBUG > 1
	void pHexChar(const uint8_t val);
	void pHex(const uint8_t *buf, uint8_t len);
#endif

#if CRC_FLASH == 1
	static uint16_t updcrc(uint8_t c, uint16_t crc);
	uint8_t crc_app_ok(void);
	void resetOnCRCFail();
#endif
