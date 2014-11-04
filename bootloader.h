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

uint8_t recData[60];
uint8_t data[60];																// copy of received data

uint8_t hasData = 0;															// flag indicate if we received data for our address

uint16_t timeoutCounter = 0;

#define MSG_RESPONSE_TYPE_ACK         0x00
#define MSG_RESPONSE_TYPE_NACK        0x80

#define FLASH_STATE_BLOCK_NOT_STARTED 0x00
#define FLASH_STATE_BLOCK_STARTED     0x01

#define MAGIC_WORD                    0x4711									// magic word for bootloader self update

int main();
void setupInterrupts();
void programPage (uint32_t pageAddr, uint8_t *buf);
void hmEncode(uint8_t *buffer);
uint8_t hmCheckAndDecodeData();

void hmEncodeAndSendData(uint8_t *msg);
void sendResponse(uint8_t *msg, uint8_t type);
void startApplication();
void startApplicationOnTimeout();
void sendBootloaderSequence();
void waitForCbMsg();
void switch_radio_to_100k_mode();
void switch_radio_to_10k_mode();
void flashFromRF();

// CRC check related functions
static uint16_t updcrc(uint8_t c, uint16_t crc);
uint8_t crc_app_ok(void);
void resetOnCRCFail();

void updateBootloaderFromRWW();

ISR(INT0_vect);
ISR(TIMER0_OVF_vect);

#if defined(PORT_STATUSLED) && defined(PIN_STATUSLED) && defined(DDR_STATUSLED)
	void blinkLED(uint16_t onTime, uint16_t offTime, uint8_t count);
#endif

#if DEBUG > 1
	void pHexChar(const uint8_t val);
	void pHex(const uint8_t *buf, uint8_t len);

	void debugData(const uint8_t *buf, uint8_t dir);
#endif
