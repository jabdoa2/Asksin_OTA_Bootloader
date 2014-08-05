#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <util/delay.h>
#include "uart/uart.h"
#include <string.h>
#include "cc.h"

uint8_t data[60];
uint8_t hasData = 0;
uint8_t hmid[3] = {0xAB, 0xCD, 0xEF};
uint8_t flasher_hmid[3];
uint16_t timeoutCounter = 0;
