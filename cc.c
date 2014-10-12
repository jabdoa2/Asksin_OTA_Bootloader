#include "cc.h"

// define initialize settings for cc1101 with 10k mode
const PROGMEM const uint8_t initVal[] = {
	CC1101_IOCFG2,   0x2E,														// non inverted GDO2, high impedance tri state
	CC1101_IOCFG1,   0x2E,														// low output drive strength, non inverted GD=1, high impedance tri state
	CC1101_IOCFG0,   0x06,														// disable temperature sensor, non inverted GDO0, asserts when a sync word has been sent/received, and de-asserts at the end of the packet. in RX, the pin will also de-assert when a package is discarded due to address or maximum length filtering
	CC1101_FIFOTHR,  0x0D,														// 0 ADC retention, 0 close in RX, TX FIFO = 9 / RX FIFO = 56 byte
	CC1101_SYNC1,    0xE9,														// Sync word
	CC1101_SYNC0,    0xCA,
	CC1101_PKTLEN,   0x3D,														// packet length 61
	CC1101_PKTCTRL1, 0x0C,														// PQT = 0, CRC auto flush = 1, append status = 1, no address check
	CC1101_FSCTRL1,  0x06,														// frequency synthesizer control

	// 868.299866 MHz
	/*
	CC1101_FREQ2,    0x21,
	CC1101_FREQ1,    0x65,
	CC1101_FREQ0,    0x6A,
	*/

	// 868.2895508  (better wireles connection)
	CC1101_FREQ2,    0x21,
	CC1101_FREQ1,    0x65,
	CC1101_FREQ0,    0x50,

	CC1101_MDMCFG4,  0xC8,
	CC1101_MDMCFG3,  0x93,
	CC1101_MDMCFG2,  0x03,
	CC1101_DEVIATN,  0x34,
	CC1101_MCSM2,    0x01,
	CC1101_MCSM1,    0x30,
	CC1101_MCSM0,    0x18,
	CC1101_FOCCFG,   0x16,
	CC1101_AGCCTRL2, 0x43,
	CC1101_FREND1,   0x56,
	CC1101_FSCAL1,   0x00,
	CC1101_FSCAL0,   0x11,
	CC1101_TEST1,    0x35,
	CC1101_PATABLE,  0xC3,
};

// initialize settings for cc1101 with 100k mode
const uint8_t PROGMEM initValUpdate[] = {
	CC1101_FSCTRL1,  0x08,
	CC1101_MDMCFG4,  0x5B,
	CC1101_MDMCFG3,  0xF8,
	CC1101_DEVIATN,  0x47,
	CC1101_FOCCFG,   0x1D,
	CC1101_BSCFG,    0x1C,
	CC1101_AGCCTRL2, 0xC7,
	CC1101_AGCCTRL1, 0x00,
	CC1101_AGCCTRL0, 0xB2,
	CC1101_FREND1,   0xB6,
	CC1101_FSCAL3,   0xEA,
};

/*
 * initialize CC1101
 */
void cc1101Init(uint8_t mode100k) {
	cli();

	bitSet(DDR_SPI,    PIN_SPI_SS);												// set B2(SS) as Output
	bitSet(DDR_SPI,    PIN_SPI_MOSI);											// set B3(MOSI) as Output
	bitClear(DDR_SPI,  PIN_SPI_MISO);											// set B4(MISO) as Input
	bitSet(DDR_SPI,    PIN_SPI_SCK);											// set B5(SCK) as Output

	bitClear(DDR_GDO0, PIN_GDO0);												// set B2(SS) as Input

	bitSet(PORT_SPI,   PIN_SPI_SS);												// set SS high
	bitSet(PORT_SPI,   PIN_SPI_SCK);											// set SCK high
	bitClear(PORT_SPI, PIN_SPI_MOSI);											// set MOSI high

	SPCR = _BV(SPE) | _BV(MSTR);												// SPI speed = CLK/4

	cc1101_Deselect();															// some deselect and selects to initialize the TRX868modul
	_delay_us(30);

	cc1101_Select();	
	_delay_us(30);

	cc1101_Deselect();
	_delay_us(45);

	cmdStrobe(CC1101_SRES);														// send reset
	_delay_us(100);

	for (uint8_t i=0; i<sizeof(initVal); i += 2) {								// write initialize value to cc1101
		writeReg(pgm_read_byte(&initVal[i]), pgm_read_byte(&initVal[i+1]));	
	}

	if (mode100k) {																// switch to 100k mode
		for (uint8_t i=0; i<sizeof(initValUpdate); i += 2) {					// write initialize value to cc1101
			writeReg(
				pgm_read_byte(&initValUpdate[i]),
				pgm_read_byte(&initValUpdate[i+1])
			);
		}
	}

	cmdStrobe(CC1101_SCAL);														// calibrate frequency synthesizer and turn it off
	_delay_ms(4);

	do {
		cmdStrobe(CC1101_SRX);
	} while (readReg(CC1101_MARCSTATE, CC1101_STATUS) != 0x0D);
	
	writeReg(CC1101_PATABLE, PA_MaxPower);										// configure PATABLE
	cmdStrobe(CC1101_SRX);														// flush the RX buffer
	cmdStrobe(CC1101_SWORRST);													// reset real time clock

	_delay_ms(3);

	sei();
}

void sendData(uint8_t *buf, uint8_t burst) {									// send data packet via RF

	/**
	 * Going from RX to TX does not work if there was a reception less than 0.5
	 * seconds ago. Due to CCA? Using IDLE helps to shorten this period(?)
	 * ccStrobe(CC1100_SIDLE);
	 * uint8_t cnt = 0xff;
	 * while(cnt-- && (ccStrobe( CC1100_STX ) & 0x70) != 2)
	 * my_delay_us(10);
	 */
 	cmdStrobe(CC1101_SIDLE);													// go to idle mode
	cmdStrobe(CC1101_SFRX );													// flush RX buffer
	cmdStrobe(CC1101_SFTX );													// flush TX buffer
	
	_delay_ms(1);																// wait a short time to set TX mode

	writeBurst(CC1101_TXFIFO, buf, buf[0]+1);									// write in TX FIFO

	cmdStrobe(CC1101_SFRX);														// flush the RX buffer
	cmdStrobe(CC1101_STX);														// send a burst

	for(uint8_t i=0; i< 200;++i) {												// after sending out all bytes the chip should go automatically in RX mode
		if( readReg(CC1101_MARCSTATE, CC1101_STATUS) == CC1101_MARCSTATE_RX)
			break;																//now in RX mode, good
		if( readReg(CC1101_MARCSTATE, CC1101_STATUS) != CC1101_MARCSTATE_TX) {
			break;																//neither in RX nor TX, probably some error
		}

		_delay_us(10);
	}
}

uint8_t receiveData(uint8_t *buf) {												// read data packet from RX FIFO
	uint8_t rxBytes = readReg(CC1101_RXBYTES, CC1101_STATUS);					// how many bytes are in the buffer

	if ((rxBytes & 0x7F) && !(rxBytes & 0x80)) {								// any byte waiting to be read and no overflow?
		buf[0] = readReg(CC1101_RXFIFO, CC1101_CONFIG);							// read data length
		
		if (buf[0] > CC1101_DATA_LEN)											// if packet is too long
			buf[0] = 0;															// discard packet
		else {
			readBurst(&buf[1], CC1101_RXFIFO, buf[0]);							// read data packet
			readReg(CC1101_RXFIFO, CC1101_CONFIG);								// read RSSI
			
			uint8_t val   = readReg(CC1101_RXFIFO, CC1101_CONFIG);				// read LQI and CRC_OK
			trx868.lqi    = val & 0x7F;
			trx868.crc_ok = bitRead(val, 7);
		}
	} else buf[0] = 0;															// nothing to do, or overflow

	cmdStrobe(CC1101_SFRX);														// flush Rx FIFO
	cmdStrobe(CC1101_SIDLE);													// enter IDLE state
	cmdStrobe(CC1101_SRX);														// back to RX state
	cmdStrobe(CC1101_SWORRST);													// reset real time clock
	
	return buf[0];																// return the data buffer
}

uint8_t sendSPI(uint8_t val) {													// send byte via SPI
	SPDR = val;																	// transfer byte via SPI
	while(!(SPSR & _BV(SPIF)));													// wait until SPI operation is terminated
	return SPDR;
}

void cmdStrobe(uint8_t cmd) {													// send command strobe to the CC1101 IC via SPI
	cc1101_Select();															// select CC1101
	wait_Miso();																// wait until MISO goes low
	sendSPI(cmd);																// send strobe command
	cc1101_Deselect();															// deselect CC1101
}

void writeBurst(uint8_t regAddr, uint8_t *buf, uint8_t len) {					// write multiple registers into the CC1101 IC via SPI
	cc1101_Select();															// select CC1101
	wait_Miso();																// wait until MISO goes low
	sendSPI(regAddr | WRITE_BURST);												// send register address
	for(uint8_t i=0 ; i<len ; i++) sendSPI(buf[i]);								// send value
	cc1101_Deselect();															// deselect CC1101
}

void readBurst(uint8_t *buf, uint8_t regAddr, uint8_t len) {					// read burst data from CC1101 via SPI
	cc1101_Select();															// select CC1101
	wait_Miso();																// wait until MISO goes low
	sendSPI(regAddr | READ_BURST);												// send register address
	for(uint8_t i=0 ; i<len ; i++) buf[i] = sendSPI(0x00);						// read result byte by byte
	cc1101_Deselect();															// deselect CC1101
}

uint8_t readReg(uint8_t regAddr, uint8_t regType) {								// read CC1101 register via SPI
	cc1101_Select();															// select CC1101
	wait_Miso();																// wait until MISO goes low
	sendSPI(regAddr | regType);													// send register address
	uint8_t val = sendSPI(0x00);												// read result
	cc1101_Deselect();															// deselect CC1101
	return val;
}

void writeReg(uint8_t regAddr, uint8_t val) {									// write single register into the CC1101 IC via SPI
	cc1101_Select();															// select CC1101
	wait_Miso();																// wait until MISO goes low
	sendSPI(regAddr);															// send register address
	sendSPI(val);																// send value
	cc1101_Deselect();															// deselect CC1101
}

