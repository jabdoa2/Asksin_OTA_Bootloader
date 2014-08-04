#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>


//const static uint8_t SS   = 4;
//const static uint8_t MOSI = 5;
//const static uint8_t MISO = 6;
//const static uint8_t SCK  = 7;

// Pin Assigment for Atmega328p (Univarsalsensor)
const static uint8_t SS   = 2;
const static uint8_t MOSI = 3;
const static uint8_t MISO = 4;
const static uint8_t SCK  = 5;

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define PORT_SPI_MISO            PINB
#define PORT_SPI_SS              PORTB
//#define BIT_SPI_MISO             6
//#define BIT_SPI_SS               4
//#define GDO0                     2

// Pin Assigment for Atmega328p (Univarsalsensor)
#define BIT_SPI_MISO             4
#define BIT_SPI_SS               2
#define GDO0                     10

#define CC1101_DATA_LEN			 60

// some register definitions for TRX868 communication
#define READ_SINGLE              0x80
#define READ_BURST               0xC0
#define WRITE_BURST              0x40										// type of transfers
	
#define CC1101_CONFIG            0x80										// type of register
#define CC1101_STATUS            0xC0
	
#define CC1101_PATABLE           0x3E										// PATABLE address
#define CC1101_TXFIFO            0x3F										// TX FIFO address
#define CC1101_RXFIFO            0x3F										// RX FIFO address

#define CC1101_SRES              0x30										// reset CC1101 chip
#define CC1101_SFSTXON           0x31										// enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). if in RX (with CCA): Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround).
#define CC1101_SXOFF             0x32										// turn off crystal oscillator
#define CC1101_SCAL              0x33										// calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
#define CC1101_SRX               0x34										// enable RX. perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1
#define CC1101_STX               0x35										// in IDLE state: enable TX. perform calibration first if MCSM0.FS_AUTOCAL=1. if in RX state and CCA is enabled: only go to TX if channel is clear
#define CC1101_SIDLE             0x36										// exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable
#define CC1101_SWOR              0x38										// start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if WORCTRL.RC_PD=0
#define CC1101_SPWD              0x39										// enter power down mode when CSn goes high
#define CC1101_SFRX              0x3A										// flush the RX FIFO buffer. only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC1101_SFTX              0x3B										// flush the TX FIFO buffer. only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC1101_SWORRST           0x3C										// reset real time clock to Event1 value
#define CC1101_SNOP              0x3D										// no operation. may be used to get access to the chip status byte

#define CC1101_PARTNUM           0x30										// status register, chip ID
#define CC1101_VERSION           0x31										// chip ID
#define CC1101_FREQEST           0x32										// frequency offset estimate from demodulator
#define CC1101_LQI               0x33										// demodulator estimate for Link Quality
#define CC1101_RSSI              0x34										// received signal strength indication
#define CC1101_MARcurStatTE         0x35										// main radio control state machine state
#define CC1101_WORTIME1          0x36										// high byte of WOR Time
#define CC1101_WORTIME0          0x37										// low byte of WOR Time
#define CC1101_PKTSTATUS         0x38										// current GDOx status and packet status
#define CC1101_VCO_VC_DAC        0x39										// current setting from PLL calibration module
#define CC1101_TXBYTES           0x3A										// underflow and number of bytes
#define CC1101_RXBYTES           0x3B										// overflow and number of bytes
#define CC1101_RCCTRL1_STATUS    0x3C										// last RC oscillator calibration result
#define CC1101_RCCTRL0_STATUS    0x3D										// last RC oscillator calibration result

#define MARcurStatTE_SLEEP          0x00
#define MARcurStatTE_IDLE           0x01
#define MARcurStatTE_XOFF           0x02
#define MARcurStatTE_VCOON_MC       0x03
#define MARcurStatTE_REGON_MC       0x04
#define MARcurStatTE_MANCAL         0x05
#define MARcurStatTE_VCOON          0x06
#define MARcurStatTE_REGON          0x07
#define MARcurStatTE_STARTCAL       0x08
#define MARcurStatTE_BWBOOST        0x09
#define MARcurStatTE_FS_LOCK        0x0A
#define MARcurStatTE_IFADCON        0x0B
#define MARcurStatTE_ENDCAL         0x0C
#define MARcurStatTE_RX             0x0D
#define MARcurStatTE_RX_END         0x0E
#define MARcurStatTE_RX_RST         0x0F
#define MARcurStatTE_TXRX_SWITCH    0x10
#define MARcurStatTE_RXFIFO_OFLOW   0x11
#define MARcurStatTE_FSTXON         0x12
#define MARcurStatTE_TX             0x13
#define MARcurStatTE_TX_END         0x14
#define MARcurStatTE_RXTX_SWITCH    0x15
#define MARcurStatTE_TXFIFO_UFLOW   0x16


#define PA_LowPower              0x03										// PATABLE values
#define PA_Normal                0x50										// PATABLE values
#define PA_MaxPower			     0xC0

// some macros for TRX868 communication
#define wait_Miso()       while(bitRead(PORT_SPI_MISO, BIT_SPI_MISO))		// wait until SPI MISO line goes low
#define cc1101_Select()   bitClear(PORT_SPI_SS, BIT_SPI_SS)					// select (SPI) CC1101
#define cc1101_Deselect() bitSet(PORT_SPI_SS, BIT_SPI_SS)					// deselect (SPI) CC1101

struct s_trx868 {															// TRX868 communication variables
	uint8_t rfState;														// RF state
	uint8_t crc_ok;															// CRC OK for received message
	uint8_t rssi;															// signal strength
	uint8_t lqi;															// link quality
}  trx868;

void init(uint8_t mode100k);
void sendData(uint8_t *buf, uint8_t burst);
uint8_t receiveData(uint8_t *buf);
uint8_t detectBurst(void);
void setPowerDownxtStatte();
uint8_t monitorStatus();
uint8_t sendSPI(uint8_t val);
void cmdStrobe(uint8_t cmd);
void writeBurst(uint8_t regAddr, uint8_t *buf, uint8_t len);
void readBurst(uint8_t *buf, uint8_t regAddr, uint8_t len);
uint8_t readReg(uint8_t regAddr, uint8_t regType);
void writeReg(uint8_t regAddr, uint8_t val);
