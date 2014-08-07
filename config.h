// Pin assignment for Atmega328p (Univarsalsensor)

// Pin assignment for the cc1101 module
#define PORT_SPI             PORTB
#define DDR_SPI              DDRB
#define PIN_SPI_SS           2
#define PIN_SPI_MOSI         3
#define PIN_SPI_MISO         4
#define PIN_SPI_SCK          5

// Pin assignment for status LED
#define PORT_GDO0            PORTD
#define DDR_GDO0             DDRD
#define PIN_GDO0             2

// Pin assignment for status LED
#define PORT_STATUSLED       PORTD
#define DDR_STATUSLED        DDRD
#define PIN_STATUSLED        4

// set to 1 to activate debug info over UART
#define DEBUG                1

// Enable CRC Check before application start. Requires firmware to include CRC checksum at the end.
#define CRC_FLASH            1

// adress of CRC checksum if CRC check is enabled. See below. Expect a CRC16 little endian
#define CODE_LEN             0x6FFE


/*****************************************
 *        Address data section           *
 * Stored at 0x7FF0 in boot loader space *
 *           See Makefile                *
 *****************************************/

#define USE_ADRESS_SECTION   1

// The model type
#define HM_TYPE              0xF1, 0x01    // DIY (HB-UW-Sen-THPL-I)			// stored at 0x7FF2

// 10 bytes serial number. Must be unique for each device
//#define HM_SERIAL            'H','B','0','D','e','f','a','u','l','t'			// stored at 0x7FF2
#define HM_SERIAL            'S','E','N','0','T','H','P','L','0','2'

#define HM_ID                0xAB, 0xCD, 0xEF									// stored at 0x7FFC
//#define HM_ID                0xAB, 0xCD, 0xEF
