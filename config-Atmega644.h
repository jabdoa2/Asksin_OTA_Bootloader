// Pin assignment for Atmega328p (Univarsalsensor)

// Pin assignment for the cc1101 module
#define PORT_SPI             PORTB
#define DDR_SPI              DDRB
#define PIN_SPI_SS           4
#define PIN_SPI_MOSI         5
#define PIN_SPI_MISO         6
#define PIN_SPI_SCK          7

// Pin assignment for status LED
#define PORT_GDO0            PORTD
#define DDR_GDO0             DDRD
#define PIN_GDO0             2

// Pin assignment for status LED
#define PORT_STATUSLED       PORTB
#define DDR_STATUSLED        DDRB
#define PIN_STATUSLED        0

// set to 1 to activate debug info over UART
#define DEBUG                1

// Enable CRC Check before application start. Requires firmware to include CRC checksum at the end.
#define CRC_FLASH            0

// adress of CRC checksum if CRC check is enabled. See below. Expect a CRC16 little endian
#define CODE_LEN             0xDEED

/*****************************************
 *        Address data section           *
 * Stored at 0x7FF0 in boot loader space *
 *           See Makefile                *
 *****************************************/

// The model type (not used from bootloader)
#define HM_TYPE              0x12, 0x34

// 10 bytes serial number. Must be unique for each device
#define HM_SERIAL            'K', 'E', 'Q', '0', '0', '0', '0', '0', '0', '1'

// 3 bytes The device address (hm_id)
#define HM_ID                0xAB, 0xCD, 0xEF
