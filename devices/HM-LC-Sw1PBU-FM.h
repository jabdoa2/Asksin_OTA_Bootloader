// Pin assignment for Atmega328p (Univarsalsensor)

// Pin assignment for the cc1101 module
#define PORT_SPI             PORTB												// The Port B Data Register
#define DDR_SPI              DDRB												// The Port B Data Direction Register
#define PIN_SPI_SS           4													// PB4 (SS)
#define PIN_SPI_MOSI         5													// PB5 (MOSI)
#define PIN_SPI_MISO         6													// PB6 (MISO)
#define PIN_SPI_SCK          7													// PB7 (SCK)

// Pin assignment for GDO0
#define PORT_GDO0            PORTD												// The Port D Data Register
#define DDR_GDO0             DDRD												// The Port D Data Direction Register
#define PIN_GDO0             2													// PD2 where the GDO0 pin of the cc1101 module is connected

// Pin assignment for status LED
#define PORT_STATUSLED       PORTB												// The Port B Data Register
#define DDR_STATUSLED        DDRB												// The Port B Data Direction Register
#define PIN_STATUSLED        0													// PB0 where the status led should connected (to ground)

// Pin assignment for config button (without hold the config button the main programm starts immediately if CRC was correct)
// config defines and enable lins below to use config button
//
#define PORT_CONFIG_BTN      PORTD											// The Port D Data Register
#define DDR_CONFIG_BTN       DDRD												// The Port D Data Direction Register
#define INPUT_CONFIG_BTN     PIND												// The Port D Input Pins Address
#define PIN_CONFIG_BTN       7												// PD7 where the button sould connected (to ground)
#define WAIT_FOR_CONFIG      10												// Wait 10 seconds after watchdog reset for press config button

/**
 * set to 1 to activate debug info over UART
 * set to 2 blockData and blockLen for each received block was printed
 */
#define DEBUG                1

// Enable CRC Check before application start. Requires firmware to include CRC checksum at the end.
#define CRC_FLASH            1

// adress of CRC checksum if CRC check is enabled. See below. Expect a CRC16 little endian
// is defined in makefile
// #define CODE_LEN             0xDEED

/*****************************************
 *        Address data section           *
 * Stored at 0x7FF0 in boot loader space *
 *           See Makefile                *
 *****************************************/

// The model type (not used from bootloader)
#define HM_TYPE              0x00, 0x96

// 10 bytes serial number. Must be unique for each device
#define HM_SERIAL            'K', 'E', 'Q', '0', '0', '0', '0', '0', '0', '1'

// 3 bytes The device address (hm_id)
#define HM_ID                0x12, 0x34, 0x56
