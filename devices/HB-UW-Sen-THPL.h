// Pin assignment for Atmega328p (Univarsalsensor)

// Pin assignment for the cc1101 module
#define PORT_SPI             PORTB												// The Port B Data Register
#define DDR_SPI              DDRB												// The Port B Data Direction Register
#define PIN_SPI_SS           2													// PB2 (SS)
#define PIN_SPI_MOSI         3													// PB3 (MOSI)
#define PIN_SPI_MISO         4													// PB4 (MISO)
#define PIN_SPI_SCK          5													// PB5 (SCK)

// Pin assignment for GDO0
#define PORT_GDO0            PORTD												// The Port D Data Register
#define DDR_GDO0             DDRD												// The Port D Data Direction Register
#define PIN_GDO0             2													// PD2 where the GDO0 pin of the cc1101 module is connected

// Pin assignment for status LED
#define PORT_STATUSLED       PORTD												// The Port D Data Register
#define DDR_STATUSLED        DDRD												// The Port D Data Direction Register
#define PIN_STATUSLED        4													// PD4 where the status led sould connected (to ground)

// Pin assignment for config button
#define PORT_CONFIG_BTN      PORTB												// The Port B Data Register
#define DDR_CONFIG_BTN       DDRB												// The Port B Data Direction Register
#define INPUT_CONFIG_BTN     PINB												// The Port B Input Pins Address
#define PIN_CONFIG_BTN       0													// PB0 where the button sould connected (to ground)
#define WAIT_FOR_CONFIG      10													// wait x seconds after watchdog reset for press config button

/**
 * set to 1 to activate debug info over UART
 * set to 2 blockData and blockLen for each received block was printed
 */
#define DEBUG                1

/*****************************************
 *        Address data section           *
 * Stored at 0x7FF0 in boot loader space *
 *           See Makefile                *
 *****************************************/

// The model type
//#define HM_TYPE              0xF1, 0x01    // DIY (HB-UW-Sen-THPL-I)			// stored at 0x7FF0
#define HM_TYPE              0x12, 0x34											// stored at 0x7FF0

// 10 bytes serial number. Must be unique for each device
#define HM_SERIAL            'H','B','0','D','e','f','a','u','l','t'			// stored at 0x7FF2

// 3 bytes The device address (hm_id)
#define HM_ID                0xAB, 0xCD, 0xEF									// stored at 0x7FFC
