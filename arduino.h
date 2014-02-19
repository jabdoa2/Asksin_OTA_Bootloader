#define PA 1
#define PB 2
#define PC 3
#define PD 4
#define PE 5

#define NOT_A_PIN 0
#define NOT_A_PORT 0

const uint16_t PROGMEM port_to_mode_PGM[] =
{
        NOT_A_PORT,
        (uint16_t) &DDRA,
        (uint16_t) &DDRB,
        (uint16_t) &DDRC,
        (uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] =
{
        NOT_A_PORT,
        (uint16_t) &PORTA,
        (uint16_t) &PORTB,
        (uint16_t) &PORTC,
        (uint16_t) &PORTD,
};

#define portModeRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_mode_PGM + (P))) )
#define portOutputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_output_PGM + (P))) )


const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] =
{
        _BV(0), /* 0, port B */
        _BV(1),
        _BV(2),
        _BV(3),
        _BV(4),
        _BV(5),
        _BV(6),
        _BV(7),
        _BV(0), /* 8, port D */
        _BV(1),
        _BV(2),
        _BV(3),
        _BV(4),
        _BV(5),
        _BV(6),
        _BV(7),
        _BV(0), /* 16, port C */
        _BV(1),
        _BV(2),
        _BV(3),
        _BV(4),
        _BV(5),
        _BV(6),
        _BV(7),
        _BV(7), /* 24, port A */
        _BV(6),
        _BV(5),
        _BV(4),
        _BV(3),
        _BV(2),
        _BV(1),
        _BV(0)
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] =
{
        PB, /* 0 */
        PB,
        PB,
        PB,
        PB,
        PB,
        PB,
        PB,
        PD, /* 8 */
        PD,
        PD,
        PD,
        PD,
        PD,
        PD,
        PD,
        PC, /* 16 */
        PC,
        PC,
        PC,
        PC,
        PC,
        PC,
        PC,
        PA, /* 24 */
        PA,
        PA,
        PA,
        PA,
        PA,
        PA,
        PA  /* 31 */
};

#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )
#define digitalPinToBitMask(P) ( pgm_read_byte( digital_pin_to_bit_mask_PGM + (P) ) )

void pinMode(uint8_t pin, uint8_t mode)
{
        uint8_t bit = digitalPinToBitMask(pin);
        uint8_t port = digitalPinToPort(pin);
        volatile uint8_t *reg, *out;

        if (port == NOT_A_PIN) return;

        // JWS: can I let the optimizer do this?
        reg = portModeRegister(port);
        out = portOutputRegister(port);

        if (mode == INPUT) {
                uint8_t oldSREG = SREG;
                cli();
                *reg &= ~bit;
                *out &= ~bit;
                SREG = oldSREG;
        } else if (mode == INPUT_PULLUP) {
                uint8_t oldSREG = SREG;
                cli();
                *reg &= ~bit;
                *out |= bit;
                SREG = oldSREG;
        } else {
                uint8_t oldSREG = SREG;
                cli();
                *reg |= bit;
                SREG = oldSREG;
        }
}

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

void
delayMicroseconds( uint16_t d )
{
  while(d--) {
    _delay_us( 1 );
  }
}

void
delay( uint16_t d )
{
  while(d--) {
    _delay_ms( 1 );
  }
}

void digitalWrite(uint8_t pin, uint8_t val)
{
        uint8_t bit = digitalPinToBitMask(pin);
        uint8_t port = digitalPinToPort(pin);
        volatile uint8_t *out;

        if (port == NOT_A_PIN) return;


        out = portOutputRegister(port);

        uint8_t oldSREG = SREG;
        cli();

        if (val == LOW) {
                *out &= ~bit;
        } else {
                *out |= bit;
        }

        SREG = oldSREG;
}
