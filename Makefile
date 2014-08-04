MCU = atmega328p
F_CPU = 8000000

SIZE = avr-size
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after:
FORMAT = ihex

TARGET = bootloader

# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(TARGET).hex
ELFSIZE = $(SIZE) -A $(TARGET).elf
AVRMEM = avr-mem.sh $(TARGET).elf $(MCU)

all:	uart_code
	avr-gcc -Wall -c -std=c99 -mmcu=$(MCU) -Wl,--section-start=.text=0x7000 -DF_CPU=$(F_CPU) -Os cc.c -o cc.o
	avr-gcc -Wall -std=c99 -mmcu=$(MCU) -Wl,--section-start=.text=0x7000 -DF_CPU=$(F_CPU) -Os bootloader.c cc.o uart/uart.o -o $(TARGET).elf
	avr-objcopy -j .text -j .data -O $(FORMAT) $(TARGET).elf $(TARGET).hex

#sizebefore:
#	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); \
#	$(AVRMEM) 2>/dev/null; echo; fi

#sizeafter:
#	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); \
#	$(AVRMEM) 2>/dev/null; echo; fi

	@echo 'Binary size:' echo; echo; $(HEXSIZE);

uart_code:
	$(MAKE) -C ./uart/

debug:	uart_code
	avr-gcc -Wall -c -std=c99 -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os cc.c -o cc.o
	avr-gcc -Wall -std=c99 -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os bootloader.c cc.o uart/uart.o -o bootloader.elf
	avr-objcopy -j .text -j .data -O $(FORMAT) bootloader.elf bootloader.hex

testpayload:
	avr-gcc -Wall -std=c99 -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os test.c -o payload.elf
	avr-objcopy -j .text -j .data -O $(FORMAT) payload.elf payload.hex
	avr-objcopy -j .text -j .data -O binary payload.elf payload.bin
	php convert.php payload.bin payload.eq3

clean:
	$(MAKE) -C ./uart/ clean
	rm bootloader.hex bootloader.elf payload.hex payload.bin payload.eq3 payload.elf *.o
