all:	uart_code
	avr-gcc -Wall -c -std=c99 -mmcu=atmega644 -Wl,--section-start=.text=0xE000 -DF_CPU=8000000 -Os cc.c -o cc.o
	avr-gcc -Wall -std=c99 -mmcu=atmega644 -Wl,--section-start=.text=0xE000 -DF_CPU=8000000 -Os bootloader.c cc.o uart/uart.o -o bootloader.elf
	avr-objcopy -j .text -j .data -O ihex bootloader.elf bootloader.hex

uart_code:
	$(MAKE) -C ./uart/

debug:	uart_code
	avr-gcc -Wall -c -std=c99 -mmcu=atmega644 -DF_CPU=8000000 -Os cc.c -o cc.o
	avr-gcc -Wall -std=c99 -mmcu=atmega644 -DF_CPU=8000000 -Os bootloader.c cc.o uart/uart.o -o bootloader.elf
	avr-objcopy -j .text -j .data -O ihex bootloader.elf bootloader.hex

testpayload:
	avr-gcc -Wall -std=c99 -mmcu=atmega644 -DF_CPU=8000000 -Os test.c -o payload.elf
	avr-objcopy -j .text -j .data -O ihex payload.elf payload.hex
	avr-objcopy -j .text -j .data -O binary payload.elf payload.bin
	php convert.php payload.bin payload.eq3

clean:
	$(MAKE) -C ./uart/ clean
	rm payload.hex payload.bin payload.eq3 payload.elf *.o
