all:
	avr-gcc -c -std=c99 -mmcu=atmega644 -Wl,--section-start=.text=0xE000 -DF_CPU=8000000 -Os cc.c -o cc.o
	avr-gcc -std=c99 -mmcu=atmega644 -Wl,--section-start=.text=0xE000 -DF_CPU=8000000 -Os bootloader.c cc.o uart/uart.o -o bootloader.elf
	avr-objcopy -j .text -j .data -O ihex bootloader.elf bootloader.hex

debug:
	avr-gcc -c -std=c99 -mmcu=atmega644 -DF_CPU=8000000 -Os cc.c -o cc.o
	avr-gcc -std=c99 -mmcu=atmega644 -DF_CPU=8000000 -Os bootloader.c cc.o uart/uart.o -o bootloader.elf
	avr-objcopy -j .text -j .data -O ihex bootloader.elf bootloader.hex
