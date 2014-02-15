all:
	avr-gcc -mmcu=atmega644 -DF_CPU=8000000 -O9 bootloader.c -o bootloader.elf
