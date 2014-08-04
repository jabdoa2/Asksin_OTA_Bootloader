Bidcos Bootloader for Atmega
======================

This version is modifyed for working with the Atmega328p at the universal sensor board

Prepare device:
* Clone repository
* Build source: make
```
make
```
* Write fuses:
```
avrdude -p m328p -P usb -c usbasp -U lfuse:w:0xE2:m -U hfuse:w:0xD0:m
```
* Flash to device:
```
avrdude -p m328p -P usb -c usbasp -V -U flash:w:bootloader.hex
```

Convert payload and flash:
* You need to convert your elf file to binary first (For arduino GUI you can find this in /tmp/buildXXXXX/)
* If CRC Check is disabled, simply use the avr-objcopy tool
```
avr-objcopy -j .text -j .data -O binary payload.elf payload.bin
```
* If CRC Check is enabled, you need to generate a CRC checksum to your binary to make it acceptable for the bootloader
* The intention of the CRC check is to prevent unfinished transfers to start and force you to do a hard reset to re-enter the bootloader
* This CRC Checksum has to be added with the tool srec_cat (from srecord: http://srecord.sourceforge.net/download.html)
* In this case, do not use avr-objcopy, simply run srec_cat as follows (use the .hex file as input, not the .elf file)
```
srec_cat <payload.hex> -intel -fill 0xFF 0x0000 0xDEED -Cyclic_Redundancy_Check_16_Little_Endian 0xDEED -o payload.bin -binary
```
* in both cases you end up with the binary, which has to go through the converter to get the EQ3 format
* Use the converter (need php-cli):
```
php convert.php payload.bin payload.eq3 # convert to eq3 hex format
tar -czf payload.tar.gz payload.eq3 # create .tar.gz for homematic windows tool
```
* Open serial with 57600 baud to see debug output
* Flash payload with flash-ota (from hmland https://git.zerfleddert.de/cgi-bin/gitweb.cgi/hmcfgusb):
* As SERIAL_NUMBER, set your desired number in the header of bootloader.c. Defaults to KEQ0000001
* You should set a different number for each device to prevent bootloader conflicts when more than 1 device is switched on at the same time
```
sudo ./flash-ota -f payload.eq3 -s <SERIAL_NUMBER>
```
* Reboot device to enter bootloader
* Wait for flash-ota to do its job

If you have feedback or problems you can ask questions or leave comments in this thread in FHEM Forum (forum is mostly german but you may also write in english): http://forum.fhem.de/index.php/topic,18071.0.html / http://forum.fhem.de/index.php/board,22.0.html
