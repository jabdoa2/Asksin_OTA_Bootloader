Bidcos Bootloader for Atmega
======================

Currently only Atmega 644 is supported. Tested on HM\-LC\-Sw1PBU\-FM (https://github.com/jabdoa2/Asksin\_HM\_LC\_Sw1PBU\_FM).

Prepare device:
* Clone repository
* Build source: make
```
make
```
* Write fuses:
```
avrdude -p m644 -P usb -c usbasp -U lfuse:w:0xFD:m -U hfuse:w:0xD8:m
```
* Flash to device:
```
avrdude -p m644 -P usb -c usbasp -V -U flash:w:bootloader.hex
```

Convert payload and flash:
* You need to convert your elf (or hex) file to binary first (For arduino gui you can find this in /tmp/buildXXXXX/)
```
avr-objcopy -j .text -j .data -O binary payload.elf payload.bin
```
* Use the converter (need php-cli):
```
php convert.php payload.bin payload.eq3
```
* Open serial with 57600 baud to see debug output
* Flash payload with flash-ota (from hmland):
```
sudo ./flash-ota -f payload.eq3 -s KEQ0123456
```
* Reboot device to enter bootloader
* Wait for flash-ota to do its job
