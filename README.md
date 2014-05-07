Bidcos Bootloader for Atmega
======================

Currently only Atmega 644 is supported. Tested on HM\-LC\-Sw1PBU\-FM (https://github.com/jabdoa2/Asksin_HM_LC_Sw1PBU_FM).

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
* You need to convert your elf file to binary first (For arduino GUI you can find this in /tmp/buildXXXXX/)
```
avr-objcopy -j .text -j .data -O binary payload.elf payload.bin
```
* Use the converter (need php-cli):
```
php convert.php payload.bin payload.eq3 # convert to eq3 hex format
tar -czf payload.tar.gz payload.eq3 # create .tar.gz for homematic windows tool
```
* Open serial with 57600 baud to see debug output
* Flash payload with flash-ota (from hmland https://git.zerfleddert.de/cgi-bin/gitweb.cgi/hmcfgusb):
```
sudo ./flash-ota -f payload.eq3 -s KEQ0123456
```
* Reboot device to enter bootloader
* Wait for flash-ota to do its job

If you have feedback or problems you can ask questions or leave comments in this thread in FHEM Forum (forum is mostly german but you may also write in english): http://forum.fhem.de/index.php/topic,18071.0.html / http://forum.fhem.de/index.php/board,22.0.html
