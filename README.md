Bidcos Bootloader for Atmega
============================

Currently tested at Atmega328p, Atmega644 not tested yet.

* Tested on HB-UW-Sen-THPL (https://github.com/kc-GitHub/Wettersensor)
* Not yest tested on HM-LC-Sw1PBU-FM. (https://github.com/jabdoa2/Asksin_HM_LC_Sw1PBU_FM)

Prepare device:
* Clone repository
* The config.h is configured to the Atmega328p (HB-UW-Sen-THPL). For the HM-LC-Sw1PBU-FM with Atmega644 you must rename the config-Atmega644.h to config.h
* Build source for HB-UW-Sen-THPL
```
make HB_UW_Sen_THPL
```
* Build source for HM-LC-Sw1PBU-FM
```
make HM_LC_Sw1PBU_FM
```
* Build source for HM-LC-Sw1PBU-FM (8k Bootloader space)
```
make HM_LC_Sw1PBU_FM_8k
```

* Write fuses:
* We set lock Bits for bootloader section mode 2 (SPM prohibited), so the bootloader can't write his self.
* LPM must allowed, so we store the adress data of the device in the bootloader section and the application
* must can read the bootloader section.
* See: http://eleccelerator.com/fusecalc/fusecalc.php?chip=atmega328p&LOW=E2&HIGH=D0&EXTENDED=06&LOCKBIT=2F
```
avrdude -p m328p -P usb -c usbasp -U lfuse:w:0xE2:m -U hfuse:w:0xD0:m -U efuse:w:0x06:m -U lock:w:0x2F:m
```
* Flash to device (Atmega328p):
```
avrdude -p m328p -P usb -c usbasp -V -U flash:w:Bootloader-AskSin-OTA-HB_UW_Sen_THPL.hex
```


* Fuse settings for Atmega644 for 4k Bootloader size (HM-LC-Sw1PBU-FM):
http://eleccelerator.com/fusecalc/fusecalc.php?chip=atmega644&LOW=FD&HIGH=DA&LOCKBIT=2F
```
avrdude -p m644 -P usb -c usbasp -U lfuse:w:0xFD:m -U hfuse:w:0xDA:m -U lock:w:0x2F:m
```

* Flash to device (Atmega644):
```
avrdude -p m644 -P usb -c usbasp -V -U flash:w:Bootloader-AskSin-OTA-HM_LC_Sw1PBU_FM.hex
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
srec_cat <payload.hex> -intel -fill 0xFF 0x0000 0x6FFE -Cyclic_Redundancy_Check_16_Little_Endian 0x6FFE -o  payload.bin -binary
```
* in both cases you end up with the binary, which has to go through the converter to get the EQ3 format
* Use the converter (need php-cli):
```
./bin2eq3.php payload.bin payload.eq3 # convert to eq3 hex format

Dependent on the flash page size of the desired AVR, a different page size can pass as third parameter in bytes. The default pages size is 256 bytes.
E.g. the Atmega328 needs a page size of 128 bytes.

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
