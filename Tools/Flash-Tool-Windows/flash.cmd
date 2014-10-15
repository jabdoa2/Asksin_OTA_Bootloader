::**************
::* Flash-Tool *
::**************

@echo off
setlocal ENABLEEXTENSIONS ENABLEDELAYEDEXPANSION

if %1!==! goto explainUsage & goto end
if %2!==! goto explainUsage & goto end

::**************************
::* convert hexfile to bin *
::**************************
bin\hex2bin -c %2 > NUL
for /f "tokens=1,2 delims=. " %%a in ("%2") do set fileBasename=%%a&set fileExt=%%b

if %3!==! if %4!==! if %5!==! (
	goto useDefaultValues
)

::**************************
::* Set the default values *
::**************************
set "defHmType1=12"
set "defHmType2=34"
set "defHmId1=AB"
set "defHmId2=CD"
set "defHmId3=EF"
set "defSerialNumber=HB0Default"

echo.

::*****************************
::* Check the for valid HM-TYPE *
::*****************************
for /f "tokens=1,2 delims=: " %%a in ("%3") do set hmType1=%%a&set hmType2=%%b
call :checkHex hmType1
call :checkHex hmType2

::*****************************
::* Check the for valid HM-ID *
::*****************************
for /f "tokens=1,2,3 delims=: " %%a in ("%4") do set hmId1=%%a&set hmId2=%%b&set hmId3=%%c
call :checkHex hmId1
call :checkHex hmId2
call :checkHex hmId3

set "hmIdError="
IF (%hmType1%) == () set hmIdError=1
IF (%hmType2%) == () set hmIdError=1

IF (%hmId1%) == () set hmIdError=1
IF (%hmId2%) == () set hmIdError=1
IF (%hmId3%) == () set hmIdError=1
IF defined hmIdError (
	call :explainUsage "The entered HM-TYPE "%3" or HM-ID "%4" is invalid. Format HM-TYPE: XX:XX, Format HM-ID: XX:XX:XX. Each X must be 0-9 or A-F."
	goto end
) else (
	set "hmType=\x%hmType1%\x%hmType2%"
	set "hmId=\x%defHmId1%\x%defHmId2%\x%defHmId3%"
)

::*********************************
::* Check the valid serial number *
::*********************************
set "serialNr=%5"
set "serialNrLen=0"
call :strLen serialNr serialNrLen
if NOT %serialNrLen% EQU 10 (
	set "serialNr=~~~"
)

set "res="&for /f "delims=0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdrefgijklmnopqrstuvwxyz" %%i in ("%serialNr%") do set "res=%%i"
IF NOT %res%!==! (
	call :explainUsage "The serial number must contains 10 characters 0-9 or A-Z."
	goto end
)


::**********************************************************
::* Write user defined HM-ID and serial number to bin-file *
::**********************************************************
@echo on
  bin\sed -b -e "s/\(\x%defHmType1%\x%defHmType2%\)\(%defSerialNumber%\)\(\x%defHmId1%\x%defHmId2%\x%defHmId3%\)/\x%hmType1%\x%hmType2%%serialNr%\x%hmId1%\x%hmId2%\x%hmId3%/" %fileBasename%.bin > %fileBasename%.tmp
 ::bin\sed -b -e "s/\(%defSerialNumber%\)\(.*\)\(\x%defHmId1%\x%defHmId2%\x%defHmId3%\)/%serialNr%\2\x%hmId1%\x%hmId2%\x%hmId3%/" %fileBasename%.bin > %fileBasename%.tmp
@echo off
echo %defHmType1%:%defHmType2% %defSerialNumber% %defHmId1%:%defHmId2%:%defHmId3%

del %fileBasename%.bin
ren %fileBasename%.tmp %fileBasename%.bin

echo Start flashing device with HM-TYPE: %hmType1%:%hmType2%, HM-ID: %hmId1%:%hmId2%:%hmId3% and serial number: %serialNr% 


:useDefaultValues

::*******************************
::* Begin flashing with avrdude *
::*******************************
if %serialNr%!==! (
	echo Start flashing device with predefined HM-TYPE, HM-ID and serial number.
)

echo.
echo ****************************************
echo * Set fusebits with bitClock 187,5 KHz *
echo ****************************************
:: First we set fuses with bitClock 187,5 KHz (fuse bits for internal osszilator)
bin\avrdude.exe -Cbin\avrdude.conf -c%1 -patmega328p -B4.0 -e -U lfuse:w:0xE2:m -U hfuse:w:0xD0:m -U efuse:w:0x06:m

:: fuse bits for external crystal
::bin\avrdude.exe -Cbin\avrdude.conf -c%1 -patmega328p -B4.0 -e -U lfuse:w:0xFD:m -U hfuse:w:0xD0:m -U efuse:w:0x06:m

echo.
echo *************************************************************************
echo * Write bootloader, application and set lock bits with bitClock 1,5 MHz *
echo *************************************************************************
:: Next we flash program and set lock bits with bitClock 1,5 MHz
bin\avrdude.exe -Cbin\avrdude.conf -c%1 -patmega328p -B0.5 -Uflash:w:%fileBasename%.bin:r -Ulock:w:0x3F:m 

::del %fileBasename%.bin

goto :end



::*********************************************************************************************************
::* At this point some subroutines defined                                                                *
::*********************************************************************************************************

::**************************
::* Write out param errors *
::**************************
: explainUsage <errorText> (
    setlocal EnableDelayedExpansion
    
	IF NOT %1!==! (
		echo Error: %1
		echo.
	)
	
	echo Usage: flash.bat ^<programmer^> ^<hexfile^> [^<HM-TYPE^> ^<HM-ID^> ^<Serial^>]
    exit /b
)

::**********************************
::* Count the length of a variable *
::**********************************
:strLen <stringVar> <resultVar> (   
::    setlocal EnableDelayedExpansion

	set "s=!%~1!#"
    set "len=0"
    for %%P in (4096 2048 1024 512 256 128 64 32 16 8 4 2 1) do (
        if "!s:~%%P,1!" NEQ "" ( 
            set /a "len+=%%P"
            set "s=!s:~%%P!"
        )
    )
) ( endlocal
    set "%~2=%len%"
    exit /b
)

::***************************************
::* Check a variable if is a hex number *
::***************************************
:checkHex <txtVar> (   
::    setlocal EnableDelayedExpansion
    setlocal

	set "res=!%~1!"
	set "var="&for /f "delims=0123456789ABCDEFabcdef" %%i in ("%res%") do set "var=%%i"

	if %var%!==! (
		rem
	) else (
		set "res="
	)
	
	set "resLen=0"
	call :strLen res resLen

	if %resLen% GTR 2 (
		set "res="
	)

	set "%~1=%res%"
    exit /b
)

::*********************************************************************************************************
::* The end.                                                                                              *
::*********************************************************************************************************
:end
echo.
