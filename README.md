# OptiForth
Fast Forth system for ATmega328 / Arduino Uno R3

It is based on Mikael Nordman's great FlashForth 5.0
(https://flashforth.com)

OptiForth includes an fully Optiboot-compatible bootloader. Once burned
the kernel is loaded as a normal Arduino-sketch.
But be aware that the bootloader runs with 250.000 baud.

To put it on an ATmega328P/ARDUINO UNO R3 you have to use a programmer:
- set the hi-fuse to 0xde and
- burn 'OptiForth56d_boot.hex' with -e (chip erase) option.
The initial config is according to the settings in 'of56d_config.inc' in
the package (with the exception 'withBOOTLOADER = 0')

OptiForth uses 115.200 baud, 8n1, no handshake and sends CR w/o LF (see
the appropriate setting of your terminal program).
It requires a terminal program capable of sending transmit delays
(e.g. TeraTerm, see 'teraterm.ini'). Usually transmit delays of 
 > '0 msec/char' and 
 > '20 msec/line' are sufficient. 
If you have trouble with this setting, double the time/line.

The OptiForth-kernel can be rewritten over USB using avrdude or similar.
The avrdude parameters look like that (all together in one line!):
 -C"[avrdude-path]/avrdude.conf"
 -v -v -patmega328p -carduino -PCOM[your COM-port] -b250000 -D 
 -Ueeprom:w:"[OptiForth-path]\OptiForth52.eep":i 
 -Uflash:w:"[OptiForth-path]\OptiForth52.hex":i 
(all together in one line!)
Be aware that:
- the bootloader runs with 250.000 baud
- not to use the -e (chip erase) option
- to put the "-Ueeprom...."-option prior to the "-Uflash..."-option


For those who are familiar with Mikael Nordman's original FlashForth 5.0:

- the use of processor registers is rearranged, so if you want to access
  them directly have a look at the definition

- the user data area is rearranged, so if you want to access it directly
  have a look at the definition

- therefore the task-switching words had to be changed, compile 'task.of'
  from the 'forth' subdirectoy and use:
  t_init  t_run  t_end  t_single

- when writing to FLASH interrupts are NOT disabled, so the system ticks
  timer keeps on running correctly. This is working because the kernel
  interrup-routines reside in NRWW-area of FLASH memory.
  If you set up your own interrupt routines, you have to switch off
  interrups before writing to FLASH.
