
; *********************************************************************
;                                                                     *
;    Filename:      of52_config.inc                                   *
;    Date:          05.12.2020                                        *
;    MCU:           ATmega328P                                        *
;    Copyright:     bitflipser                                        *
;    Author:        bitflipser                                        *
;                                                                     *
; *********************************************************************
; OptiForth is MIT licensed                                           *
; *********************************************************************

; OptiForth device configuration for Atmega devices
;
.NOLIST
; select the include file for your micro controller
;.include "m2561def.inc"
;.include "m2560def.inc"
;.include "m128def.inc"
;.include "m168pdef.inc"

; ##### build and tested for this ATmega328p-config ONLY !!!) #####
; #																  #
.include "m328pdef.inc"   ; tested fuses: E:0xff H:0xde L:0xff    #
; #																  #
; #################################################################

;.include "m328def.inc"
;.include "m32adef.inc"
;.include "m644pdef.inc"
.LIST

;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.equ optimizeNUM		= 1 	; optimized math and numeric conversion
.equ optimizingCOMPILER	= 1 	; speed optimizing compiler (may increase code size)
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

; set to 1 when the terminal program requires LF to be send with CR
; (use 'Setup - Terminal... - New Line - Receive: CR' with TeraTerm)
.equ CR_with_LF = 0

; set to 1 to allow 'Ctrl-O' to reset OptiForth from the operator UART
.equ CTRL_O_WARM_RESET = 1

; debug flash and eeprom writes
; prints time [ms] for every FLASH block write, or '+' for every EEPROM byte write
.equ DEBUG_FLASH = 0;1

; set to 1 for power save when CPU is idle
.set IDLE_MODE = 0;1

; enable the cpu load measurement (for MS_TIMER <> 1 only, sets IDLE_MODE = 1)
; ################# NO load measurement when MS_TIMER == 1 #####################
.set CPU_LOAD = 1
			
; CPU load indicator led definitions (sets IDLE_MODE = 1)
.equ CPU_LOAD_LED = 1

; UART buffer sizes
.equ RX0_BUF_SIZE = 0x100				; 0x100, 0x200, 0x300, 0x400 ... RAMSIZE-0x200
;.equ RX0_OFF_FILL = RX0_BUF_SIZE - 50	; fill level for XOFF - not in use, no handshaking

; default number base
.equ BASE_DEFAULT = 10					; 16 = hexadecimal, 10 = decimal

; define the startup delay [ms] for the turnkey word
.equ TURNKEY_DELAY = 2000				; milliseconds

.equ CPU_LOAD_DDR = DDRB
.equ CPU_LOAD_PORT= PORTB				; avr-p28: portc	arduinomega: portb	arduinouno: PORTB
.equ CPU_LOAD_BIT = 5					; avr-p28: pin5  	arduinomega: pin7 	arduinouno: PB5
.equ CPU_LOAD_LED_POLARITY = 1			; avr-p28:  0 (= low) on port turns on led 
										; arduino:  1 (=high) on port turns on led 

; oscillator frequency in herz
.equ FREQ_OSC = 16000000

;;; UART0 configuration
;;; Note: With Arduino Uno R3 and MEGA R3 the USB to serial bridge latency and queues 
;;; disturb the XON/XOFF flow control. 
;;; The workaround is NOT to use flow control and INCREASE buffer size. Initial setup
;;; is RX0_BUF_SIZE = 1024 (0x400) - the size of a SCREEN in early forth implementations.
;;; You can set any multiple of 0x100 up to (RAM_SIZE - 0x200).
;;; Send source files ONLY, that do not EXCEED that buffer size! Or use a terminal 
;;; program capable of inserting waiting times between sending lines.
;;; (You may also try the ff-shell.py from the original FlashForth 5.0-package)
.equ BAUDRATE0 = 115200             ; serial baudrate UART0
.equ U0FC_TYPE = 0                  ; 0 = no handshaking, 1 = XON/XOFF, 2=CTS/RTS
;.equ U0RTS_PORT= PORTD
;.equ U0RTS_DDR = DDRD
;.equ U0RTS_BIT = 3

;;; USER AREA sizes for the OPERATOR task
.equ RETURN_STACK_SIZE    = 64       ; 32 cells return stack
.equ PARAMETER_STACK_SIZE = 64       ; 32 cells parameter stack
.equ TIB_SIZE = 90                   ; 80 chars tib size + 10 chars hold area

; =============================================================================
; =============================================================================
; =============================================================================
; ############## options requiring reburning of bootloader part ###############
; =============================================================================
; =============================================================================
; =============================================================================

.equ withBOOTLOADER	= 0;1 			; append bootloader part (needs programmer)

; define the UART used for the operator
.equ OPERATOR_UART	= 0 			; 0 or 1 (requires reburning of bootloader)

;;; UART1 configuration	(not on ATmega328/p)
.equ BAUDRATE1 = 38400              ; Serial baudrate UART1
.equ U1FC_TYPE = 1                  ; 1 = XON/XOFF, 2=CTS/RTS
.equ U1RTS_PORT= PORTD
.equ U1RTS_DDR = DDRD
.equ U1RTS_BIT = 4

.equ RX1_BUF_SIZE = 32  ; 8,16,32,64
.equ RX1_OFF_FILL = 4   ; Fill level for XOFF

; select the timer to use for the system millisecond ticks (0, 1, 2)
.equ MS_TIMER = 0					; 0, 1, 2 (requires reburning of bootloader)

; ############### NO load measurement when MS_TIMER = 1 #####################
	.if MS_TIMER == 1
		.set CPU_LOAD = 0
	.endif
; ############################################################################

.if (CPU_LOAD == 1) || (CPU_LOAD_LED == 1)
	.set IDLE_MODE = 1
.endif

#if defined(__ATmega2560__)
#define partstring "ATmega2560"
#elif defined(__ATmega328P__)
#define partstring "ATmega328P"
#elif defined(__ATmega328__)
#define partstring "ATmega328"
#elif defined(__ATmega128__)
#define partstring "ATmega128"
#elif defined(__ATmega2561__)
#define partstring "ATmega2561"
#elif defined(__ATmega644__)
#define partstring "ATmega644"
#else
#define partstring "ATmega"
#endif

.if MS_TIMER == 0
  .ifdef OC0Aaddr
	.equ MS_TIMER_ADDR = OC0Aaddr
  .else
	.equ MS_TIMER_ADDR = OC0addr
  .endif
.endif

.if MS_TIMER == 1
  .ifdef OC1Aaddr
	.equ MS_TIMER_ADDR = OC1Aaddr
  .else
	.equ MS_TIMER_ADDR = OC1addr
  .endif
.endif

.if MS_TIMER == 2
  .ifdef OC2Aaddr
	.equ MS_TIMER_ADDR = OC2Aaddr
  .else
	.equ MS_TIMER_ADDR = OC2addr
  .endif
.endif

