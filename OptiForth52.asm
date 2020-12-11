
;***********************************************************************************
;                                                                                  *
;    Filename:      OptiForth52.asm                                                *
;    Date:          05.12.2020                                                     *
;    File Version:  5.2                                                            *
;    MCU:           Atmega328/P                                                    *
;    Copyright:     bitflipser                                                     *
;    Author:        bitflipser                                                     *
;                                                                                  * 
;***********************************************************************************
; MIT License                                                                      *
;                                                                                  *
; Copyright (c) 2020 bitflipser                                                    *
;                                                                                  *
; Permission is hereby granted, free of charge, to any person obtaining a copy     *
; of this software and associated documentation files (the "Software"), to deal    *
; in the Software without restriction, including without limitation the rights     *
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell        *
; copies of the Software, and to permit persons to whom the Software is            *
; furnished to do so, subject to the following conditions:                         *
;                                                                                  *
; The above copyright notice and this permission notice shall be included in all   *
; copies or substantial portions of the Software.                                  *
;                                                                                  *
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR       *
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,         *
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE      *
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER           *
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,    *
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE    *
; SOFTWARE.                                                                        *
;                                                                                  *
;***********************************************************************************
;                                                                                  *
; OptiForth is a standalone Forth system for AVR ATmega microcontrollers that can  *
; flash their own flash memory.                                                    *
;                                                                                  *
; It is based on Mikael Nordman's FlashForth 5.0 (https://flashforth.com)          *
;                                                                                  *
; Optimized and tested for the ATmega328P/Arduino UNO R3 ONLY !!                   *
;                                                                                  *
; Modified versions of OptiForth must be clearly marked as such, in the name of    *
; this file, and in the identification displayed when OptiForth starts.            *
;***********************************************************************************

; define the OF version date string
#define DATE       "05.12.2020"

; include the OptiForth configuration file
.include "of52_config.inc"
.NOLIST
; include the OptiForth macro library
.include "of_macros.inc"
.LIST

; Register definitions
  .def UP		= R2		; not in interrupt
   .def upL		= R2		; not in interrupt
   .def upH		= R3		; not in interrupt
  .def r_one	= R6		; read only one
  .def r_zero	= R7		; read only zero
  .def #hold	= R8		; not in interrupt (used by <# # #s #>)
  .def t8		= R8		; not in interrupt
  .def wflags	= R9		; not in interrupt

.if CPU_LOAD == 1
	  .def loadreg	  = R18
.else
	  .def t9		  = R18	; not in interrupt
.endif

  .def X_intSafe	=  R4 	; 16 bit, interrupt only !
;   .def X_intSafeL	=   R4	; interrupt only !
;   .def X_intSafeH	=   R5	; interrupt only !
  .def INTvector	= R19	; interrupt only !
  .def SREG_intSafe = R19 	; interrupt only !

  .def IBASE		= R12	; not in interrupt
   .def ibaseL		= R12	; not in interrupt
   .def ibaseH		= R13	; not in interrupt
  .def MS_COUNT		= R14	; not in interrupt
   .def ms_countL	= R14	; not in interrupt
   .def ms_countH	= R15	; not in interrupt

  .def t0 = R16		.equ regt0   = 0x00
  .def t1 = R17		.equ regt1   = 0x10
  .def t2 = R0          	; not in interrupt (see 'OF_ISR' and 'OF_ISR_EXIT')
  .def t3 = R1          	; not in interrupt (------------------------------)
  .def t4 = R26				; XL
  .def t5 = R27				; XH
  .def t6 = R30				; ZL
  .def t7 = R31				; ZH

  .def t1t0 = R16	.equ regt1t0 = 0x00		; 16-bit
  .def t3t2 = R0							; 16-bit
  .def t5t4 = R26	.equ regt5t4 = 0xa0		; 16-bit (=X)
  .def t7t6 = R30	.equ regt7t6 = 0xe0		; 16-bit (=Z)

  .def A  = R10				; A register
   .def al = R10
   .def ah = R11
  .def P  = R20				; P register and FOR..LOOP INDEX variable
   .def pl = R20
   .def ph = R21	.equ regP	= 0x40

  .def TOP  = R24	.equ regTOP = 0x80
   .def tosl = R24	  .equ regtosl= 0x80
   .def tosh = R25	  .equ regtosh= 0x90
					.equ regX	= 0xa0
					.equ regZ	= 0xe0

  .def FLAGS1 = R22     	; not in interrupt
  .def FLAGS2 = R23     	; not in interrupt
  .equ FLAGS3 = GPIOR0


; symbol naming compatilibity
; UART0 symbols for Atmega32
.ifndef UCSR0A
	.equ UCSR0A=UCSRA
	.equ UDR0_=UDR
	.equ UCSR0B=UCSRB
	.equ UCSR0C=UCSRC
	.equ RXEN0=RXEN
	.equ TXEN0=TXEN
	.equ RXCIE0=RXCIE
	.equ UCSZ00=UCSZ0
	.equ USBS0=USBS
	.equ UBRR0H=UBRRH
	.equ UBRR0L=UBRRL
	.equ URSEL_=0x80
.else
	.equ UDR0_=UDR0
	.equ URSEL_=0
.endif

.ifndef SPMCSR
	.equ SPMCSR=SPMCR
.endif

.ifndef SPMEN
	.equ SPMEN=SELFPRGEN
.endif

.ifndef EEWE
	.equ EEWE=EEPE
.endif

.ifndef EEMWE
	.equ EEMWE=EEMPE
.endif

.if OPERATOR_UART == 1
	.equ OP_TX_=TX1_
	.equ OP_RX_=RX1_
	.equ OP_RXQ=RX1Q
.elif OPERATOR_UART == 0
	.equ OP_TX_=TX0_
	.equ OP_RX_=RX0_
	.equ OP_RXQ=RX0Q
.endif

#define ubrr0val (FREQ_OSC/ 8/BAUDRATE0) - 1		; double speed mode
#define ubrr1val (FREQ_OSC/16/BAUDRATE1) - 1

.if FREQ_OSC < 16384000 ; Hz
	.equ ms_value_tmr0 = ((FREQ_OSC/1000/64) - 1)
	.equ ms_value_tmr1 = ((FREQ_OSC/1000) - 1)
	.equ ms_value_tmr2 = ((FREQ_OSC/1000/64) - 1)
 .ifdef TCCR0B
	.equ ms_pre_tmr0   = 3
 .endif
 .ifdef TCCR0
	.equ ms_pre_tmr0   = 4
 .endif
 .ifdef TCCR2B
	.equ ms_pre_tmr2   = 4
 .endif
 .ifdef TCCR2
	.equ ms_pre_tmr2   = 3
 .endif
.else 					; FREQ_OSC >= 16384000 Hz
	.equ ms_value_tmr0 = ((FREQ_OSC/1000/256) - 1)
	.equ ms_value_tmr1 = ((FREQ_OSC/1000) - 1)
	.equ ms_value_tmr2 = ((FREQ_OSC/1000/128) - 1)
 .ifdef TCCR0B
	.equ ms_pre_tmr0   = 4
 .endif
 .ifdef TCCR0
	.equ ms_pre_tmr0   = 6
 .endif
 .ifdef TCCR2B
	.equ ms_pre_tmr2   = 5
 .endif
 .ifdef TCCR2
	.equ ms_pre_tmr2   = 4
 .endif
.endif

.equ CPU_LOAD_VAL  = (FREQ_OSC*128/100000)

;..............................................................................
;program specific constants (literals used in code)
;..............................................................................
; flash page size
.equ PAGESIZEB=PAGESIZE*2		; page size in bytes 

; forth word header flags
.equ NFA		= 0x80			; name field mask
.equ IMMED		= 0x40			; immediate mask
.equ INLINE		= 0x20			; inline mask for 1, 2 and 3 cell code
.equ COMPILE	= 0x10			; compile only mask
.equ NFAmask	= 0x0f			; name field length mask

  .if optimizingCOMPILER == 1
	.equ INLINE4	= 0x20			; inline mask for 4 cell code
	.equ INLINE5	= 0x20			; inline mask for 5+ cell code
  .else
	.equ INLINE4	= 0x00
	.equ INLINE5	= 0x00
  .endif

.equ NFAbit		= 7
.equ IMMEDbit	= 6
.equ INLINEbit	= 5
.equ COMPILEbit	= 4

; FLAGS3 (GPIOR0)
;----------------
;						= 7
;						= 6
;						= 5
;						= 4
.equ fLEAVE 			= 3		; LEAVE encountered in DO..LOOP
.equ fLOCK				= 2		; write protect EEPROM and FLASH
.equ idirty				= 1		; flash write buffer modified
.equ fFLASH_PAGE_CLEAR	= 0		; actual flash page in erased state -> no need to erase before flushing buffer


; FLAGS2 (R23)
;-------------
.equ fSTATE			= 7		; 0 = interpret, 1 = compile
;+++++++++++++ do not change the following two bit positions ++++++++++++
.equ fIMMED			= IMMEDbit	; (6) create an IMMEDIATE-marked word
.equ fINLINE		= INLINEbit ; (5) create an INLINE-marked word
;+++++++++++++ do not change the upper two bit positions ++++++++++++++++
.equ fDOTsign		= 4		; write '-' sign 
.equ fWORDSall		= 3		; list all words in WORDS
.equ fDUMPxxx		= 3		; DUMP 3-digit numbers (for base < 16)
  .if IDLE_MODE == 0
	;				= 2
	.equ fSINGLE	= 1		; single task (operator) only
  .else
	.equ fTX0pending= 2		; waiting for UDRE0 (enable UDRIE0 in 'IDLE_LOAD')
	.equ fIDLE		= 1		; 0 = busy, 1 = idle
  .endif
.equ fLOADled		= 0		; 0 = no load-LED, 1 = load-LED on


; FLAGS1 (R22)		; used in COMPILE-state only
;-------------
.equ fLIT		= 7			; literal compiled
;.equ noclear	= 6			; dont clear optimisation flags (replaced by doclear to save space in 'constant')
.equ doclear	= 6			; clear optimization flags
.equ idup		= 5			; use dupzeroequal instead of zeroequal
.equ izeroeq	= 4			; use brne instead of breq if zeroequal
.equ f2LIT		= 3			; 2 subsequent literals (used by '!', 'c!', 'mtst', 'mtst0', 'mset', 'mclr')
.equ iLITeq		= 2			; 'LIT =' compiled
.equ fTAILC		= 1			; prevent tail jump optimization
.equ icarryeq	= 0			; use brcs instead of brne

;;; for flow Control
.equ XON	= 0x11
.equ XOFF	= 0x13

.equ CR_	= 0x0d
.equ LF_	= 0x0a
.equ BS_	= 0x08
.equ TAB_	= 0x09

;;; memory mapping prefixes
.equ PRAM    = 0x0000                 ; 2 kB of RAM    (ATmega328)
.equ PEEPROM = RAMEND+1               ; 1 kB of EEPROM (ATmega328)

.if (FLASHEND == 0x1ffff)             ; 128 Kwords FLASH
	.equ OFLASH  = PEEPROM+EEPROMEND+1    ; 52 Kbytes available for OptiForth(atm2560)
	.equ PFLASH  = 0
	.equ RAMPZV  = 3
	.equ KERNEL_SIZE=0x0d80
.elif (FLASHEND == 0xffff)              ; 64 Kwords FLASH
	.equ OFLASH  = PEEPROM+EEPROMEND+1    ; 56 Kbytes available for OptiForth(atm128)
	.equ PFLASH  = 0
	.equ RAMPZV  = 1
	.equ KERNEL_SIZE=0x0d00
.elif (FLASHEND == 0x7fff)              ; 32 Kwords FLASH
	.equ OFLASH = PEEPROM+EEPROMEND+1     ; 56 Kbytes available for OptiForth
	.equ PFLASH = 0
	.equ RAMPZV  = 0
	.equ KERNEL_SIZE=0x0d00
.elif (FLASHEND == 0x1fff)              ; 8  Kwords FLASH
	.equ OFLASH = 0xC000                  ; 16 Kbytes available for OptiForth
	.equ PFLASH = OFLASH
	.equ RAMPZV  = 0
	.equ KERNEL_SIZE=0x0c80

;=========================ATmega328/Arduino UNO===================================
.elif (FLASHEND == 0x3fff)              ; 16 Kwords FLASH
	.equ OFLASH = 0x8000                  ; 32 Kbytes available for OptiForth
	.equ PFLASH = OFLASH
	.equ RAMPZV  = 0
	.set KERNEL_SIZE = 0x1101
;=================================================================================
.endif

	.if CR_with_LF == 1
		.set KERNEL_SIZE = KERNEL_SIZE +  0x02
	.endif
	.if CPU_LOAD_LED == 1
		.set KERNEL_SIZE = KERNEL_SIZE +  0x11
	.endif
	.if CPU_LOAD == 1
		.set KERNEL_SIZE = KERNEL_SIZE +  0x23
	.endif
	.if IDLE_MODE == 1
		.set KERNEL_SIZE = KERNEL_SIZE +  0x1a
	.endif
	.if DEBUG_FLASH == 1
		.set KERNEL_SIZE = KERNEL_SIZE +  0x0d
	.endif
	.if optimizeNUM == 1
		.set KERNEL_SIZE = KERNEL_SIZE +  0xd4
	.endif
	.if optimizingCOMPILER == 1
		.set KERNEL_SIZE = KERNEL_SIZE + 0x148
	.endif

.equ BOOT_SIZE   =0x100
.equ BOOT_START  =FLASHEND   - BOOT_SIZE + 1  ; atm128: 0xff00, atm328: 0x3f00 
.equ KERNEL_START=BOOT_START - KERNEL_SIZE

;;;  high values for memory areas
.equ FLASH_HI  = 0xffff - (BOOT_SIZE*2) - (KERNEL_SIZE*2)
.equ EEPROM_HI = PEEPROM + EEPROMEND
.equ RAM_HI    = RAMEND
        
;;; USER AREA for the OPERATOR task
.equ ursize=       RETURN_STACK_SIZE
.equ ussize=       PARAMETER_STACK_SIZE
.equ utibsize=     TIB_SIZE

;;; user variables and area
.equ us0=          -28         ; start of parameter stack
.equ ur0=          -26         ; start of ret stack
.equ uemit=        -24         ; user EMIT vector
.equ ukey=         -22         ; user KEY vector
.equ ukeyq=        -20         ; user KEY? vector
.equ ubase=        -18         ; number Base
.equ utib=         -16         ; TIB address
.equ utask=        -14         ; task area pointer
.equ ulink=        -12         ; task link
.equ ustatus=      -10
.equ uflg=         -9
.equ usource=      -8          ; two cells
.equ utoin=        -4          ; input stream
.equ ursave=       -2          ; saved ret stack pointer
.equ uhp=           0          ; hold pointer


;;; variables in EEPROM
.equ eeprom=       PEEPROM
.equ dp_start=     eeprom + 0x0000 ; TURNKEY
.equ dp_flash=     eeprom + 0x0002 ; FLASH dictionary pointer
.equ dp_eeprom=    eeprom + 0x0004 ; EEPROM dictionary pointer
.equ dp_ram=       eeprom + 0x0006 ; RAM dictionary pointer
.equ latest=       eeprom + 0x0008 ; pointer to latest dictionary word
.equ prompt=       eeprom + 0x000a ; deferred prompt
.equ ehere=        eeprom + 0x000c

;****************************************************
.dseg
.org SRAM_START
rbuf0:        .byte RX0_BUF_SIZE		; do not move rbuf0 away from SRAM_START
ibuf:         .byte PAGESIZEB			; must (!) be placed on page boundary (0x..00)
ivec:         .byte INT_VECTORS_SIZE	; must not (!) reach into next page

rx0queue:
rbuf0_wr:    .byte 2		.equ _wr0=0		.equ _wr1=1
rbuf0_rd:    .byte 2		.equ _rd0=2		.equ _rd1=3
rbuf0_lv:    .byte 2		.equ _lv0=4		.equ _lv1=5

.ifdef UCSR1A
	rx1queue:
	rbuf1_wr:    .byte 1
	rbuf1_rd:    .byte 1
	rbuf1_lv:    .byte 1
	rbuf1:       .byte RX1_BUF_SIZE
.endif

RAMvarBase:
dpSTART:    .byte 2		.equ _dpSTART	=dpSTART -RAMvarBase
; DP's and LATEST in RAM
dpFLASH:    .byte 2		.equ _dpFLASH	=dpFLASH -RAMvarBase
dpEEPROM:   .byte 2		.equ _dpEEPROM	=dpEEPROM-RAMvarBase
dpRAM:      .byte 2		.equ _dpRAM		=dpRAM   -RAMvarBase
dpLATEST:   .byte 2		.equ _dpLATEST	=dpLATEST-RAMvarBase

iaddrl:     .byte 1
iaddrh:     .byte 1

.ifdef RAMPZ
	iaddru:	    .byte 1
	ibaseu:	    .byte 1
.endif

.if CPU_LOAD == 1       
	load_res:	.byte 1			; load result [%]
.endif

litbuf0:    .byte 1				; used in COMPILE-state only
;litbuf1:    .byte 1
.equ litbuf1 = GPIOR2			; used in COMPILE-state only

LEAVEadr:	.byte 2				; used in COMPILE-state only
.equ _LEAVEadr	=LEAVEadr -RAMvarBase

;cse:    .byte 1 				; current data section 0=FLASH, 2=EEPROM, 4=RAM
.equ cse = GPIOR1
;state:   .byte 1 				; compilation state 0=INTERPRET, 1=COMPILE
;++ moved to fSTATE in FLAGS2

uvars:   .byte (-us0)
up0:     .byte 2
urbuf:   .byte ursize
usbuf:   .byte ussize
utibbuf: .byte utibsize
dpdata:

.eseg
.org 0
		.dw 0xffff				; force first cell of EEPROM to 0xffff

;*******************************************************************
; Start of kernel
;*******************************************************************
.cseg
.org KERNEL_START

	.if (FLASHEND == 0x1ffff)
        	fdw		PAUSE_L		; ### check ###
	WDON_L:
        	.db		NFA|3,"wd+"
	WDON:
        	cli
        	wdr
        	lds tosh,WDTCSR
        	ori tosh,(1<<WDCE)|(1<<WDE)
        	sts WDTCSR,tosh
        	andi tosl,7
        	ori tosl,(1<<WDE)
        	sts WDTCSR,tosl
        	sei
        	rjmp DROP

	; WD- ( -- )    stop the watchdog
			fdw		WDON_L
	WDOFF_L:
			.db		NFA|3,"wd-"
	WDOFF:
			cli
			wdr
		.ifdef MCUSR
					out MCUSR,r_zero
		.else
					out MCUCSR,r_zero
		.endif
	       ldi t0,(1<<WDCE)|(1<<WDE)
	       sts WDTCSR,t0
	       sts WDTCSR,r_zero
	       sei
	       ret

	; WDR ( -- )    kick the dog
	       fdw		WDOFF_L
	CWD_L:
	       .db		NFA|INLINE|3,"cwd"
	CWD:
	       wdr
	       ret
	.endif
;***************************************************
FLASHHI:
		.dw		FLASH_HI
		.dw		EEPROM_HI
		.dw		RAM_HI

MEMQADDR_N:
		fdw		ROM_N
		fdw		EROM_N
		fdw		FRAM_N

;;; ************************************************
;;; WARM user area data
.equ warmlitsize= 20
WARMLIT:
		.dw		utibbuf-4			; S0
		.dw		usbuf-1				; R0
		fdw		OP_TX_
		fdw		OP_RX_
		fdw		OP_RXQ
		.dw		BASE_DEFAULT		; BASE
		.dw		utibbuf				; TIB
		fdw		OPERATOR_AREA		; TASK
		.dw		up0					; Task link
;;; ************************************************
;;; EMPTY dictionary data
; *******************************************************************
.equ coldlitsize=12
COLDLIT:
STARTV: .dw		0
DPC:    .dw		OFLASH
DPE:    .dw		ehere
DPD:    .dw		dpdata
LW:     fdw		lastword
STAT:   fdw		DOTSTATUS
;*******************************************************************


; PAD ( -- a-addr )				user pad buffer
		fdw		PCSTORE_L
PAD_L:
		.db		NFA|3,"pad"
PAD:
		movw Z,UP				; TIB 
		sbiw Z,(-utib)			;	( TIU @ )
		pushtos
		ld t0,Z+
		ld t1,Z+
		ld tosl,Z+				; TIBSIZE 
		ld tosh,Z+				;	( TASK @ 8 + @ )
		adiw TOP,8
		movw Z,TOP
		sub_pflash_z
		lpm tosl,Z+
		lpm tosh,Z+

		add tosl,t0				; +
		adc tosh,t1
		ret						; 16 / 27+4


		fdw		BSLASH_L
; [ ( -- )						enter interpretive state
LEFTBRACKET_L:
		.db		NFA|IMMED|1,"["
;LEFTBRACKET:
		cbr FLAGS2,(1<<fSTATE)
		ret


		fdw		DOTS_L
DOTQUOTE_L:
		.db		NFA|IMMED|COMPILE|2,".",0x22,0xff
;DOTQUOTE:
		rcall SQUOTE
		rcall DOCOMMAXT
		fdw TYPE
		ret


		fdw		RX0_L
; RP@							fetch the return stack pointer
RPFETCH_L:
		.db		NFA|INLINE|COMPILE|3,"rp@"
;RPFETCH:
		pushtos
		in tosl,spl
		in tosh,sph				; 4 / 6
		ret


		fdw		CHAR_L
; CELL ( -- n )					size of one cell
CELL_L:
		.db		NFA|INLINE4|4,"cell",0xff
;CELL:
		pushtos
;CELL_0:
		ldi tosl,2
		ldi tosh,0				; 4 / 6
		ret


		fdw		DECIMAL_L
; ALIGNED ( addr -- a-addr )	align given addr
ALIGNED_L:
		.db		NFA|INLINE|7,"aligned"
;ALIGNED:
		adiw TOP,1
		cbr tosl,1				; 2 / 3
		ret


		fdw		CELLS_L
; CELL+ ( a-addr1 -- a-addr2 )	add cell size
;	2 + ;
CELLPLUS_L:
		.db		NFA|INLINE|5,"cell+"
;CELLPLUS:
		adiw TOP,2				; 1 / 2
		ret


		fdw		CHARPLUS_L
; CELLS ( n1 -- n2 )			cells->adrs units
CELLS_L:
		.db		NFA|INLINE|5,"cells"
;CELLS:
		lsl tosl
		rol tosh				; 2 / 2
		ret


		fdw		CHARS_L
; CHAR+ ( c-addr1 -- c-addr2 )	add char size
CHARPLUS_L:
		.db		NFA|INLINE|5,"char+"
;CHARPLUS:
		adiw TOP,1				; 1 / 2
		ret


		fdw		CMOVE_L
; CHARS ( n1 -- n2 )			chars->adrs units
CHARS_L:
		.db		NFA|INLINE|5,"chars"
;CHARS:
		ret


		fdw		RIGHTBRACKET_L
; \								skip the rest of the line
BSLASH_L:
		.db		NFA|IMMED|1,0x5c
;BSLASH:
		movw Z,UP
		sbiw Z,(-(usource+2))
		ld t0,Z+
		ld t1,Z+
		;adiw Z,<..utoin>		; Z already at utoin
		st Z+,t0
		st Z+,t1
		cbr FLAGS1,(1<<doclear)	; don't clear flags in case of \
		ret


		fdw		MEMHI_L
; ei ( -- )						enable interrupts
EI_L:
		.db		NFA|INLINE|2,"ei",0xff
		sei						; 1 / 1
		ret


		fdw		DO_L
; di  ( -- )					disable interrupts
DI_L:
		.db		NFA|INLINE|2,"di",0xff
		cli						; 1 / 1
		ret


		fdw		SIGNQ_L
; RSAVE ( -- a-addr )			saved return stack pointer
RSAVE_L:
		.db		NFA|INLINE4|5,"rsave"
;RSAVE_:
		inline_DOUSER ursave	; 4 / 7
		ret


		fdw		UNTIL_L
; ULINK ( -- a-addr )			link to next task
ULINK_L:
		.db		NFA|INLINE4|5,"ulink"
;ULINK_:
		inline_DOUSER ulink		; 4 / 7
		ret


		fdw		THEN_L
; TASK ( -- a-addr )			TASK pointer
TASK_L:
		.db		NFA|INLINE4|4,"task",0xff
;TASK:   
		inline_DOUSER utask		; 4 / 7
		ret


		fdw		ICOMMA_L
; HP ( -- a-addr )				HOLD pointer
HP_L:
		.db		NFA|INLINE|2,"hp",0xff
;HP:
		pushtos
		movw TOP,UP			; 3 / 5
		;sbiw TOP,(-uhp)		; uhp = 0
		ret

.if IDLE_MODE == 0
		fdw		CELL_L
.elif IDLE_MODE == 1
		fdw		BUSY_L
.else .error "illegal value: IDLE_MODE"
.endif
; BASE ( -- a-addr )			holds conversion radix
BASE_L:
		.db		NFA|INLINE4|4,"base",0xff
;BASE:
		inline_DOUSER ubase		; 4 / 7
		ret


		fdw		USER_L
; umin ( u1 u2 -- u )			unsigned minimum
;	2DUP U> IF SWAP THEN DROP ;
UMIN_L:
		.db		NFA|INLINE5|4,"umin",0xff
UMIN:
		pop_t1t0
UMIN_0:
		cp tosl,t0
		cpc tosh,t1
		brcs PC+2
		  movw TOP,t1t0			; 6 / 8
		ret


		fdw		UMIN_L
; umax ( u1 u2 -- u )			unsigned maximum
;	2DUP U< IF SWAP THEN DROP ;
UMAX_L:
		.db		NFA|INLINE5|4,"umax",0xff
UMAX:
		pop_t1t0
UMAX_0:
		cp t0,tosl
		cpc t1,tosh
		brcs PC+2
		  movw TOP,t1t0			; 6 / 8
		ret


		fdw 	TICKS_L
SWOPMINUS_L:
		.db		NFA|INLINE4|5,"swap-"
SWOPMINUS:
		pop_t1t0
SWOPMINUS_0:
		sub tosl,t0
		sbc tosh,t1				; 4 / 6
		ret


		fdw		BIN_L
AND_L:
		.db		NFA|INLINE4|3,"and"
AND_:
		pop_t1t0
AND_0:
		and tosl,t0
		and tosh,t1				; 4 / 6
		ret


		fdw		PSTORE_L
OR_L:
		.db		NFA|INLINE4|2,"or",0xff
OR_:
		pop_t1t0
OR_0:
		or tosl,t0
		or tosh,t1				; 4 / 6
		ret


		fdw		ZFL_L
XOR_L:
		.db		NFA|INLINE4|3, "xor"
XOR_:
		pop_t1t0
XOR_0:
		eor tosl,t0
		eor tosh,t1				; 4 / 6
		ret


		fdw		LATEST_L
INVERT_L:
		.db		NFA|INLINE|6,"invert",0xff
;INVERT:
		com tosl
		com tosh				; 2 / 2
		ret


		fdw		ONEMINUS_L
ONEPLUS_L:
		.db		NFA|INLINE|2,"1+",0xff
;ONEPLUS:
		adiw TOP,1				; 1 / 2
		ret


		fdw		TWOSTORE_L
ONEMINUS_L:
		.db		NFA|INLINE|2,"1-",0xff
;ONEMINUS:
		sbiw TOP,1				; 1 / 2
		ret


		fdw		TWOMINUS_L
TWOPLUS_L:
		.db		NFA|INLINE|2,"2+",0xff
;TWOPLUS:
		adiw TOP,2				; 1 / 2
		ret


		fdw		ABORT_L
TOBODY_L:
		.db		NFA|INLINE|5,">body"
;TOBODY:
		adiw TOP,4				; 1 / 2
		ret


		fdw		TWOPLUS_L
TWOSTAR_L:
		.db		NFA|INLINE|2,"2*",0xff
;TWOSTAR:
		lsl tosl
		rol tosh				; 2 / 2
		ret



		fdw		UDSTAR_L
UTWOSLASH_L:
		.db		NFA|INLINE|3,"u2/"
;UTWOSLASH:
		lsr tosh
		ror tosl				; 2 / 2
		ret


		fdw		TWOFETCH_L
TWOSLASH_L:
		.db		NFA|INLINE|2,"2/",0xff
;TWOSLASH:
		asr tosh
		ror tosl				; 2 / 2
		ret


		fdw		DUP_L
DTWOSLASH_L:
		.db		NFA|INLINE5|3,"d2/"
		pop_t1t0
;DTWOSLASH_0:
		asr tosh
		ror tosl
		ror t1
		ror t0
		push_t1t0				; 8 / 12
		ret


		fdw		DTWOSLASH_L
DTWOSTAR_L:
		.db		NFA|INLINE5|3,"d2*"
		pop_t1t0
;DTWOSTAR_0:
		lsl t0
		rol t1
		rol tosl
		rol tosh
		push_t1t0				; 8 / 12
		ret


		fdw		DZEROEQUAL_L
DZEROLESS_L:
		.db		NFA|INLINE4|3,"d0<"
;DZEROLESS:
		adiw Y,2				; NIP d.l
		lsl tosh
		sbc tosl,tosl
		sbc tosh,tosh			; 4 / 5
		ret


		fdw		FUNLOCK_L
;;; disable writes to FLASH and EEPROM
FLOCK_L:
		.db		NFA|INLINE|3,"fl-"
		sbi FLAGS3,fLOCK		; 1 / 1
		ret


		fdw		FOR_L
;;; enable writes to FLASH and EEPROM
FUNLOCK_L:
		.db		NFA|INLINE|3,"fl+"
		cbi FLAGS3,fLOCK		; 1 / 1
		ret

		fdw		MS_L
; fast RAM access
MFETCH_L:
   	    .db		NFA|INLINE|2,"m@",0xff
;MFETCH:
		movw Z,TOP
		ld tosl,Z+
		ld tosh,Z+			; 3 / 5
		ret


		fdw		MIN_L
MCFETCH_L:
   	    .db		NFA|INLINE|3,"mc@"
;MCFETCH:
		movw Z,TOP
		ld tosl,Z+
		ldi tosh,0x00		; 3 / 4
		ret


		fdw		FCY_L
; DUP must not be reachable from user code with rcall
DUP_L:
		.db		NFA|INLINE|3, "dup"
DUP:
		duptos					; 2 / 4
		ret


		fdw		ONEPLUS_L
ZEROEQUAL_L:
		.db		NFA|INLINE|2, "0=",0xff
ZEROEQUAL:
		sbiw    TOP, 1
		sbc     tosl, tosl
		sbc     tosh, tosh		; 3 / 4
		ret


		fdw		ZEROEQUAL_L
ZEROLESS_L:
		.db		NFA|INLINE|2, "0<",0xff
ZEROLESS:
		lsl     tosh
		sbc     tosl, tosl
		sbc     tosh, tosh		; 3 / 3
		ret


		fdw		XA_FROM_L
; store the execution vector addr to the return stack
; leave the updated return stack pointer on the data stack
; x>r ( addr rsp -- rsp' )
X_TO_R_L:
		.db		NFA|3,"x>r"
;X_TO_R:
		movw Z,TOP
		poptos
		sub_pflash_tos
		lsr tosh
		ror tosl
		st  Z,tosl
		st -Z,tosh
.ifdef EIND
			st -Z,r_one
.endif
		st -Z,r_zero
		movw TOP,Z
		ret


		fdw		FEXECUTE_L
TO_XA_L:
		.db NFA|INLINE|3,">xa"
;TO_XA:
		sub_pflash_tos
		;rampv_to_c
		;ror tosh
		lsr tosh
		ror tosl				; 3 / 3
		ret


		fdw		XOR_L
XA_FROM_L:
		.db NFA|INLINE|3,"xa>"
;XA_FROM:
		lsl tosl
		rol tosh
		add_pflash_tos			; 3 / 3
		ret


		 fdw    R_TO_P_L
PFL_L:
		.db		NFA|3,"pfl"
;PFL:
		rcall DOCREATE
		.dw OFLASH

		fdw    STORE_P_TO_R_L
;	.dw		0
ZFL_L:
		.db		NFA|3, "zfl"
;ZFL:
		rcall DOCREATE
		.dw RAMPZV


		fdw		FALSE_L
; leave clear top of return stack
ENDIT_L:
		.db		NFA|COMPILE|0x20|5,"endit"
;ENDIT:						; ++++ must be inlined ++++
		 in ZH,SPH
		 in ZL,SPL
		 std Z+1,r_zero
		 std Z+2,r_zero			; 4 / 6
		 ret


        fdw		 RSAVE_L
; RDROP compile a pop
RDROP_L:
        .db		NFA|COMPILE|0x20|5,"rdrop"
;RDROP:						;++++ must be inlined ++++
        pop     t0
        pop     t0				; 2 / 4
        ret


        fdw		SHB_L
STOD_L:
        .db		NFA|INLINE5|3,"s>d"
;STOD:
        duptos
        lsl tosh				; mov sign to C
        sbc tosl,tosl
		sbc tosh,tosh			; 5 / 7
		ret


        fdw		 HP_L
MEMHI_L:
        .db		NFA|2,"hi",0xff
;MEMHI:
		 pushtos
;MEMHI_0:
		 in ZL,cse
		 clr ZH
		 subi ZL, LOW(-(FLASHHI<<1))
		 sbci ZH,HIGH(-(FLASHHI<<1))
		 lpm tosl,Z+
		 lpm tosh,Z+
		 ret


        fdw		 A_FROM_L
L_FETCH_P:
        .db      NFA|INLINE|2,"@p", 0
;FETCH_P:
        pushtos
;FETCH_P_0:
        movw TOP,pl				; 3 / 5
        ret


        fdw		 PAD_L
; P2+  ( n -- )					add 2 to P
L_PTWOPLUS:
        .db		NFA|INLINE|3,"p2+"
;PTWOPLUS:
		subi pl,-2
		sbci ph,-1				; 2 / 2
        ret


        fdw		TX0_L
; TIU  ( -- a-addr )			Terminal Input Buffer user variable 
TIU_L:
        .db		NFA|INLINE4|3,"tiu"
;TIU:
		 inline_DOUSER utib		; 4 / 7
		 ret


        fdw		TO_PRINTABLE_L
; >IN  ( -- a-addr )			holds offset into TIB
; in RAM
TOIN_L:
        .db		NFA|INLINE4|3,">in"
TOIN:
		 inline_DOUSER utoin	; 4 / 7
		 ret


        fdw		SLASHSTRING_L
; 'SOURCE ( -- a-addr )			two cells: len, adrs
; in RAM
TICKSOURCE_L:
        .db		NFA|INLINE4|7,"'source"
;TICKSOURCE: 				; two cells !!!!!!
		 inline_DOUSER usource	; 4 / 7
		 ret


        fdw		PFETCH_L
PPLUS_L:
        .db		NFA|INLINE|2,"p+",0xff
;PPLUS:
        add pl,r_one
        adc ph,r_zero			; 2 / 2
        ret   


        fdw		L_PTWOPLUS
PNPLUS_L:
        .db		NFA|INLINE4|3,"p++"
;PNPLUS:
        add pl,tosl
        adc ph,tosh
        poptos					; 4 / 6
        ret


        fdw		UKEYQ_L
UEMIT_L:
kernellink_long:
        .db		NFA|INLINE4|5,"'emit"
;UEMIT_:
		 pushtos
UEMIT_0:
		 movw TOP,UP
		 sbiw TOP,(-uemit)		; 4 / 7
		 ret

        
        fdw		TWODUP_L
UKEY_L:
        .db		NFA|INLINE4|4,"'key",0xff
;UKEY_:
		 pushtos
UKEY_0:
		 movw TOP,UP
		 sbiw TOP,(-ukey)		; 4 / 7
		 ret


        fdw		TWODROP_L
UKEYQ_L:
        .db		NFA|INLINE4|5,"'key?"
;UKEYQ_:
		 pushtos
UKEYQ_0:
		 movw TOP,UP
		 sbiw TOP,(-ukeyq)		; 4 / 7
		 ret


        fdw		POSTPONE_L
OPERATOR_L:
        .db		NFA|8,"operator",0xff
OPERATOR:
		call DOCREATE			; ### 'call..' for 'tasks' to work ###
		fdw   OPERATOR_AREA
OPERATOR_AREA:					; description of operator task:
		.dw  up0				; location of the user area
		.dw  0		,ursize		; size of:  user variable area	, return stack	
		.dw  ussize	,utibsize	;			data stack			, tib


        fdw		MTST_L
; mset  ( mask addr -- )
;   dup >r c@ or r> c! ;
MSET_L:
        .db		NFA|INLINE5|4,"mset",0xff
MSET:
		pop_t1t0
MSET_0:
		movw Z,TOP
		ld tosl,Z
		or tosl,t0
		st Z,tosl
		poptos					; 8 / 14
		ret
        

        fdw		MSET_L
; mclr  ( mask addr -- )
;   dup >r c@ swap invert and r> c! ;
MCLR_L:
        .db		NFA|INLINE5|4,"mclr",0xff
MCLR:
		pop_t1t0
MCLR_0:
		movw Z,TOP
		ld tosl,Z
		com t0
		and tosl,t0
		st Z,tosl
		poptos					;  9 / 15
		ret


		fdw		PARSE_L
; mtst0 ( mask addr -- )		(NO flag on stack - to be used with '0until' and '0if')
;   c@ and drop ;
MTSTZ_L:
		.db		NFA|INLINE5|5,"mtst0"
MTSTZ:
		pop_t1t0				; mask in t0
MTSTZ_0:
		movw Z,TOP
		poptos
		ld t1,Z
		and t1,t0				; 7 / 12
		ret						; leave valid Z-flag


		fdw		NEXT_L
; mtst ( mask addr -- flag )
;   c@ and ;
MTST_L:
		.db		NFA|INLINE5|4,"mtst",0xff
MTST:
		pop_t1t0
MTST_0:
		movw Z,TOP
		ld tosl,Z
		and tosl,t0
		ldi tosh,0x00			; 6 / 9
		ret						; leav valid Z-flag


.if IDLE_MODE == 1
			fdw		IRQ_V_L
	IDLE_L:
			.db		NFA|INLINE|4,"idle",0xff
	;IDLE:
			sbr FLAGS2,(1<<fIDLE)	; 1 / 1
			ret

			fdw		CELL_L
	BUSY_L:
			.db		NFA|INLINE|4,"busy",0xff
	;BUSY:
			cbr FLAGS2,(1<<fIDLE)	; 1 / 1
			ret
.endif


        fdw		TICK_L
; #   ( ud1 -- ud2 )			convert 1 digit of output and put into PAD
;   base @ ud/mod rot >digit hold ;
NUM_L:
        .db		NFA|1,"#"
NUM:
.if optimizeNUM == 0
		movw Z,UP
		sbiw Z,(-ubase)
		ld t4,Z					; divisor
		ldi t5,0

		clr t2
		ldd t6,Y+0				; ud.l
		ldd t7,Y+1				; skip udslashmod0 for ud = 0x0000.0000.
		or t6,tosl				; (for leading zeros in UDOTR)
		or t7,tosh
		or t6,t7
		  breq NUM_aaa			; +7 / +9..-(320..380)

		rcall udslashmod0		; ud'.h: t7t6	rem: t3t2
		pushtos					; ud'.l
NUM_aaa:
		push_t7t6				; ud'.h
		movw TOP,t3t2			; remainder (digit)
		rcall TODIGIT
		rjmp HOLD

.elif optimizeNUM == 1
								; unsignd division 32 -- 32	, t0: rem8
			movw Z,UP
			sbiw Z,(-ubase)
			ld t4,Z					; base in t4
			ldd t6,Y+0
			ldd t7,Y+1				; ud.l: t7t6
									; ud.h: TOP
			mov t0,t6				; save LSB for digit
 			cpi t4,2
			  breq UDbinSLASHMOD
			cpi t4,16
			  breq UDhexSLASHMOD
			;cpi t4,8
			;  breq UDoctSLASHMOD	; (activate when needed)
			;cpi t4,4
			;  breq UDquadSLASHMOD	; (activate when needed)

			clr t5					; divisor.h (divisor.l = base)
			clr t0
			or t6,tosl
			or t7,tosh
			or t6,t7
			  breq UDbaseSM_00		; skip division for ud = 0x0000.0000. (for leading zeros in UDOTR)
				
	UDbaseSM_32:					; ( ud.l  ud.h )					u  : t5t4
			rcall udslashmod0		; ( ud'.l	   )	ud'h in t7:t6	rem: t3t2

			mov t0,t2				; rem/digit
			movw t3t2,TOP			; save ud'.l for zero check in NUMS
			pushtos
			movw TOP,t7t6			; ( ud'.l ud'.h  )
			;rjmp NUM_TODIGIT
	NUM_TODIGIT:
			cpi t0,0x0a
			brlt PC+2
			  subi t0,-0x27
			subi t0,-0x30
			rjmp NUM_HOLD

	UDhexSLASHMOD:					; 42+4
			lsr tosh
			ror tosl
			ror t7
			ror t6
	UDoctSLASHMOD:					; 40+4
			lsr tosh
			ror tosl
			ror t7
			ror t6
	UDquadSLASHMOD: 				; 38+4
			lsr tosh
			ror tosl
			ror t7
			ror t6
	UDbinSLASHMOD:					; 28+4	
			lsr tosh
			ror tosl
			ror t7
			ror t6
	
			dec t4					; base - 1
			and t0,t4				; separate digit
			std Y+1,t7
			std Y+0,t6
	UDbaseSM_00:
			movw t3t2,t7t6			; save ud'.l for zero check in NUMS
			rjmp NUM_TODIGIT
.else .error "illegal value: optimizeNUM"
.endif

	
        fdw		PLUSSTORE_L
; #S  ( ud1 -- 0. )				convert remaining digits
;   begin # 2dup or 0= until ;
NUMS_L:
        .db		NFA|2,"#s",0xff
NUMS:
.if optimizeNUM == 0
	NUMS_0:
          rcall NUM
		  ldd t0,Y+0			; ud' = 0 ?
		  ldd t1,Y+1
		  or t0,tosl
		  or t1,tosh
		  or t0,t1
        brne NUMS_0
        ret
.elif optimizeNUM == 1
			movw Z,UP
			sbiw Z,(-ubase)
			ld t4,Z

			cpi t4,10
			breq NUMS10
	NUMS_0:
			  rcall NUM				; ud'.l saved in t3t2
			  or t2,tosl			; ud' = 0 ?
			  or t3,tosh
			  or t2,t3
			brne NUMS_0
			ret

	NUMS10_loop:
			  rcall NUM
	NUMS10:
			  sbiw TOP,0
			brne NUMS10_loop		; ud.h <> 0 -> get a digit
									; ud.h =  0 -> ...
	NUMS_ten:					; 16-bit base10 conversion for u in NEXT
			ldd tosl,Y+0			; (keep stack space)
			ldd tosh,Y+1
	WordToDec5:						; (t1:t0:tosh:tosl:t3) = WordToDec5(TOP)

		.def x6     = R16			; t0
		.def x5536L = R16
		.def x5536H = R16
		.def x10    = R26			; t4

			ldi t1,'0'						; 10.000
			ldi x6,6						; multiply by 6,5536 to shift out the first digit
			mul tosl,x6											; * 6,5536 as:
						movw t7t6,R1:R0
			mul tosh,x6											; * 6
									add t7,R0		adc t1,R1
		
			ldi x5536L,0xb9										; .. + *0,553604126 (0x8db9)
			ldi t5,0x80											; enough for correct rounding
			mul tosh,x5536L
			add t5,R0	adc t6,R1	adc t7,r_zero	adc t1,r_zero
	
			ldi x5536H,0x8d
			mul tosh,x5536H
			sec			adc t6,R0	adc t7,R1		adc t1,r_zero
			mul tosl,x5536H
			add t5,R0	adc t6,R1	adc t7,r_zero	adc t1,r_zero	; 30
	
			ldi t0,'0'						;  1.000
			ldi x10,10						; multiply by 10 to shift out the second digit
			mul t6,x10
						movw TOP,R1:R0
			mul t7,x10
									add tosh,R0		adc t0,R1
			mul t5,x10
						add tosl,R1	adc tosh,r_zero	adc t0,r_zero	; 44

											; ......... third digit
			mul tosl,x10					;    100
						movw t7t6,R1:R0
			mul tosh,x10
			ldi tosh,'0'			add t7,R0	adc tosh,R1			; 52

											; ........ fourth digit
			ldi tosl,'0'					;     10
			mul t6,x10
									mov t5,R1
			mul t7,x10
									add t5,R0	adc tosl,R1			; 60
											; ......... fifth digit
			ldi t7,'0'						;      1
			mul t5,x10
												add R1,t7	; t3	; 64 

								; HOLD
			movw Z,UP				; t7:t6
			;sbiw Z,(-uhp)			; uhp = 0
			ld XL,Z+				; t4
			ld XH,Z+				; t5
			st -X,t3				; ....x HOLD
			inc #hold

			cpi t1,'0'										;
			  brne Nt_all									;	 	91		leading digit non-zero -> write all 5 digits
			  					; skip leading zeros
			cp  t1,t0				; at this point: t1 = '0'
			cpc t1,tosh
			cpc t1,tosl	
			  breq Nt_xxx									;		    95			
			st -X,tosl				; ...x. HOLD
			inc #hold

			cp  t1,t0
			cpc t1,tosh	
			  breq Nt_xxx									;			   101
			st -X,tosh				; ..x.. HOLD
			inc #hold

			cp  t1,t0	
			  breq Nt_xxx									;				   106
			st -X,t0				; .x... HOLD
			inc #hold

			rjmp Nt_xxx										;					   110

	Nt_all:
			st -X,tosl				; ...x. HOLD
			st -X,tosh				; ..x.. HOLD
			st -X,t0				; .x... HOLD
			st -X,t1				; x.... HOLD
			ldi t0,4
			add #hold,t0									;       99
	Nt_xxx:
			st -Z,XH
			st -Z,XL
			;;clr tosl 										; put 0x0000.0000. on stack
			;;clr tosh
			;;pushtos
			;sbiw Y,2										; let it be any double (will be 2drop-ed in '#>' anyway)
															; .. no need (stack space kept)
															;   	 5	 1	 2	 3	 4 #digits 
			ret												;      104  99 105 110 114
															;   	+4	+4	+4	+4	+4
.else .error "illegal value: optimizeNUM"
.endif


        fdw		SQUAREROOT_L
; SKIP  ( c-addr u c -- c-addr' u' )	skip matching chars
; c-addr must point to RAM
; u (count) must be smaller than 256
SKIP_L:
        .db		NFA|4,"skip",0xff
SKIP:
		 mov t0,tosL			; c in t0
		 poptos					; u in tosl
SKIP_0:
		 pop_Z					; c-addr in Z
SKIP_loop:
		   sbiw TOP,1
             brcs SKIP_leave
		   ld t1,Z+				; fetched char
		   cpi t1,TAB_
		     breq SKIP_loop
		   cp t1,t0
		 breq SKIP_loop
SKIP_mismatch:
SCAN_match:
		 sbiw Z,1				; set c-addr back to found match
SKIP_leave:
SCAN_leave:
		 adiw TOP,1
		 push_Z
		 ret


		fdw		SIGN_L
; SCAN  ( c-addr u c -- c-addr' u' )	find matching chars
; c-addr must point to RAM
SCAN_L:
		.db		NFA|4,"scan",0xff
SCAN:
		 mov t0,tosl			; c in t0
		 poptos					; u in tos
SCAN_0:
		 pop_Z					; c-addr in Z
SCAN_loop:
		   sbiw TOP,1
		     brcs SCAN_leave
		   ld t1,Z+
		   cpi t1,TAB_
		     breq SCAN_match
		   cp t1,t0
		 brne SCAN_loop
		 rjmp SCAN_match


		fdw		FLOCK_L
FCY_L:
		.db		NFA|3,"Fcy"
		rcall DOCREATE
		.dw FREQ_OSC/1000


		fdw		TWOCONSTANT_L
VARIABLE_L:
		.db		NFA|8,"variable",0xff
VARIABLE_:
		 ldi t1,2
VARIABLE_1:
		 rcall HERE				; (uses t0)
		 clr t0
		 add t1,tosl			; t0:t1 (++ high<>low swapped ++) 
		 adc t0,tosh
		 st -Z,t0				; (Z-pointer valid from HERE)
		 st -Z,t1
		 rjmp CONSTANT_


		fdw		IMMEDIATE_L
TWOVARIABLE_L:
		.db		NFA|9,"2variable"
TWOVARIABLE_:
		 ldi t1,4
		 rjmp VARIABLE_1


		fdw		EIGHTRSHIFT_L
EIGHTLSHIFT_L:
		.db		NFA|INLINE|3,"8<<"
;EIGHTLSHIFT:
		mov tosh,tosl
		ldi tosl,0				; 2 / 2
		ret
	
		fdw		TOIN_L
EIGHTRSHIFT_L:
		.db		NFA|INLINE|3,"8>>"
;EIGHTRSHIFT:
		mov tosl,tosh
		ldi tosh,0				; 2 / 2
		ret

		fdw		TO_A_L
; ><							swap bytes
SWAPB_L:
		.db		NFA|INLINE|2,"><",0xff
;SWAPB:
		mov t0,tosl
		mov tosl,tosh
		mov tosh,t0				; 3 / 3
		ret


		fdw		IHERE_L
; set the current memory section
; ..FLASH
FLASH_L:
ROM_N:  
		.db		NFA|INLINE|5,"flash"
;ROM_:
		out cse,r_zero			; 1 / 1
		ret

		fdw		IFLUSH_L
; ..EEPROM
EEPROM_L:
EROM_N: 
		.db		NFA|INLINE|6,"eeprom",0xff
;EROM:
		ldi t0,2
		out cse,t0				; 2 / 2
		ret

		fdw		ROT_L
; ..RAM
RAM_L:
FRAM_N: 
		.db		NFA|INLINE|3,"ram"
FRAM:
		ldi t0,4
		out cse,t0				; 2 / 2
		ret


		fdw		EI_L
; DP  ( -- a-addr )				current dictionary pointer (in RAM)
DP_L:
		.db		NFA|INLINE5|2,"dp",0xff
DP:
		 pushtos
DP_0:
		 in tosl,cse
		 clr tosh
		 subi tosl, LOW(-dpFLASH)
		 sbci tosh,HIGH(-dpFLASH)	; 6 / 8
		 ret


		fdw		HOLD_L
; HERE  ( -- addr )				get current data space ptr
;   DP @ ;
HERE_L:
		.db		NFA|4,"here",0xff
HERE:
		 pushtos
HERE_0:
		 in ZL,cse
		 clr ZH
		 subi ZL, LOW(-dpFLASH)
		 sbci ZH,HIGH(-dpFLASH)
		 ld tosl,Z+
		 ld tosh,Z+				; 8 / 10
		 ret


		fdw		CWD_L
COMMAXT_L:
		.db		NFA|3, "cf,"
COMMAXT:
		duptos
		lds t0,dpFLASH
		lds t1,dpFLASH+1

  .if FLASHEND < 0x4000
			cpi tosh,0xf0
			brcs COMMAXT_xxx
			  ;cbr tosh,HIGH(PFLASH)	; 'rcall ...' with wrap around at flashhi
			  andi tosh,0x0f
			  ori tosh,HIGH(PFLASH-0x1000)
	COMMAXT_xxx:
  .endif

		sub tosl,t0
		sbc tosh,t1
		;sbrc tosh,7			; (size vs. speed)
		;  rcall negate
		brpl COMMAXT_yyy
		  com tosl
		  com tosh
		  adiw TOP,1
COMMAXT_yyy:
		cpi tosl,0xfc
		ldi tosl,0x0f
		cpc tosh,tosl
		poptos
		  brcs STORECF1
STORECFF1: 
		sub_pflash_tos			; compile 'call ..'
		lsr tosh
		ror tosl
STORECFF2:
		pushtos
  .ifdef EIND
			;.dw     0x940F 	; on ATmega 2560 all code is on 128 - 256 kword area.
			ldi tosl,0x0f		; 'ecall ..'
			ldi tosh,0x94
  .else
			;.dw     0x940E
			ldi tosl,0x0e  		; 'call ..'
			ldi tosh,0x94
  .endif
		rcall ICOMMA_
		rjmp ICOMMA_

STORECF1:
		;rcall IHERE			; compile 'rcall ..'
		;rcall MINUS			;  2 / 29
;		lds t0,dpFLASH
;		lds t1,dpFLASH+1		; (still there)
		sub tosl,t0
		sbc tosh,t1
		sbiw TOP,2
		asr tosh
		ror tosl
		andi tosh,0x0f
		ori tosh,0xd0
ICOMMA___:
		rjmp ICOMMA_


		fdw		TWOSWAP_L
; 2DROP ( x1 x2 -- )			drop 2 cells
;	DROP DROP ;
TWODROP_L:
		.db		NFA|INLINE|5,"2drop"
TWODROP:
		adiw Y,2				; NIP
		poptos					; 3 /  6
		ret


		fdw		BASE_L
; 2DUP  ( x1 x2 -- x1 x2 x1 x2 )	dup top 2 cells
;   OVER OVER ;
TWODUP_L:
		.db		NFA|INLINE5|4,"2dup",0xff
TWODUP:
		ldd t0,Y+0
		ldd t1,Y+1
		duptos
		push_t1t0				; 6 / 12
		ret


		fdw		TOBODY_L
; 2SWAP  ( x1 x2 x3 x4 -- x3 x4 x1 x2 )
TWOSWAP_L:
		.db		NFA|5,"2swap"
TWOSWAP:
		movw t7t6,TOP
		pop_t5t4
		poptos
		pop_t1t0
		push_t5t4
		push_t7t6
		push_t1t0				; 13 / 25
		ret


		fdw		STATE_L
; SPACE ( -- )					output a space
;	BL EMIT ;
SPACE_L:
		.db		NFA|5,"space"
SPACE_:  
		ldi t0,' '
		rjmp EMIT_t0


		fdw		TICKSCOMPUTE_L
; SPACES ( n -- )				output n spaces
;	BEGIN DUP WHILE SPACE 1- REPEAT DROP ;
SPACES_L:
		.db		NFA|6,"spaces",0xff
SPACES:
SPCS1:
		sbiw TOP,0
		breq SPCS2
SPCS3:
		  rcall SPACE_
		  sbiw TOP,1
		brne SPCS3
SPCS2:  
		rjmp DROP


		fdw		DUMP_L
DROP_L:
		.db		NFA|INLINE|4,"drop",0xff
DROP:
		poptos					; 2 / 4
		ret


		fdw		TASK_L
SWOP_L:
		.db		NFA|INLINE5|4,"swap",0xff
SWOP:
		movw t1t0,TOP
		poptos
		push_t1t0				; 5 / 9
		ret


		fdw		QUIT_L
; ( xu ... x0 u -- xu ... x0 xu )
PICK_L:
		.db		NFA|INLINE5|4,"pick",0xff
PICK:
		lsl tosl
		movw Z,Y
		add ZL,tosl
		adc ZH,r_zero
		ld tosl,Z+
		ld tosh,Z+				; 8 / 10
		ret


		fdw 	PICK_L
OVER_L:
		.db		NFA|INLINE4|4,"over",0xff
OVER:
		pushtos
		ldd tosl,Y+2
		ldd tosh,Y+3			; 4 / 8
		ret


		fdw		FETCHPP_L
TOR_L:
		.db		NFA|COMPILE|0x20|2,">r",0xff
;TOR:					;++++ must be inlined ++++
		 push tosl
		 push tosh
		 poptos					; 4 /  8
		 ret

		fdw		RFETCH_L
RFROM_L:
		.db		NFA|COMPILE|0x20|2,"r>",0xff
;RFROM:					;++++ must be inlined ++++
		 pushtos
		 pop tosh
		 pop tosl				; 4 /  8
		 ret


		fdw		SQUOTE_L
RFETCH_L:
		.db		NFA|COMPILE|0x20|2,"r@",0xff
;RFETCH:				;++++ must be inlined ++++
		 pushtos
		 in ZL,SPL
		 in ZH,SPH
		 ldd tosl,Z+2
		 ldd tosh,Z+1			; 6 / 10
		 ret				

		fdw		AND_L
; ABS ( n -- |n| )				absolute value of n
ABS_L:
		.db		NFA|INLINE|3,"abs"
ABS_:
		 tst tosh
		 brpl ABS_1
		   com tosl
		   com tosh
		   adiw TOP,1
ABS_1:							; 5 / 3..6
		 ret


		fdw		COMMA_L
PLUS_L:
		.db		NFA|INLINE4|1, "+"
PLUS:
		pop_t1t0
PLUS_0:
		add tosl,t0
		adc tosh,t1				; 4 / 6
		ret


		fdw		ALLOT_L
; ALIGN ( -- )					align DP
ALIGN_L:
		.db		NFA|5,"align"
;ALIGN:
		 rcall HERE
		 adiw TOP,1
		 cbr tosl,1
		 rjmp ALLOT_1			; Z-pointer valid from HERE


		fdw		CREATE_L
; ACCEPT  ( c-addr u -- u' )	get line or max u char from terminal and put into TIB
ACCEPT_L:
		.db		NFA|6,"accept",0xff
ACCEPT:
		movw t1t0,TOP
		ldd tosl,Y+0
		ldd tosh,Y+1
		add t0,tosl				; ( c-addr c-addr ) end-addr in t1:t0
		adc t1,tosh
ACC0:
		push t0
		push t1
ACC1:
		  rcall KEY_A			; ( c-addr c-addr c )

		  movw X,UP
		  sbiw X,(-uflg)

		  cpi tosl, LF_
			breq ACC_LF
		  cpi tosl, BS_
			breq ACC_BS_DEL
		  cpi tosl, 127
			breq ACC_BS_DEL
		  cpi tosl, CR_
			brne ACC3
ACC_CR:
		  st X,tosl
		  poptos
;			rjmp ACC6
ACC6:
		  pop t0
		  pop t0
		  rjmp SWOPMINUS

ACC_BS_DEL:
		  st X,r_zero
		  poptos
		  ldd t0,Y+0
		  ldd t1,Y+1
		  sub t0,tosl
		  sbc t1,tosh
		breq ACC1
		  sbiw TOP,1
		  rcall XSQUOTE
		  .db   3,8,0x20,8
		call TYPE
		rjmp ACC1

ACC_LF:
		  poptos
		  ld t0,X
		  tst t0
			breq ACC6
		  st X,r_zero
		rjmp ACC1

ACC3:
		  mov t0,tosl
		  rcall EMIT_t0
		  pop_X
		  st X+,tosl
		  movw TOP,X

		  pop t1
		  pop t0
		  cp  t0,tosl
		  cpc t1,tosh
		brne ACC0
		rjmp SWOPMINUS

		fdw		SQUARE_L
; SP@ ( -- addr )				get parameter stack pointer
SPFETCH_L:
		.db		NFA|INLINE|3,"sp@"
SPFETCH:
		movw Z,Y
		pushtos
		movw TOP,Z				; 4 / 6
		ret


		fdw		S0_L
SQUOTE_L:
		.db      NFA|IMMED|COMPILE|2,"s",0x22,0xff
SQUOTE:
		rcall DOCOMMAXT
		fdw  XSQUOTE
		out cse,r_zero			; 'flash'
		rcall CQUOTE
		rjmp  FRAM


		fdw		MINUS_FETCH_L
CQUOTE_L:
		.db		NFA|2,",",0x22,0xff
CQUOTE: 
		pushtos
		ldi tosl,'"'
		;ldi tosh,0
		rcall PARSE
		rcall HERE
		rcall OVER
		adiw TOP,2
		cbr tosl,1
		rcall ALLOT
		rjmp  PLACE


		fdw     FIND_L
; EXIT							compile a return
;		variable link
EXIT_L:
		.db		NFA|COMPILE|IMMED|4,"exit",0xff
EXIT:
		rjmp SEMICOLON_0


		fdw		TICKSOURCE_L
; within ( x min max -- f )		min <= x < max ?
WITHIN_L:
		.db		NFA|6,"within",0xff
;WITHIN:
		pop_t1t0				; min
		pop_t5t4				; x
								; max in TOP
		cp  t0,t4
		cpc t1,t5
		breq PC+2
		  brcc WITHIN_flag		; C=0 -> x < min -> put FALSE
		cp  t4,tosl
		cpc t5,tosh				; C=0 -> max <= x -> put FALSE
WITHIN_flag:					; C=1 -> min <= x < max -> put TRUE
		sbc tosl,tosl			; .. make it a stack-flag
		sbc tosh,tosh
		ret						; 12 / 14..15+4


		fdw		SWAPB_L
NOTEQUAL_L:
		.db		NFA|INLINE4|2,"<>",0xff
NOTEQUAL:
		 pop_t1t0
NOTEQUAL_0:
		 sub tosl,t0
		 sbc tosh,t1			; 4 / 6
		 ret


		fdw		GREATER_L
EQUAL_L:
		.db		NFA|INLINE5|1, "="
EQUAL:
		 pop_t1t0
EQUAL_0:
		 sub tosl,t0
		 sbc tosh,t1
		 sbiw TOP,1
		 sbc tosl,tosl
		 sbc tosh,tosh			; 7 / 10
		 ret


		fdw		EQUAL_L
LESS_L:
		.db		NFA|1,"<"
LESS:
		pop_t1t0
;LESS_0:						; #### check LESSC_ when changing ####
		sub t0,tosl
		sbc t1,tosh
		brvc PC+2
		  com t1
		lsl t1
		sbc tosl,tosl
		sbc tosh,tosh			; 9 / 11
		ret


		fdw		FETCH_L
GREATER_L:
		.db		NFA|1,">"
GREATER:						; #### check GREATERC_ when changing ####
		pop_t1t0
;GREATER_0:
		sub tosl,t0
		sbc tosh,t1
;GREATER_1:
		brvc PC+2
		  com tosh
		lsl tosh
		sbc tosl,tosl
		sbc tosh,tosh			; 9 / 11
		ret


		fdw		UGREATER_L
ULESS_L:
		.db		NFA|INLINE5|2,"u<",0xff
ULESS:							; #### check ULESSC_ when changing ####
		pop_t1t0
;ULESS_0:
		cp  t0,tosl
		cpc t1,tosh
		sbc tosl,tosl
		sbc tosh,tosh			; 6 / 8
		ret


		fdw		UPTR_L
UGREATER_L:
		.db		NFA|INLINE5|2, "u>",0xff
UGREATER:						; #### check UGREATERC_ when changing ####
		 pop_t1t0
;UGREATER_0:
		 cp  tosl,t0
		 cpc tosh,t1
		 sbc tosl,tosl
		 sbc tosh,tosh			; 6 / 8
		 ret


		fdw		NUMGREATER_L
STORE_P_L:
		.db		NFA|INLINE|2,"!p",0xff
STORE_P:
		movw P,TOP
		poptos
		ret

.if optimizingCOMPILER == 1
	TO_A_C_:						; fdw TO_A in TOP
	TO_A_C_0:
			rcall ldi16_t1t0_C_
			ldi16 tos,0x0158		; 'movw A,t1t0'
			rjmp ICOMMA_

	STORE_P_TO_R_C_:				; fdw STORE_P_TO_R in TOP
			ldi t0,8
			rcall IDPMINUS
			ldi16 tos,0x934f		; 'push pl'
			rcall ICOMMA_
			pushtos
			ldi16 tos,0x935f		; 'push ph'
			rcall ICOMMA_
;			ldi t0,2				; point to 'ldi tosh,<lit.1>'
		ldi t0,2
		ldi t1,regP
		rjmp ldi16_thtl_C_0

	STORE_P_C_:						; (fdw STORE_P) | (fdw STORE_P_TO_R) in TOP
			ldi t1,regP				; -> P
		rcall ldi16_thtl_C_
		rjmp DROP

.endif
		
		fdw		UKEY_L
STORE_P_TO_R_L:
		.db		NFA|COMPILE|0x20|4,"!p>r",0xff
STORE_P_TO_R:				;++++ must be inlined ++++
		push pl
		push ph
		movw P,TOP
		poptos					; 5 / 9
		ret


		fdw		RAM_L
R_TO_P_L:
		.db		NFA|COMPILE|0x20|3,"r>p"
;R_TO_P:					;++++ must be inlined ++++
		pop ph
		pop pl
		ret						; 2 / 4


;		.db		NFA|INLINE|3,"?0="
ZEROSENSE:
		or tosh,tosl
		poptos
		ret


		fdw		TO_XA_L
; >pr  ( c -- c )				filter a character to printable 7-bit ASCII
TO_PRINTABLE_L:
		.db		NFA|INLINE5|3,">pr"
TO_PRINTABLE:
		clr tosh
		cpi tosl,0x20
		  brcs nonPRINT
		sbrc tosl,7
nonPRINT: ldi tosl,'.'			; 5 / 5
		ret


		fdw		DOT_L
MINUS_L:
		.db		NFA|INLINE5|1, "-"
MINUS:
		pop_t1t0
;MINUS_0:
		sub t0,tosl
		sbc t1,tosh
		movw TOP,t1t0			; 5 / 7
		ret


		fdw 	PNPLUS_L
NIP_L:
		.db		NFA|INLINE|3,"nip"
NIP:
		adiw Y,2				; 1 / 1
		ret
    

		fdw		TYPE_L
; TUCK
;   swap over ;
TUCK_L:
		.db		NFA|INLINE5|4,"tuck",0xff
TUCK:
		pop_t1t0
		duptos
		push_t1t0				; 6 / 12
		ret


		fdw		MICROS_L
UPTR_L:
		.db		NFA|INLINE4|2,"up",0xff
UPTR:
		pushtos 				; up in R3:R2 -> addr = 0x0002
UPTR_0:
		ldi tosl,2
		ldi tosh,0				; 4 / 6
		ret

.if IDLE_MODE == 0
		fdw		IRQ_V_L
.elif IDLE_MODE == 1
		fdw		IDLE_L
.else .error "ilegal value: IDLE_MODE"
.endif
HOLD_L:
		.db		NFA|4,"hold",0xff
HOLD:
		mov t0,tosl
		poptos
NUM_HOLD:
		movw Z,UP
		;sbiw Z,(-uhp)			; uhp = 0 !!
		ld XL,Z+
		ld XH,Z+
		st -X,t0
		inc #hold
		st -Z,XH
		st -Z,XL
		ret


		fdw		NOTEQUAL_L
; <# ( -- )						begin numeric conversion
;	PAD HP ! ;					(initialize hold pointer)
LESSNUM_L:
		.db		NFA|2,"<#",0xff
LESSNUM:
		rcall PAD
		movw Z,UP
		std Z+0,tosl
		std Z+1,tosh
		clr #hold
		rjmp DROP				; 6 / 46+4


		fdw		DOES_L
; digit ( n -- c )				convert to 0..9 a..z
TODIGIT_L:
		.db		NFA|INLINE4|5,"digit"
TODIGIT:
		cpi tosl,0x0a
		brcs PC+2
		  subi tosl,-0x27
		subi tosl,-0x30			; 4 / 4
		ret


		fdw		NUMS_L
; #>  ( ud1 -- c-addr u )		end conversion, get string
;   2drop hp @ pad over - ;
NUMGREATER_L:
		.db		NFA|2,"#>", 0
NUMGREATER:
		adiw Y,2				; "2drop"
		;poptos
		;pushtos				; HP @
		movw Z,UP
		;sbiw Z,(-uhp)			; (uhp = 0)
		ld tosl,Z+
		ld tosh,Z+
		pushtos
		mov tosl,#hold			;  8 / 12+4
		ret			


.if optimizeNUM == 1
	UDOTR_ten:
			push XL					; print count
			push XH					; leading char when <> 0 (optional)
			rcall NUMS_ten			; Z points to uhp, X points to leading digit in PAD
			pop tosh				; leading char
			ldi t1,'0'
			cpse tosh,r_zero
			  mov t1,tosh
			pop tosl				; print count
			sub tosl,#hold			; sub # of converted digits
			  breq allDOT_finish
			  brcc UDOTR_ten_more

	UDOTR_ten_less:
			sub XL,tosl				; mov X back skipping unused leading digits
			sbci XH,-1
			add #hold,tosl
			rjmp UDOTR_ten_xxx

	UDOTR_ten_more:
			  st -X,t1				; append leading char to PAD
			  inc #hold
			dec tosl
			brne UDOTR_ten_more

	UDOTR_ten_xxx:
			st Z+,XL
			st Z+,XH
	UDOTR_ten_done:
			rjmp allDOT_finish
.endif

		fdw		UTWOSLASH_L
; U.R  ( u +n -- )				display u unsigned in field of n. 1<n<=255 
;    0 swap <# 1- for # next #s #> type space ;
UDOTR_L:
		.db		NFA|3,"u.r"
UDOTR:
		movw X,TOP				; n <= 255 -> byte loop count in XL
		;poptos
		;pushtos				; ZERO 
		clr tosl
		clr tosh
		rcall LESSNUM			; (leaves Z = UP)
	  .if optimizeNUM == 1
			;movw Z,UP
			sbiw Z,(-ubase)
			ld t0,Z
			cpi t0,10
			  breq UDOTR_ten
	  .endif
		rjmp UDOTR_2
UDOTR_loop:
		  push XL
		  rcall NUM
		  pop XL				; byte loop count
UDOTR_2: 
		  subi XL,1
		brcc UDOTR_loop

		rjmp allDOT_finish


		fdw		SKIP_L
; SIGN ( n -- )					add minus sign if n<0
;	0< IF 2D HOLD THEN ;
SIGN_L:
		.db		NFA|4,"sign",0xff
SIGN:
		sbrs tosh,7
		  rjmp DROP
		ldi tosl,'-'
		rjmp HOLD


		fdw		USLASH_L
; U.  ( u -- )					display u unsigned
;   <# 0 #S #> TYPE SPACE ;
UDOT_L:
		.db		NFA|2,"u.",0xff
UDOT:
		rcall ZERO
		;pushtos				; (speed vs. size)
		;clr tosl
		;clr tosh				; +3 / -7
UDOT_0:
		rcall LESSNUM			;   1 /  53
		;call PAD				; (speed vs. size)
		;movw Z,UP
		;st Z+,tosl
		;st Z+,tosh
		;clr #hold
		;poptos					;  +6 /  -9
		;<inline PAD>			; +16 / -25

		rcall NUMS

		ldi t0,'-'
		sbrc FLAGS2,fDOTsign
		  rcall NUM_HOLD		; append '-' sign
		cbr FLAGS2,(1<<fDOTsign)

allDOT_finish:
		rcall NUMGREATER
		rcall TYPE
SPACE_A:
		rjmp SPACE_


		fdw		UMSTAR_L
UDDOT_L:
		.db		NFA|3,"ud."
;UDDOT:
		rjmp UDOT_0


		fdw		SLASH_L
; . ( n -- )					display n signed
;	<# DUP ABS #S SWAP SIGN #> TYPE SPACE ;
DOT_L:
		.db		NFA|1,"."
DOT:
		tst tosh
		brpl UDOT
		  sbr FLAGS2,(1<<fDOTsign)
		  com tosl
		  com tosh
		  adiw TOP,1
		  rjmp UDOT


		fdw		DLESS_L
DDOT_L:
		.db		NFA|2,"d.",0
;DDOT:
		tst tosh
		brpl UDOT_0
		  sbr FLAGS2,(1<<fDOTsign)
		  call DNEGATE
		  rjmp UDOT_0
	  

		fdw		OPERATOR_L
CONSTANT_L:
		.db		NFA|8,"constant",0xff
CONSTANT_:
.if optimizingCOMPILER == 0
		 rcall CREATE0
		 rcall LITERAL_A		; compile 'pushtos   ldi tosl,..   ldi tosh,..'
		 rjmp ADD_RETURN_1

.elif optimizingCOMPILER == 1
		 	sbr FLAGS2,(1<<fIMMED)	; compile an 'immediate' word
			;rcall COLON
			rcall CREATE0
		 	rcall LITERAL_A			; compile 'pushtos   ldi tosl,..   ldi tosh,..'
			pushtos
			ldi16 tos,((CONST_yyy<<1)+PFLASH)
			rcall INLINE0			; compile CONST_yyy-sequence
			;cbr FLAGS2,(1<<fSTATE)	; SEMICOLON
		 	rjmp ADD_RETURN_1

	CONST_yyy:
			sbrc FLAGS2,fSTATE
			;-------------------------------------------------------------------------
			  jmp LITERAL			; 'compile' -> compile lit (### must be 'jmp' ###)
			;-------------------------------------------------------------------------
			ret						; 'interpret' - > ready
.else .error "illegal value: optimizingCOMPILER"
.endif

		fdw		TWOVARIABLE_L
TWOCONSTANT_L:
		.db		NFA|9,"2constant"
TWOCONSTANT_:
		rcall SWOP
		rcall CREATE0
		rcall LITERAL_A
		rcall LITERAL_A
		rjmp ADD_RETURN_1


		fdw		WARM_L
; USER  ( n -- )				create user variable
USER_L:
		.db		NFA|4,"user",0xff
;USER:
	  .if optimizingCOMPILER == 1
		sbr FLAGS2,(1<<fINLINE)	; compile 'INLINE'-marked user variable
	  .endif
		 rcall CREATE0
		 rcall LITERAL_A
		 cbr FLAGS1,(1<<fLIT)|(1<<f2LIT)
		 pushtos
		 ldi16 tos,((XDOUSER<<1)+PFLASH)
		 rcall INLINE0
		 rjmp ADD_RETURN_1

XDOUSER:
		 add tosl,upL
		 adc tosh,upH
		 ret


		fdw		PAUSE_L
; PARSE  ( char -- c-addr u )
PARSE_L:
		.db		NFA|5,"parse"
PARSE:
		movw Z,UP
		sbiw Z,(-usource)
		ld XL,Z+
		ld XH,Z+				; a in X
		pushtos
		ld tosl,Z+
		ld tosh,Z+				; ( c u )
		ld t0,Z+
		ld t1,Z+				; n in t1:t0
		add XL,t0
		adc XH,t1				; a = a + n in X
		push_X					; ( c a u )
		sub tosl,t0				; u = u - n (new tib len)
		sbc tosh,t1				; ( c a u )
		push tosl				; 'dup >r'
		push tosh				; ( c a u			R: u )
		ldd t0,Y+2				; c in t0
		rcall SKIP_0			; ( c astart u' )	Z: astart	t0: c
		push ZL					; ( c astart u'		R: u astart )
		push ZH
		adiw Y,4				; ( u' )			Z: astart	t0: c
		rcall SCAN_loop			; ( aend u" )		Z: aend
		sbiw TOP,0
		breq PC+2
		  sbiw TOP,1
		pop t3
		pop t2					; astart in t3:t2
		sub ZL,t2
		sbc ZH,t3				; aend-astart (wlen)
		std Y+1,t3
		std Y+0,t2				; ( astart u" )
		pop t1
		pop t0					; u
		sub t0,tosl
		sbc t1,tosh				; u = u - u" in t1:t0
		movw TOP,Z				; ( astart wlen )
		movw Z,UP
		sbiw Z,(-utoin)
		ld XL,Z+				; u >in +!
		ld XH,Z+
		add XL,t0
		adc XH,t1
		st -Z,XH
		st -Z,XL
		ret						; ( astart wlen )
     

;		fdw		UEMIT_L
	.dw		0
; WORD ( char -- c-addr )		word delimited by char and/or TAB
WORD_L:
		.db		NFA|4,"word",0xff
WORD:
		rcall PARSE				; c-addr wlen
		pop_Z
		st -Z,tosl
		tst tosl				; leave Z-flag for caller
		movw TOP,Z
		ret


		fdw		INLINE_L
; IMMED? ( nfa -- f )			fetch immediate flag
IMMEDQ_L:
		.db		NFA|6,"immed?",0xff
IMMEDQ: 
		rcall CFETCH_A
		mov wflags,tosl  		; COMPILE and INLINE flags for the compiler
		andi tosl,IMMED
FINDx: 
		ret


		fdw		HERE_L
; FIND	( c-addr -- c-addr 0 )	if not found
;		(			xt     1 )	if immediate
;		(			xt    -1 )	if normal
; 3 chains for kernel words to speed up search
FIND_L:
		.db		NFA|4,"find",0xff
FIND:   
		duptos
FIND_0:
		movw Z,TOP
		ld t0,Z+
		ldi16 tos,((kernellink_short<<1)+PFLASH)
		cpi t0,3
		  brcs FIND_1
		ldi16 tos,((kernellink_mid<<1)+PFLASH)
		cpi t0,5
		  brcs FIND_1
		ldi16 tos,((kernellink_long<<1)+PFLASH)
FIND_1:
		rcall findi
		  brne FINDx
		lds16 tos,dpLATEST
		rjmp findi


		fdw		TIB_L
; TI# ( -- n )					size of TIB
;	ti# task @ 8 + @ ;
TIBSIZE_L:
		.db		NFA|3,"ti#"
TIBSIZE:
		pushtos
TIBSIZE_0:
		movw Z,UP
		sbiw Z,(-utask)
		ld tosl,Z+
		ld tosh,Z+
		adiw TOP,8
		rjmp FETCH_A


		fdw		TIU_L
; TIB ( -- a-addr )				Terminal Input Buffer
TIB_L:
		.db		NFA|INLINE5|3,"tib"
TIB:
		pushtos
TIB_0:
		movw Z,UP
		sbiw Z,(-utib)
		ld tosl,Z+
		ld tosh,Z+				; 6 / 11
		ret


		fdw		QABORTQ_L
; >NUMBER ( 0 0 addr u -- ud.l ud.h addr' u' )
;										  0  ) done
;										<>0  ) not a number
;								 convert string to number (u < 0x100)
TONUMBER_L:
		.db		NFA|7,">number"
;TONUMBER:
		movw Z,UP
		sbiw Z,(-ubase)
		ld ah,Z					; base in ah
TONUMBER_0:
		mov al,r_one			; init 'not a number'-flag

TONUM_loop:
		  tst tosl	      		; ( ud.l ud.h addr u )
			breq TONUM_exit
		  push tosl				; (byte value)		
		  poptos				; ( ud.l ud.h addr		R: u )
		  push tosl
		  push tosh				; ( ud.l ud.h addr		R: u addr )
		  rcall CFETCH_A
		  cpi tosl,'.'
			breq TONUM_next
;TONUM_digitQ:				; inline DIGITQ
		  cpi tosl,0x3a			; ( ud.l ud.h c )
			brcs TONUM_dQ1
		  cpi tosl,0x61
		    brcs TONUM_noDIGIT
		  subi tosl,0x27
TONUM_dQ1:
		  subi tosl,0x30		; 1
			brcs TONUM_noDigit
		  cp tosl,ah
			brcs TONUM_digit
TONUM_noDigit:
		  pop tosh
		  pop tosl				; (ud.l ud.h addr		R: u )
		  pushtos				; r> (byte value)
		  pop tosl				; ( ud.l ud.h addr u )
TONUM_exit:
		  clr tosh
		  add tosl,al			; generate 'not a number'-flag, leave valid Z-flag
		ret

TONUM_digit:
;baseStarPlus:				; multiply by base and add digit
	 	  mov t8,tosl			; save digit
		  pop_t7t6				; ud.h
		  pop_t5t4				; ud.l
		  mul t6,ah
		  movw TOP,R1:R0
		  mul t4,ah
		  movw t1t0,R1:R0
		  mul t7,ah
		  add tosh,R0
		  mul t5,ah
		  add t0,t8				; add digit
		  adc t1,R0
		  adc tosl,R1
		  adc tosh,r_zero		; ud.l' in t1t0
		 					; ++++ end baseStarPlus ++++
		  push_t1t0				; ( ud'.l ud'.h 		R: u addr )
								; 18 / 28
		  mov al,r_zero			; clear 'not a number'-flag
		  pushtos

TONUM_next:
		  pop tosh
		  pop tosl				; ( ud'.l ud'.h addr		R: u )
		  adiw TOP,1			; <addr+1>
		  pushtos
		  pop tosl				; byte value
		  subi tosl,1			; ( ud'.l ud'.h addr+1 u-1 )
		rjmp TONUM_loop


		fdw		STAR_L
; ( ( -- )						skip input until ')'
PAREN_L:
		.db		NFA|IMMED|1,"("
;PAREN:
		rcall DOLIT
		.dw   ')'
		rcall PARSE
		cbr FLAGS1,(1<<doclear)	; don't clear flags in case of '('
		rjmp TWODROP


		fdw		TWOSLASH_L
TWOMINUS_L:
		.db		NFA|INLINE|2,"2-",0xff
;TWOMINUS:
waste9:
		sbiw TOP,2				; 1 / 2
		ret


		fdw		CSTORE_L
; BL ( -- char )				ASCII space
BL_L:
		.db		NFA|INLINE4|2,"bl",0xff
BL:
		pushtos
;BL_0:
		ldi tosl,' '
		ldi tosh,0x00			; 4 / 6
		ret

BL_WORD:
		pushtos
		ldi tosl,' '
		rjmp WORD


		fdw		DOTID_L
; findi	( c-addr nfa -- c-addr 0 )	if not found
;		(				xt     1 )	if immediate
;		(				xt    -1 )	if normal
; first in RAM, second in FLASH, Z-flag
BRACFIND_L:
kernellink_mid:
		.db		NFA|3,"(f)"
findi:
findi_loop:
		  ldd XL,Y+0			; c-addr -> X
		  ldd XH,Y+1
		  movw t3t2,TOP			; save NFA
		  rcall NEQUAL_0		; leaves valid Z-flag
			breq findi_match
		  movw Z,t3t2
		  sbiw Z,2				; NFA -> LFA
		  set					; 'fetch word' flag
		  call IFETCH
		sbiw TOP,0				; LFA 0=?
		brne findi_loop			; no -> valid new NFA -> loop
		;sez
		ret						; yes-> end of dict -> return

findi_match:					; TOP = 0 
		adiw Z,1				; 'aligned'
		cbr ZL,1
		add_pflash_z
		std Y+0,ZL				; CFA > next
		std Y+1,ZH
		mov wflags,t0  			; COMPILE and INLINE flags for the compiler (loaded to t0 in NEQUAL)
		sbiw TOP,1				; 'normal'		Z clear	N set
		sbrc wflags,IMMEDbit
		  adiw TOP,2			; ->'immediate'	Z clear N clear
;;		clz
		ret


PLUSC_:
		lds tosl,litbuf0		; optimize 'LIT +'
		in  tosh,litbuf1
;PLUSC_0:
		com tosl
		com tosh
		add tosl,r_one
		adc tosh,r_zero
		out litbuf1,tosh
		rcall ANDIC1

	.if CPU_LOAD == 1
			ldi t1,0x40			; 'sbci ..'
	.else
			ldi t9,0x40			; 'sbci ..'
	.endif

		rjmp ORIC_1

ANDIC0:
		lds tosl,litbuf0
ANDIC1:
		ldi t0,8
		rcall IDPMINUS
ANDIC2:
		mov tosh,tosl
		swap tosh
		andi tosl,0x0f
		andi tosh,0x0f
		ret

ANDIC_:
	.if CPU_LOAD == 1
			ldi t1,0x70			; 'andi ..'
	.else
			ldi t9,0x70			; 'andi ..'
	.endif

		rjmp ORIC_0

ORIC_:							; optimize 'LIT or'
	.if CPU_LOAD == 1
			ldi t1,0x60			; 'ori ..'
	.else
			ldi t9,0x60			; 'ori ..'
	.endif
ORIC_0:
		rcall ANDIC0
ORIC_1:
		ori tosl,regtosl

	.if CPU_LOAD ==1
			mov t8,t1
			or tosh,t8
	.else
			or tosh,t9
	.endif

		sbrs tosh,5
	  	  ori tosh,0x10			; 'subi ..' >< 'sbci ..'
		rcall ICOMMA_

		pushtos
		in tosl,litbuf1
		rcall ANDIC2
		ori tosl,regtosh

	.if CPU_LOAD == 1
			or tosh,t8
	.else
			or tosh,t9
	.endif

		rjmp ICOMMA_

MINUSC_:
	.if CPU_LOAD == 1
			ldi t1,0x40			; 'sbci ..'
	.else
			ldi t9,0x40			; 'sbci ..'
	.endif

		rjmp ORIC_0


.if optimizingCOMPILER == 1
	STARIC_:						; fdw STAR in TOP
			rcall ldi16_t1t0_C_
			adiw TOP,4
			in t1,litbuf1
			cpse t1,r_zero
			  rjmp INLINE0			; -> fdw STAR_0
			ldi t0,2
			rcall IDPMINUS
			adiw TOP,16				; -> fdw cSTAR_0
			rjmp INLINE0

	ULESSC_:						; fdw ULESS in TOP
			adiw TOP,40				; -> fdw UGREATER_0	(+24) (TOP >< t1:t0 swapped)

	UGREATERC_:						; fdw UGREATER in TOP
			adiw TOP,4				; -> fdw ULESS_0 	(-16) (TOP >< t1:t0 swapped)

	GREATERC_:						; fdw GREATER in TOP
			sbiw TOP,20				; -> fdw LESS_0		(-20) (TOP >< t1:t0 swapped)
			
			rcall checkDUP

			ldi t1,regt1t0
			ldi t0,8
			sbrc FLAGS1,idup
			  ldi t0,12
			rcall ldi16_thtl_C_DUP_
			
			sbr FLAGS1,(1<<icarryeq)
			rjmp INLINE0

	LESSC_:							; fdw LESS in TOP
			adiw TOP,32				; -> fdw GREATER_1	(+32) (TOP >< t1:t0 swapped)
			pushtos
			rcall MINUSC_
			sbr FLAGS1,(1<<icarryeq)
			rjmp INLINE0



	EQUALC_:						; fdw EQUAL in TOP
			rcall checkDUP
			rcall MINUSC_
			sbr FLAGS1,(1<<izeroeq)|(1<<iLITeq)
			inline_DOLIT ((ZEROEQUAL<<1)+PFLASH)
			rjmp INLINE0

	checkDUP:
			cbr FLAGS1,(1<<idup)
			rcall IHERE
			sbiw TOP,12				; look for 'dup' preceeding LIT
			rcall FETCH_A			; (valid addr+2 returned in Z)
			cpi tosl, LOW(0x939a)	; 'pushtosH'?
			  brne checkDUP_non
			cpi tosh,HIGH(0x939a)
			  brne checkDUP_non
			;rcall FETCH_Zplus		; (skip check to save space)
			;cpi tosl, LOW(0x938a)	; 'pushtosL'?
			;  brne _noDUP
			;cpi tosh,HIGH(0x938a)
			;  brne _noDUP
			sbr FLAGS1,(1<<idup)
	checkDUP_non:
			rjmp DROP
.endif

SWOP_A:
		rjmp SWOP

.if optimizingCOMPILER == 1
	optMEM_helper:
			lds tosl,litbuf0
			in  tosh,litbuf1
			ldi t0,8
			rjmp IDPMINUS

	STOREC_2LIT:
			rcall ldi16_t1t0_C_
			ldi t4,regt1			; register 0x11 -> t1
			rcall comp_sts_
			rjmp comp_sts_t0

	STOREC_:
			rcall optMEM_helper
			duptos
			adiw TOP,1				; high byte <addr+1>
			sbrc FLAGS1,f2LIT
			  rjmp STOREC_2LIT
	STOREC_1LIT:
			ldi t4,regtosh			; register 0x19 -> tosh
			rcall comp_sts_
			rjmp STOREC_helper

	CSTOREC_:
			rcall optMEM_helper
			sbrc FLAGS1,f2LIT
			  rjmp CSTOREC_2LIT
	CSTOREC_1LIT:
	STOREC_helper:
		 	ldi t4,regtosl			; register 0x18 -> tosl
	STOREC_help1:
			rcall comp_sts_
	STOREC_help2:
			pushtos
			ldi16 tos,((DROP<<1)+PFLASH)
			rjmp INLINE0

	CSTOREC_2LIT:
			rcall ldi16_t1t0_C_
			ldi t0,2
			rcall IDPMINUS			; wipe 'ldi t1,<lit.1>'
			rjmp comp_sts_t0	
.endif

		.dw		0
; INTERPRET ( c-addr u -- )		interpret given buffer (in RAM)
INTERPRET_L:
		.db		NFA|9,"interpret"
INTERPRET:
		pop_t1t0
		movw X,UP
		sbiw X,(-usource)
		st X+,t0
		st X+,t1
		st X+,tosl
		st X+,tosh
		poptos
		st X+,r_zero			; utoin
		st X+,r_zero
IPARSEWORD:
		rcall BL_WORD
		brne PC+2				; valid Z-flag left by 'WORD'
		  rjmp DROP				; INOWORD
		rcall FIND 				; sets also wflags, leaves valid Z-flag
						; 0 = not found, -1 = normal, 1 = immediate
		poptos
		brne PC+2
		  rjmp INUMBER			; 'not found' -> is it a number?
		brpl IPARSEWORD_3		; 'immediate'
		sbrc FLAGS2,fSTATE
		  rjmp ICOMPILE_1		; 'normal & compiling' -> go compile
IPARSEWORD_3:
								; ..'immediate' or 'interpreting'
		sbrc wflags,COMPILEbit
		sbrc FLAGS2,fSTATE
		  rjmp IEXECUTE			; (not a compile only word) OR ('compile only' && 'compiling') -> execute
		rcall XSQUOTE
		.db 12,"COMPILE ONLY",0xff
		rjmp QABORT_1
IEXECUTE:
		sbr FLAGS1,(1<<doclear)
		call EXECUTE
		sbrc FLAGS1,doclear		; cleared by '\', '(', 'literal' and 'constant'
		  cbr FLAGS1,(1<<izeroeq)|(1<<icarryeq)|(1<<iLITeq)|(1<<idup)|(1<<fLIT)|(1<<f2LIT)
		rjmp IPARSEWORD

ICOMPILE_1:
		cbr FLAGS1,(1<<izeroeq)|(1<<icarryeq)
								; check for 0=, modifies IF, WHILE and UNTIL to use brne
		ldi t1, HIGH((ZEROEQUAL<<1) + PFLASH)
		cpi tosl,LOW((ZEROEQUAL<<1) + PFLASH)
		cpc tosh,t1
		  brne ICOMPILE_2
		sbr FLAGS1,(1<<izeroeq)	; mark '0= encountered in compilation'
		rjmp ICOMPILE

ICOMPILE_2:
		sbrs FLAGS1,fLIT
		  rjmp ICOMPILE_00

.if optimizingCOMPILER == 0

			ldi t1, HIGH((AND_<<1) + PFLASH)
			cpi tosl,LOW((AND_<<1) + PFLASH)
			cpc tosh,t1
			  brne ICOMPILE_3
			rcall ANDIC_
			rjmp ICLRFLIT
	ICOMPILE_3:
			ldi t1, HIGH((OR_<<1) + PFLASH)
			cpi tosl,LOW((OR_<<1) + PFLASH)
			cpc tosh,t1
			  brne ICOMPILE_4
			rcall ORIC_
			rjmp ICLRFLIT
	ICOMPILE_4:
			ldi t1, HIGH((PLUS<<1) + PFLASH)
			cpi tosl,LOW((PLUS<<1) + PFLASH)
			cpc tosh,t1
			  brne ICOMPILE_5
			rcall PLUSC_
			rjmp ICLRFLIT
	ICOMPILE_5:
			ldi t1, HIGH((MINUS<<1) + PFLASH)
			cpi tosl,LOW((MINUS<<1) + PFLASH)
			cpc tosh,t1
			  brne ICOMPILE_00
			rcall MINUSC_
			rjmp ICLRFLIT

.elif optimizingCOMPILER == 1

			ldi16 Z,(optTAB*2)
			ldi t4,optTAB_count
	optLoop:
			  lpm t0,Z+
			  lpm t1,Z+
			  cp  tosl,t0
			  cpc tosh,t1
			    breq foundOpt
			dec t4
			brne optLoop
			  rjmp ICOMPILE_00		; -> go on with no opt found
	
	foundOpt:
			cpi t4,WORDIC_check
			brcc fO_yyy
			  					; WORDIC_ optimization		
			  rcall ldi16_t1t0_C_	; replace LIT by 'ldi t0,<lit.0> ldi t1,<lit.1>'
			  adiw TOP,4			; skip 'pop_t1t0' at beginning of word
			  rjmp ICOMPILE

		fO_yyy:
			cpi t4,PEEPROMcheck
			brcc fO_xxx
			  in t1,litbuf1			; @ c@ ! c!
			  cpi t1,HIGH(PEEPROM)	; ..for ram access only (lit.h < PEEPROM)
			  brcc ICOMPILE_00

		fO_xxx:
			adiw Z,(optTAB_count*2-2)
			lpm t0,Z+				; read address of .... opt routine
			lpm t1,Z+
			movw Z,t1t0
			icall					; call opt routine
			rjmp ICLRFLIT			; clear fLIT and f2LIT, go on parsing

	.equ optTAB_count = 28
	.equ WORDIC_check =  7
	.equ PEEPROMcheck = WORDIC_check + 4

	optTAB:
		fdw AND_
		fdw OR_
		fdw PLUS
		fdw MINUS
			fdw STAR
			fdw NOTEQUAL
			fdw EQUAL
			fdw ULESS
			fdw UGREATER
			fdw LESS
			fdw GREATER
			fdw TO_A
			fdw STORE_P
			fdw STORE_P_TO_R
			fdw MSET
			fdw MCLR
			fdw MTST
			fdw MTSTZ
		;.equ PEEPROMcheck = WORDIC_check + 4
			fdw STORE				; words requiring an address check 'below PEEPROM?'
			fdw CSTORE
			fdw FETCH
			fdw CFETCH
		;.equ WORDIC_check = 5
			fdw XOR_				; words not requiring a special routine
			fdw UMIN				; .. by simply loading LIT to t1:t0
			fdw UMAX
			fdw MIN
			fdw MAX
			fdw SCALE

		.dw ANDIC_
		.dw ORIC_
		.dw PLUSC_
		.dw MINUSC_
			.dw STARIC_
			.dw MINUSC_
			.dw EQUALC_
			.dw ULESSC_
			.dw UGREATERC_
			.dw LESSC_
			.dw GREATERC_
			.dw TO_A_C_
			.dw STORE_P_C_
			.dw STORE_P_TO_R_C_
			.dw MSETC_
			.dw MCLRC_
			.dw MTSTC_
			.dw MTSTZC_
		; PEEPROMcheck ...
			.dw STOREC_
			.dw CSTOREC_
			.dw FETCHC_
			.dw CFETCHC_
		; WORDIC_check - no entries needed
.else .error "illegal value: optimizingCOMPILER"
.endif

ICOMPILE_00:
		cbr FLAGS1, (1<<idup)	; clear 'DUP encountered in compilation'
		ldi t1, HIGH((DUP<<1) + PFLASH)
		cpi tosl,LOW((DUP<<1) + PFLASH)
		cpc tosh,t1				; check for DUP, modies IF and UNTIL to use DUPZEROSENSE
		brne ICOMPILE
		  sbr FLAGS1,(1<<idup)	; mark DUP encountered during compilation
ICOMPILE:
		sbrs wflags,INLINEbit	; inline check
		  rjmp ICOMMAXT
		rcall INLINE0
		rjmp ICLRFLIT

ICOMMAXT:
		rcall COMMAXT
		cbr FLAGS1,(1<<fTAILC)	; allow tail jump  optimization
		sbrc wflags,COMPILEbit	; compile only ?
		  sbr FLAGS1,(1<<fTAILC); ->  prevent tail jump  optimization
ICLRFLIT:
		cbr FLAGS1,(1<<fLIT)|(1<<f2LIT)
		rjmp IPARSEWORD

INUMBER: 
		cbr FLAGS1,(1<<izeroeq)|(1<<icarryeq)|(1<<iLITeq)|(1<<idup)|(1<<f2LIT)
;		poptos
		rcall NUMBERQ
		sbiw TOP,0
		  breq IUNKNOWN
		sbrs FLAGS2,fSTATE
		  rjmp INUMBER1			; 'interpret' -> 
		mov t0,tosl
		poptos
		sbrs t0,1
		  rjmp ISINGLE
IDOUBLE:
		rcall SWOP
		rcall LITERAL_A
ISINGLE:
		rcall LITERAL_A
		rjmp IPARSEWORD

INUMBER1:
		poptos
		rjmp ICLRFLIT

IUNKNOWN:
		cbr FLAGS1,(1<<fLIT)|(1<<f2LIT)
		poptos
		rcall DP_TO_RAM
		rcall CFETCHPP
		rcall TYPE
		rjmp QABORTQ_0			; never returns & resets the stacks


		fdw		TICKStoNEXT_L
; NUMBER?	( c-addr --     n  1 ) single
;			(		 -- dl dh  2 ) double
;			(		 -- c-addr 0 ) if convert error
NUMBERQ_L:
		.db		NFA|7,"number?"
NUMBERQ:
		duptos					; a a  (for the 'not a number'-case)
		st -Y,r_zero
		st -Y,r_zero
		st -Y,r_zero
		st -Y,r_zero			; a 0 0 a
		movw X,TOP
		ld tosl,X+
		clr tosh				; a 0 0 n	X: a+1
		ld t0,X				; inline 'sign?'
		subi t0,'+'
		  breq isSign
		cpi t0,2				; '-'
		  breq isSign
		clr t0
		rjmp prefix
isSign:
		  sbiw TOP,1			; a 0 0 n-1
		  adiw X,1				;			X: a+1
prefix:
		push t0					; save sign

		movw Z,UP				; get current base
		sbiw Z,(-ubase)
		ld ah,Z

		ld t0,X					; check for base-prefix
		subi t0,'#'
		  breq baseQ_dec		; '#' (=0)
		cpi t0,2
		  breq baseQ_bin		; '%' (=2)
		  brcc BASEQ1			; no prefix
baseQ_hex:						; '$' (=1)
		subi t0,-5
baseQ_dec:
		subi t0,-10

baseQ_bin:
		mov ah,t0				; set temporary base (from prefix)
		sbiw TOP,1				; a 0 0 n-1
		adiw X,1				;			X: a+1
BASEQ1:
		push_X					; a 0 0 a' n'
		rcall TONUMBER_0		; leaves valid Z-flag
		poptos					; a ud.l ud.h a' 
		  brne QNUM_ERR

		sbiw TOP,1				; read last char from buf
		rcall CFETCH_A			; a ud.l ud.h c

		subi tosl,'.'			; check for 'double'-sign
		poptos					; a ud.l ud.h
		pop t0					;				t0: sign
		  brne QNUM_single
;QNUM_double:
		cpse t0,r_zero
		  rcall DNEGATE
		rcall ROT				; d.l d.h a
		ldi tosl,2
		ldi tosh,0
		ret

QNUM_single:
		poptos					; a ud.l	t0: sign
		cpse t0,r_zero
		  rcall NEGATE
		std Y+0,tosl			; d.l d.l
		std Y+1,tosh
		movw TOP,r_one			; d.l 1 - single precision number
		ret

QNUM_ERR:					; not a number
		pop t0					; drop sign-flag from R
		adiw Y,4
		clr tosl				; a 0 (convert error)
		clr tosh
		ret


; DOCREATE						 code action of CREATE
; fetch the next cell from program memory to the parameter stack
DOCREATE_L:
		.db		NFA|3, "(c)"	; just for 'see' to work
DOCREATE:
		m_pop_zh
		pop ZH
		pop ZL
		;rjmp FETCHLIT
FETCHLIT:
		pushtos
		lsl ZL
		rol ZH
		lpm_ tosl,Z+
		lpm_ tosh,Z+
		ret


		.db		NFA|3, "(,)"	; just for 'see' to work
DOCOMMAXT:
		m_pop_t0
		pop ZH
		pop ZL
		rcall FETCHLIT
		ror ZH					; C-flag set by FETCHLIT
		ror ZL
		push ZL
		push ZH
		m_push_t0
		rjmp COMMAXT


		fdw		ZEROIF_L
; .st ( -- )					output a string with current data section and current base info
;	base @ dup decimal <#  [char] , hold #s  [char] < hold #> type 
;	<# [char] > hold cse @ #s #> type base ! ;
DOTSTATUS_L:
		.db		NFA|3,".st"
DOTSTATUS:
;### no runtime otimization
		ldi t0,'<'
		rcall EMIT_t0
		;rcall DOTBASE
;DOTBASE:					; ++++ begin inline DOTBASE ++++
		movw Z,UP
		sbiw Z,(-ubase)
		ld t1, Z
		ldi t0,'$'				; hex
		cpi t1,0x10
		  breq DOTBASE_done
;		ldi t0,'%'				; bin
;		cpi t1,2				; replaced by '2'
;		  breq DOTBASE_done
		ldi t0,'#'				; decimal
		cpi t1,10
		  breq DOTBASE_done
		ldi t0, '?'
		  brcc DOTBASE_done
		subi t1,-'0'			; less than #10 -> display as decimal digit
		mov t0,t1
DOTBASE_done:					; ++++ end inline ++++
		rcall EMIT_t0
		ldi t0,','
		rcall EMIT_t0
		;rcall MEMQ

; M? ( -- caddr count )			current data space string
MEMQ:						; +++ begin inline MEMQ +++
		pushtos
		in ZL,cse
		clr ZH
		subi ZL, LOW(-(MEMQADDR_N<<1))
		sbci ZH,HIGH(-(MEMQADDR_N<<1))
		lpm tosl,Z+
		lpm tosh,Z+
		rcall CFETCHPP
		andi tosl,NFAmask
							; +++ end inline +++
		rcall   TYPE
		ldi t0,'>'
		rcall EMIT_t0
		rjmp DOTS


; >dp ( -- )					copy (only changed) turnkey, dp's and latest from RAM to EEPROM
		.db		NFA|3,">dp"		; just for 'see' to work
DP_TO_EEPROM:
		ldi16 Z,(dp_start+9-PEEPROM)
		ldi16 X,(dpSTART+10)
DP_TO_EEPROM_loop:
DTE_wait: sbic EECR,EEWE	rjmp DTE_wait	; EEPROM ready?

		  out EEARH,ZH
		  out EEARL,ZL
		  out EECR,r_one		; (1<<EERE)
		  in t0,EEDR
		  ld t1,-X
		  cp t0,t1
		  breq DP_TO_EEPROM_0
			out EEDR,t1			; value changed -> write to EEPROM
			in_ t0,SREG
			cli
			ldi t1,(EEMPE<<1)
			out EECR,t1
			sbi EECR,EEWE
			out_ SREG,t0
;	sei
		  .if DEBUG_FLASH == 1	; write a '+' for every changed byte
				ldi t0,'+'
			DF_wait:
				  in_ t1,UCSR0A
				  sbrs t1,UDRE0	; USART0 Data Register Empty
				rjmp DF_wait
				out_ UDR0_,t0
		  .endif
DP_TO_EEPROM_0:
		subi ZL,1				; dp_start on page boundary!!
		brcc DP_TO_EEPROM_loop
		ret


; dp> ( -- )					copy ini, dps and latest from EEPROM to RAM
		.db		NFA|3,"dp>"		; just for 'see' to work
DP_TO_RAM:
		ldi16 Z,(dp_start+9-PEEPROM)
		ldi16 X,(dpSTART+10)

DTR_wait: sbic EECR,EEWE     rjmp DTR_wait	; EEPROM ready?

		out EEARH,ZH
DP_TO_RAM_loop:
		  out EEARL,ZL
		  out EECR,r_one		; (1<<EERE)
		  in t1,EEDR
		  st -X,t1
		subi ZL,1				; dp_start on page boundary!!
		brcc DP_TO_RAM_loop
		ret


		fdw		FLASH_L
FALSE_L:
		.db		NFA|INLINE4|5,"false"
FALSE_:							; put 0000 (FALSE) on data stack
ZERO:
		pushtos
;FALSE_pushed:
		clr tosl
		clr tosh				; 4 / 6
		ret


		fdw		TUCK_L
TRUE_L:
		.db		NFA|INLINE4|4,"true",0xff
;TRUE_:							; put ffff (TRUE) on data stack
		pushtos
;TRUE_pushed:
NEQUAL_noMatch:
		ser tosl
		ser tosh				; 5 / 7
SHIFT_ret:
		ret


		fdw		NEGATE_L
; LSHIFT ( x1 u -- x2 )			shift left u bit positions
LSHIFT_L:
		.db		NFA|6,"lshift",0xff
LSHIFT:
		movw Z,TOP
		poptos
LSHIFT1:
		  sbiw Z,1
			brmi SHIFT_ret
		  lsl tosl
		  rol tosh
		rjmp LSHIFT1
	

		fdw		SOURCE_L
; RSHIFT ( x1 u -- x2 )			shift right u bit positions
RSHIFT_L:
		.db		NFA|6,"rshift",0xff
RSHIFT:
		movw Z,TOP
		poptos
RSHIFT1:
		  sbiw Z,1
			brmi SHIFT_ret
		  lsr tosh
		  ror tosl
		rjmp RSHIFT1


		fdw		OR_L
; N= ( c-addr nfa -- n )		compare strings
;					 0    ) s1==s2
;                    ffff ) s1!=s2
; N= is specificly used for finding dictionary entries
; it can also be used for comparing strings shorter than 16 characters,
; but the first string must be in RAM and the second in program memory (FLASH).
NEQUAL_L:
		.db		NFA|2,"n=",0xff
;NEQUAL:
		pop_X
NEQUAL_0:
		movw Z,TOP
		sub_pflash_z
		ld tosh,X+
		lpm tosl,Z+
		mov t0,tosl				; save length+flags for caller
		andi tosl,0x0f			; length
		cp tosl,tosh
		  brne NEQUAL_noMatch	; Z flag valid

NEQUAL_loop:					; length in tosh as loop count
		  lpm tosl,Z+
		  ld t1,X+
		  cp t1,tosl
			brne NEQUAL_noMatch
		dec tosh
		brne NEQUAL_loop
NEQUAL_match:					; Z flag valid
		clr tosl				; n=0: s1==s2
		;clr tosh				; (already there)
		ret

;NEQUAL_noMatch:
;		rjmp TRUE_pushed		; n=ffff: s1!=s2


		fdw		EEPROM_L
; DIGIT? ( c -- n f )			check char for beeing a valid digit (base < 0xd0)
DIGITQ_L:
		.db		NFA|6,"digit?",0xff
;DIGITQ:						; '0' = 0x30    'a' = 0x61
		cpi tosl,0x3a
		  brcs DIGITQ1
		cpi tosl,0x61
		  brcs noDIGIT
		subi tosl,0x27
DIGITQ1:
		subi tosl,0x30			; '0'
		brcc DIGITQ2			; not needed for base < 0xd0
noDIGIT:  rjmp FALSE_
DIGITQ2:
		duptos
		movw Z,UP
		sbiw Z,(-ubase)
		ld tosh,Z				; base in tosh
		sub tosl,tosh			; n < base -> C -> TRUE
		sbc tosl,tosl
		sbc tosh,tosh
		ret


.if optimizingCOMPILER == 1
	; bit testing 8 bits (for RAM addresses only)
	; ( mask addr -- )   (NO flag on stack - to be used with '0until' and '0if')
	MTSTZC_:						; fdw MTSTZ in TOP
			ldi t0,8
			rcall IDPMINUS
			sbrs FLAGS1,f2LIT
			  rjmp MTSTZC_1LIT
	MTSTZC_2LIT:
			ldi t0,8
			rcall IDPMINUS			; leaves dpFLASH in X
			movw TOP,X
			adiw TOP,4
			rcall FETCH_A			; 'ldi tosl,mask'
			cbr tosh,0xf0
			ori tosh,HIGH(0x7000)	; 'andi tosl,mask'
			cbr tosl,0xf0			; 'andi t0  ,mask'
			rcall comp_lds_t0
			rjmp ICOMMA_

	MTSTZC_1LIT:
			rcall comp_lds_t0
			ldi16 tos,0x2380		; 'and tosl,t0'
			rcall ICOMMA_
			pushtos
			ldi16 tos,((DROP<<1) + PFLASH)
			rjmp INLINE0
	
	; bit testing 8 bits (for RAM addresses only)
	; ( mask addr -- f )		

	MTSTC_:							; fdw MTST in TOP
			ldi t0,8
			rcall IDPMINUS
			sbrs FLAGS1,f2LIT
			  rjmp MTSTC_1LIT
	MTSTC_2LIT:
			rcall IHERE
			sbiw TOP,4
			rcall FETCH_A			;   'ldi tosl,<lit1.0>'
			cbr tosh,0xf0
			ori tosh,HIGH(0x7000)	;->'andi tosl,<lit1.0>'
			ldi t0,4
			rcall IDPMINUS			; "wipe" 'ldi tosl,<lit1.0>    ldi tosh,<lit1.1>'
			ldi t4,regtosl			; register 0x18 -> tosl
			rcall comp_lds_t4		; 'in/lds  tosl,<addr>'
			;rcall ICOMMA			; 'andi tosl,<lit1.0>'
			;rjmp ldi_tosh_0_C_		; 'ldi  tosh,0x00'
			rjmp MTSTC_xxx
	
	MTSTC_1LIT:
			rcall comp_lds_t0		; 'in/lds t0,<addr>
			pushtos
			ldi16 tos,0x2380		; 'and tosl,t0'
	MTSTC_xxx:
			rcall ICOMMA
			rjmp ldi_tosh_0_C_		; 'ldi tosh,0x00'
	
	comp_lds_t0:
			ldi t4,regt0			; register 0x10 -> t0
	comp_lds_t4:
			cbr FLAGS1,(fLIT<<1)|(f2LIT<<1)
			pushtos
			lds tosl,litbuf0
			in  tosh,litbuf1
	comp_lds_:
			clt						; 'read'-flag ('in/lds .... ')
			rjmp comp_xts_
			 
	MCLRC_:							; fdw MCLR in TOP
			rcall optMEM_helper
			sbrs FLAGS1,f2LIT
			  rjmp MCLRC_1LIT
	MCLRC_2LIT:						; ( addr -- )
			rcall IHERE
			sbiw TOP,4
			rcall FETCH_A			;    ldi tosl,<lit1.0>'
			cbr tosh,0xf0
			ori tosh,HIGH(0x7000)	;  'andi tosl,<lit1.0>'
			cbr tosl,0xf0			;->'andi t0  ,<lit1.0>'
			ldi t0,0x0f
			eor tosl,t0
			eor tosh,t0				; "com <lit1.0>"
			rjmp comp_2LIT
	
	MCLRC_1LIT:
			pushtos
			ldi16 tos,0x2308		; 'and t0,tosl'
			pushtos
			ldi16 tos,0x9580		; 'com tosl'
			rcall comp_lds_t0		; 'in/lds t0,<addr>
			rcall ICOMMA
			rcall comp_sts			; ICOMMA, 'out/sts <addr>,t0'
			rjmp STOREC_help2		; 'drop'
	
	MSETC_:							; fdw MSET in TOP
			rcall optMEM_helper
	MSETC_0: 
			sbrs FLAGS1,f2LIT
			  rjmp MSETC_1LIT
	MSETC_2LIT:						; ( addr -- )
			rcall IHERE
			sbiw TOP,4
			rcall FETCH_A			;   'ldi tosl,<lit1.0>'
			cbr tosh,0x80			; ->'ori tosl,<lit1.0>'
			cbr tosl,0xf0			; ->'ori t0  ,<lit1.0>'
	comp_2LIT:
			ldi t0,8
			rcall IDPMINUS			; "wipe" 'lit1'
			rcall comp_lds_t0		; 'in/lds t0,<addr>
	comp_sts:						; compile command ('andi ..' or 'ori ..')
			rcall ICOMMA
	comp_sts_t0:
			ldi t4,regt0			; register 0x10 -> t0
	comp_sts_:
			set						; 'write'-flag ('out/sts ... ')
	comp_xts_:
			cpi tosl,0x60			; check addr range
			cpc tosh,r_zero
			  brcc comp_xts_pure
			cpi tosl,0x20
			  brcs comp_xts_pure
								; compile 'in/out <t4>,<addr-0x20>'
			subi tosl,0x20			; make it an io-addr
			mov tosh,tosl
			andi tosl,0x0f
			andi tosh,0xf0
			swap tosh
			lsl tosh
			ori tosh,0xb1
			bld tosh,3				; insert 'read/write'-flag
			or tosl,t4
			rjmp ICOMMA

	comp_xts_pure:				; compile 'lds/sts <t4>,<addr>'
			pushtos	
			mov tosl,t4
			ldi tosh,HIGH(0x9100)	; R16..R31
			bld tosh,1				; insert 'read/write'-flag
			rcall ICOMMA
			rjmp ICOMMA

	MSETC_1LIT:
			pushtos
			ldi16 tos,0x2b08		; 'or t0,tosl'
			rcall comp_lds_t0		; 'lds t0,<addr>
			rcall comp_sts			; ICOMMA, 'sts <addr>,t0'
			rjmp STOREC_help2		; 'drop'
.endif


		fdw		SPACE_L
; SIGN? ( addr n -- addr' n' f )	get optional sign (n<0x100)
;							+ leaves $0000 flag
;							- leaves $0002 flag
SIGNQ_L:
		.db		NFA|5,"sign?"
;SIGNQ:
		 ldd ZL,Y+0				; addr
		 ldd ZH,Y+1
		 mov t0,tosl			; n
		 pushtos				; OVER c@

		 ;call CFETCH_Zplus
		 ld tosl,Z+				; in RAM

		 subi tosl,'+'
		   breq SIGNQIS
		 cpi tosl,2				; '-'
		   breq SIGNQIS
		 clr tosl
		 ;ldi tosh,0x00			; (unchanged)
		 ret
SIGNQIS:
		 std Y+2,ZL				; addr'
		 std Y+3,ZH
		 dec t0
		 std Y+0,t0				; n'
		 ;std Y+1,r_zero		; (unchanged)
		 ;ldi tosh,0x00			; (unchanged)
		 ret


		fdw		WITHIN_L
UMSLASHMOD_L:
		.db		NFA|6,"um/mod",0xff
UMSLASHMOD:
		movw t5t4,TOP			; divisor
		poptos					; ( ud.l ud.h			u in t5:t4 )
		rcall udslashmod0		; (t7:t6:tosh:tosl)		rem in t3:t2
		push_t3t2				; ( rem ud'.l )
		ret


		fdw		UDOTR_L
STARSLASH_L:
		.db		NFA|3,"u*/"
STARSLASH: 						; unsigned values only
		push tosl				; divisor
		push tosh
		poptos
		rcall umstar0			; product.l in tos, product.h in t7:t6
		pop t5
		pop t4
		pushtos
		movw TOP,t7t6
		rjmp udslashmod0		; ( ud'.l		ud'.h in t7:t6		rem in t3:t2 )


		fdw		UDSLASHMOD_L
USSMOD_L:
		.db		NFA|6,"u*/mod",0xff
USSMOD:
		rcall STARSLASH			; remainder returned in t3:t2
		push_t3t2
		ret


		fdw		DEFER_L
; CMOVE ( src dst u -- )		copy u bytes from src to dst
;	swap !p>r for c@+ pc! p+ next drop ;
CMOVE_L:
		.db		NFA|5,"cmove"
CMOVE:
		push pl
		push ph
		ld pl,Y+
		ld ph,Y+				; dst to P
		movw X,TOP				; u to X
		poptos					; src
		rjmp CMOVE2
CMOVE1:
		  push XL
		  push XH
		  rcall CFETCHPP
		  rcall PCSTORE
		  add pl,r_one
		  adc ph,r_zero
		  pop XH
		  pop XL
CMOVE2:
		  sbiw X,1
		brcc CMOVE1

		pop ph
		pop pl
		rjmp DROP


		fdw		MINUS_L
; , ( x -- )					append cell to current data space
;	HERE ! CELL ALLOT ;
COMMA_L:
		.db		NFA|1,","
COMMA:
		rcall HERE
		adiw TOP,2
		st -Z,tosh				; Z-pointer valid from HERE
		st -Z,tosl
		sbiw TOP,2
		rjmp STORE


		fdw		CFETCH_L 
; c, ( c -- )					append char to current data space
;	HERE C! 1 ALLOT ;
CCOMMA_L:
		.db		NFA|2,"c,",0xff
CCOMMA:
		rcall HERE
		adiw TOP,1
		st -Z,tosh				; Z-pointer valid from HERE
		st -Z,tosl
		sbiw TOP,1
		rjmp CSTORE


		fdw    NIP_L
; N>C ( nfa -- cfa )			name addr -> code field
NTOC_L:
		.db		NFA|3,"n>c"
NFATOCFA:
		 rcall CFETCH
		 andi tosl,0x0f
		 add ZL,tosl			; Z valid from CFETCH
		 adc ZH,r_zero
		 adiw Z,1
		 cbr ZL,1
		 movw TOP,Z
		 ret


		fdw    CFETCHPP_L
; C>N ( cfa -- nfa )			code field addr -> name field addr
; moded to match 'see': checks for valid length and chars < 0x80
CTON_L:
		.db		NFA|3,"c>n"
CFATONFA:
		ldi t0,-2
CTN_loop:
		  rcall MINUS_FETCH
		  subi t0,-2
		  sbrs tosl,7
			breq CFATONFA_3		; skip  first high byte for len > 1
		  cpi tosh,0x7f
			brcc CTN_noname
		  cpi tosh,0x20
		    brcs CTN_noname
CFATONFA_3:
		  cpi tosl,0x7f
		    breq CTN_noname
			brcc CTN_hit
CFATONFA_2:
		  cpi tosl,0x20
		    brcs CTN_noname
		  poptos
		  cpi t0,14
		brne CTN_loop
CTN_noname:
		adiw Y,2				; NIP address
		ldi tosl,7				; 'r_zero' as length, if no NFA was found
		clr tosh
		ret
CTN_hit:
		andi tosl,0x0f			; length
		  breq CTN_noname
		cbr tosl,1
		cp tosl,t0				; same lenght?
		  brne CTN_noname
		rjmp DROP


		fdw		RDROP_L
; place ( src n dst -- )		place as counted str
PLACE_L:
		.db		NFA|5,"place"
PLACE:
		rcall TWODUP
PLACE_twodupped:
		rcall CSTORE
		adiw TOP,1
		rcall SWOP
		rjmp CMOVE
		 

FETCH_A:
		rjmp FETCH

CFETCH_A:
		rjmp CFETCH


		fdw		RX0Q_L
; QUIT ( -- 	R: i*x -- )		interpret from kbd
QUIT_L:
		.db		NFA|4,"quit",0xff
QUIT:
		movw Z,UP
		sbiw Z,(-ur0)
		ld t4,Z+
		ld t5,Z+
		in_ t0,SREG
		cli
		out SPL,t4
		out SPH,t5
		out_ SREG,t0
;	sei
		cbr FLAGS2,(1<<fSTATE)	; 'INTERPRET'
		ldi t0,4
		out cse,t0				; 'RAM'
QUIT0:  
		  rcall   DP_TO_RAM
QUIT1: 
check_sp:						; check stack pointer position
			movw Z,UP
			sbiw Z,(-us0)
			ld t4,Z+			; SP R0 S0 WITHIN
			ld t5,Z+
			cp  t4,YL
			cpc t5,YH
			  brcs SP_error		; s0 < sp -> error
			ld t4,Z+			; ur0
			ld t5,Z+
			cp  t4,YL
			cpc t5,YH
			brcs SP_ok
SP_error:
			  rcall XSQUOTE
			  .db   3,"SP?"
			  rjmp QABORT_1		; never returns
SP_ok:
			rcall CR
			rcall TIB
			duptos
			rcall TIBSIZE
			sbiw  TOP,10		; reserve 10 bytes for hold buffer
			rcall ACCEPT
			rcall SPACE_
			rcall INTERPRET
			sbrc FLAGS2,fSTATE
		  rjmp QUIT1

		  rcall IFLUSH
		  rcall DP_TO_EEPROM
		  rcall XSQUOTE
		  .db   3," ok"
		  rcall TYPE
		  rcall PROMPT_
		rjmp  QUIT0


		fdw		REPEAT_L
PROMPT_L:
		.db		NFA|6,"prompt",0xff
PROMPT_:
		rcall DEFER_DOES
		.dw  prompt


		fdw		AGAIN_L
; ABORT ( i*x -- 	R: j*x -- )	clear stack & QUIT
ABORT_L:
		.db		NFA|5,"abort"
ABORT:
		movw Z,UP
		sbiw Z,(-us0)
		ld YL,Z+
		ld YH,Z+
		rjmp QUIT				; QUIT never rets


		fdw		QNEGATE_L
; ?ABORT ( f -- )				abort & print '?'
QABORTQ_L:
		.db		NFA|7,"?abort?"
QABORTQ:
		sbiw TOP,0
		poptos
		  brne QAQ_ret
QABORTQ_0:
		rcall SPACE_
		ldi t0,'?'
		rcall EMIT_t0
		rjmp ABORT


		fdw		BRACCHAR_L
; ?ABORT ( f c-addr u -- )		abort & print msg if flag is false
QABORT_L:
		.db		NFA|6,"?abort",0xff
QABORT:
		ldd t0,Y+2
		ldd t1,Y+3
		or t0,t1
		  brne QABO1
QABORT_1:
		rcall SPACE_
		rcall TYPE
		rjmp ABORT  			; ABORT never returns
QABO1:
		rjmp DOTS_2				; (drop drop drop)


		fdw		ACCEPT_L
; ABORT" ( i*x 0  -- i*x   R: j*x -- j*x ) x1=0
;		 ( i*x x1 --       R: j*x --     ) x1<>0
ABORTQUOTE_L:
		.db		NFA|IMMED|COMPILE|6,"abort",0x22,0xff
ABORTQUOTE:
		rcall SQUOTE
		rcall DOCOMMAXT
		fdw  QABORT
QAQ_ret:
		ret


udslashmod0:				; ( 32 / 16 -> 32		rem16: t3t2 )
								; ( ud.l ud.h					u: t5t4 )
		sbiw TOP,0
		  breq udsm0_16
udsm0_32:
		rcall uslashmod0		; ( ud.l q.h	r.h  : t3t2		u: t5t4 )
		movw t7t6,TOP
		poptos					; ( ud.l		r.h  : t3t2		u: t5t4		q.h:  t7t6 )
		set
;		cp  t2,r_zero			; r.h == 0 ?
;		cpc t3,r_zero			; (speed >< size)
;		brne PC+2
;		  clt
		rjmp udsm0_second		; ( ud'l		rem16: t3t2		u: t5t4		ud'h: t7t6 )   

udsm0_16:
		movw t7t6,TOP			; = 0 (unchanged in uslashmod0)
		poptos
		rjmp uslashmod0			; ( ud'l		rem16: t3t2		u: t5t4		ud'h: t7t6(=0) )


		fdw		UMSLASHMOD_L
; UD/MOD ( ud u -- r ud' )
UDSLASHMOD_L:
		.db		NFA|6,"ud/mod",0xff
;UDSLASHMOD:
		rcall UMSLASHMOD		; ( rem ud'.l	ud'h: t7t6 )
		pushtos					
		movw TOP,t7t6			; ( rem ud'.l ud'.h )
		ret


		fdw		MAX_L
; LIT ( -- x )					fetch inline 16 bit literal to the stack
DOLIT_L:
		.db		NFA|3, "lit"
DOLIT:
		m_pop_zh
		pop ZH
		pop ZL
		rcall FETCHLIT			; 1 / 12+7
		ror ZH
		ror ZL
		mijmp    ; (z)


		fdw		PAREN_L
; ' ( "name" -- xt )			find word in dictionary
TICK_L:
		.db		NFA|1,0x27		; 27h = '
TICK:
		rcall BL_WORD
		;rcall WORD
		rcall FIND
		rjmp QABORTQ


		fdw		DABS_L
; CHAR ( -- char )				parse ASCII character
CHAR_L:
		.db		NFA|4,"char",0xff
CHAR:
		rcall BL
		rcall PARSE
		rcall DROP
		rjmp CFETCH


		fdw		LEAVE_L
; IHERE ( -- a-addr )			put code dictionary ptr onto data stack
;   IDP @ ;
IHERE_L:
		.db		NFA|INLINE5|5,"ihere"
IHERE:
		 pushtos
;IHERE_0:
		 lds16 tos,dpFLASH		; 6 / 8
		 ret


		fdw		ABORTQUOTE_L
; [CHAR] ( -- )					compile character as literal
BRACCHAR_L:
		.db		NFA|IMMED|COMPILE|6,"[char]",0xff
BRACCHAR:
		rcall CHAR
LITERAL_A:
		rjmp LITERAL


; helper for CREATE (CONSTANT_, VARIABLE, MARKER ...)
; creates an empty dictionary entry w/o appending DOCREATE and current dp
CREATE0:
		rcall BL_WORD
		;rcall WORD
		rcall FIND				; ( c-addr 0 ) if not found -> go on
		  breq CREATE_1			; Z-flag valid from 'FIND'
		;adiw Y,2				; (drop address) QABORT will do
		clr tosl
		clr tosh
		cbr FLAGS2,(1<<fINLINE) | (1<<fIMMED)	; clear flags
		rcall XSQUOTE
		.db 15,"ALREADY DEFINED"
		rcall QABORT			; ABORT if word has already been defined

CREATE_1:						;			  ( c-addr 0 )
		ldd ZL,Y+0				; CFETCH: we are in TIB -> RAM
		ldd ZH,Y+1
		ld tosl,Z				;			  ( c-addr len )
		tst tosl
		  breq CREATE_2			; len = 0 -> abort
		cpi tosl,16
		brcs CREATE_3			; len < 16 -> go on
CREATE_2:						; len = 0 or len >= 16
		  rcall QABORTQ			; invalid length -> abort
CREATE_3:						; (c-addr true )
		lds16 tos,dpLATEST		; 			  ( c-addr [latest] )
		rcall ICOMMA			; Link field  ( c-addr )
		rcall CFETCHPP			; str len	  ( addr' len)
		rcall TUCK				; len str len ( len addr' len )
								; IHERE LATEST !
		lds16 Z,dpFLASH			; IHERE
		sts16 dpLATEST,Z
		adiw Z,1				; IHERE+1
		push_Z					;			  ( len addr' ihere+1 len )

		duptos					;			  ( len addr' ihere+1 len len )
		ori tosl,NFA
		mov t0,FLAGS2
		andi t0,(IMMED|INLINE)	; mask out fIMMED and fINLINE
		or tosl,t0				; ..and set header bits
							;			  ( len addr' ihere+1 len NFA[0] )
		cbr FLAGS2,(1<<fINLINE)|(1<<fIMMED)		; clear flags
		sbiw Z,1				; IHERE
		clt
		rcall STORE_Z			; 			  ( len addr' ihere+1 len )
		rcall CMOVE				; 			  ( len )
								; align length and increase IDP
		adiw TOP,2
		cbr tosl,1				;			  ( len' )
;		rjmp IALLOT				; the header has now been created
IALLOT:
		clr ZL
		rjmp ALLOT_0


		fdw		DIGITQ_L
; CREATE ( "name" -- )			create an empty definition
; create a definition header and append DOCREATE and the current data space dictionary pointer
; examples :
; : table create 10 cells allot does> swap cells + ;
; ram table table_a     flash table table_b    eeprom table table_c
; ram variable  qqq
; eeprom variable www ram
; flash variable  rrr ram 
; eeprom create calibrationtable 30 allot ram
; 
CREATE_L:
		.db		NFA|6,"create",0xff
CREATE:
		rcall CREATE0
		pushtos
		ldi tosl, LOW(DOCREATE)	; compiles the runtime routine to fetch the next dictionary cell to the parameter stack
		ldi tosh,HIGH(DOCREATE)
		rcall STORECFF2			; append an exeution token (call !)
		rcall HERE				; compile the current dataspace dp into the dictionary
		in t0,cse
		tst t0
		brne PC+2
		  adiw TOP,2			; in FLASH
		rjmp ICOMMA				; dp now points to a free cell


		fdw    VARIABLE_L
; POSTPONE
POSTPONE_L:
		.db		NFA|IMMED|COMPILE|8,"postpone",0xff
;POSTPONE:
		rcall BL_WORD
		rcall FIND				; leaves valid Z-flag
		brne PC+2
		  rcall QABORTQ			; never returns
		tst tosh
		poptos
		  brpl POSTPONE_immediate
POSTPONE_normal:				; -1
		rcall DOCOMMAXT
		fdw DOCOMMAXT
		rjmp ICOMMA_

POSTPONE_immediate:				;  1
		rjmp COMMAXT


		fdw		DPLUS_L
; CR ( -- )						output newline
CR_L:
		.db		NFA|2,"cr",0xff
CR:
		ldi t0,0x0d
	  .if CR_with_LF == 1
		rcall EMIT_t0
		ldi t0,0x0a				; LF \n
	  .endif
EMIT_t0:
		pushtos
		mov tosl,t0
		rjmp EMIT

KEY_A:
		rjmp KEY


; (DOES>) ( -- )				run-time action of DOES>
		.db		NFA|7,"(does>)"	; just for 'see' to work
XDOES:
		pushtos
XDOES_pushed:
		lds16 tos,dpLATEST		; LATEST_ @
		rcall NFATOCFA			; 'call DOCREATE'
		adiw TOP,2				; point to address
		movw Z,TOP
		pop tosh				; get caller return address
		pop tosl
		set
		rjmp STORE_Z


		fdw		EMPTY_L
; DOES> ( -- )					declare run-time action
DOES_L:
		.db		NFA|IMMED|COMPILE|5,"does>"
;DOES:
		rcall DOCOMMAXT
		fdw  XDOES
		rcall DOCOMMAXT
		fdw  DODOES
		ret


		fdw		STORE_P_L
; ] ( -- )						enter compiling state
RIGHTBRACKET_L:
		.db		NFA|INLINE|1,"]"
RIGHTBRACKET:
		sbr FLAGS2,(1<<fSTATE)
		ret


		fdw		SEMICOLON_L
; : ( -- )						begin a colon definition
COLON_L:
		.db		NFA|1,":"
COLON:
		 sbr FLAGS2,(1<<fSTATE)	; RIGHTBRACKET
		 rjmp CREATE0


		fdw		TONUMBER_L
; :noname ( -- a )				define headerless forth code
NONAME_L:
		.db		NFA|7,":noname"
NONAME:
		rcall IHERE
		rjmp RIGHTBRACKET


		fdw		LESS_L
; ; ( -- )						end a colon definition
SEMICOLON_L:
		.db		NFA|IMMED|COMPILE|1,";"
SEMICOLON:
		cbr FLAGS2,(1<<fSTATE)	; LEFTBRAKET
SEMICOLON_0:
		sbrc FLAGS1,fTAILC
		  rjmp ADD_RETURN_1
		rcall IHERE
		rcall MINUS_FETCH
		movw t1t0,TOP
		andi t1,0xf0
		subi t1,0xd0
		  breq RCALL_TO_RJMP
		poptos
		rcall MINUS_FETCH
		cpi tosh,0x94
.ifdef EIND
			sbci tosl,0x0f
.else
			sbci tosl,0x0e
.endif
		brne ADD_RETURN
CALL_TO_JMP:
.ifdef EIND
			ldi tosl,0x0d
.else
			ldi tosl,0x0c
.endif
		rjmp SWOP_STORE

RCALL_TO_RJMP:
		cbr tosh,0x10			; 0xd... (rcall) -> 0xc... (rjmp)
SWOP_STORE:
		pop_Z
SWOP_STORE_Z:
		set
		rjmp STORE_Z

ADD_RETURN:
		adiw Y,2				; 2drop
		rjmp ADD_RETURN_2
ADD_RETURN_1:
		pushtos
ADD_RETURN_2:
		ldi tosl, LOW(0x9508)	; compile a 'ret'
		ldi tosh,HIGH(0x9508)
ICOMMA_:
		rjmp ICOMMA

.if optimizingCOMPILER == 1
	; recompile 'lit' ('pushtos  ldi16 tos,<lit>')
	;..to 			  ('ldi16 <t1>,<lit>')

	ldi16_t1t0_C_:
			ldi t1,regt1t0			; -> t1:t0
	ldi16_thtl_C_:
			ldi t0,8
	ldi16_thtl_C_DUP_:
			rcall IDPMINUS			; replace 'LIT' by
			subi t0,2
	ldi16_thtl_C_0:
			rcall IHERE
			add tosl,t0
			adc tosh,r_zero
			rcall FETCH				;  'ldi tosh,<lit.1>'
			andi tosl,0x1f
			or tosl,t1				; -> 'ldi th,<lit.1>'
			pushtos
			sbiw Z,4				; valid addr in Z returned by 'FETCH'
			rcall FETCH_Zplus		;  'ldi tosl,<lit.0>'
			andi tosl,0x1f
			or tosl,t1				; -> 'ldi tl,<lit.0>
			rcall ICOMMA			; ..'ldi tl,..'
			rjmp ICOMMA				; ..'ldi th,..'
.endif	


		fdw		DOTQUOTE_L
MINUS_FETCH_L:
		.db		NFA|2,"-@",0xff
MINUS_FETCH:
		sbiw TOP,2
		duptos
		rjmp FETCH


		fdw		SWOPMINUS_L
; STATE ( -- f )				put compiler state onto data stack
STATE_L:
		.db		NFA|INLINE5|5,"state"
STATE_:
		pushtos
STATE_0:
		movw TOP,r_one
		sbrs FLAGS2,fSTATE
		  clr tosl				; 5 / 7
		ret


		fdw		LSHIFT_L
; LATEST ( -- a-addr )
LATEST_L:
		.db		NFA|INLINE4|6,"latest",0xff
LATEST_:
		 pushtos
LATEST_0:
		 ldi16 tos,dpLATEST		; 4 / 6	
		 ret


		fdw		TO_L
; S0 ( -- a-addr )				start of parameter stack
S0_L:
		.db		NFA|INLINE4|2,"s0",0xff
S0:
		inline_DOUSER us0		; 4 / 7
		ret


		fdw		RFROM_L
; R0 ( -- a-addr )				start of parameter stack
R0_L:
		.db		NFA|INLINE4|2,"r0",0xff
R0_:
		inline_DOUSER ur0		; 4 / 7
		ret


		fdw		USLASHMOD_L
; ticks ( -- u )				system ticks (0-ffff) in milliseconds
TICKS_L:
		.db		NFA|INLINE|5,"ticks"
TICKS:  
		pushtos
TICKS_0:
		movw TOP,MS_COUNT		; 3 / 5
		ret


		fdw		USSMOD_L
; ticks= ( u -- t )				leave time in ms from u to actual ms_count
TICKSCOMPUTE_L:
		.db		NFA|INLINE4|6,"ticks=",0xff
TICKSCOMPUTE:
		movw t1t0,TOP
		movw TOP,MS_COUNT
		sub tosl,t0
		sbc tosh,t1				; 4 / 4
		ret

	
		fdw		TURNKEY_L
; ticks>n  ( x -- u x )			push system ticks to NEXT
TICKStoNEXT_L:
		.db		NFA|INLINE|7,"ticks>n"
TICKStoNEXT:
		movw t1t0,MS_COUNT
		push_t1t0				; 3 / 5
		ret


		.dw		0
; us ( u -- )					pause for u microseconds
;	begin 1- dup while waste9 waste2 repeat drop ;
; for ATmega328/p with 16 MHz
MICROS_L:
		.db		NFA|2,"us",0xff
MICROS:							; CPU ticks
		  sbiw TOP,1			; 2(2)
			breq MICROS_xxx		; 1(2)
		  rcall waste9			; 9				(2-)
		  adiw TOP,2			; 2 
		rjmp MICROS				; 2______________16 ticks (= 1 us) per loop
MICROS_xxx:
		poptos					;  (4)
		ret						;(4+4) call/ret__16 ticks for entry+exit


		fdw		BEGIN_L
ALLOT_L:
		.db		NFA|5,"allot"
ALLOT:
		 in ZL,cse
ALLOT_0:
		 clr ZH
		 subi ZL, LOW(-dpFLASH)
		 sbci ZH,HIGH(-dpFLASH)	; DP
		 ld t0,Z+
		 ld t1,Z+				; 	@
		 add tosl,t0
		 adc tosh,t1			;		+
ALLOT_1:
		 st -Z,tosh
		 st -Z,tosl				;		  !
		 rjmp DROP

		
		fdw		IRQ_SEMI_L
; 2@ ( a-addr -- x1 x2 )		fetch 2 cells
;	DUP @ SWAP CELL+ @ ;
TWOFETCH_L:
		.db		NFA|2,"2@",0xff
TWOFETCH:
;		rcall FETCHPP
;		rcall SWOP
;		rjmp  FETCH
	rcall FETCH
	pushtos
	rjmp FETCH_Zplus


		fdw		SPFETCH_L
; SHB ( bitmask -- )			set header bit
SHB_L:
		.db		NFA|3,"shb"
SHB:
		 mov t1,tosl			; save mask
SHB0:
		 lds16 Z,dpLATEST
		 rcall CFETCH_Zplus
		 sbiw Z,1
		 or tosl,t1
		 clt					; 'write char'-flag
		 rjmp STORE_Z			; store tosl to (Z)


		fdw		INTERPRET_L
IMMEDIATE_L:
		.db		NFA|9,"immediate" ; 
IMMEDIATE:
		 ldi t1,IMMED
pre_SHB:
		 pushtos
		 rjmp SHB0


		fdw		LITERAL_L
INLINED_L:
		.db		NFA|7,"inlined" ; 
INLINED:
		 ldi t1,INLINE
		 rjmp pre_SHB


		fdw		DOTSTATUS_L
;  .id ( nfa -- ) 
DOTID_L:
		.db		NFA|3,".id"
DOTID:
		rcall CFETCHPP
		andi tosl,NFAmask
		rjmp TYPE


		fdw		TWOSTAR_L
; 2! ( x1 x2 a-addr -- )		store 2 cells
;	SWAP OVER ! CELL+ ! ;
TWOSTORE_L:
		.db		NFA|2,"2!",0xff
TWOSTORE:
		rcall TUCK
		adiw TOP,2
		rcall STORE
		rjmp STORE


		fdw 	TIBSIZE_L
; SQR  ( u -- u^2 )				16-bit square
; valid results for u < 256 only
SQUARE_L:
		.db		NFA|INLINE|3,"sqr"
;SQUARE:
		mul tosl,tosl
		movw TOP,R1:R0			; 2 / 3
		ret


		fdw		SWOP_L
; SQRT  ( u -- u' )				16-bit square root
; no rounding, no remainder on stack
SQUAREROOT_L:
		.db		NFA|4,"sqrt",0xff
;SQUAREROOT:
		movw t1t0,TOP
		clr tosl
		ldi tosh,0x80
SQRT_loop:
		  eor tosl,tosh
		  mul tosl,tosl
		  cp  t0,R0
		  cpc t1,R1
		  brcc PC+2
			eor tosl,tosh
		lsr tosh
		brne SQRT_loop

		ret


		fdw		PLUS_L
; 								16 x 16 bit to 16 bit multiply
STAR_L:
		.db		NFA|INLINE5|1,"*"
STAR: 
		pop_t1t0
STAR_0:
		movw t7t6,TOP
		mul t0,t6
		movw TOP,R1:R0
		mul t1,t6
		add tosh,R0
		mul t0,t7
		add tosh,R0				;  9 / 14
		ret						;  7 / 10 for  'LIT *'

.if optimizingCOMPILER == 1
	cSTAR_0:						; inline code for 'cLIT *'
			 movw t7t6,TOP
			 mul t0,t6
			 movw TOP,R1:R0
			 mul t0,t7
			 add tosh,R0			;  5 / 7
			 ret
.endif


; (S" ( -- c-addr u )			run-time code for S"
		.db      NFA|3,"(s",0x22	; just for 'see' to work
XSQUOTE:				; #### do NOT change distance to 'STAR:' for 'see' to work ####
		m_pop_zh
		pop ZH
		pop ZL
		lsl ZL
		rol ZH
		lpm_ t0,Z+
		pushtos
		movw TOP,Z
		add_pflash_tos
		pushtos
		mov tosl,t0
		add ZL,t0
		adc ZH,r_zero
		adiw Z,1
		;rampv_to_c
		;ror zh
		lsr ZH
		ror ZL
		mijmp


		fdw		ZEROUNTIL_L
; WORDS   ( "filter" -- ) 		list words beginning with 'filter'
WORDS_L:
		.db		NFA|5,"words"
WORDS:
		rcall CR
		rcall BL_WORD				; c-addr, valid Z-flag
		brne PC+2
		  sbr FLAGS2,(1<<fWORDSall)	; list all words
		pushtos
		ldi16 tos,((kernellink_short<<1)+PFLASH); (c-addr NFA)
		rcall WORDS_dict
		ldi16 tos,((kernellink_mid<<1)+PFLASH)	; (c-addr NFA)
		rcall WORDS_dict
		ldi16 tos,((kernellink_long<<1)+PFLASH)	; (c-addr NFA)
		rcall WORDS_dict
		lds16 tos,dpLATEST			; (c-addr NFA)
		rcall WORDS_dict

		cbr FLAGS2,(1<<fWORDSall)
		rjmp TWODROP_A

WORDS_dict:
		sbrc FLAGS2,fWORDSall
		  rjmp WORDS_match
		movw Z,TOP				; NFA
		sub_pflash_z
		ldd XL,Y+0				; c-addr
		ldd XH,Y+1
		ld  t0,X+				; check length
		lpm t1,Z+
		andi t1,0x0f
		cp t1,t0
		  brcs WORDS_next		; <string> longer than dict entry
WORDS_loop:
		  ld t2,X+
		  lpm t3,Z+
		  cp t2,t3
			brne WORDS_next		; char mismatch
		dec t0
		brne WORDS_loop

WORDS_match:					; type name
		duptos					; (c-addr NFA NFA)
		rcall DOTID
		rcall SPACE_A

WORDS_next:
		sbiw TOP,2				; (c-addr LFA)
		rcall FETCH				; (c-addr NFA')
		sbiw TOP,0
		  brne WORDS_dict
								; end of dict
		rjmp CR


; .S ( -- )						print stack contents
;	space sp@ s0 @ 2- begin 2dup < while -@ u. repeat 2drop ;
		fdw		ZEROLESS_L
DOTS_L:
		.db		NFA|2,".s",0xff
DOTS:
		rcall SPACE_A
		pushtos
		rcall SPFETCH
		rcall S0
		rcall FETCH
		sbiw TOP,2
DOTS_1:
		  ldd t0,Y+0
		  ldd t1,Y+1
		  cp t0,tosl
		  cpc t1,tosh
			brcc DOTS_2
		  rcall MINUS_FETCH
		  rcall UDOT
		rjmp DOTS_1
DOTS_2:
		adiw Y,4
		rjmp DROP_A


		fdw		 UMAX_L
; TYPE ( c-addr u -- )			type line to terminal u < $100
;	for c@+ emit next drop ;
TYPE_L:
		.db		NFA|4,"type",0xff
TYPE:
		mov t8,tosl
		poptos
		tst t8
		  breq TYPE_2
TYPE_loop:
		  rcall CFETCHPP
		  rcall EMIT
		dec t8
		brne TYPE_loop
TYPE_2:
		rjmp DROP_A


		fdw		ELSE_L
; DUMP ( addr u -- )			display memory
DUMP_L:
		.db		NFA|4,"dump",0xff
DUMP:
		ldi t0,4
dump_0:
		  lsr tosh
		  ror tosl
		dec t0
		brne dump_0

		movw Z,UP
		sbiw Z,(-ubase)
		ld t4,Z
		cpi t4,16
		brcc PC+2
		  sbr FLAGS2,(1<<fDUMPxxx)	; dump 3-digit numbers for base<16
		movw X,TOP				; save line count
		poptos
DUMP_loop:  
		  push XL
		  push XH
		  rcall CR
		  duptos
		  pushtos
		  clr tosh
		  ldi tosl,4
		  sbrc FLAGS2,fDUMPxxx
			inc tosl
		  rcall UDOTR			; type line address

		  ldi XL,15				; byte loop count	(hex dump)
  DUMP2:
			push XL				; type 16 byte values
			rcall CFETCHPP
			pushtos
			clr tosh
			ldi tosl,2
			sbrc FLAGS2,fDUMPxxx
			  inc tosl
			rcall UDOTR
			pop XL				; byte loop count
		  subi XL,1
		  brcc DUMP2
  
		  sbiw TOP,16
		  ldi t0,16
		  mov t8,t0				; byte loop count	(ASCII dump)
  DUMP4:
			rcall CFETCHPP		; type 16 ASCII char
			rcall TO_PRINTABLE
			rcall EMIT
		  dec t8
		  brne DUMP4
  	
		  pop XH
		  pop XL
		sbiw X,1
		brcc DUMP_loop

		cbr FLAGS2,(1<<fDUMPxxx)
		;rjmp DROP
		ldi tosl,0x0d			; CR
		rjmp EMIT


; ,?0= ( -- addr )				compile '?0=' and make place for a branch instruction
		.db		NFA|4, ",?0=",0xff    ; just for 'see' to work
COMMAZEROSENSE:
		pushtos
		sbrc FLAGS1,idup
		  rjmp COMMAZEROSENSE1
		ldi16 tos,((ZEROSENSE<<1)+PFLASH)
		rjmp INLINE0

COMMAZEROSENSE1:				; DUPZEROSENSE
		ldi t0,4
		rcall IDPMINUS
		ldi16 tos,0x9700		; 'sbiw TOP,0'		
		cbr FLAGS1,(1<<idup)
		rjmp ICOMMA

IDP6MINUS:
		ldi t0,6
IDPMINUS:
		ldi16 Z,dpFLASH
		ld XL,Z+
		ld XH,Z+
		sub XL,t0
		sbc XH,r_zero
		st -Z,XH
		st -Z,XL
		ret
		 

BRNEC:
		inline_DOLIT 0xf409 	; brne PC+2
		sbrc FLAGS1,izeroeq
		  andi tosh, ~4			; breq PC+2
		rjmp ICOMMA


		fdw		IS_L
; IF ( -- adrs )				conditional forward branch
; leaves address of branch instruction and compiles the condition byte
IF_L:
		.db		NFA|IMMED|COMPILE|2,"if",0xff
IF_:
		rcall IF_createBranch
IF_1:
		cbr FLAGS1,(1<<izeroeq)|(1<<icarryeq)|(1<<iLITeq)

IHERE_ZERO_RJMPC:
		rcall IHERE
		inline_DOLIT 0xc000		; 'rjmp 0' as dummy
		rjmp ICOMMA

IF_createBranch:
		sbrc FLAGS1,icarryeq
		  rjmp carryIF_
		sbrc FLAGS1,idup
		sbrs FLAGS1,iLITeq
		  rjmp IF_zeroeq

IF_dupLITeq:					; (idup && iLITeq)
		inline_DOLIT 0xf009		; 'breq PC+2'
		rcall IHERE
		sbiw TOP,10
		rcall FETCH				; 'subi tosl,lit.0' (leaves valid adr+2 in Z)
		andi tosh,0x0f
		ori tosh,0x30			; -> 'cpi tosl,lit.0'
		inline_DOLIT 0xf411		; 'brne PC+3'
		pushtos
		rcall FETCH_Zplus		; 'sbci tosh,lit.1'
		andi tosh,0x0f
		ori tosh,0x30			; -> 'cpi tosh,lit.1'
		ldi t0,14
		rcall IDPMINUS
		rcall ICOMMA
		rcall ICOMMA
		rcall ICOMMA
		rjmp ICOMMA

IF_zeroeq:
		sbrc FLAGS1,izeroeq
		  rcall IDP6MINUS
		rcall COMMAZEROSENSE
		rjmp BRNEC

carryIF_:
		sbrs FLAGS1,idup
		  rjmp carryIF_noDUP
carryIF_DUP:					; for ULESS, UGREATER and GREATER only
		inline_DOLIT 0xf008		; 'brcs PC+2'
		ldi t0,4				; skip 'drop'
		rcall IDPMINUS
		rjmp ICOMMA

carryIF_noDUP:
		ldi t0,4
		rcall IDPMINUS
		pushtos
		ldi16 tos,((BRCS_seq<<1)+PFLASH)	; 'brcs PC+2'
		rjmp INLINE0

BRCS_seq:
		poptos
		brcs PC+2
		ret


		fdw		EIGHTLSHIFT_L
; 0if   ( -- adrs )				foreward branch on Z-flag - NO stack action
ZEROIF_L:
		.db		NFA|IMMED|COMPILE|3,"0if"
ZEROIF:
		sbr FLAGS1,(1<<izeroeq)
		rcall BRNEC
		rjmp IHERE_ZERO_RJMPC

	
		fdw		EMIT_L
; ELSE ( adrs1 -- adrs2 )		branch for IF..ELSE
; leave adrs2 of bra instruction and store breq in adrs1
; leave adress of branch instruction and FALSE flag on stack
ELSE_L:
		.db		NFA|IMMED|COMPILE|4,"else",0xff
ELSE_:
		rcall IHERE_ZERO_RJMPC
		rcall SWOP_A			; else-addr  if-addr 
		rjmp THEN_


		fdw		TRUE_L
; THEN ( adrs  -- )				resolve forward branch
THEN_L:
		.db		NFA|IMMED|COMPILE|4,"then",0xff
THEN_:
		ldi t0,0xfe				; -2
		ldi t1,0xff
THEN_0:
		movw Z,TOP				; write forward branch at 'if', 'else','do' or 'for'
		lds16 tos,dpFLASH
		sub tosl,ZL
		sbc tosh,ZH
		add tosl,t0
		adc tosh,t1
		asr tosh
		ror tosl
		andi tosh,0x0f
		;ori tosl, LOW(0xc000)
		ori tosh,HIGH(0xc000)
		sbr FLAGS1,(1<<fTAILC)	; prevent tail jump  optimization
		rjmp SWOP_STORE_Z		; write TOP at Z


		fdw		CELLPLUS_L
; BEGIN ( -- adrs )				target for backward branch
BEGIN_L:
		.db		NFA|IMMED|COMPILE|5,"begin"
BEGIN:
		rjmp IHERE


		fdw		VALUE_L
; UNTIL ( adrs -- )				branch backwards if true
UNTIL_L:
		.db		NFA|IMMED|COMPILE|5,"until"
UNTIL:
		sbr FLAGS1,(1<<fTAILC)	; prevent tail jump  optimization
		rcall IF_createBranch
		cbr FLAGS1,(1<<izeroeq)|(1<<icarryeq)|(1<<iLITeq)
		rjmp AGAIN_


		fdw		SCALE_L
; 0until   ( adrs -- )      backward branch on Z-flag - NO stack action
; ########## small loops ONLY ##########
ZEROUNTIL_L:
		.db		NFA|IMMED|COMPILE|6,"0until",0xff
ZEROUNTIL:
		 mov t4,r_one			; b001 for Z-flag
		 ;rjmp UNTILflag
		 						; branch on cpu-flag - NO stack action
UNTILflag:
		 sbr FLAGS1,(1<<fTAILC)	; prevent tail jump  optimization
		 lds t0,dpFLASH			; IHERE
		 ;lds t1,dpFLASH+1		; not needed ...
		 sub tosl,t0
		 ;sbc tosh,t1			; not needed ...
		 ;sbiw TOP,2
		 subi tosl,2
		 andi tosl, LOW(0x00fe)
		 andi tosh,HIGH(0x00fe)	; distance in bytes (already shifted left by 1)
		 lsl tosl	rol tosh
		 lsl tosl	rol tosh	; distance in words (shifted left by 3) to be inserted into 'br..'-command
		 or tosl,t4				; which flag
		 ori tosh,HIGH(0xf400)	; branch on cleared
		 rjmp ICOMMA


		fdw		ALIGN_L
; AGAIN ( adrs -- )				uncondional backward branch
AGAIN_L:
		.db		NFA|IMMED|COMPILE|5,"again"
AGAIN_:
		sbr FLAGS1,(1<<fTAILC)	; prevent tail jump optimization
		lds t0,dpFLASH
		lds t1,dpFLASH+1
		sub tosl,t0
		sbc tosh,t1
		sbiw TOP,2
;		rjmp RJMPC

; rjmp, ( rel-addr -- )
RJMPC:
		asr tosh
		ror tosl
		andi tosh,0x0f
		;ori tosl, LOW(0xc000)	; 'rjmp ...'
		ori  tosh,HIGH(0xc000)
		rjmp ICOMMA


		fdw		WORDS_L
; WHILE ( addr1 -- addr2 addr1 )	branch for WHILE loop
; addr1: address of BEGIN
; addr2: address where to store breq instruction
WHILE_L:
		.db		NFA|IMMED|COMPILE|5,"while"
WHILE_:
		rcall IF_
		rjmp SWOP_A


		fdw		RSHIFT_L
; REPEAT ( addr2 addr1 -- )		resolve WHILE loop
REPEAT_L:
		.db		NFA|IMMED|COMPILE|6,"repeat",0xff
REPEAT_:
		rcall AGAIN_
		rjmp THEN_


		fdw		INVERT_L
INLINE_L:
		.db      NFA|IMMED|COMPILE|6,"inline",0xff
;INLINE
		cbr FLAGS1,(1<<izeroeq)|(1<<idup)|(1<<icarryeq)
		rcall TICK
		rjmp INLINE0


		fdw		KEY_L
; in, ( addr -- )
;	begin @+ dup $9508 <> while i, repeat 2drop ;
INLINEC_L:
		.db      NFA|3,"in,"
INLINE0:
		  rcall FETCHPP
		  ldi t1, HIGH(0x9508)	; 'ret'
		  cpi tosl,LOW(0x9508)
		  cpc tosh,t1
			breq INLINE1
		  rcall ICOMMA
		rjmp INLINE0
TWODROP_A:
INLINE1:
		adiw Y,2
		rjmp DROP_A


		fdw		GCD_L
; FOR  ( -- bc-addr bra-addr )
FOR_L:
		.db		NFA|IMMED|COMPILE|3,"for"
FOR:
		inline_DOLIT ((XFOR<<1)+PFLASH)

	.if optimizingCOMPILER == 1
			sbrs FLAGS1,fLIT
			  rjmp FOR_0
		FORC_:
			cbr FLAGS1,(1<<fLIT)|(1<<f2LIT)
			ldi t1,regX
			rcall ldi16_thtl_C_		; -> 'load literal to X'
			adiw TOP,6				; -> 'XFOR1'
		FOR_0:
	.endif

		rcall INLINE0
		rcall IHERE
		sbiw TOP,4				; skip backwards 'push XL	push XH'
		duptos					; bc-addr
		sbiw TOP,2				; bra-addr
		ret

XFOR:
		movw X,TOP
		poptos
XFOR1:
		rjmp PC+1				; dummy address
		push XL
		push XH
		ret


		fdw		DP_L
; do (limit index --  R: -- limit index )
DO_L:
		.db		NFA|IMMED|COMPILE|2,"do",0xff
DO:
		inline_DOLIT ((XDO<<1)+PFLASH)
		rcall INLINE0
		rcall IHERE
		sbiw TOP,8				; adjust to '_LOOP:'
		ret

XDO:
		movw t5t4,TOP			; index
		poptos
		movw t1t0,TOP			; limit
		poptos
;_LOOP:
		push t0					; limit
		push t1
		push t4					; index
		push t5
		ret

.if CPU_LOAD_LED == 0
		fdw		MTSTZ_L
.elif CPU_LOAD_LED == 1
		fdw		LOADON_L
.else .error "illegal value: CPU_LOAD_LED"
.endif
; leave ( --  R: limit index -- )	  #### one LEAVE per word ONLY!!! ####
LEAVE_L:							; #### do not open a loop after LEAVE !!! ####
		.db		NFA|IMMED|COMPILE|5,"leave"
LEAVE:
		inline_DOLIT ((XLEAVE<<1)+PFLASH)
		rcall INLINE0
		sbi FLAGS3,fLEAVE
		ldi16 Z,RAMvarBase
		ldd t4,Z+_dpFLASH
		ldd t5,Z+_dpFLASH+1
		sbiw t5t4,2
		std Z+_LEAVEadr+1,t5
		std Z+_LEAVEadr  ,t4
		ret

XLEAVE:
		pop t0
		pop t0
		pop t0
		pop t0
		rjmp PC+1				; dummy to be replaced by 'LOOP'
		ret


		fdw		MCLR_L
; loop ( --  R: limit index -- limit index+1 )
;      (     R: limit index --  )
LOOP_L:
		.db		NFA|IMMED|COMPILE|4,"loop",0xff
LOOP:
		inline_DOLIT ((XLOOP<<1)+PFLASH)
		rcall INLINE0
		sbis FLAGS3,fLEAVE
		  rjmp AGAIN_			; write backward branch to '_LOOP'
LOOP_LEAVE:
		cbi FLAGS3,fLEAVE
		pushtos
		lds tosl,LEAVEadr
		lds tosh,LEAVEadr+1
		ldi t0,0				; 0
		ldi t1,0
		rcall THEN_0			; write forward branch at LEAVE
		rjmp AGAIN_				; write backward branch to '_LOOP'

XLOOP:
		pop t5					; index
		pop t4
		pop t1					; limit
		pop t0
		adiw t5t4,1
		cp  t0,t4
		cpc t1,t5
		  brcs PC+2
		ret

		fdw		outerINDEX_L
; i	( -- index  R: limit index -- limit index )
innerINDEX_L:
		.db		NFA|0x20|1,"i"
innerINDEX:			; ++++ must be inlined ++++
		pushtos
		in ZL,SPL
		in ZH,SPH
		ldd tosl,Z+2			; index
		ldd tosh,Z+1			; 6 / 10
		ret


		fdw		LEFTBRACKET_L
; j ( -- index'  R: limit' index' limit index -- limit' index' limit index )
outerINDEX_L:
		.db		NFA|0x20|1,"j"
outerINDEX:			; ++++ must be inlined ++++
		pushtos
		in ZL,SPL
		in ZH,SPH
		ldd tosl,Z+6			; index'
		ldd tosh,Z+5			; 6 / 10
		ret


		fdw		OVER_L
; NEXT ( bc-addr bra-addr -- )
NEXT_L:
		.db		NFA|IMMED|COMPILE|4,"next",0xff
NEXT:
		ldi t0,2				; +2
		ldi t1,0
		rcall THEN_0			; write forward branch at 'for'
		inline_DOLIT ((XNEXT<<1)+PFLASH)
		rcall INLINE0
		lds t0,dpFLASH
		lds t1,dpFLASH+1
		sub tosl,t0
		sbc tosh,t1
		sbiw TOP,2
		rjmp RJMPC


; (next) decrement top of return stack
;		.db		NFA|0x20|6,"(next)",0xff
XNEXT:
		pop XH
		pop XL
		sbiw X,1
		brcs PC+2
		ret


		fdw		EXECUTE_L
DNEGATE_L:
		.db		NFA|7,"dnegate"
DNEGATE:
		 pop_t1t0				; d.l in t1:t0
DNEGATE_0:
		sub t0,r_one
		sbc t1,r_zero
		sbc tosl,r_zero
		sbc tosh,r_zero
		rjmp DINVERT_0


		fdw		CONSTANT_L
QDNEGATE_L:
		.db		NFA|8,"?dnegate",0xff
QDNEGATE:
		lsl tosh				; mov sign to C-flag
		poptos
		  brcs DNEGATE
		ret


		fdw		DROP_L
DABS_L:
		.db		NFA|4,"dabs",0xff
DABS:
		sbrc tosh,7
		  rjmp DNEGATE
		ret


		fdw		MFETCH_L
; m+  ( d n -- d1 )
MPLUS_L:
		.db		NFA|2, "m+",0xff
;MPLUS:
		 movw t1t0,TOP			; n		(d2.l)
		 lsl tosh				; s>d	(d2.h)
		 sbc tosl,tosl
		 sbc tosh,tosh
		 rjmp DPLUS_0


		fdw		DMINUS_L
DPLUS_L:
		.db		NFA|2,"d+",0xff
DPLUS:
		pop_t1t0				; d2.l
DPLUS_0:
		pop_t7t6				; d1.h
		pop_t5t4				; d1.l
		add t0,t4
		adc t1,t5
		adc tosl,t6
		adc tosh,t7
		push_t1t0
		ret


		fdw		DDOT_L
DMINUS_L:
		.db		NFA|2,"d-",0xff
DMINUS:
		 pop_t1t0				; d2.l
DMINUS_0:
		 movw t7t6,TOP			; d2.h
		 poptos					; d1.h
		 pop_t5t4				; d1.l
		 sub t4,t0
		 sbc t5,t1
		 sbc tosl,t6
		 sbc tosh,t7
		 push_t5t4				; Z-flag valid
		 ret					; 13 / 21+4


		fdw		DNEGATE_L
DINVERT_L:
		.db		NFA|INLINE5|7,"dinvert"
DINVERT:
		pop_t1t0
DINVERT_0:
		com t0
		com t1
		com tosl
		com tosh
		push_t1t0				; 8 / 12
		ret


		fdw		DTWOSTAR_L
DZEROEQUAL_L:
		.db		NFA|INLINE5|3,"d0="
DZEROEQUAL:
		pop_t1t0
DZEROEQUAL_0:
		or tosl,tosh
		or tosl,t0
		or tosl,t1
		sbiw TOP,1				; TOP == 0 -> C-flag
		sbc tosl,tosl
		sbc tosh,tosh			; 8 / 11
		ret


		fdw		DGREATER_L
DEQUAL_L:
		.db		NFA|2,"d=",0xff
		rcall DMINUS			; d1 d2 -
		sbiw TOP,1
		rjmp putFlag


		fdw		DEQUAL_L
DLESS_L:
		.db		NFA|2,"d<",0xff
DLESS:
		rcall DMINUS			; d1 d2 -
		brvc PC+2
DG_2:
		  com tosh
DG_1:
		lsl tosh
putFlag:
		sbc tosl,tosl
		sbc tosh,tosh
		adiw Y,2				; NIP d'.l (from DMINUS)
		ret


		fdw		DI_L
DGREATER_L:
		.db		NFA|2,"d>",0xff
DGREATER:
		rcall DMINUS
		  breq putFlag			; d1 == d2 -> C=0 -> put FALSE
		brvs DG_1
		brvc DG_2



.if FLASHEND > 0x3fff
;;; x@ ( addrl addru -- x )		not on ATmega328/p
			fdw		LEFTBRACKET_L
  XFETCH_L:
			.db		NFA|2, "x@",0xff
  .ifdef RAMPZ
			out_    RAMPZ, tosl
  .endif
			poptos
			movw    z, tosl
			lpm_    tosl, z+		; Fetch from Flash directly
			lpm_    tosh, z+
  .ifdef RAMPZ
			ldi     t0, RAMPZV
			out_    RAMPZ, t0
  .endif
			ret
	
;;; x! ( x addrl addru -- )		not on ATmega328/p
			fdw		XFETCH_L
  XSTORE_L:
			.db		NFA|2, "x!",0xff
    		mov     t0, tosl
		 	poptos
			rcall   XUPDATEBUF
			rjmp    ISTORE1
.endif


		.dw     0
; FORGET ( "name" -- ) 
;  bl word latest @ (f) ?abort?
;  c>n 2- dup @ ?abort?
;  dup flash dp ! @ latest ! ram
FORGET_L:
		.db		NFA|6,"forget",0xff
FORGET:
		rcall BL_WORD
		rcall LATEST_
		rcall FETCH
		rcall findi
		rcall QABORTQ
		rcall CFATONFA
		sbiw TOP,2				; LFA
		duptos
		rcall FETCH
		rcall QABORTQ
		sts16 dpFLASH,tos		; dup idp !
		rcall FETCH
		rcall LATEST_
		rjmp STORE


		fdw		FORGET_L
; marker ( "name" -- )
MARKER_L:
lastword:
		.db		NFA|6,"marker",0xff
MARKER:
		out cse,r_zero			; 'flash'
		rcall CREATE
		rcall DOLIT 
		.dw dp_start
		rcall IHERE
		rcall TEN_CMOVE
		rcall IHERE
		adiw TOP,10
		sts16 dpFLASH,tos
		ldi t0,4
		out cse,t0				; 'ram'
		rcall XDOES_pushed
		rcall DODOES
;		rcall INI
; ini ( -- a-addr )				ini variable contains the user-start xt
; In RAM
INI:	pushtos
		ldi16 tos,dpSTART

TEN_CMOVE:
		rcall DOLIT
		.dw 10
		rjmp CMOVE


		fdw		TOR_L
TO_A_L:
		.db		NFA|INLINE|2, ">a",0xff
TO_A:
		movw A,TOP
		poptos					; 3 / 5
		ret


		fdw		BL_L
A_FROM_L:
		.db		NFA|INLINE|2, "a>",0xff
A_FROM:
		pushtos
A_FROM_0:
		movw TOP,A				; 3 / 5
		ret


.if CPU_LOAD_LED == 1
	;;; Enable load-LED
			fdw		LOADOFF_L
	LOADON_L:
			.db		NFA|INLINE|5,"load+"
			sbr FLAGS2,(1<<fLOADled)
			ret

	;;; Disable load-LED
			fdw		MTSTZ_L
	LOADOFF_L:
			.db		NFA|INLINE|5,"load-"
			cbr FLAGS2,(1<<fLOADled)
			.if CPU_LOAD_LED_POLARITY == 1
				cbi_ CPU_LOAD_PORT,CPU_LOAD_BIT
			.elif CPU_LOAD_LED_POLARITY == 0
				sbi_ CPU_LOAD_PORT,CPU_LOAD_BIT
			.else .error "illegal value: CPU_LOAD_LED_POLARITY"
			.endif
			ret
.endif
 
.if CPU_LOAD == 1
			fdw		LOOP_L
	LOAD_L:
			.db		NFA|INLINE|4,"load",0xff
			pushtos
			lds tosl,load_res
		 	clr tosh
			ret
.endif

.ifdef UCSR1A
  ; TX1   c --    output character to UART 1
			fdw		RX0_Q_L			; ### check ###
  TX1_L:
			.db		NFA|3,"tx1"
  TX1_:
			cpi     tosl, XON
			breq    XXON_TX1_TOS
			cpi     tosl, XOFF
			breq    XXOFF_TX1_TOS
  TX1_LOOP:
			rcall   PAUSE
  TX1_quick:
			in_     t0, UCSR1A
			sbrs    t0, UDRE1
			rjmp    TX1_LOOP
			out_    UDR1, tosl
			poptos
			ret

  XXON_TX1_TOS:
			poptos
			rjmp    XXON_TX1_1
  XXON_TX1:
			sbrs    FLAGS2, ixoff_tx1
			ret
  XXON_TX1_1:
			cbr     FLAGS2, (1<<ixoff_tx1)
			ldi     zh, XON
			rjmp    TX1_SEND

  XXOFF_TX1_TOS:
			poptos
			rjmp    XXOFF_TX1_1
  XXOFF_TX1:
			sbrc    FLAGS2, ixoff_tx1
			ret     
  XXOFF_TX1_1:
			sbr     FLAGS2, (1<<ixoff_tx1)
			ldi     zh, XOFF
  TX1_SEND:
			in_     zl, UCSR1A
			sbrs    zl, UDRE1
			rjmp    TX1_SEND
			out_    UDR1, zh
			ret

  ; RX1    -- c    get character from the serial line
			fdw		TX1_L
  RX1_L:
			.db		NFA|3,"rx1"
  RX1_:
			rcall   PAUSE
			rcall   RX1Q
			sbiw TOP,0
			poptos
			breq    RX1_
			pushtos
			ldi     zl, low(rbuf1)
			ldi     zh, high(rbuf1)
			lds     xl, rbuf1_rd
			add     zl, xl
			adc     zh, r_zero
			ld      tosl, z
			clr     tosh
			in      t0, SREG
			cli
			inc     xl
			andi    xl, (RX1_BUF_SIZE-1)
			sts     rbuf1_rd, xl
			lds     xl, rbuf1_lv
			dec     xl
			sts     rbuf1_lv, xl
			out     SREG, t0
			ret

  ; RX1?  -- n    return the number of characters in queue
			fdw		RX1_L
  RX1Q_L:
			.db		NFA|4,"rx1?",0xff
  RX1Q:
			lds     xl, rbuf1_lv
			cpse    xl, r_zero
			rjmp     TRUE_
  .if U1FC_TYPE == 1
			rcall   XXON_TX1
  .elif U1FC_TYPE == 2
			cbi_    U1RTS_PORT, U1RTS_BIT
  .else .error "illegal value: U1FC_TYPE"
  .endif
			rjmp     FALSE_

  RX1_ISRR:
			in_     xh, UDR1
  .if OPERATOR_UART == 1
    .if CTRL_O_WARM_RESET == 1
				cpi     xh, 0xf
				brne    PC+2
				rjmp    RESET_
    .endif
  .endif
			lds     xl, rbuf1_lv
			cpi     xl, RX1_BUF_SIZE-2
			breq    RX1_OVF
			inc     xl
			sts     rbuf1_lv, xl
			cpi     xl, RX0_OFF_FILL
			brmi    RX1_ISR_SKIP_XOFF
  .if U1FC_TYPE == 1
			rcall   XXOFF_TX1_1
  .elif U1FC_TYPE == 2
			sbi_    U1RTS_PORT, U1RTS_BIT
  .else .error "illegal value: U1FC_TYPE"
  .endif
  RX1_ISR_SKIP_XOFF:
			ldi     zl, low(rbuf1)
			ldi     zh, high(rbuf1)
			lds     xl, rbuf1_wr
			add     zl, xl
			adc     zh, r_zero
			st      z, xh
			inc     xl
			andi    xl, (RX1_BUF_SIZE-1)
			sts     rbuf1_wr, xl
			rjmp    OF_ISR_EXIT
  RX1_OVF:
			ldi     zh, '|'
			rcall   TX1_SEND
			rjmp    OF_ISR_EXIT
.endif


.if IDLE_MODE == 1
  IDLE_LOAD:
;			sbrs FLAGS2,fIDLE			; (moved to PAUSE)
;			  rjmp IDLE_LOAD1			; 'busy' -> leave
			ldi t0,LOW(up0)
			cp upL,t0
			  brne IDLE_LOAD1			; not in 'operator' task -> leave
			in_ t0,SREG
			cli
			lds t2,rbuf0_lv				; in OPERATOR task -> check rbuf0_lv/rbuf1_lv
			lds t3,rbuf0_lv+1
			or t2,t3
			out_ SREG,t0
;	sei
    	  .ifdef rbuf1_lv
			  lds t1,rbuf1_lv
			  or  t2,t1
    	  .endif
			brne IDLE_LOAD1				; rbuf0/rbuf1 not empty -> leave

		  .if CPU_LOAD_LED == 1
			sbrc FLAGS2,fLOADled
    		  .if CPU_LOAD_LED_POLARITY == 1	; LED off
				cbi_ CPU_LOAD_PORT,CPU_LOAD_BIT
    		  .else
				sbi_ CPU_LOAD_PORT,CPU_LOAD_BIT
    		  .endif
		  .endif

		  .ifdef SMCR					; buffers empty -> enable sleep mode
			out_ SMCR,r_one
		  .else
			in_ t0,MCUCR
			sbr t0,(1<<SE)
			out_ MCUCR,t0
		  .endif

		  .if CPU_LOAD == 1
       		out_ TCCR1B,r_zero    		; stop load-counter (restarted by any int)
		  .endif

		  .if OPERATOR_UART == 0		; #### must be placed right before 'sleep' ####
			ldi t0,(1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0)
			sbrc FLAGS2,fTX0pending		; if a char for TX0 is pending
			  out_ UCSR0B,t0			; .. enable UDRE0-interrupt (disabled by UDRE_ISR)
		  .endif
			sleep		       			; enter sleep mode 'IDLE'

		  .ifdef SMCR					; disable sleep mode
			out_ SMCR,r_zero
		  .else
			in_ t0,MCUCR
			cbr t0,(1<<SE)
			out_ MCUCR,r_zero
		  .endif

		  .if CPU_LOAD_LED == 1
			sbrc FLAGS2,fLOADled		; LED on
    		  .if CPU_LOAD_LED_POLARITY == 1
				sbi_ CPU_LOAD_PORT,CPU_LOAD_BIT
    		  .else
				cbi_ CPU_LOAD_PORT,CPU_LOAD_BIT
    		  .endif
		  .endif

  IDLE_LOAD1:
			ret
.endif


; unsigned 16/16 -> 32 multiply
umstar0:						; product.l in tos, product.h in t7:t6
		 pop_t1t0
umstar0_0:
		 mul tosl,t0
		 movw t5t4,R1:R0
		 mul tosh, t1
		 movw t7t6,R1:R0
		 mul tosh, t0
		 add t5,R0
		 adc t6,R1
		 adc t7,r_zero
		 mul tosl, t1
		 add t5,R0
		 adc t6,R1
		 adc t7,r_zero
		 movw TOP,t5t4
		 ret					; 16 / 21+4


		fdw		ALIGNED_L
QNEGATE_L:
		.db		NFA|7,"?negate"
;QNEGATE:
		lsl tosh				; copy sign into C-flag
		poptos
		  brcs NEGATE
		ret


		fdw		PROMPT_L
NEGATE_L:
		.db		NFA|INLINE|6, "negate",0xff
NEGATE:
		com tosl
		com tosh
		adiw TOP,1				; 3 / 4
		ret


;;; Resolve the runtime action of the word created by using does>
DODOES_L:
		.db		NFA|3,"(d)"		; just for 'see' to work
DODOES:
		m_pop_xh
		pop XH
		pop XL
		m_pop_zh
		pop ZH
		pop ZL
		rcall FETCHLIT
		movw Z,X
		mijmp					; 7 / 30


; rx0 complete interrupt
RX0_ISR:
		in_ t0,UDR0_

	.if (OPERATOR_UART == 0) && (CTRL_O_WARM_RESET == 1)
			cpi t0,0xf
			brne PC+2
			  rjmp RESET_
	.endif
		 ldi ZL, LOW(rx0queue)
		 ldi ZH,HIGH(rx0queue)
		 ldd XL,Z+_lv0
		 ldd XH,Z+_lv1
		 cpi XH,HIGH(RX0_BUF_SIZE)	; (buf size is a multiple of 0x100)
		   brcc RX0_OVF
		 adiw X,1
		 std Z+_lv1,XH
		 std Z+_lv0,XL

	.if U0FC_TYPE > 0
		 	cpi XL, LOW(RX0_OFF_FILL)
		 	ldi t1,HIGH(RX0_OFF_FILL)
		 	cpc XH,t1
		 	  brcs RX0_ISR_SKIP_XOFF
	  .if U0FC_TYPE == 1
				sbrs FLAGS2,ixoff_tx0
	       		  rcall XXOFF_TX0
	  .elif U0FC_TYPE == 2
				sbi_ U0RTS_PORT,U0RTS_BIT
	  .endif
	  RX0_ISR_SKIP_XOFF:
	.endif

		 ldd XL,Z+_wr0
		 ldd XH,Z+_wr1
		 st X+,t0
		 cpi XH,HIGH(rbuf0 + RX0_BUF_SIZE)	; (buf size is a multiple of 0x100)
		 brcs PC+2
		   ldi XH,HIGH(rbuf0)	;clr XL		; wrap around pointer (rbuf0 on page boundary!!)
;RX0_ISR_0:
		 std Z+_wr1,XH
		 std Z+_wr0,XL
		 rjmp OF_ISR_EXIT

RX0_OVF:
		 ldi ZH,'|'
		 rcall TX0_SEND
		 rjmp OF_ISR_EXIT


	.ifdef UCSR1A
	  RX1_ISR:		rjmp RX1_ISRR
	.endif


		fdw		ABS_L
; ['] ( "name" -- )				find word & compile as DOLITeral
BRACTICK_L:
		.db		NFA|IMMED|COMPILE|3,"[']"
;BRACTICK:
		rcall TICK				; get xt of 'xxx'
		rjmp LITERAL


		fdw		WHILE_L
VALUE_L:
		.db		NFA|5,"value"
VALUE:
		rcall CREATE
		rcall COMMA
		rcall XDOES
VALUE_DOES:
		rcall DODOES
		rjmp FETCH


		fdw		TODIGIT_L
DEFER_L:
		.db		NFA|5,"defer"
DEFER:
		rcall CREATE
		inline_DOLIT ((ABORT<<1)+PFLASH)
		rcall COMMA
		rcall XDOES
DEFER_DOES:
		rcall DODOES
		rjmp FEXECUTE


		fdw		MPLUS_L
IS_L:
		.db		NFA|IMMED|2,"is",0xff
IS:
		rcall TICK				; -> fdw TURNKEY
		adiw TOP,4				; '.dw dpSTART'
		rcall FETCH
		sbrs FLAGS2,fSTATE
		  rjmp STORE
		rcall LITERAL
		rcall DOCOMMAXT
		fdw STORE
		ret


		fdw		UDOT_L
TO_L:
		.db		NFA|IMMED|2,"to",0xff
TO:
		rjmp IS


		fdw		QDNEGATE_L
TURNKEY_L:
		.db		NFA|7,"turnkey"
TURNKEY:
		;--------------------------------------------------
		call VALUE_DOES      	; 'call ..' for IS to work!
		;--------------------------------------------------
		.dw dpSTART


		fdw		STARSLASH_L
; TX0  ( c -- )					output character to UART0
TX0_L:
		.db		NFA|3,"tx0"
TX0_:
	  .if U0FC_TYPE == 1
		cpi tosl,XON
		 breq XXON_TX0_TOS
		cpi tosl,XOFF
		 breq XXOFF_TX0_TOS
	  .endif

TX0_LOOP:
		.if IDLE_MODE == 1
		  sbr FLAGS2,(1<<fTX0pending)
		.endif
		  rcall PAUSE
TX0_quick:
		  in_ tosh,UCSR0A
		  sbrs tosh,UDRE0		; USART0 Data Register Empty
		rjmp TX0_LOOP

		out_ UDR0_,tosl
		poptos
	  .if IDLE_MODE == 1
		cbr FLAGS2,(1<<fTX0pending)
	  .endif

		ret

	.if U0FC_TYPE == 1
	  XXON_TX0_TOS:
			poptos
	  XXON_TX0:
			cbr FLAGS2,(1<<ixoff_tx0)
			ldi ZH,XON
			rjmp TX0_SEND

	  XXOFF_TX0_TOS:
			poptos
	  XXOFF_TX0:
			sbr FLAGS2,(1<<ixoff_tx0)
			ldi ZH,XOFF
	.endif

TX0_SEND:
		  in_ ZL,UCSR0A
		  sbrs ZL,UDRE0			; USART0 Data Register Empty
		rjmp TX0_SEND
		out_ UDR0_,ZH
		ret


.if IDLE_MODE == 0
SLEEP_:
	  .ifdef SMCR				; buffers empty -> enable sleep mode
		out_ SMCR,r_one
	  .else
			in_ t0,MCUCR
			sbr t0,(1<<SE)
			out_ MCUCR,t0
	  .endif

		sleep					; enter sleep mode 'IDLE'

	  .ifdef SMCR				; disable sleep mode
		out_ SMCR,r_zero
	  .else
			in_ t0,MCUCR
			cbr t0,(1<<SE)
			out_ MCUCR,r_zero
	  .endif
		ret
.endif

;***************************************************
; RX0    -- c    get character from the UART0 buffer
		fdw		STOD_L
RX0_L:
		.db		NFA|3,"rx0"
RX0_:
		  rcall PAUSE
		  ldi ZL, LOW(rx0queue)
		  ldi ZH,HIGH(rx0queue)
		  in_ t0,SREG
		  cli
		  ldd XL,Z+_lv0
		  ldd XH,Z+_lv1
		  sbiw X,1
			brcc RX0_1

		  out_ SREG,t0
;	sei
								; buffer empty -> switch off LED
		  sbrs FLAGS2,fLOADled	; .. if it is NOT in use by CPU_LOAD_LED
			.if CPU_LOAD_LED_POLARITY == 1
				cbi_ CPU_LOAD_PORT,CPU_LOAD_BIT
			.else
				sbi_ CPU_LOAD_PORT,CPU_LOAD_BIT
			.endif
		  
		  .if U0FC_TYPE == 1	; .. send 'XON' if this kind of handshake is enabled
				sbrc FLAGS2,ixoff_tx0
				  rcall XXON_TX0
		  .elif U0FC_TYPE == 2	; .. set 'RTS' if this kind of handshake is enabled
				cbi_ U0RTS_PORT, U0RTS_BIT
		  .endif

		.if IDLE_MODE == 0
		  sbrc FLAGS2,fSINGLE
			rcall SLEEP_
		.endif
		rjmp RX0_

RX0_1:

		std Z+_lv1,XH
		std Z+_lv0,XL
		ldd XL,Z+_rd0
		ldd XH,Z+_rd1
		pushtos
		ld tosl,X+
		clr tosh
		cpi XH,HIGH(rbuf0+RX0_BUF_SIZE)
		cpc XL,r_zero
		brne RX0_2
		  ldi XH,HIGH(rbuf0)	;clr XL	; wrap around pointer, rbuf0 on page boundary!!

RX0_2:
		std Z+_rd1,XH
		std Z+_rd0,XL
		out_ SREG,t0
;	sei
								; char read  -> switch on LED
		sbrs FLAGS2,fLOADled	; .. if it is NOT in use by CPU_LOAD_LED
		  .if CPU_LOAD_LED_POLARITY == 1
			sbi_ CPU_LOAD_PORT,CPU_LOAD_BIT
		  .else
			cbi_ CPU_LOAD_PORT,CPU_LOAD_BIT
		  .endif
		ret

;***************************************************
		fdw		SCAN_L
; RX0?  ( -- n )				return the number of characters in queue
RX0Q_L:
		.db		NFA|4,"rx0?",0xff
RX0Q:
		pushtos
		in_ t0,SREG
		cli
		lds tosl,rbuf0_lv
		lds tosh,rbuf0_lv+1
		out_ SREG,t0
;	sei
		ret


		fdw		RPFETCH_L
ROT_L:
		.db		NFA|3, "rot"
ROT:
		 pop_t5t4
		 pop_t1t0
		 push_t5t4
		 pushtos
		 movw TOP,t1t0			; 9 / 17
		 ret


; c@+  ( addr -- addr+1 n )
;   dup 1+ swap c@ ;
		fdw		COMMAXT_L
CFETCHPP_L:
		.db		NFA|3,"c@+"
CFETCHPP:
		rcall CFETCH
		push_Z
		ret


; @+   ( addr -- addr+2 n )
;   dup 2+ swap @ ;
		fdw		L_FETCH_P
FETCHPP_L:
		.db		NFA|2,"@+",0xff
FETCHPP:
		rcall FETCH
		push_Z
		ret


.if (FLASHEND < 0x1ffff)
			fdw		WDOFF_L
	; WD+ ( n -- )				n < 8, start watchdog timer
	WDON_L:
			.db		NFA|3,"wd+"
	WDON:
			cli
			wdr
			andi tosl,7
			ori tosl,(1<<WDE)
			ldi tosh,(1<<WDCE)|(1<<WDE)
			sts WDTCSR,tosh
			sts WDTCSR,tosl
			sei
			rjmp DROP_A


			fdw		X_TO_R_L
	; WD- ( -- )				stop the watchdog 
	WDOFF_L:
			.db		NFA|3,"wd-"
	WDOFF:
			cli
			wdr
  	.ifdef MCUSR
				out MCUSR,r_zero
  	.else
				out MCUCSR,r_zero
  	.endif
			ldi t0,(1<<WDCE)|(1<<WDE)
			sts WDTCSR,t0
			sts WDTCSR,r_zero
			sei
			ret


			fdw 	DZEROLESS_L
	; cwd  ( -- )				kick watchdog
	CWD_L:
			.db		NFA|INLINE|3,"cwd"
	;CWD:
			wdr
			ret
.endif

		 fdw		IMMEDQ_L
IFLUSH_L:
		.db		NFA|6,"iflush",0xff
IFLUSH:
		sbic FLAGS3,idirty
		  rjmp IWRITE_BUFFER
		ret


.ifdef UCSR1A
		fdw		RX1_Q_L				; ### check ###
.else
		fdw		ENDIT_L
.endif
EMPTY_L:
		.db		NFA|5,"empty"
EMPTY:
		rcall DOLIT
		fdw COLDLIT
		rcall DOLIT
		.dw dp_start
		rcall DOLIT
		.dw coldlitsize
		rcall CMOVE				; burn DP's to EEPROM
		rjmp DP_TO_RAM			; .. and copy to RAM


		fdw		WORD_L
WARM_L:
		.db		NFA|4,"warm",0xff
WARM_:
		clr YH

	.ifdef MCUCSR
			in_ R0,MCUCSR
			out_ MCUCSR,YH
	.endif
	.ifdef MCUSR
			in_ R0,MCUSR
			out_ MCUSR,YH
	.endif
WARM_0:
		in_ R1,SREG	
		cli
; zero memory
		ldi XL,2
		ldi XH,0
WARM_1:							; clear register R2..R25 (r_zero, FLAGS1, FLAGS2, TOP included)
		  st X+,YH
		  cpi XL,26
		brne WARM_1

		in_ t0,OSCCAL			; save OSCCAL for ...

		ldi XL,28  				; clear ram from y register upwards (FLAGS3, cse included)
WARM_2:
		  st X+,r_zero
		  cpi XH,HIGH(PEEPROM)	; up to the end of SRAM
		brne WARM_2

		out_ OSCCAL,t0			; .. flash + eeprom write timing

; init empty flash buffer
	    dec ibaseH				; 0xff

	.ifdef RAMPZ
			sts ibaseu,ibaseH
	.endif

; init Stack pointer
		ldi YL, low(utibbuf-4)	; 2 cells safety
		ldi YH,high(utibbuf-4)
; init Return stack pointer
		ldi t0, LOW(usbuf-2)	; 1 cell safety
		ldi t1,HIGH(usbuf-2)
		out SPL,t0
		out SPH,t1

		;rcall   INIT_012		; Init constant registers
;INIT_012:
		;clr r_zero				; already there
		inc r_one				; from 0 to 1
;		inc r_two	inc r_two	; no r_two anymore

		rcall WDOFF
; init user pointer
		ldi t0, LOW(up0)
		ldi t1,HIGH(up0)
		movw UP,t1t0

	.ifdef RAMPZ
  ; set RAMPZ for correct flash addressing
			ldi t0,RAMPZV
			out_ RAMPZ,t0
	.endif
	.ifdef EIND
			out_ EIND,r_one
	.endif

; init warm literals
		rcall DOLIT
		fdw WARMLIT
		rcall DOLIT
		.dw uvars
		rcall DOLIT
		.dw warmlitsize
		rcall CMOVE

		;out cse,r_zero			; initialized to 0 (= 'flash')
		;out state,r_two		; initialized to 0 (= 'interpret') with FLAGS2

; init cold data to eeprom
		ldi ZL, LOW(dp_start)
		ldi ZH,HIGH(dp_start)
;		pushtos					; stack is empty -> nothing to save
		set
		rcall EFETCH			; Z+ included
		adiw TOP,1				; if 'turnkey' is 0xffff then burn DP's to EEPROM
;		poptos					; nothing pushed yet
		brne PC+2
		  rcall EMPTY
								; move interrupts to boot flash section
		out_ MCUCR,r_one   		; (1<<IVCE)
		ldi t7,2				; (1<<IVSEL)
		out_ MCUCR,t7

; init MS timer
	.if MS_TIMER == 0
	  .ifdef TIMSK0
			out_ TCCR0A,t7			; 0x02 (CTC mode)
	       	ldi t0,ms_pre_tmr0
	       	out_ TCCR0B,t0
	       	ldi t0,ms_value_tmr0
	       	out_ OCR0A,t0
			out_ TIMSK0,t7			; 0x02 (1<<OCIE0A)
	  .endif
	  .ifdef TIMSK
			ldi t0,(ms_pre_tmr0 | (1<<WGM01))
			out_ TCCR0,t0
			ldi t0,ms_value_tmr0
			out_ OCR0,t0
			ldi t0,(1<<OCIE0)
			out_ TIMSK,t0
	  .endif
	.elif MS_TIMER == 1
			ldi     t0, 9      		; CTC, clk/1
			out_    TCCR1B, t0
			ldi     t0, high(ms_value_tmr1)
			out_    OCR1AH, t0
			ldi     t0, low(ms_value_tmr1)
			out_    OCR1AL, t0
  	  .ifdef TIMSK
			ldi     t0, (1<<OCIE1A)
			out_    TIMSK, t0
  	  .endif
  	  .ifdef TIMSK1
			out_ TIMSK1,t7			; 0x02 (1<<OCIE1A)
  	  .endif
	.elif MS_TIMER == 2
  	  .ifdef TIMSK2
			out_ TCCR2A,t7			; 0x02 (CTC mode)
			ldi     t0, ms_pre_tmr2
			out_    TCCR2B, t0
			ldi     t0, ms_value_tmr2
			out_    OCR2A, t0
			out_ TIMSK2,t7			; 0x02(1<<OCIE2A)
  	  .endif
  	  .ifdef TIMSK
			ldi     t0, (ms_pre_tmr2 | ( 1<<WGM21 ))
			out_    TCCR2, t0
			ldi     t0, ms_value_tmr2
			out_    OCR2, t0
			ldi     t0, (1<<OCIE2)
			out_    TIMSK, t0
  	  .endif
	.endif

	.if CPU_LOAD == 1
  			ldi t0, LOW(CPU_LOAD_VAL - 1)
			ldi t1,HIGH(CPU_LOAD_VAL - 1)
			out_ OCR1AH,t1
			out_ OCR1AL,t0			; load counter is started by any interrupt
  	  .ifdef TIMSK
			ldi t0,(1<<OCIE1A)
			out_ TIMSK,t0
  	  .endif
  	  .ifdef TIMSK1
			out_ TIMSK1,t7			; 0x02(1<<OCIE1A)
  	  .endif
	.endif

; init UART0
.ifdef UBRR0L
		ldi t0, LOW(RX0_ISR)
		ldi t1,HIGH(RX0_ISR)

  .ifdef URXC0addr
  			sts (URXC0addr+ivec+1),t1
			sts (URXC0addr+ivec)  ,t0
  .else
  			sts (URXCaddr+ivec+1),t1
			sts (URXCaddr+ivec)  ,t0
  .endif

		ldi t0,ubrr0val				; set baud rate
		out_ UBRR0L,t0
		out_ UCSR0A,t7				; 0x02 -> set double speed mode			
		ldi t0,(1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)	; enable receiver, transmitter, rx0 interrupts
		out_ UCSR0B,t0
		ldi t0,(3<<UCSZ00)|URSEL_	; set frame format: 8data, 1stop bit
		out_ UCSR0C,t0

  .if U0FC_TYPE == 1
			sbr FLAGS2,(1<<ixoff_tx0)
  .elif U0FC_TYPE == 2
			sbi_ U0RTS_DDR,U0RTS_BIT
  .endif
.endif
								; init rbuf0 pointer
		;sts rbuf0_lv  ,r_zero		; level starts at 0000
		;sts rbuf0_lv+1,r_zero
		ldi t1,HIGH(rbuf0)			; rbuf0 on page boundary!!
		sts rbuf0_wr+1,t1
		sts rbuf0_rd+1,t1


	; init UART1
	.ifdef UBRR1L
			rcall   DOLIT
			.dw     RX1_ISR
			rcall   DOLIT
			.dw     URXC1addr+ivec
			rcall   STORE
			; Set baud rate
	;		out_    UBRR1H, r_zero
			ldi     t0, ubrr1val
			out_    UBRR1L, t0
		; enable receiver and transmitter, rx1 interrupts
			ldi     t0, (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1)
			out_    UCSR1B,t0
		; set frame format: 8data, 1stop bit
			ldi     t0, (3<<UCSZ10)
			out_    UCSR1C,t0
  	  .if U1FC_TYPE == 1
			sbr     FLAGS2, (1<<ixoff_tx1)
  	  .endif
  	  .if U1FC_TYPE == 2
			sbi_    U1RTS_DDR, U1RTS_BIT
  	  .endif
	.endif
	.ifdef rbuf1_lv
			sts     rbuf1_lv, r_zero
			sts     rbuf1_wr, r_zero
	.endif

		rcall DP_TO_RAM

		sei
		 ;rcall RQ_EMIT
;RQ_EMIT:
		ldi t0,' '
		sbrc R0,BORF
		  ldi t0,'B'
		sbrc R0,WDRF
		  ldi t0,'W'
		sbrc R0,EXTRF
		  ldi t0,'E'
		sbrc R0,PORF			; (MCUSR reset value in R0)
		  ldi t0,'P'
		sbrc R1,6				; T with MATH error (SREG reset value in R1)
		  ldi t0,'M'
		rcall EMIT_t0

		rcall VER
; init LED
		sbi_ CPU_LOAD_DDR,CPU_LOAD_BIT
       .if CPU_LOAD_LED_POLARITY == 1
			cbi_ CPU_LOAD_PORT,CPU_LOAD_BIT
       .else
       		sbi_ CPU_LOAD_PORT,CPU_LOAD_BIT
       .endif

WARM_4:							; write protect bootloader section
		  sbic EECR,EEPE	rjmp WARM_4	; EEPROM write in progress?


		movw Z,r_one
		ldi t0,0xef				; BLB11 = 0
		mov R0,t0
		ldi t1,(1<<BLBSET) | (1<<SPMEN)
		rcall DO_SPM

; check 'turnkey'
		rcall TURNKEY
		sbiw TOP,0
		  breq STARTQ2
		rcall XSQUOTE
		.db  6,"start?",0
		rcall TYPE
		ldi16 tos,TURNKEY_DELAY
		rcall MS
		rcall KEYQ
		sbiw TOP,0
		  brne STARTQ2
STARTQ1:						; no key -> execute
		rcall TURNKEY
		rcall EXECUTE
STARTQ2:
		rjmp ABORT

;.equ partlen = strlen(partstring)
.equ datelen = strlen(DATE)

		fdw		WDON_L
VER_L:
		.db		NFA|3,"ver"
VER:
		rcall XSQUOTE
		;        123456789012345678901234567890123456    7
		;.db 37," OptiForth 5.2  ATmega328 dd.mm.yyyy",0xd
		.db 27+datelen," OptiForth 5.2  ATmega328 ",DATE,0xd		;partstring
TYPE_A:
		rjmp TYPE


;*******************************************************
; ;i ( -- )						end definition of user interrupt routine
		fdw		LESSNUM_L
IRQ_SEMI_L:
		.db		NFA|IMMED|2,";i",0xff
;IRQ_SEMI:
		rcall DOLIT

	.ifdef EIND
			.dw 0x940D			; jmp <page 1>
	.else
			.dw 0x940C			; jmp <page 0>
	.endif

		rcall ICOMMA
		rcall DOLIT
		.dw OF_ISR_EXIT
		cbr FLAGS2,(1<<fSTATE)
		rjmp ICOMMA


		fdw		KEYQ_L
; int! ( addr n  --  )			store to interrupt vector number
IRQ_V_L:
		.db		NFA|4,"int!",0xff
;IRQ_V:
		subi tosl,1
		lsl tosl
		ldi ZL, LOW(ivec)
		ldi ZH,HIGH(ivec)
		add ZL,tosl
		adc ZH,r_zero
		poptos
		sub_pflash_tos			; TO_XA (virtual addr -> real addr)
		lsr tosh
		ror tosl
		rjmp SRZ_w				; store word to RAM


		fdw		NUMBERQ_L
; DOLITERAL ( x -- )			compile DOLITeral x as native code
LITERAL_L:
		.db		NFA|IMMED|7,"literal"
LITERAL:
		pushtos
		ldi16 tos,((DUP<<1)+PFLASH)
		rcall INLINE0
;LITERAL_2:
		sbrc FLAGS1,fLIT
		  sbr FLAGS1,(1<<f2LIT)
		sbr FLAGS1,(1<<fLIT)
		cbr FLAGS1,(1<<izeroeq)|(1<<idup)|(1<<icarryeq)|(1<<doclear)
		sts litbuf0,tosl
		out litbuf1,tosh
		duptos
		mov tosh,tosl
		ldi t1,regtosl			; 'ldi regl,<lit.0>'
		rcall LITERAL_1
		mov tosl,tosh
		ldi t1,regtosh			; '... regh,..'
LITERAL_1:
		swap tosh
		andi tosl,0x0f
		andi tosh,0x0f
		or tosl,t1
		ori tosh,0xe0			; 'ldi ..,..'
		rjmp ICOMMA

#if 0
LITERALruntime:
		st -Y,tosh    	; 0x939a
		st -Y,tosl    	; 0x938a
		ldi tosl,0x12	; 0xe1r2 r=8 (r24)
		ldi tosh,0x34	; 0xe3r4 r=9 (r25)
#endif


;*****************************************************************
.if optimizingCOMPILER == 1
	FETCHC_:
			 rcall FETCHC_helper	; compile 'lds tosl,<addr>'
			 adiw TOP,1				; <addr+1>
			 ldi t4,0x90			; register 0x19 -> tosh
			 rjmp comp_lds_			; compile 'lds tosh,<addr+1>'

	CFETCHC_:
			 rcall FETCHC_helper
	ldi_tosh_0_C_:
			 ldi16 tos,0xe090		; replace <addr> by 'ldi tosh,0x00'
			 rjmp ICOMMA
	
	FETCHC_helper:
			 ldi t0,4
			 rcall IDPMINUS
			 lds tosl,litbuf0		; lit in TOP
			 in tosh,litbuf1
	FETCHC_help0:
			 duptos					; dup <addr>
			 ldi t4,0x80			; register 0x18 -> tosl
			 rjmp comp_lds_
.endif	


		fdw		IF_L
ICOMMA_L:
		.db		NFA|2, "i,",0xff
ICOMMA:
		set
		ldi t0,2
ICOMMA_0:
		ldi16 X,dpFLASH
		ld ZL,X+
		ld ZH,X+
		add ZL,t0
		adc ZH,r_zero
		st -X,ZH
		st -X,ZL
		sub ZL,t0
		sbc ZH,r_zero
		rjmp STORE1


		fdw		INLINEC_L
; ic,  ( c -- )					write char at ihere
;   IHERE c! 1 CHARS IALLOT ;
ICCOMMA_L:
		.db		NFA|3,"ic,"
ICCOMMA:
		 clt
		 ldi t0,1
		 rjmp ICOMMA_0


		fdw		SPACES_L
; SOURCE  ( -- addr n )			current input buffer
;	'SOURCE 2@ ;				length is at higher adrs
SOURCE_L:
		.db		NFA|6,"source",0xff
;SOURCE:
		 pushtos
		 movw Z,UP
		 sbiw Z,(-usource)
		 ld tosl,Z+
		 ld tosh,Z+
		 pushtos
		 ld tosl,Z+
		 ld tosh,Z+				; 10 / 19
		 ret


		fdw		 NONAME_L
; /STRING ( a u n -- a+n u-n )	trim string
;	swap over - >r + r> ;
SLASHSTRING_L:
		.db		NFA|7,"/string"
;SLASHSTRING:
		 movw t5t4,TOP			; n
		 poptos					; u
		 pop_t1t0				; a
		 sub tosl,t4			; u-n
		 sbc tosh,t5
		 add t0,t4				; a+n
		 adc t1,t5
		 push_t1t0				; 11 / 17
		 ret


		fdw		DINVERT_L
; DECIMAL ( -- )				set number base to decimal
;	#10 BASE ! ;
DECIMAL_L:
		.db		NFA|7,"decimal"
;DECIMAL: 
		 ldi t0,0x0a
BASE_STORE:
		 movw Z,UP
		 sbiw Z,(-ubase)
		 st Z+,t0
		 st Z+,r_zero			; 5 / 8
		 ret

		fdw		ICCOMMA_L
; HEX ( -- )					set number base to hex
;	#16 BASE ! ;
HEX_L:
		.db		NFA|3,"hex"
;HEX:
		 ldi t0,0x10
		 rjmp BASE_STORE

		fdw		CTON_L
; BIN ( -- )					set number base to binary
;	#2 BASE ! ;
BIN_L:
		.db		NFA|3,"bin"
;BIN:
		 ldi t0,2
		 rjmp BASE_STORE


IFETCH:
		sub_pflash_z

	.ifdef RAMPZ
			lds t0,ibaseu
			cpi t0,RAMPZV
			  brne IIFETCH
	.endif

		cpse ZH,ibaseH
		  rjmp IIFETCH
		mov t0,ZL
		andi t0,~(PAGESIZEB-1)
		cp t0,ibaseL
		  brne IIFETCH
		mov XL,ZL
		andi XL,(PAGESIZEB-1)
		;add XL, LOW(ibuf)		; ibuf on page boundary !!!
		ldi XH,HIGH(ibuf)
		ld tosl,X+
		brtc IFETCH_1
		  ld tosh,X+
		  adiw Z,1
IFETCH_1:
		adiw Z,1				; for 'FETCH_Zplus'
		add_pflash_z
		ret

IIFETCH:
		lpm_ tosl,Z+	     	; fetch from FLASH directly
		brtc IIFETCH_1
		  lpm_ tosh,Z+
IIFETCH_1:
		add_pflash_z
		ret

ISTORE_Z:
		rcall IUPDATEBUF
		ldi ZH,high(ibuf)
		lds ZL,iaddrl
		andi ZL,(PAGESIZEB-1)
		st Z+,tosl
		brtc ISTO_xxx
		  st Z+,tosh			; T-flag -> 'store word'
ISTO_xxx:
		sbi FLAGS3,idirty
		rjmp DROP_A

FETCH1:
		set
CFETCH1:
		cpi ZH,HIGH(OFLASH)
		  brcc IFETCH

EFETCH:   sbic EECR,EEWE	rjmp EFETCH	; EEPROM ready?

		subi ZH,high(PEEPROM)
		out EEARH,ZH
		out EEARL,ZL
		out EECR,r_one			; (1<<EERE)
		in tosl,EEDR
		  brtc EFETCH_leave
		adiw Z,1
		out EEARH,ZH				
		out EEARL,ZL
		out EECR,r_one			; (1<<EERE)
		in tosh,EEDR
		;rjmp EFETCH_leave
EFETCH_leave:
		adiw Z,1				; for 'FETCH_Zplus'
		subi ZH,HIGH(-PEEPROM)
		ret


STORE1:
		sbic FLAGS3,fLOCK
		  rjmp ISTORERR			; EEPROM + FLASH write protected
		cpi ZH,high(OFLASH)
		  brcc ISTORE_Z
;ESTORE_Z:
		subi ZH,HIGH(PEEPROM)

ECS_wait: sbic EECR,EEWE	rjmp ECS_wait	; wait EEPROM ready

		  out EEARH,ZH
		  out EEARL,ZL
		  out EEDR,tosl
		  in_ t0,SREG
		  cli
		  ldi tosl,(EEMPE<<1)
		  out EECR,tosl
		  sbi EECR,EEWE
		  out_ SREG,t0
;	sei
		    brtc DROP_A
		  adiw Z,1
		  mov tosl,tosh
		  clt
		rjmp ECS_wait


.if FLASHEND > 0x3fff
		fdw		XSTOR_E_L			; ### check ###
.else
		fdw		innerINDEX_L
.endif
FETCH_L:
		.db		NFA|1, "@"
FETCH:
		movw Z,TOP
FETCH_Zplus:
		cpi ZH,HIGH(PEEPROM)
		  brcc FETCH1
		ld tosl,Z+
		ld tosh,Z+
		ret


		fdw		L_PCFETCH
; pc!  ( c -- )					store char to address in pointer
PCSTORE_L:
		.db		NFA|3,"pc!"
PCSTORE:
		clt
		movw Z,P
		rjmp STORE_Z


		fdw		CR_L
; c@  ( adr -- c )				fetch char from adr
CFETCH_L:
		.db		NFA|2, "c@",0xff
CFETCH:
		movw Z,TOP
CFETCH_Zplus:
		clr tosh
		clt
		cpi ZH,HIGH(PEEPROM)
		  brcc CFETCH1
		ld tosl,Z+
		ret


		fdw		NUM_L
; !  ( n adr -- )				store n to adr
STORE_L:
kernellink_short:
		.db		NFA|1, "!"
STORE:
		set						; 'store word' flag
STORE_0:
		movw Z,TOP
		poptos
STORE_Z:
		cpi ZH,high(PEEPROM)
		  brcc STORE1
STORE_RAM_Z:
		brtc SRZ_b
SRZ_w:
		  std Z+1,tosh
SRZ_b:
		std Z+0,tosl
DROP_A:
		poptos
		ret


		fdw		CCOMMA_L
; c!  ( c adr -- )				store char to adr
CSTORE_L:
		.db		NFA|2, "c!",0xff
CSTORE:
		clt						; 'store char' flag
		rjmp STORE_0


		fdw		PFL_L
; pc@ ( -- c )					fetch char from address in pointer
L_PCFETCH:
		.db		NFA|3,"pc@"
;PCFETCH:
		pushtos
		movw Z,P
		rjmp CFETCH_Zplus


		fdw		R0_L
; p@  ( -- n )					fetch word from address in pointer
PFETCH_L:
		.db		NFA|2,"p@",0xff
PFETCH:
		pushtos
		movw Z,P
		rjmp FETCH_Zplus


		fdw		PPLUS_L
; p!  ( n -- )					store word to address in pointer
PSTORE_L:
		.db		NFA|2,"p!",0xff
PSTORE:
		set
		movw Z,P
		rjmp STORE_Z


		fdw		NEQUAL_L
; ms  ( u -- )					pause for u milliseconds
;	ticks +
;	begin pause dup ticks = until drop ;
;
MS_L:
		.db		NFA|2,"ms",0xff
MS:
		movw t1t0,MS_COUNT
		add tosl,t0
		adc tosh,t1
MS1:
		.if IDLE_MODE == 0
		  sbrc FLAGS2,fSINGLE
			rcall SLEEP_		; enter sleep mode when in 'single task'
		.endif
		  rcall PAUSE
		  movw t1t0,MS_COUNT
		  cp  t0,tosl
		  cpc t1,tosh
		brne MS1
		rjmp DROP_A


		fdw		PLACE_L
; PAUSE ( -- )					switch task
PAUSE_L:
		.db		NFA|5,"pause"
PAUSE:
	  .if IDLE_MODE == 0
		sbr FLAGS2,(1<<fSINGLE)	; set 'single task'-flag
	  .elif IDLE_MODE == 1
		sbrc FLAGS2,fIDLE
		  rcall IDLE_LOAD		; 'idle' -> check for sleep
	  .endif

		wdr						; watchdog reset
		movw Z,UP
		sbiw Z,(-ulink)
		ld XL,Z+				; get task link
		ld XH,Z+
		cp  XL,upL				; up = ulink?
		cpc XH,upH
		  breq PAUSE_exit		; ..yes -> single task, no switching

	  .if IDLE_MODE == 0
		cbr FLAGS2,(1<<fSINGLE)	; clear 'single task'-flag
	  .endif
							; switch task
		in_ t1,SREG
		cli
		push YH					; SP
		push YL
		push tosh      			; TOS
		push tosl
		push ph					; P
		push pl
		movw Z,UP
		in t0,SPH				; write rsp to 'ursave'
		st -Z,t0
		in t0,SPL
		st -Z,t0
		movw UP,X
		ld t0,-X
		out SPH,t0
		ld t0,-X
		out SPL,t0
		pop pl
		pop ph
		pop tosl
		pop tosh
		pop YL
		pop YH
		out_ SREG,t1
;	sei
PAUSE_exit:
		ret						; 36 / 14..55+4


		fdw		QABORT_L
SCALE_L:
		.db		NFA|6,"1024*/",0xff
SCALE:
		pop_t1t0
SCALE_0:
		rcall umstar0_0			;	product.l in TOP, product.h in t7:t6
		lsr t7	ror t6	ror tosh	; '1.024 /' by shifting right 
		lsr t7	ror t6	ror tosh		
		mov tosl,tosh				; .. and dropping lower 8 bits
		mov tosh,t6
		adc tosl,r_zero			; rounding
		adc tosh,r_zero
		ret						; 14 / 38+4


		fdw		UDDOT_L
; UD* ( ud u -- ud' )			unsigned 32 x 16 bit multiply to 32 bit
UDSTAR_L:
		.db		NFA|3,"ud*"
;UDSTAR:
		movw t1t0,TOP			; u    in t1t0 
		poptos					; ud.h in TOP
		rcall umstar0_0			; ud.h * u -> t7:t6:TOP, t1t0 unchanged
		movw A,TOP				; save low part, drop high part (t7:t6)
		poptos					; ud.l in TOP
		rcall umstar0_0			; ud.l * u -> t7:t6:TOP
		pushtos					; push low part
		movw TOP,t7t6			; get high part
		add tosl,al				; accumulate high part to ud'.h
		adc tosh,ah				; 13 / 65
		ret


		fdw		VER_L
UMSTAR_L:
		.db		NFA|3,"um*"
UMSTAR:
		 rcall umstar0			; product.l in tos, product.h in t7:t6
		 pushtos
		 movw TOP,t7t6
		 ret


		fdw		ULINK_L
USLASHMOD_L:
		.db		NFA|5,"u/mod"
USLASHMOD:
		rcall USLASH			; (tosh:tosl) / (t5:t4) -> (tosh:tosl) (t3:t2[rem])
		push_t3t2
		ret

		fdw		NTOC_L
MOD_L:
		.db		NFA|3,"mod"
MOD:						; unsigned values only
		 rcall USLASH			; remainder returned in t3:t2
		 movw TOP,t3t2
		 ret


usm0_err:
		set
		rjmp WARM_


		fdw		HEX_L
; gcd (u1 u2 -- gcd )		greatest common divider
GCD_L:
		.db		NFA|3,"gcd"
GCD:
		movw t5t4,TOP
		poptos
GCD_0:
		  rcall uslashmod0
		  movw TOP,t5t4
		  movw t5t4,t3t2
		or t3,t2
		brne GCD_0
		ret


		fdw		ULESS_L
USLASH_L:
		.db		NFA|2,"u/",0xff
USLASH:
		movw t5t4,TOP
		poptos
		;rjmp uslashmod0
							; unsigned 16 16 -- 16 16 division
uslashmod0:						; (tosh:tosl) / (t5:t4) -> (tosh:tosl) (t3:t2[rem])
		clr t2
		clr t3
		clt
udsm0_second:					; for t3t2 < t5t4 ONLY
		ldi t0,16/2

.if optimizeNUM == 0
		sbiw t5t4,0
		  breq usm0_err
		;rjmp usm_full_16

.elif optimizeNUM == 1
		tst t5
		  brne usm0_x_16
		tst t4
		  breq usm0_err

  usm0_x_8:						; divisorH = 0
		brts usm0_full_8
							; t3t2 == 0
		tst tosh
		  brne usm0_full_8

		cp tosl,t4
		  brcs usm0_below		; -> (divident < divisor)
		  brne usm0_8_8
  ;		  breq usm0_equal		; -> (divident = divisor)
  usm0_equal:
		movw TOP,r_one
		ret						; (uslashmod0: 15+4 for divident = divisor, both < 0x0100)
								; (            14+4 for divident = divisor, both > 0x0100)
	usm0_8_8_zero:
		dec t0
		breq usm0_8_8_done
  usm0_8_8:						; (divident > divisor)
	usm0_8_8_loop:
		  lsl tosl
		  rol t2
		    brcs usm0_8_8_one
		  cp t2,t4
		  brcs usm0_8_8_zero
	usm0_8_8_one:
		    sub t2,t4
			inc tosl
	usm0_8_8_next:
		dec t0
		brne usm0_8_8_loop
	usm0_8_8_done:
		;push_t3t2				; remainder in t3t2
		ret						; (uslashmod0: 79..95+4 for divident + divisor < 0x0100)

  usm0_full_8:
  ;		ldi t0,16/2				; 2-bit loop (save 10..24 ticks using 8 words)
	usm0_full_8_loop:
		  lsl tosl
		  rol tosh
		  rol t2
		    brcs usm0_full_8_one1
		  cp t2,t4
		  brcs usm0_full_8_bit2
	usm0_full_8_one1:
			sub t2,t4
			inc tosl
	
	usm0_full_8_bit2:
		  lsl tosl
		  rol tosh
		  rol t2
		    brcs usm0_full_8_one2
		  cp t2,t4
		  brcs usm0_full_8_next
	usm0_full_8_one2:
			sub t2,t4
			inc tosl
	usm0_full_8_next:
		dec t0
		brne usm0_full_8_loop

		;push_t3t2				; remainder in t3t2
		ret						; (uslashmod0: 146..164+4 for divisor < 0x0100)

  usm0_below:
		movw t3t2,TOP			
		clr tosl
		clr tosh
		ret						; (uslashmod0: 17+4 for divident < divisor, both < 0x0100)
								; (            15+4 for divident < divisor, both > 0x0100)
  usm0_x_16:					; divisorH > 0
		brts usm0_full_16
								
		cp  tosl,t4
		cpc tosh,t5
		  brcs usm0_below		; -> (divident < divisor)
		  breq usm0_equal		; -> (divident = divisor)
								; (divident > divisor)
  usm0_16_16:		
		mov t2,tosh			; .. (t3t2==0) -> 8<< (fast shift first 8)
		clr tosh
	;	ldi t0,16-8
	usm0_16_16_loop:
		  lsl tosl
		  rol t2
		  rol t3
		    brcs usm0_16_16_one
		  cp  t2,t4
		  cpc t3,t5
		  brcs usm0_16_16_next
	usm0_16_16_one:
		    sub t2,t4
			sbc t3,t5
			inc tosl
	usm0_16_16_next:
		dec t0
		brne usm0_16_16_loop
		; push_t3t2				; remainder in t3t2
		ret						; (uslashmod0: 100..116+4 for divisor > 0x00ff)
.endif ;(optimizeNUM == 1)

usm0_full_16:
	usm0_full_16_loop:			; 2-bit loop (save 10..24 ticks using 11 words)
		  lsl tosl
		  rol tosh
		  rol t2
		  rol t3
			brcs usm0_full_16_one1
		  cp t2,t4
		  cpc t3,t5
		  brcs usm0_full_16_bit2
	usm0_full_16_one1:
			sub t2,t4
			sbc t3,t5
			inc tosl

	usm0_full_16_bit2:
		  lsl tosl
		  rol tosh
		  rol t2
		  rol t3
			brcs usm0_full_16_one2
		  cp t2,t4
		  cpc t3,t5
		  brcs usm0_full_16_next
	usm0_full_16_one2:
			sub t2,t4
			sbc t3,t5
			inc tosl
	usm0_full_16_next:
		dec t0
		brne usm0_full_16_loop
	usm0_full_16_done:
		; push_t3t2				; remainder in t3t2
		ret						; (uslashmod0: 177..209+4 for divisor > 0x00ff)


		fdw		COLON_L
SLASH_L:
		.db		NFA|1,"/"
;SLASH:
		movw t5t4,TOP			; divisor
		poptos					; divident
		mov t0,t5
		eor t0,tosh
		push t0					; save 'sign xor sign'
		sbrc tosh,7
		  rcall NEGATE
		tst t5
		brpl SLASH_1
		  com t4
		  com t5
		  adiw t5t4,1
SLASH_1:
		rcall uslashmod0

		pop t0					; resulting sign
		sbrc t0,7
		  rjmp NEGATE
		ret


		fdw		CQUOTE_L
PLUSSTORE_L:
		.db		NFA|2,"+!",0xff
;PLUSSTORE:
/*	movw Z,TOP
	cpi ZH,HIGH(PEEPROM)
	  brcc PLUSSTORE_1
	pop_t1t0
	ld tosl,Z+
	ld tosh,Z+
	add tosl,t0
	adc tosh,t1
	st -Z,tosh
	st -Z,tosl
	poptos
	ret
PLUSSTORE_1:
		 rcall FETCH1			; leaves valid Z+2
*/		rcall FETCH
		sbiw Z,2
		pop_t1t0
		add tosl,t0
		adc tosh,t1
		set						; write word
		rjmp STORE_Z


		fdw		MCFETCH_L
MAX_L:
		.db		NFA|3,"max"
MAX:
		 pop_t1t0				; NEXT
;MAX_0:
		 movw t7t6,t1t0
		 eor t7,tosh
		 movw t7t6,t1t0
		   brmi MAX_1
		 sub t6,tosl			; NEXT-TOP
		 sbc t7,tosh
MAX_1:
		 sbrs t7,7				; -> max = TOP
		   movw TOP,t1t0		; max = NEXT
		 ret


		fdw		MOD_L
MIN_L:
		.db		NFA|3,"min"
MIN:
		 pop_t1t0				; NEXT
;MIN_0:
		 movw t7t6,TOP
		 eor t7,t1				; check signs
		 movw t7t6,TOP
		   brmi MIN_1			; -> different signs
		 sub t6,t0				; TOP-NEXT
		 sbc t7,t1
MIN_1:
		 sbrs t7,7				; -> min = TOP
		   movw TOP,t1t0		; min = NEXT
		 ret


		fdw		EXIT_L
; EMIT ( c -- )					output character to the UEMIT vector
EMIT_L:
		.db		NFA|4,"emit",0xff
EMIT:
		ldi t0,(-uemit)
UEXECUTE:
		movw X,UP
		sub XL,t0
		sbc XH,r_zero
		ld ZL,X+
		ld ZH,X+
		rjmp EXECUTE_0			; 7 / 15


		fdw		DOLIT_L
; KEY  ( -- c )					get char from UKEY vector
KEY_L:
		.db		NFA|3,"key"
KEY:
		ldi t0,(-ukey)
		rjmp UEXECUTE			; 2 / 17


.if CPU_LOAD == 0
		fdw		LOOP_L
.elif CPU_LOAD == 1
		fdw		LOAD_L
.else .error "illegal value: CPU_LOAD"
.endif
; KEY  ( -- c )					check for char from UKEYQ vector
KEYQ_L:
		.db		NFA|4,"key?",0xff
KEYQ:
		ldi t0,(-ukeyq)
		rjmp UEXECUTE			; 2 / 17


		fdw		INLINED_L
EXECUTE_L:
		.db		NFA|7,"execute"
EXECUTE:
		movw Z,TOP
		poptos
EXECUTE_0:
		sub_pflash_z
		lsr ZH
		ror ZL
		mijmp					; 7 / 10


		fdw		BRACTICK_L
FEXECUTE_L:
		.db		NFA|3,"@ex"
FEXECUTE:
		rcall FETCH
		rjmp EXECUTE


;*************************************************************
ISTORERR:				; write access to kernel
		rcall DOTS
		rcall XSQUOTE
		.db 3,"AD?"
		rcall TYPE
		jmp ABORT
		
; Coded for max 256 byte pagesize !
; if ((ibaselo != (iaddrlo & ~(PAGESIZEB-1))) | (ibaseH != iaddrh) | (ibaseu != iaddru))
;   if (idirty)
;       writebuffer_to_imem
;   endif
;   fillbuffer_from_imem
;   ibaselo = iaddrlo & ~(PAGESIZEB-1)
;   ibasehi = iaddrhi
;endif
IUPDATEBUF:
		sub_pflash_Z
	.ifdef  RAMPZ
	    	ldi t0,RAMPZV
	.endif
;XUPDATEBUF:
		sts iaddrl,ZL
		sts iaddrh,ZH
	.ifdef RAMPZ
			sts iaddru,t0
	    	cpi t0,RAMPZV
	    	  brne XUPDATEBUF2
	.endif
		cpi ZL, LOW(FLASH_HI-PFLASH+1) ; don't allow kernel writes
		ldi t1,HIGH(FLASH_HI-PFLASH+1)
		cpc ZH,t1
		  brcc ISTORERR
XUPDATEBUF2:	
		andi ZL,~(PAGESIZEB-1)
		cp  ZL,ibaseL
		cpc ZH,ibaseH
		  brne IFILL_BUFFER
	.ifdef RAMPZ
			lds t0,iaddru
			lds t1,ibaseu
			cpse t0,t1
			  rjmp IFILL_BUFFER
	.endif
		ret

IFILL_BUFFER:
		sbic FLAGS3, idirty
		  rcall IWRITE_BUFFER
		lds t0,iaddrl
		andi t0,~(PAGESIZEB-1)
		mov ibaseL,t0
		lds ibaseH,iaddrh
	.ifdef RAMPZ
	    	lds t0,iaddru
	    	sts ibaseu,t0
	    	out_ RAMPZ, t0
	.endif
IFILL_BUFFER_1:					; fill buffer from FLASH
		ldi t0,0xff
		movw Z,IBASE
		ldi16 X,ibuf				; ibuf on page boundary !!
IFILL_BUFFER_2:
		  lpm_ t1,Z+
		  st X+,t1
		  and t0,t1					; check for programmed bytes
		  cpi XL, LOW(ibuf+PAGESIZEB)	; ibuf on page boundary !!
		brne IFILL_BUFFER_2
		
		cbi FLAGS3,fFLASH_PAGE_CLEAR
		cpi t0,0xff
		brne PC+2
		  sbi FLAGS3,fFLASH_PAGE_CLEAR	; all bytes in page = 0xff -> don't need to erase page

	.ifdef RAMPZ
			ldi t0,RAMPZV
			out_ RAMPZ,t0
	.endif
		ret

IWRITE_BUFFER:
	.if OPERATOR_UART == 0
	  .if U0FC_TYPE == 1
			sbrs FLAGS2,ixoff_tx0
			  rcall XXOFF_TX0
	  .elif U0FC_TYPE == 2
			sbi_ U0RTS_PORT, U0RTS_BIT
	  .endif
	.else  ;; UART1
	  .if U1FC_TYPE == 1
			rcall   DOLIT
			.dw     XOFF
			rcall    EMIT
	  .elif U1FC_TYPE == 2
			sbi_    U1RTS_PORT, U1RTS_BIT
	  .endif
	.endif

	.ifdef RAMPZ
	    	lds t0,ibaseu
	    	out_ RAMPZ,t0
	.endif

IWR_BUF_waitEE:	 sbic EECR,EEPE		rjmp IWR_BUF_waitEE	; EEPROM write in progress?

	.if DEBUG_FLASH == 1
		 	pushtos
		 	movw TOP,MS_COUNT
	.endif
		
;		push R0
;		push R1
		push R20				; pl (boot loader compatibility)
		ldi16 X,ibuf
		rcall WRITE_FLASH_PAGE	; leaves Z = ibase
		pop R20
;		pop R1
;		pop R0
		
		ldi16 X,ibuf			; read back and check
IWRITE_BUFFER2:
		  lpm_ R0,Z+
		  ld R1,X+
		  cpse R0,R1
		    rjmp WARM_     		; reset if write error
		  cpi XL,LOW(ibuf+PAGESIZEB)
		brne IWRITE_BUFFER2

		cbi FLAGS3,idirty
		clr ibaseH	dec ibaseH	; 'ibuf empty'-marker

	.ifdef RAMPZ
			sts ibaseu,t0
			ldi t0,RAMPZV
			out_ RAMPZ,t0
	.endif

	.if OPERATOR_UART == 0
	  .if U0FC_TYPE == 1
			 rcall XXON_TX0
	  .elif U0FC_TYPE == 2
			cbi_ U0RTS_PORT,U0RTS_BIT
	  .endif
	.else
	  .if U1FC_TYPE == 1
			rcall   DOLIT
			.dw     XON
			rcall    EMIT
	  .elif U1FC_TYPE == 2
			cbi_    U1RTS_PORT, U1RTS_BIT
	  .endif
	.endif

	.if DEBUG_FLASH == 1
		 	 sub tosl,ms_countL		; write time [ms] as single ASCII to operator UART
		 	 neg tosl
		 	 subi tosl,-'0'
		.if   OPERATOR_UART == 0
			 	rjmp TX0_quick
		.elif OPERATOR_UART == 1
			 	rjmp TX1_quick
		.else 
			 	rjmp DROP
		.endif
	.else	; (DEBUG_FLASH <> 1)
			 ret
	.endif


OF_ISR_EXIT:
;	 pop t3						; uncomment when programming interrupt-words
;	 pop t2							; ------------------------------------------
		 pop t1
		 pop t0
		 pop ZH					; t7	
		 pop ZL					; t6
		 out_ SREG,SREG_intSafe
  		 movw X,X_intSafe		; t5:t4
		 reti


.if CPU_LOAD == 1
	MS_TIMER_LOAD:
		inc ms_countL
		brvs MTL_0				; 128 ms load measurement intervall
		brne MTL_xxx
		  inc ms_countH
	MTL_0:
		  sts load_res,loadreg
		  ldi loadreg,HIGH(CPU_LOAD_VAL / 2)	; rounding
		  out_ TCNT1H,loadreg
		  out_ TCNT1L,r_zero
		  clr loadreg			; reset load-counter
	MTL_xxx:
		out_ SREG,SREG_intSafe
		reti
.elif CPU_LOAD > 1
		.error "illegal value: CPU_LOAD"
.endif

.org BOOT_START - 0x20			; DO NOT CHANGE !! (requires reburning of bootloader part)
MS_TIMER_ISR:
  .if CPU_LOAD == 0
		in_ SREG_intSafe,SREG
		add ms_countL,r_one
		adc ms_countH,r_zero
		out_ SREG,SREG_intSafe
		reti
  .elif CPU_LOAD == 1			
		ldi INTvector,9
		out_ TCCR1B,INTvector	; restart load-timer (CTC, /1)

		in_ SREG_intSafe,SREG	; (SREG_intSafe = INTvector = R19)
		rjmp MS_TIMER_LOAD
  .else .error "illegal value: CPU_LOAD"
  .endif


.org BOOT_START - 0x1b			; DO NOT CHANGE !! (requires reburning of bootloader part)
OF_ISR:
		movw X_intSafe,X		; t5:t4
	.if CPU_LOAD == 1
		  ldi XL,9
		  out_ TCCR1B,XL		; restart load-timer (CTC, /1)
	.endif

		mov XL,INTvector
		in_ SREG_intSafe,SREG	; (INTvector = SREG_intSafe = R19)
		push ZL					; t6
		push ZH					; t7
		push t0
		push t1
;	push t2						; uncomment when programming interrupt-words
;	push t3						; ------------------------------------------
		ldi XH,HIGH(ivec)
		ld ZL,X+
		ld ZH,X+
		mijmp
	.dw 0xffff,0xffff			; ... and comment out this line
								; -----------------------------
	.if CPU_LOAD == 0
		  .dw 0xffff,0xffff,0xffff
	.elif CPU_LOAD == 1
	.else .error "illegal value: CPU_LOAD"
	.endif

.org BOOT_START - 0x0b			; DO NOT CHANGE !! (requires reburning of bootloader part)
INTx16_VECTOR:
  .if CPU_LOAD == 0
			ldi INTvector,0x16 + LOW(ivec)
		    rjmp OF_ISR
			.dw 0xffff
  .elif CPU_LOAD == 1			; -> load-timer interrupt
			;in_ SREG_intSafe,SREG	; (bootloader does)
			inc loadreg			; inc load-counter
			out_ SREG,SREG_intSafe
			reti
  .else .error "illegal value: CPU_LOAD"
  .endif 

.org BOOT_START - 0x08			; DO NOT CHANGE !! (requires reburning of bootloader part)
UDRE_VECTOR:
  .if IDLE_MODE == 0
			ldi INTvector,0x26 + LOW(ivec)
		    rjmp OF_ISR
			.dw 0xffff,0xffff,0xffff,0xffff,0xffff
  .elif IDLE_MODE == 1
	.if CPU_LOAD == 0
	  UDRE_ISR:
			ldi INTvector,(1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)
			out_ UCSR0B,INTvector	; disable UDRE0-interrupt
			reti
			.dw 0xffff,0xffff,0xffff
	.elif CPU_LOAD == 1
	  UDRE_ISR:
			ldi INTvector,9
		  	out_ TCCR1B,INTvector	; restart load-timer (CTC, /1)
			ldi INTvector,(1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)
			out_ UCSR0B,INTvector	; disable UDRE0-interrupt
			reti
	.else .error "illegal value: CPU_LOAD" 
	.endif  ; (CPU_LOAD)
  .else .error "illegal value: IDLE_MODE"
  .endif	; (IDLE_MODE)

.org BOOT_START - 0x01			; DO NOT CHANGE !! (requires reburning of bootloader part)
WARM_VECTOR:
			rjmp WARM_0

; ################################################################################ ;
;                                                                                  ;
; OptiForth bootloader                                                             ; 
;                                                                                  ;
; based on Optiboot (https://githib.com/Optiboot/optiboot)                         ;
;  "Although it has evolved considerably, Optiboot builds on the original work     ;
;   of Jason P. Kyle (stk500boot.c), Arduino group (bootloader),                   ;
;   Spiff (1K bootloader), AVR-Libc group and Ladyada (Adaboot).                   ;
;                                                                                  ;
;   Optiboot is the work of Peter Knight (aka Cathedrow). Despite some             ;
;   misattributions, it is not sponsored or supported by any organisation          ;
;   or company including Tinker London, Tinker.it! and Arduino.                    ;
;   Maintenance of optiboot was taken over by Bill Westfield (aka WestfW) in 2011.";
;                                                                                  ;
; fits into 256 words of FLASH                                                     ; 
;                                                                                  ;
; extended functions:                                                              ;
;   - save MCUSR reset status in R0                                                ;
;   - increased USART speed   (250.000 baud standard)                              ;
;   - read/write EEPROM                                                            ;
;   - read FUSE                                                                    ;
;   - read/write LOCK bits                                                         ;
;   - read SIGNATURE from MCU (if enabled)                                         ;
;   - read OSCCAL             (if enabled)                                         ;
;                                                                                  ;
; MIT License                                                                      ;
;                                                                                  ;
; Copyright (c) 2020 bitflipser                                                    ;
;                                                                                  ;
; Permission is hereby granted, free of charge, to any person obtaining a copy     ;
; of this software and associated documentation files (the "Software"), to deal    ;
; in the Software without restriction, including without limitation the rights     ;
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell        ;
; copies of the Software, and to permit persons to whom the Software is            ;
; furnished to do so, subject to the following conditions:                         ;
;                                                                                  ;
; The above copyright notice and this permission notice shall be included in all   ;
; copies or substantial portions of the Software.                                  ;
;                                                                                  ;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR       ;
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,         ;
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE      ;
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER           ;
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,    ;
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE    ;
; SOFTWARE.                                                                        ;
;                                                                                  ;
; ################################################################################ ;

; 1 - ###############
;     select MCU type

; ######### tested for ATmega328/P ONLY ###########
.equ ATmega =328
; #################################################

; 2 - ###################
;     select MCU sub type

;.equ subType=0			; no letter
;.equ subType=1			; A
;.equ subType=2			; L
.equ subType=3			; P
;.equ subType=4			; PA
;.equ subType=5			; V

; 3 - ################
;     select frequency

;.equ F_CPU=20			; 20 MHz
.equ F_CPU=16			; 16 MHz
;.equ F_CPU=10			; 10 MHz
;.equ F_CPU= 8			;  8 MHz
;.equ F_CPU= 1			;  1 MHz

; 4 - ##############
;     select Vtarget

.equ Vtarget =50		; 5,0 V
;.equ Vtarget =33		; 3,3 V
;.equ Vtarget =xy		; x,y V - set value

; 5 - ##############################################
;     select standard or high-speed USART connection

;.equ USARTspeed=0			; standard baud rates 
.equ USARTspeed=1			; increased USART speed, better timing accuracy

; 6 - ########################
;     select Watchdog Time Out

;.equ WDtimeOut = 0x0c		; 250 ms
;.equ WDtimeOut = 0x0d		; 500 ms
.equ WDtimeOut = 0x0e		;   1 s
;.equ WDtimeOut = 0x0f		;   2 s
;.equ WDtimeOut = 0x28		;   4 s
;.equ WDtimeOut = 0x29		;   8 s

; 7 - ######################
;     memory saving specials

.equ noADDRpreset		 =1	; uncomment to save  2 words: (it's not needed with avrdude)
.equ noLED_START_FLASH	 =1	; uncomment to save 14 words: LED will NOT blink at start
.equ noVtarget			 =1	; uncomment to save  4 words: Vtarget-reply will be 2.0 V
.equ noOSCCAL			 =1	; uncomment to save  5 words: STK_READ_OSCCAL will return 0
.equ noSIGfromMCU		 =1	; uncomment to save 10 words: SIGNATURE taken from bootloader code instead of MCU
;   includes noOSCCAL=1

; 8 - ###################
;     set LED port values

.equ LED_DDR=DDRB
.equ LED_PORT=PORTB
.equ LED_PIN=PINB
.equ LEDpin=5


.equ SIG_1=0x1e

.if ATmega == 328
	.set SIG_2=0x95
	.if subType==0
		.equ SIG_3=0x14
		.NOLIST
;		.include "m328def.inc"	; NOT in use, see 'of52_config.inc'
		.LIST
	.elif subType==3
		.equ SIG_3=0x0f
		.NOLIST
;		.include "m328pdef.inc"	; NOT in use, see 'of52_config.inc'
		.LIST
	.else
		.error "No such ATmega328 sub type"
	.endif
.endif

.if F_CPU == 20
	.if USARTspeed == 0
		; standard baud rates 							 Error
		.set Baud = 0x0015				;  115.200 baud: -1,4%
		.set doubleSpeed = 1
	.elif USARTspeed == 1
		; high-speed baud rates
		.set Baud = 0x0004				;  250.000 baud:  0,0%
		.set doubleSpeed = 0
	.endif
.endif

.if F_CPU == 16
	.if USARTspeed == 0
		; standard baud rates 							 Error
		.set Baud = 0x0010				;  115.200 baud:  2,1%
		.set doubleSpeed = 1
	.elif USARTspeed == 1
		; high-speed baud rates
		.set Baud = 0x0003				;  250.000 baud:  0,0%
		;.set Baud = 0x0001				;  500.000 baud:  0,0%
		;.set Baud = 0x0000				;1.000.000 baud:  0,0%
		.set doubleSpeed = 0
	.endif
.endif

.if F_CPU == 10
	.if USARTspeed == 0
		; standard baud rates 							 Error
		.set Baud = 0x0015				;   57.600 baud: -1,4%
		;.set Baud = 0x000a				;  115.200 baud: -1,4% 
		.set doubleSpeed = 1
	.elif USARTspeed == 1
		; high-speed baud rates
		.set Baud = 0x0004				;  250.000 baud:  0,0%
		.set doubleSpeed = 1
	.endif
.endif

.if F_CPU == 8
	.if USARTspeed == 0
		; standard baud rates 							 Error
		.set Baud = 0x0010				;   57.600 baud: -0,8%
		.set doubleSpeed = 1
	.elif USARTspeed == 1
		; high-speed baud rates
		.set Baud = 0x0001				;  250.000 baud:  0,0%
		;.set Baud = 0x0000				;  500.000 baud:  0,0%
		.set doubleSpeed = 0
	.endif
.endif

.if F_CPU == 1
	.if USARTspeed == 0
		; standard baud rates 							 Error
		
		.set Baud = 0x000c				;    4.800 baud:  0,2%
		.set doubleSpeed = 0
		/*
		.set Baud = 0x000c				;    9.600 baud:  0,2%
		.set doubleSpeed = 1
		*/
	.elif USARTspeed == 1
		; high-speed baud rates
		.set Baud = 0x0000				;   62.500 baud:  0,0%
		.set doubleSpeed = 0
	.endif
.endif

.equ USARTbase	= UCSR0A
.equ oUCSR0A	= UCSR0A-USARTbase		; register offsets to be accessed with ldd/std and Y+d
.equ oUCSR0B	= UCSR0B-USARTbase
.equ oUCSR0C	= UCSR0C-USARTbase
.equ oUBRR0L	= UBRR0L-USARTbase
.equ oUBRR0H	= UBRR0H-USARTbase
.equ oUDR0		= UDR0  -USARTbase

.equ buff = 0x200						; bootloader buffer addr in SRAM - DO NOT CHANGE!!!

.if withBOOTLOADER == 0				; w/o bootloader
	.equ RESET_				= BOOT_START
	.equ DO_PAGE_ERASE		= BOOT_START + 0x34
	.equ DO_SPM				= BOOT_START + 0x35
	.equ WRITE_FLASH_PAGE	= BOOT_START + 0xf0
.else 								;with bootloader

	.cseg
	.org BOOT_START
	RESET_:		rjmp BOOT
				.dw 0xffff
	.org BOOT_START + 0x02
				ldi INTvector,0x02 + LOW(ivec)
	            rjmp OF_ISR
	.org BOOT_START + 0x04
				ldi INTvector,0x04 + LOW(ivec)
	            rjmp OF_ISR
	.org BOOT_START + 0x06
				ldi INTvector,0x06 + LOW(ivec)
	            rjmp OF_ISR
	.org BOOT_START + 0x08
	.if MS_TIMER_ADDR == 0x08
	            rjmp  MS_TIMER_ISR
				.dw 0xffff
	.else
				ldi INTvector,0x08 + LOW(ivec)
	            rjmp OF_ISR
	.endif
	.org BOOT_START + 0x0a
				ldi INTvector,0x0a + LOW(ivec)
	            rjmp OF_ISR
	.org BOOT_START + 0x0c
				ldi INTvector,0x0c + LOW(ivec)
	            rjmp OF_ISR
	.org BOOT_START + 0x0e
	.if MS_TIMER_ADDR == 0x0e
	            rjmp  MS_TIMER_ISR
				.dw 0xffff
	.else
				ldi INTvector,0x0e + LOW(ivec)
	            rjmp OF_ISR
	.endif
	.org BOOT_START + 0x10
				ldi INTvector,0x10 + LOW(ivec)
	            rjmp OF_ISR
	.org BOOT_START + 0x12
	.if MS_TIMER_ADDR == 0x12
	            rjmp  MS_TIMER_ISR
				.dw 0xffff
	.else
				ldi INTvector,0x12 + LOW(ivec)
	            rjmp OF_ISR
	.endif
	.org BOOT_START + 0x14
	.if MS_TIMER_ADDR == 0x14
	            rjmp  MS_TIMER_ISR
				.dw 0xffff
	.else
				ldi INTvector,0x14 + LOW(ivec)
	            rjmp OF_ISR
	.endif
	.org BOOT_START + 0x16					; TIMER1_COMPA
	.if MS_TIMER_ADDR == 0x16
	            rjmp  MS_TIMER_ISR
				.dw 0xffff
	.else
				in_ SREG_intSafe,SREG
				rjmp INTx16_VECTOR
	.endif
	.org BOOT_START + 0x18
	.if MS_TIMER_ADDR == 0x18
	            rjmp  MS_TIMER_ISR
				.dw 0xffff
	.else
				ldi INTvector,0x18 + LOW(ivec)
	            rjmp OF_ISR
	.endif
	.org BOOT_START + 0x1a
	.if MS_TIMER_ADDR == 0x1a
	            rjmp  MS_TIMER_ISR
				.dw 0xffff
	.else
				ldi INTvector,0x1a + LOW(ivec)
	            rjmp OF_ISR
	.endif
	.org BOOT_START + 0x1c
	.if MS_TIMER_ADDR == 0x1c
	            rjmp  MS_TIMER_ISR
				.dw 0xffff
	.else
				ldi INTvector,0x1c + LOW(ivec)
	            rjmp OF_ISR
	.endif
	.org BOOT_START + 0x1e
	.if MS_TIMER_ADDR == 0x1e
	            rjmp  MS_TIMER_ISR
				.dw 0xffff
	.else
				ldi INTvector,0x1e + LOW(ivec)
	            rjmp OF_ISR
	.endif
	.org BOOT_START + 0x20
	.if MS_TIMER_ADDR == 0x20
	            rjmp  MS_TIMER_ISR
				.dw 0xffff
	.else
				ldi INTvector,0x20 + LOW(ivec)
	            rjmp OF_ISR
	.endif
	.org BOOT_START + 0x22
	.if MS_TIMER_ADDR == 0x22
	            rjmp  MS_TIMER_ISR
				.dw 0xffff
	.else
				ldi INTvector,0x22 + LOW(ivec)
	            rjmp OF_ISR
	.endif
	.org BOOT_START + 0x24
				ldi INTvector,0x24 + LOW(ivec)
				rjmp OF_ISR
	.if 0x26 < INT_VECTORS_SIZE
	.org BOOT_START + 0x26
	  .if UDREaddr == 0x26
				rjmp UDRE_VECTOR
				.dw 0xffff
	  .else
				ldi INTvector,0x26 + LOW(ivec)
				rjmp OF_ISR
	  .endif
	.endif
	.if 0x28 < INT_VECTORS_SIZE
	.org BOOT_START + 0x28
				ldi INTvector,0x28 + LOW(ivec)
	            rjmp OF_ISR
	.endif
	.if 0x2a < INT_VECTORS_SIZE
	.org BOOT_START + 0x2a
	  .if MS_TIMER_ADDR == 0x2a
	            rjmp  MS_TIMER_ISR
				.dw 0xffff
	  .else
				ldi INTvector,0x2a + LOW(ivec)
	            rjmp OF_ISR
	  .endif
	.endif
	.if 0x2c < INT_VECTORS_SIZE
	.org BOOT_START + 0x2c
				ldi INTvector,0x2c + LOW(ivec)
	            rjmp OF_ISR
	.endif
	.if 0x2e < INT_VECTORS_SIZE
	.org BOOT_START + 0x2e
				ldi INTvector,0x2e + LOW(ivec)
	            rjmp OF_ISR
	.endif
	.if 0x30 < INT_VECTORS_SIZE
	.org BOOT_START + 0x30
				ldi INTvector,0x30 + LOW(ivec)
	            rjmp OF_ISR
	.endif
	.if 0x32 < INT_VECTORS_SIZE
	.org BOOT_START + 0x32
				ldi INTvector,0x32 + LOW(ivec)
	            rjmp OF_ISR
	.endif
	
	.org BOOT_START + 0x34			; DO NOT CHANGE !! ########################
	DO_PAGE_ERASE:
	        ldi t1,(1<<PGERS)|(1<<SPMEN) 	; R17
	.org BOOT_START + 0x35			; DO NOT CHANGE !! ########################
	DO_SPM:
	          in_ t0,SPMCSR					; R16
	          sbrc t0,SPMEN
	        rjmp DO_SPM       		; Wait for previous write/erase to complete
	DO_SPM_0:
	;+++++++++++++++++++++++++++++++++
			in_ t0,SREG						; R16
			cli
	        out_ SPMCSR,t1					; R17
	        spm
			out_ SREG,t0
	;+++++++++++++++++++++++++++++++++
	        ret


	BOOT:
		clr YH						; const YH = 0 (fix all program)
		in R0, MCUSR				; save MCUSR for application program
		out MCUSR, YH
		bst R0, 1
		  brts extReset
	appStart:						; no external reset
		clr R24						; -> stop watchdog		
		rcall watchdogConfig
		rjmp WARM_VECTOR			; start OptiForth
	
	extReset:
		ldi YL, LOW(USARTbase)		; use Y as IObase pointer (fix all program)
	;	ldi YH,HIGH(USARTbase)		; (already there)
		
	.ifndef noADDRpreset
		clr ibaseL					; set addressH:L to 0x0000 by default
		clr ibaseH
	.endif

		ldi XH, 2					; const XH = 2 (while running bootloader)
	.if doubleSpeed == 1
		std Y+oUCSR0A, XH			; USART0 double speed mode
	.endif
		ldi R18, 0x18				; (fix in bootloader - do not change)
		std Y+oUCSR0B, R18			; RX enable, TX enable
	;	ldi t0, 0x06				; reset value - no need to write
	;	std Y+oUCSR0C, t0			; 8.1 no parity
	.if Baud == 0
	;	std Y+oUBRR0L, YH			; reset value - no need to write
	.else
		ldi R25, Baud
		std Y+oUBRR0L, R25
	.endif
	
		ldi R24, WDtimeOut
		rcall watchdogConfig

	.ifndef noLED_START_FLASH
							; LED_START_FLASH
	 .if F_CPU == 16 || F_CPU == 20
		ldi t0, 12					; blink 6 times
	 .else
		ldi t0, 6					; blink 3 times
	 .endif

		sbi LED_DDR, LEDpin		; LED 'output'
	 ;	cbi LED_PORT, LEDpin		; reset value - no need to write
		ldi R25, 1

	 .if F_CPU > 1					; start TC1
		out_ TCCR1B, XH				; XH = 2 -> prescale /8
	 .else
		out_ TCCR1B, R25			; prescale /1
	 .endif

	 blinkLED:
		  out_ TIFR1, R25			; clear TOV1
	   TC1wait:
		    sbis TIFR1, TOV1		; wait TOV1
		  rjmp TC1wait
		  sbi LED_PIN, LEDpin		; toggle LED 'on/off'
		  wdr
		dec t0
		brne blinkLED
		out_ TCCR1B, t0				; stop TC1
	.else
			; noLED_START_FLASH	
	.endif
	
	foreverLoop:
		rcall getch
	checkA:
		cpi R24, 0x41				; 'A' - STK_GET_PARAMETER
		  brne checkB

	getParameter:
		rcall getch
	  .ifndef noVtarget
			mov t1, R24				; save 2nd command byte
			rcall verifySpace
			ldi R24, Vtarget			; = 3.3 / 5.0 V
			cpi t1, 0x84
			  breq byte_response_R24
		byte_response_0:				; all other 'A'-params
			clr R24						; 0
	  .else						; noVtarget == 1
			rcall verifySpace
	  .endif
	byte_response_R24:
		rcall putch					; send response
		rjmp putchSTK_OK			; send 'OK'

	checkB:
		cpi R24, 0x42				; 'B' - STK_SET_DEVICE
		ldi R25, 0x14
		  breq getThat				; go ignore cmd by ..

	checkE:
		cpi R24, 0x45				; 'E' - STK_SET_DEVICE_EXT
		  brne checkU
	
	setDeviceExt:
		ldi R25, 0x05				; ignore cmd by ..
	getThat:
		rcall getNch_R25			; .. dropping next #R25 chars + verifySpace
		rjmp putchSTK_OK			; send 'OK'
	
	checkU:
		cpi R24, 0x55				; 'U' - STK_LOAD_ADDRESS
		  brne checkV
	
	loadAddress:					; little endian, in words
		rcall getch
		mov ibaseL, R24				; addressH:L ibase
		rcall getch
		mov ibaseH, R24
		lsl ibaseL					; make it byte address
		rol ibaseH
		rjmp nothing_response		; verifySpace + STK_OK

	checkV:
		cpi	R24, 0x56				; 'V' - STK_UNIVERSAL
		  brne check_d
	
	Universal:
		rcall getch
		ldi ZL, 1
		clr ZH
		ldi R25, 2
		cpi R24, 0xac
		  brne maybe_fuse

	write_cmd:						; check for 'write lock bits'
		rcall getch
		cpi R24, 0xe0
		  brne getThat
	write_lock:
		rcall getNch_R25			; last received in R0 + verifySpace
		ldi R25, (1<<BLBSET) | (1<<SPMEN)
		out SPMCSR, R25
		spm
		rjmp putchSTK_OK
	
	maybe_fuse:
		cpi R24, 0x58
		  breq fuse_cmd
		clr ZL
		cpi R24, 0x50
		  breq fuse_cmd
	no_fuse_read:
		ldi	R25, 0x03				; not a fuse read
	fuse_read_wrong:
		rcall getNch_R25			; ..drop next . chars
	  .ifndef noVtarget
			rjmp byte_response_0
	  .else
			clr R24
			rjmp byte_response_R24
	  .endif

	fuse_cmd:
		rcall getch
		cpi R24, 0x00
		  breq go_read_fuses
		cpi R24, 0x08
		  brne fuse_read_wrong
		sbr ZL, 1<<1
	go_read_fuses:
		rcall getNch_R25			; ..drop next . chars
		ldi R25, (1<<BLBSET) | (1<<SPMEN)
		out SPMCSR, R25
		lpm R24, Z
		rjmp byte_response_R24

	check_d:
		cpi R24, 0x64				; 'd' - STK_PROG_PAGE
		  brne check_t
	
	progPage:
		rcall page_header 			; get length and set T upon memtype
	
	;	ldi16 X, buff				; buff on page boundary
		ldi XL, LOW(buff)			; buff on page boundary
	;	ldi XH, 2					; constant XH = 2 (while running bootloader)
	fillBuff:
		  rcall getch
		  st X+, R24
		cp R20, XL					; keep R20 as lengthL !! buff on page boundary !!
		brne fillBuff

		rcall verifySpace
	progPage_memtype:
	;	ldi16 X, buff
		ldi XL, LOW(buff)			; buff on page boundary
	;	ldi XH, 2					; unchanged on ATmega328 (constant XH = 2 (while running bootloader))
		brts progPage_eeprom		; T = 0 -> FLASH, T = 1 -> EEPROM
	
	; FLASH							; memtype 'even' (='F') -> flash
		rcall progPage_flash
		rjmp putchSTK_OK
	
	progPage_eeprom:				; memtype 'odd' (='E')  -> eeprom
		  out_ EEARH, ibaseH
		  out_ EEARL, ibaseL			; load EEPROM target address
		  ld	R24, X+					; get value from buff
		  out_ EEDR, R24				; put as EEPROM target data
		  ldi R24, 1<<EEMPE
		  out_ EECR, R24				; enable Prog
		  sbi EECR, EEPE				; start Prog

	  wait_eeprom_ready:
			sbic EECR, EEPE				; wait EEPROM ready
		  rjmp wait_eeprom_ready

		  inc ibaseL					; targetAdr += 1
		dec R20						; lengthL (scratched)
		brne progPage_eeprom

		rjmp putchSTK_OK
	
	check_t:
		cpi R24, 0x74				; 't' - STK_READ_PAGE
		  brne check_u

	readPage:
		rcall page_header 			; get length and set T upon memtype
		rcall verifySpace
		movw Z, IBASE

	readPage_loop:
		  brtc read_flash

	  read_eeprom:
		  out_ EEARH, ZH
		  out_ EEARL, ZL
		  sbi EECR, EERE			; EEReadEnable
		  in_ r24, EEDR
		  adiw Z, 1
		  rjmp rP_0

	  read_flash:
		  lpm R24, Z+ 
	  rP_0:
		  rcall putch				; send
		dec R20						; lengthL (scratched)
		brne readPage_loop
	
		rjmp putchSTK_OK
	
	check_u:
		cpi R24, 0x75				; 'u' - STK_READ_SIGN
		  brne check_v

	readSignature:
		rcall verifySpace
	.ifndef noSIGfromMCU
			ldi ZL, 0x00				; SIG0
			rcall read_sig_
			ldi ZL, 0x02				; SIG1
			rcall read_sig_
			ldi ZL, 0x04				; SIG2
		readLastSig:
			rcall read_sig_
			rjmp putchSTK_OK

		read_sig_:
			clr ZH
			ldi R24, (1<<SIGRD) | (1<<SPMEN)
			out_ SPMCSR, R24
			lpm R24, Z
			rjmp putch
	.else
			ldi R24, SIG_1				; 0x1e - Atmel
			rcall putch
			ldi R24, SIG_2				; 0x9. - ATmega ...
			rcall putch
			ldi R24, SIG_3
			rjmp byte_response_R24
	.endif

	check_v:
	.ifndef noOSCCAL
	 .ifndef noSIGfromMCU		
			cpi R24, 0x76				; 'v' - STK_READ_OSCCAL
			  brne checkQ

		readOSCCAL:
			rcall verifySpace
			ldi ZL, 0x01
			rjmp readLastSig
	  .endif
	.endif

	checkQ:
		cpi	R24, 0x51				; 'Q' - STK_LEAVE_PROGMODE
		  brne allElse

	leaveProgmode:
		ldi	R24, 0x08				; force WD-RESET w/i 16 ms
		rcall watchdogConfig
	
	allElse:
	nothing_response:
		rcall verifySpace
	putchSTK_OK:
		ldi	R24, 0x10				; STK_OK
		rcall putch
		rjmp foreverLoop
	
	page_header:
		rcall getch					; drop lengthH (=0) 
		rcall getch					; get  lengthL - big endian
		mov R20, R24				; R20 = lengthL
		rcall getch
		bst R24, 0					; put Bit0 into T ('E' set, 'F' clear)
		ret	

	putchSTK_INSYNC:
		ldi	R24, 0x14				; STK_INSYNC
	
	putch:
		  ldd	R25, Y+oUCSR0A		; wait USART Data Register empty
		  sbrs R25, UDRE0
		rjmp putch
		std Y+oUDR0, R24			; put Data
		ret
	
	getch:
		  ldd	R24, Y+oUCSR0A		; wait USART Receive complete
		  sbrs R24, RXC0
		rjmp getch

		sbrs R24, FE0				; skip WDR on Framing Error
		  wdr
		ldd	R24, Y+oUDR0			; get char received
		ret

	watchdogConfig:
	;	ldi	R18, (1>>WDE | 1>>WDCE)	; (still there)
		out_ WDTCSR, R18			; const R18 = 0x18 (fix all program - do not change)
		out_ WDTCSR, R24
		ret

	getNch_R25:
		  rcall getch
		dec R25
		brne getNch_R25

		mov R0, R24					; save last received
	;	rjmp verifySpace
	
	verifySpace:
		rcall getch
		cpi	R24, 0x20				; ' '
	 	  breq putchSTK_INSYNC
	not_INSYNC:
	;	ldi	R24, 0x08				; set WD time-out 16 ms
	;	rcall watchdogConfig		; (save space)
	wait_watchdog:
		rjmp wait_watchdog

	
	.org BOOT_START + 0xf0			; DO NOT CHANGE !! ########################
	WRITE_FLASH_PAGE:				; (call with buffer addr in X)
		ldi R20, PAGESIZEB
	progPage_flash:					; (call with buffer addr in X)
		movw Z, IBASE
		sbis FLAGS3, fFLASH_PAGE_CLEAR	; (always clear in bootloader [cleared by chip reset])
		  rcall DO_PAGE_ERASE

		ldi t1, (1<<SPMEN)
	loadPageBuf:
		  ld R0, X+					; data from buffer
		  ld R1, X+
		  rcall DO_SPM
		  adiw Z, 2
		subi R20, 2					; lengthL (scratched)
		brne loadPageBuf

	writePage:
		movw Z, IBASE

		ldi t1, (1<<PGWRT) | (1<<SPMEN)
		rcall DO_SPM
	
		ldi t1, (1<<RWWSRE) | (1<<SPMEN)
		rjmp DO_SPM

.endif ; (appendBOOTLOADER == 1)

