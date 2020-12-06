; *********************************************************************
;                                                                     *
;    Filename:      of_macros.inc                                     *
;    Date:          30.11.2020                                        *
;    MCU:           ATmega328P                                        *
;    Copyright:     bitflipser                                        *
;    Author:        bitflipser                                        *
;                                                                     *
; *********************************************************************
; OptiForth is MIT licensed                                           *
; *********************************************************************

.macro poptos 
	ld tosl, Y+
	ld tosh, Y+
.endmacro

.macro pop_t1t0
	ld t0,Y+
	ld t1,Y+
.endm

.macro pop_t3t2
	ld t2,Y+
	ld t3,Y+
.endm

.macro pop_t5t4
	ld t4,Y+
	ld t5,Y+
.endm

.macro pop_t7t6
	ld t6,Y+
	ld t7,Y+
.endm

.macro pop_X
	ld XL,Y+
	ld XH,Y+
.endm

.macro pop_Z
	ld ZL,Y+
	ld ZH,Y+
.endm

.macro pushtos
	st -Y,tosh
	st -Y,tosl
.endmacro

.macro duptos
	st -Y,tosh
	st -Y,tosl
.endmacro

.macro push_t1t0
	st -Y,t1
	st -Y,t0
.endm

.macro push_t3t2
	st -Y,t3
	st -Y,t2
.endm

.macro push_t5t4
	st -Y,t5
	st -Y,t4
.endm

.macro push_t7t6
	st -Y,t7
	st -Y,t6
.endm

.macro push_X
	st -Y,XH
	st -Y,XL
.endm

.macro push_Z
	st -Y,ZH
	st -Y,ZL
.endm

.macro inline_DOLIT
		pushtos
		ldi	tosl, LOW(@0)
		ldi	tosh,HIGH(@0)
.endmacro

.macro inline_DOUSER
		pushtos
		movw TOP,UP
		sbiw TOP,(-@0)
.endm

.macro in_
 .if (@1 < $40)
		in @0,@1
 .else
		lds @0,@1
 .endif
.endmacro

.macro out_
 .if (@0 < $40)
		out @0,@1
 .else
		sts @0,@1
 .endif
.endmacro

.macro sbi_
 .if (@0 < $20)
		sbi @0,@1
 .else
		in_ @2,@0
		ori @2,exp2(@1)
		out_ @0,@2
 .endif
.endmacro

.macro cbi_
 .if (@0 < $20)
		cbi @0,@1
 .else
		in_ @2,@0
		andi @2,~(exp2(@1))
		out_ @0,@2
 .endif
.endmacro

.macro lpm_
 .if (FLASHEND < 0x8000) ; word address
		lpm @0,@1
 .else
		elpm @0,@1
 .endif
.endmacro

.macro ldi16
	ldi @0L, LOW(@1)
	ldi @0H,HIGH(@1)
.endm

.macro lds16
	lds @0L,@1
	lds @0H,@1+1
.endm

.macro sts16
	sts @0+1,@1H
	sts @0  ,@1L
.endm

.macro sub_pflash_z
 .if (PFLASH > 0)
		subi ZH,HIGH(PFLASH)
 .endif
.endmacro

.macro add_pflash_z
 .if (PFLASH > 0)
		subi ZH,HIGH(0x10000-PFLASH)
 .endif
.endmacro

.macro sub_pflash_tos
 .if (PFLASH > 0)
		subi tosh,HIGH(PFLASH)
 .endif
.endmacro

.macro add_pflash_tos
 .if (PFLASH > 0)
		subi tosh,HIGH(0x10000-PFLASH)
 .endif
.endmacro

.macro rampv_to_c
 .if (FLASHEND >= 0x8000)
		bset 0
 .else
		bclr 0
 .endif
.endmacro

.macro fdw
  .dw ((@0<<1)+PFLASH)
.endmacro

.macro m_pop_zh
 .ifdef EIND
		pop ZH
 .endif
.endmacro

.macro m_pop_xh
 .ifdef EIND
		pop XH
 .endif
.endmacro

.macro m_pop_t0
 .ifdef EIND
		pop t0
 .endif
.endmacro

.macro m_push_t0
 .ifdef EIND
		push t0
 .endif
.endmacro

.macro mijmp
 .ifdef EIND
		eijmp
 .else
		ijmp
 .endif
.endmacro
