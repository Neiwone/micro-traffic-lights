.def led0 = r16
.def led1 = r17
.def temp = r18
.def PORTAD = r19
.def TOPL = r24
.def TOPH = r25


; Interrupt Vector
.org 0x0000
jmp RESET

.org OC1Aaddr
jmp TIMER_ISR

RESET:
	; Timer/Counter 1 Setup
	#define CLOCK 16.0e6 ;clock speed
	#define DELAY 1 ;seconds
	.equ PRESCALE = 0b101
	.equ PRESCALE_DIV = 1024
	.equ WGM = 0b0100
	.equ TOP = int(0.5 + ((CLOCK/PRESCALE_DIV)*DELAY))
	.if TOP > 65535
	.error "TOP is out of range"
	.endif

	; Seta o valor de TOP 
	ldi TOPH, high(TOP)
	sts OCR1AH, TOPH
	ldi TOPL, low(TOP)
	sts OCR1AL, TOPL

	; Configura o modo CTC e PRESCALE
	ldi temp, ((WGM&0b11) << WGM10)
	sts TCCR1A, temp
	ldi temp, ((WGM>> 2) << WGM12)|(PRESCALE << CS10)
	sts TCCR1B, temp ;start counter

	; Interrupt Setup
	lds	r17, TIMSK1
	sbr r17, 1 << OCIE1A
	sts TIMSK1, r17
	sei

	; I/O Port Setup
	ldi temp, $FF
	out DDRC, temp
	out DDRD, temp
	
	ldi led0, 0b00000000
	ldi led1, 0b00000000


	ldi PORTAD, 0b00100100

	main:
		out PORTD, PORTAD
		
		ori led0, 1 << 4
		out PORTC, led0
		rcall delay
		ori led1, 1 << 5
		out PORTC, led1
		rcall delay
		jmp main
	nop

TIMER_ISR:
	inc led1
	cpi led1, 0b101010
	brne sair
	ldi led1, 0b00000000
	inc led0
	sair:
		reti

delay:
	.equ ClockMHz = 16 ;16MHz
	.equ DelayMs = 2 ;2ms
	ldi r29, byte3 (ClockMHz * 1000 * DelayMs / 5)
	ldi r28, high (ClockMHz * 1000 * DelayMs / 5)
	ldi r27, low(ClockMHz * 1000 * DelayMs / 5)
	
	subi r27,1
	sbci r28,0
	sbci r29,0
	brcc pc-3

	ret