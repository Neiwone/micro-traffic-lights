.def led0 = r16 ;	Clock Tens Register
.def led1 = r17 ;	Clock Units Register
.def temp = r18 ;	Temporary Register
.def PORTAD = r19


; _______________________________________
;|            Interrupt Vector           |
;|_______________________________________|
.org 0x0000
jmp RESET
.org OC1Aaddr
jmp TIMER_ISR



RESET:
	; _______________________________________
	;|         Timer/Counter 1 Setup         |
	;|_______________________________________|

	;	Set TOP value

	#define CLOCK 16.0e6 ;clock speed
	#define DELAY 1 ;seconds
	.equ PRESCALE = 0b101
	.equ PRESCALE_DIV = 1024
	.equ WGM = 0b0100
	.equ TOP = int(0.5 + ((CLOCK/PRESCALE_DIV)*DELAY))
	.if TOP > 65535
	.error "TOP is out of range"
	.endif

	;	Load TOP in output compare registers for Timer/Counter 1

	ldi temp, high(TOP)
	sts OCR1AH, temp
	ldi temp, low(TOP)
	sts OCR1AL, temp

	;	Configure CTC mode and PRESCALE

	ldi temp, ((WGM&0b11) << WGM10)
	sts TCCR1A, temp
	ldi temp, ((WGM>> 2) << WGM12)|(PRESCALE << CS10)
	sts TCCR1B, temp ;start counter
	;________________________________________

	; _______________________________________
	;|       Interrupt Registers Setup       |
	;|_______________________________________|

	lds	r17, TIMSK1
	sbr r17, 1 << OCIE1A
	sts TIMSK1, r17
	sei
	;________________________________________

	; _______________________________________
	;|            I/O Ports Setup            |
	;|_______________________________________|

	ldi temp, $FF
	out DDRC, temp
	out DDRD, temp
	;________________________________________

	; _______________________________________
	;|           Start Definition            |
	;|_______________________________________|

	ldi PORTAD, 0b00100100
	rcall clear_CUD
	rcall clear_CTD

	;________________________________________



	;/~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\
	;|                                        |
	;|            > MAIN PROGRAM <            |
	;|                                        |
	;\~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~/

	main:
		out PORTD, PORTAD
		
		out PORTC, led0
		rcall delay

		out PORTC, led1
		rcall delay

		jmp main

	;_________________________________________



; _______________________________________
;|   Timer1 Interrupt Service Routine    |
;|_______________________________________|
TIMER_ISR:
	inc led1
	;	Compare Clock Unit Value to 9
	cpi led1, 0b00101010
	brne exit_T1ISR

	;	Reset Clock Unit Value
	rcall clear_CUD
	inc led0
	;	Compare Clock Ten Value to 9
	cpi led0, 0b00011010
	brne exit_T1ISR

	;	Reset Clock Ten Value
	rcall clear_CTD

	exit_T1ISR: 
		reti


; _______________________________________
;|         Start Timer Definition        |
;|_______________________________________|
	
; > TENS DISPLAY
clear_CTD:
;	Start with 0
ldi led0, 0b00000000
;	Set transistor value of Clock Tens Display to 1 
ori led0, 1 << 4
ret

; > UNITS DISPLAY
clear_CUD:
;	Start with 0
ldi led1, 0b00000000
;	Set transistor value of Clock Units Display to 1 
ori led1, 1 << 5
ret
;________________________________________

; _______________________________________
;|		 	  Delay function			 |
;|---------------------------------------|
;|  Keep the CPU busy for 2ms.           |
;|_______________________________________|
delay:
	.equ ClockMHz = 16 ; 16MHz
	.equ DelayMs = 2 ; 2ms
	ldi r29, byte3 (ClockMHz * 1000 * DelayMs / 5)
	ldi r28, high (ClockMHz * 1000 * DelayMs / 5)
	ldi r27, low(ClockMHz * 1000 * DelayMs / 5)
	
	subi r27,1
	sbci r28,0
	sbci r29,0
	brcc pc-3

	ret
;________________________________________

