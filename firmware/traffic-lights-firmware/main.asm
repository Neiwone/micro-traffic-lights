.def led0 = r16 ;	Clock Tens Register
.def led1 = r17 ;	Clock Units Register
.def temp = r18 ;	Temporary Register


.def current_stateH = r19 ;		Traffic lights 3 and 4 signal
.def current_stateL = r20 ;		Traffic lights 1 and 2 signal
.def state = r21          ;		Current state
.def change_state_timer = r22 ;	Remaining seconds to change between states


; _______________________________________
;|            Interrupt Vector           |
;|_______________________________________|

.org 0x0000
jmp RESET
.org OC1Aaddr
jmp TIMER_ISR



; _______________________________________
;|            Defining States            |
;|---------------------------------------|
;|     0b10010000'00100100 -> RRRR       |
;|     0b00100100'00100100 -> GGRR       |
;|     0b01000100'00100100 -> GYRR       |
;|     0b10000100'00100001 -> GRGR       |
;|     0b10001000'00100010 -> YRYR       |
;|     0b10010000'00001100 -> RRRG       |
;|     0b10010000'00010100 -> RRRY       |
;|_______________________________________|

Table_States:
.dw 0b1001000000100100, 0b0010010000100100, 0b0100010000100100, 0b1000010000100001, 0b1000100000100010, 0b1001000000001100, 0b1001000000010100

; ___________________________________________________________________________________________________________
;|          Defining States String																			 |
;|-----------------------------------------------------------------------------------------------------------|
;|    state0: "Semaforo 0: Vermelho, Semaforo 1: Vermelho, Semaforo 2: Vermelho, Semaforo 3: Vermelho ", "#" |
;|    state1: "Semaforo 0: Verde, Semaforo 1: Verde, Semaforo 2: Vermelho, Semaforo 3: Vermelho ", "#"       |
;|    state2: "Semaforo 0: Verde, Semaforo 1: Amarelo, Semaforo 2: Vermelho, Semaforo 3: Vermelho ", "#"     |
;|    state3: "Semaforo 0: Verde, Semaforo 1: Vermelho, Semaforo 2: Verde, Semaforo 3: Vermelho ", "#"       |
;|    state4: "Semaforo 0: Amarelo, Semaforo 1: Vermelho, Semaforo 2: Amarelo, Semaforo 3: Vermelho ", "#"   |
;|    state5: "Semaforo 0: Vermelho, Semaforo 1: Vermelho, Semaforo 2: Vermelho, Semaforo 3: Verde ", "#"    |
;|    state6: "Semaforo 0: Vermelho, Semaforo 1: Vermelho, Semaforo 2: Vermelho, Semaforo 3: Amarelo ", "#"  |
;|___________________________________________________________________________________________________________|

Message_States:
state0_msg: .db "Semaforo 0: Vermelho, Semaforo 1: Vermelho, Semaforo 2: Vermelho, Semaforo 3: Vermelho", "#"   ;	RRRR
state1_msg: .db "Semaforo 0: Verde, Semaforo 1: Verde, Semaforo 2: Vermelho, Semaforo 3: Vermelho", "#"		    ;	GGRR
state2_msg: .db "Semaforo 0: Verde, Semaforo 1: Amarelo, Semaforo 2: Vermelho, Semaforo 3: Vermelho", "#"       ;	GYRR
state3_msg: .db "Semaforo 0: Verde, Semaforo 1: Vermelho, Semaforo 2: Verde, Semaforo 3: Vermelho", "#"         ;	GRGR
state4_msg: .db "Semaforo 0: Amarelo, Semaforo 1: Vermelho, Semaforo 2: Amarelo, Semaforo 3: Vermelho", "#"     ;   YRYR
state5_msg: .db "Semaforo 0: Vermelho, Semaforo 1: Vermelho, Semaforo 2: Vermelho, Semaforo 3: Verde", "#"      ;	RRRG
state6_msg: .db "Semaforo 0: Vermelho, Semaforo 1: Vermelho, Semaforo 2: Vermelho, Semaforo 3: Amarelo", "#"    ;	RRRY


RESET:
	; Stack initialization
	ldi temp, low(RAMEND)
	out SPL, temp
	ldi temp, high(RAMEND)
	out SPH, temp
	; _______________________________________
	;|            Setup of USART0			 |
	;|_______________________________________|

	.equ UBRRvalue = 103 ;16 mhz clock speed, 9600 baud UBRR = 103

	ldi temp, high(UBRRvalue)  
	sts UBRR0H, temp
	ldi temp, low(UBRRvalue)
	sts UBRR0L, temp

	; 8 bits, 1 stop, no parity, asynchronous mode
	ldi temp, (3<<UCSZ00)
	sts UCSR0C, temp

	ldi temp, (1 << TXEN0) ; TX Enable
	sts UCSR0B, temp 

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
	ldi temp, ((WGM>> 2) << WGM12) | (PRESCALE << CS10)
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
	out DDRB, temp
	out DDRC, temp
	out DDRD, temp
	;________________________________________

	; _______________________________________
	;|           Start Definition            |
	;|_______________________________________|

	ldi state, 1
	rcall clear_CUD
	rcall clear_CTD

	rcall STATE1

	;________________________________________



	;/~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\
	;|                                        |
	;|            > MAIN PROGRAM <            |
	;|                                        |
	;\~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~/

	main:
		clc
		;	Compare if is time to change state
		tst change_state_timer
		brne CONTINUE
		inc state

		; if is time to change state, which state should we go now?
		; here we find out by the ´state´ variable
		TEST0:
		ldi temp, 0
		cpse state, temp
		jmp TEST1
		rcall STATE0
		TEST1:
		ldi temp, 1
		cpse state, temp
		jmp TEST2
		rcall STATE1
		TEST2:
		ldi temp, 2
		cpse state, temp
		jmp TEST3
		rcall STATE2
		TEST3:
		ldi temp, 3
		cpse state, temp
		jmp TEST4
		rcall STATE3
		TEST4:
		ldi temp, 4
		cpse state, temp
		jmp TEST5
		rcall STATE4
		TEST5:
		ldi temp, 5
		cpse state, temp
		jmp TEST6
		rcall STATE5
		TEST6:
		ldi temp, 6
		cpse state, temp
		jmp TEST7
		rcall STATE6
		TEST7:
		ldi temp, 7
		cpse state, temp
		jmp CONTINUE
		ldi state, 0
		rcall STATE0

		CONTINUE:
		; Send traffic lights signals by PORTD & PORTB
		out PORTB, current_stateH
		out PORTD, current_stateL
		
		; Send timer display signals by PORTC
		out PORTC, led0
		rcall delay
		out PORTC, led1
		rcall delay

		rjmp main

	;_________________________________________



; _______________________________________
;|   Timer1 Interrupt Service Routine    |
;|_______________________________________|
TIMER_ISR:
	subi change_state_timer, 1

	;	Count down by 1 on the display
	dec led1
	;	Compare Clock Unit Value to 31
	cpi led1, 0b00011111
	brne exit_T1ISR

	;	Reset Clock Unit Value
	rcall clear_CUD
	dec led0
	;	Compare Clock Ten Value to 15
	cpi led0, 0b00001111
	brne exit_T1ISR

	;	Reset Clock Ten Value
	rcall clear_CTD

	exit_T1ISR: 
		reti

; _______________________________________
;|         Clear timer functions         |
;|---------------------------------------|
;| clear_CTD: Set display of tens to 9   |
;| clear_CUD: Set display of units to 9  |
;|_______________________________________|
; > TENS DISPLAY
clear_CTD:
	;	Start with 9
	ldi led0, 0b00001001
	;	Set transistor value of Clock Tens Display to 1 
	ori led0, 1 << 4
	ret

; > UNITS DISPLAY
clear_CUD:
	;	Start with 9
	ldi led1, 0b00001001
	;	Set transistor value of Clock Units Display to 1 
	ori led1, 1 << 5
	ret
;________________________________________

; _______________________________________
;|		 	  Delay function			 |
;|---------------------------------------|
;|  Keep the CPU busy for 1ms.           |
;|_______________________________________|
delay:
	.equ ClockMHz = 16 ; 16MHz
	.equ DelayMs = 1 ; 2ms
	ldi r29, byte3 (ClockMHz * 1000 * DelayMs / 5)
	ldi r28, high (ClockMHz * 1000 * DelayMs / 5)
	ldi r27, low(ClockMHz * 1000 * DelayMs / 5)
	
	subi r27,1
	sbci r28,0
	sbci r29,0
	brcc pc-3

	ret
;________________________________________

; ________________________________________
;|		 		 State 0				  |
;|----------------------------------------|
;|  All lights are red                    |
;|                                        |
;|  - Load ´change_state_timer´ with 15s  |
;|  - Reload Z pointer with a new address |
;|________________________________________|
STATE0:
	ldi change_state_timer, 15

	; Load traffic lights signal distribution from memory
	ldi ZH, high(Table_States+0<<1)  
    ldi ZL, low(Table_States+0<<1)
    lpm current_stateH, Z+
	lpm current_stateL, Z

	ldi ZH, high(state0_msg<<1)
    ldi ZL, low(state0_msg<<1)
    rcall send_string

	ret

; ________________________________________
;|		 		 State 1				  |
;|----------------------------------------|
;|  Traffic lights:                       |
;|   > 1 and 2 are green                  |
;|   > 3 and 4 are red                    |
;|                                        |
;|  - Load ´change_state_timer´ with 25s  |
;|  - Reload Z pointer with a new address |
;|  - Change display start value          |
;|________________________________________|
STATE1:
	ldi change_state_timer, 25

	ldi ZH, high(Table_States+1<<1)  
    ldi ZL, low(Table_States+1<<1)
    lpm current_stateH, Z+
	lpm current_stateL, Z

	;	Count down 90 sec

	;	Load 9 on led0
	ldi led0, 0b00001001
	;	Set transistor value of Clock Tens Display to 1 
	ori led0, 1 << 4

	;	Load 0 on led1
	ldi led1, 0b00000000
	;	Set transistor value of Clock Units Display to 1 
	ori led1, 1 << 5

	ldi ZH, high(state1_msg<<1)
    ldi ZL, low(state1_msg<<1)
    rcall send_string

	ret

; ________________________________________
;|		 		 State 2				  |
;|----------------------------------------|
;|  Traffic lights:                       |
;|   > 1 is green                         |
;|   > 2 is yellow                        |
;|   > 3 and 4 are red                    |
;|                                        |
;|  - Load ´change_state_timer´ with 5s   |
;|  - Reload Z pointer with a new address |
;|________________________________________|
STATE2:
	ldi change_state_timer, 5

	ldi ZH, high(Table_States+2<<1)  
    ldi ZL, low(Table_States+2<<1)
    lpm current_stateH, Z+
	lpm current_stateL, Z

	ldi ZH, high(state2_msg<<1)
    ldi ZL, low(state2_msg<<1)
    rcall send_string

	ret

; ________________________________________
;|		 		 State 3				  |
;|----------------------------------------|
;|  Traffic lights:                       |
;|   > 1 and 3 are green                  |
;|   > 2 and 4 are red                    |
;|                                        |
;|  - Load ´change_state_timer´ with 60s  |
;|  - Reload Z pointer with a new address |
;|________________________________________|
STATE3:
	ldi change_state_timer, 60

	ldi ZH, high(Table_States+3<<1)  
    ldi ZL, low(Table_States+3<<1)
    lpm current_stateH, Z+
	lpm current_stateL, Z

	ldi ZH, high(state3_msg<<1)
    ldi ZL, low(state3_msg<<1)
    rcall send_string

	ret

; ________________________________________
;|		 		 State 4				  |
;|----------------------------------------|
;|  Traffic lights:                       |
;|   > 1 and 3 are yellow                 |
;|   > 2 and 4 are red                    |
;|                                        |
;|  - Load ´change_state_timer´ with 5s   |
;|  - Reload Z pointer with a new address |
;|  - Change display start value          |
;|________________________________________|
STATE4:
	ldi change_state_timer, 5

	ldi ZH, high(Table_States+4<<1)  
    ldi ZL, low(Table_States+4<<1)
    lpm current_stateH, Z+
	lpm current_stateL, Z

	;	Count down 05 sec

	;	Load 0 on led0
	ldi led0, 0b00000000
	;	Set transistor value of Clock Tens Display to 1 
	ori led0, 1 << 4

	;	Load 5 on led1
	ldi led1, 0b00000101
	;	Set transistor value of Clock Units Display to 1 
	ori led1, 1 << 5

	ldi ZH, high(state4_msg<<1)
    ldi ZL, low(state4_msg<<1)
    rcall send_string

	ret

; ________________________________________
;|		 		 State 5				  |
;|----------------------------------------|
;|  Traffic lights:                       |
;|   > 1, 2 and 3 are red                 |
;|   > 4 is green                         |
;|                                        |
;|  - Load ´change_state_timer´ with 25s  |
;|  - Reload Z pointer with a new address |
;|  - Change display start value          |
;|________________________________________|
STATE5:
	ldi change_state_timer, 25

	ldi ZH, high(Table_States+5<<1)  
    ldi ZL, low(Table_States+5<<1)
    lpm current_stateH, Z+
	lpm current_stateL, Z

	;	Count down 45 sec

	;	Load 4 on led0
	ldi led0, 0b00000100
	;	Set transistor value of Clock Tens Display to 1 
	ori led0, 1 << 4

	;	Load 5 on led1
	ldi led1, 0b00000101
	;	Set transistor value of Clock Units Display to 1 
	ori led1, 1 << 5

	ldi ZH, high(state5_msg<<1)
    ldi ZL, low(state5_msg<<1)
    rcall send_string

	ret

; ________________________________________
;|		 		 State 6				  |
;|----------------------------------------|
;|  Traffic lights:                       |
;|   > 1, 2 and 3 are red                 |
;|   > 4 is yellow                        |
;|                                        |
;|  - Load ´change_state_timer´ with 5s   |
;|  - Reload Z pointer with a new address |
;|________________________________________|
STATE6:
	ldi change_state_timer, 5

	ldi ZH, high(Table_States+6<<1)  
    ldi ZL, low(Table_States+6<<1)
    lpm current_stateH, Z+
	lpm current_stateL, Z

	ldi ZH, high(state6_msg<<1)
    ldi ZL, low(state6_msg<<1)
    rcall send_string

	ret

; _______________________________________
;| Send string via USART0 (Z ptr)        |
;|---------------------------------------|
;| Entrada: Z aponta para string em ROM  |
;| String termina em caractere '#'       |
;|_______________________________________|

send_string:
    lpm temp, Z+             ; lê caractere da flash
    cpi temp, '#'            ; chegou no fim?
    breq send_done
wait_udre:
    lds r27, UCSR0A
    sbrs r27, UDRE0          ; espera buffer vazio
    rjmp wait_udre
    sts UDR0, temp           ; transmite caractere
    rjmp send_string
send_done:
    ret
