
.include "m2560def.inc"

	.def row = r16 ; current row number
	.def col = r17 ; current column number
	.def rmask = r18 ; mask for current row during scan
	.def cmask = r19 ; mask for current column during scan
	.def temp1 = r20
	.def temp2 = r21
	.equ PORTADIR = 0xF0 ; PD7-4: output, PD3-0, input
	.equ INITCOLMASK = 0xEF ; scan from the rightmost column,
	.equ INITROWMASK = 0x01 ; scan from the top row
	.equ ROWMASK = 0x0F ; for obtaining input from Port D


.macro do_lcd_command
	ldi r16, @0
	rcall lcd_command
	rcall lcd_wait
.endmacro
.macro do_lcd_data
	ldi r16, @0
	rcall lcd_data
	rcall lcd_wait
.endmacro
.macro do_lcd_data1
	mov r16, @0
	rcall lcd_data
	rcall lcd_wait
.endmacro

.org 0
	jmp RESET

RESET:
	ldi temp1, low(RAMEND) ; initialize the stack
	out SPL, temp1
	ldi temp1, high(RAMEND)
	out SPH, temp1
	ldi temp1, PORTADIR ; PA7:4/PA3:0, out/in
	sts DDRL, temp1
	ser temp1 ; PORTC is output
	out DDRC, temp1
	out PORTC, temp1


	ser r16
	out DDRF, r16
	out DDRA, r16
	clr r16
	out PORTF, r16
	out PORTA, r16

main:
	ldi cmask, INITCOLMASK ; initial column mask
	clr col ; initial column
	colloop:
	cpi col, 4
	breq main ; If all keys are scanned, repeat.
	sts PORTL, cmask ; Otherwise, scan a column.
	ldi temp1, 0xFF ; Slow down the scan operation.
delay: 
	dec temp1
	brne delay
	lds temp1, PINL ; Read PORTA
	andi temp1, ROWMASK ; Get the keypad output value
	cpi temp1, 0xF ; Check if any row is low
	breq nextcol
	; If yes, find which row is low
	ldi rmask, INITROWMASK ; Initialize for row check
	clr row ; 
rowloop:
	cpi row, 4
	breq nextcol ; the row scan is over.
	mov temp2, temp1
	and temp2, rmask ; check un-masked bit
	breq convert ; if bit is clear, the key is pressed
	inc row ; else move to the next row
	lsl rmask
	jmp rowloop
nextcol: ; if row scan is over
	lsl cmask
	inc col ; increase column value
	jmp colloop ; go to the next column
convert:
	cpi col, 3 ; If the pressed key is in col.3
	breq letters ; we have a letter
	; If the key is not in col.3 and
	cpi row, 3 ; If the key is in row3,
	breq symbols ; we have a symbol or 0
	mov temp1, row ; Otherwise we have a number in 1-9
	lsl temp1
	add temp1, row
	add temp1, col ; temp1 = row*3 + col
	subi temp1, -1 ; Add the value of character ‘1’
	jmp convert_end
letters:
	ldi temp1, 'A'
	add temp1, row ; Get the ASCII value for the key
	jmp convert_end
symbols:
	cpi col, 0 ; Check if we have a star
	breq star
	cpi col, 1 ; or if we have zero
	breq zero
	ldi temp1, '#' ; if not we have hash
	jmp convert_end
star:
	ldi temp1, '*' ; Set to star
	jmp convert_end
zero:
	ldi temp1, 10 ; Set to zero
convert_end:
	ldi r17, 1
	ldi r16, 1
	push r16
	clr r16
	out DDRG, r16
	pop r16
	mov r18, temp1
	push r18
	cpi r18, 9
		breq floor9
		brge floor10
	rjmp leftshift
floor10:
	push temp1
	ser temp1
	out DDRG, temp1 ;set Port G
	pop temp1
	ldi r18, 3
	out PORTG, r18
	rjmp leftshift
floor9:
	push temp1
	ser temp1
	out DDRG, temp1 ;set Port G
	pop temp1
	ldi r18, 1
	out PORTG, r18
leftshift:
	cp r17, temp1
		breq end
	lsl r16
	subi r16, -1
	inc r17
	rjmp leftshift
end:
	mov temp1, r16
	out PORTC, temp1 ; Write value to PORTC

	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_5ms
	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_1ms
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00001000 ; display off?
	do_lcd_command 0b00000001 ; clear display
	do_lcd_command 0b00000110 ; increment, no display shift
	do_lcd_command 0b00001110 ; Cursor on, bar, no blink
	do_lcd_command 0b11000000


	do_lcd_data 'F'
	do_lcd_data 'l'
	do_lcd_data 'o'
	do_lcd_data 'o'
	do_lcd_data 'r'
	do_lcd_data ' '
	do_lcd_data 'r'
	do_lcd_data 'e'
	do_lcd_data 'q'
	do_lcd_data 'u'
	do_lcd_data 'e'
	do_lcd_data 's'
	do_lcd_data 't'
	do_lcd_data ' '
	pop r18
	subi r18, -48
	cpi r18, 58
		breq display10
	do_lcd_data1 r18

	jmp main ; Restart main loop
display10:
	do_lcd_data '1'
	do_lcd_data '0'

	jmp main


halt:
	rjmp halt

.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4

.macro lcd_set
	sbi PORTA, @0
.endmacro
.macro lcd_clr
	cbi PORTA, @0
.endmacro

;
; Send a command to the LCD (r16)
;

lcd_command:
	out PORTF, r16
	rcall sleep_1ms
	lcd_set LCD_E
	rcall sleep_1ms
	lcd_clr LCD_E
	rcall sleep_1ms
	ret

lcd_data:
	out PORTF, r16
	lcd_set LCD_RS
	rcall sleep_1ms
	lcd_set LCD_E
	rcall sleep_1ms
	lcd_clr LCD_E
	rcall sleep_1ms
	lcd_clr LCD_RS
	ret

lcd_wait:
	push r16
	clr r16
	out DDRF, r16
	out PORTF, r16
	lcd_set LCD_RW
lcd_wait_loop:
	rcall sleep_1ms
	lcd_set LCD_E
	rcall sleep_1ms
	in r16, PINF
	lcd_clr LCD_E
	sbrc r16, 7
	rjmp lcd_wait_loop
	lcd_clr LCD_RW
	ser r16
	out DDRF, r16
	pop r16
	ret

.equ F_CPU = 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4
; 4 cycles per iteration - setup/call-return overhead

sleep_1ms:
	push r24
	push r25
	ldi r25, high(DELAY_1MS)
	ldi r24, low(DELAY_1MS)
delayloop_1ms:
	sbiw r25:r24, 1
	brne delayloop_1ms
	pop r25
	pop r24
	ret

sleep_5ms:
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	ret