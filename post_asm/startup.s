    ; File: startup.asm
    ; ~/src/vasm/vasmm68k_mot -Fbin -m68000 startup.s -o startup.bin
    ORG     $100000        ; Start in ROM at 0x100000

; Exception vector table (256 longwords, 1KB)
RESET_VECTORS:
    DC.L    $000FFFFE      ; Vector 0: Initial Stack Pointer (top of RAM)
    DC.L    START          ; Vector 1: Initial Program Counter
    DC.L    BUS_ERROR      ; Vector 2: Bus Error
    DC.L    ADDRESS_ERROR  ; Vector 3: Address Error
    DC.L    ILLEGAL_INSTR  ; Vector 4: Illegal Instruction
    ; Vectors 5-255: Explicitly fill with DEFAULT_HANDLER
    REPT    251
    DC.L    DEFAULT_HANDLER
    ENDR

; Startup code
START:
    MOVE.W  #$2700,SR      ; Supervisor mode, disable interrupts
    MOVE.L  $800000.L,D1   ; Dummy read to disable ROM remap
    LEA     RESET_VECTORS,A0  ; Source: ROM vector table at 0x100000
    LEA     $000000,A1        ; Destination: RAM at 0x000000
    MOVE.W  #255,D0           ; Copy 256 longwords
COPY_LOOP:
    MOVE.L  (A0)+,(A1)+       ; Copy longword from ROM to RAM
    DBRA    D0,COPY_LOOP      ; Decrement and loop
    MOVE.L  #$000FFFFE,A7     ; Set stack pointer
    BRA     POST              ; Call POST routine
HALT:
    STOP    #$2700            ; Stop CPU
    BRA     HALT              ; Infinite loop

; Exception handlers
DEFAULT_HANDLER:
BUS_ERROR:
ADDRESS_ERROR:
ILLEGAL_INSTR:
    MOVE.W #10,D3
    BRA BLINK_NUMBER_FOREVER                   ; Infinite blink: 10 blinks
    RTE                       ; Return from exception

POST:

;    MOVE.B #$55,D1
;    JSR SERIAL_TX

;    MOVE.L #100,D0
;DELAY_CHAR:
;    SUBQ.L #1,D0
;    BNE.S DELAY_CHAR

;    BRA     POST

    JSR NEW_LINE
    ; Send the string
    LEA     MY_STRING,A0     ; Load string address
    JSR     SERIAL_STRING     ; Transmit string

;    JSR     MONITOR

; ================================================================
; POST Check 1: ROM Checksum
; Sum first EPROM_SIZE - CHECKSUM_SIZE bytes, compare with 32-bit checksum at 0x100000+EPROM_SIZE-CHECKSUM_SIZE
; ================================================================
; Constants
EPROM_SIZE    SET 16384
CHECKSUM_SIZE SET 4

    LEA $100000,A0
    MOVE.W #(EPROM_SIZE-CHECKSUM_SIZE-1),D0  ; 16380 bytes
    CLR.L D1                         ; Sum accumulator
    CLR.L D2                         ; Also clear the whole D2 so we can use MOVE.B
ROM_SUM_LOOP:
    MOVE.B (A0)+,D2                  ; Read byte
    ADD.L D2,D1                      ; Add to sum
    DBRA D0,ROM_SUM_LOOP

    JSR HEX_LWORD       ; print sum in D1
    JSR NEW_LINE

    MOVE.L D1,D0        ; move sum to D0
    ; Compare with stored checksum (32-bit, big-endian at 0x103FFC)
    MOVE.L ($100000+EPROM_SIZE-CHECKSUM_SIZE),D1  ; Read checksum

    JSR HEX_LWORD       ; print stored checksum in D1
    JSR NEW_LINE

    CMP.L D0,D1
    BEQ.S ROM_TEST_DONE

ROM_ERROR:
    LEA     POST_ROM_ERROR,A0     ; Load string address
    JSR     SERIAL_STRING     ; Transmit string
    MOVE.W #2,D3
    BRA BLINK_NUMBER_FOREVER                   ; Error 1: ROM checksum fail

ROM_TEST_DONE:
    LEA     POST_ROM_OK,A0     ; Load string address
    JSR     SERIAL_STRING     ; Transmit string


; ================================================================
; RAM TEST
; Tests 256 bytes starting at RAM_TEST_BASE
; ================================================================
RAM_TEST_BASE SET  $000400    ; Start of your .data (safe if we restore)
RAM_TEST_SIZE SET  30720        ; 256 bytes = 64 longwords

    ; --- Save .data area if needed (optional, skip for speed) ---
    ; We'll test in .data, then restore from ROM later if critical

    LEA     POST_RAM_55,A0     ; Load string address
    JSR     SERIAL_STRING     ; Transmit string
    ; --- Test 1: 0x55 / 0xAA Pattern ---
    LEA RAM_TEST_BASE,A0
    MOVE.W #(RAM_TEST_SIZE/4-1),D0
    MOVE.L #$55555555,D1
RAM_FILL_55:
    MOVE.L D1,(A0)+
    DBRA D0,RAM_FILL_55

    LEA.L RAM_TEST_BASE,A0
    MOVE.W #(RAM_TEST_SIZE/4-1),D0
RAM_CHECK_55:
    MOVE.L (A0)+,D2
    CMP.L D1,D2
    BNE RAM_ERROR_5
    DBRA D0,RAM_CHECK_55

    LEA     POST_RAM_AA,A0     ; Load string address
    JSR     SERIAL_STRING     ; Transmit string
; --- Test 2: 0xAA ---
    LEA.L RAM_TEST_BASE,A0
    MOVE.W #(RAM_TEST_SIZE/4-1),D0
    MOVE.L #$AAAAAAAA,D1
RAM_FILL_AA:
    MOVE.L D1,(A0)+
    DBRA D0,RAM_FILL_AA

    LEA.L RAM_TEST_BASE,A0
    MOVE.W #(RAM_TEST_SIZE/4-1),D0
RAM_CHECK_AA:
    MOVE.L (A0)+,D2
    CMP.L D1,D2
    BNE RAM_ERROR_5
    DBRA D0,RAM_CHECK_AA

    LEA     POST_RAM_WALKING_1S,A0     ; Load string address
    JSR     SERIAL_STRING     ; Transmit string
; --- Test 3: Walking 1s (detect address faults) ---
    MOVE.W #31,D3                        ; Test bits 0..31
WALKING_LOOP:
    MOVE.W D3,D4
    MOVE.L #1,D1
    LSL.L D4,D1                         ; D1 = 1 << bit
    MOVE.L D1,D2

    ; --- Fill phase ---
    LEA.L RAM_TEST_BASE,A0              ; Reset pointer each time
    MOVE.W #(RAM_TEST_SIZE/4-1),D0
WALKING_FILL:
    MOVE.L D2,(A0)+
    DBRA D0,WALKING_FILL

    ; --- Verify phase ---
    LEA.L RAM_TEST_BASE,A0              ; Reset pointer again
    MOVE.W #(RAM_TEST_SIZE/4-1),D0
WALKING_CHECK:
    MOVE.L (A0)+,D2
    CMP.L D1,D2
    BNE RAM_ERROR_5
    DBRA D0,WALKING_CHECK

    SUBQ.W #1,D3
    BPL WALKING_LOOP

    LEA     POST_RAM_ADDRESS_IN_DATA,A0     ; Load string address
    JSR     SERIAL_STRING     ; Transmit string
; --- Test 4: Address-in-Data (unique per location) ---
    LEA.L RAM_TEST_BASE,A0
    MOVE.W #(RAM_TEST_SIZE/4-1),D0
ADDR_FILL:
    MOVE.L A0,D1
    EOR.L #RAM_TEST_BASE,D1         ; D1 = offset from base
    MOVE.L D1,(A0)+
    DBRA D0,ADDR_FILL

    LEA.L RAM_TEST_BASE,A0
    MOVE.W #(RAM_TEST_SIZE/4-1),D0
ADDR_CHECK:
    MOVE.L A0,D1
    EOR.L #RAM_TEST_BASE,D1
    MOVE.L (A0)+,D2
    CMP.L D1,D2
    BNE RAM_ERROR_5
    DBRA D0,ADDR_CHECK

; --- RAM TEST PASS ---
    BRA.S RAM_TEST_DONE

RAM_ERROR_5:
    LEA     POST_RAM_ERROR,A0     ; Load string address
    JSR     SERIAL_STRING     ; Transmit string
    MOVE.W #3,D3
    BRA BLINK_NUMBER_FOREVER                   ; Infinite blink: 3 blinks

RAM_TEST_DONE:
    LEA     POST_RAM_OK,A0     ; Load string address
    JSR     SERIAL_STRING     ; Transmit string
;    BRA BLINK_FOREVER

    BRA     MONITOR_START

; ========================================
; FULLY STACK-LESS BLINK (NO JSR/RTS AT ALL)
; D3 = number of flashes per sequence
; ========================================
BLINK_NUMBER_FOREVER:
    MOVE.W  D3,D2                ; D2 = flash counter

BLINK_SEQUENCE:
    ; === Blink D2 times ===
BLINK_LOOP:
    ; LED ON
    MOVE.W  #1,$200000           ; Turn LED on
    ; --- Inline 0.2s delay ---
    MOVE.L  #53332,D0
ON_DELAY_LOOP:
    SUBQ.L  #1,D0
    BNE.S   ON_DELAY_LOOP

    ; LED OFF
    MOVE.W  #0,$200000           ; Turn LED off
    ; --- Inline 0.2s delay ---
    MOVE.L  #53332,D0
OFF_DELAY_LOOP:
    SUBQ.L  #1,D0
    BNE.S   OFF_DELAY_LOOP

    SUBQ.W  #1,D2
    BNE.S   BLINK_LOOP            ; Repeat blink

    ; === 1 second delay between sequences ===
    MOVE.L  #266664,D0
SEQ_DELAY1:
    SUBQ.L  #1,D0
    BNE.S   SEQ_DELAY1

    BRA     BLINK_NUMBER_FOREVER        ; Repeat forever


; ========================================
; BLINK LED FOREVER - FULLY STACK-LESS
; Toggles bit 0 on $200000 every 0.5s
; No JSR, no RTS, no stack
; ========================================
BLINK_FOREVER:
    MOVE.W  #0,D1                ; D1 = initial state (0 = off)

TOGGLE_LOOP:
    ; --- Write current state to LED ---
    MOVE.W  D1,$200000           ; Output D1 to latch (bit 0 = LED)

    ; --- Inline 0.5s delay ---
    MOVE.L  #133333,D0
DELAY_LOOP:
    SUBQ.L  #1,D0
    BNE.S   DELAY_LOOP

    ; --- Toggle bit 0 ---
    EOR.W   #1,D1                ; Flip LED state

    BRA     TOGGLE_LOOP

; ========================================
; SERIAL INPUT (9600 baud, 8N1)
; Output: D1.B = received byte
; Input: $200000 bit 0 (RX pin, adjust as needed)
; Uses: Stack for subroutines, 4.8 MHz clock
; ========================================
SERIAL_RX:
    MOVEM.L D0/D2-D4,-(SP)    ; Save registers
    ; --- Wait for start bit (0 = low) ---
SERIAL_RX_WAIT_START:
    MOVE.W ($200000),D4       ; Read RX pin
;    MOVE.W D4,($200000)       ; Echo to TX pin for debugging. Replace by 4 NOPs if disabled
    NOP
    NOP
    NOP
    NOP
    AND.W #1,D4
    BNE.S SERIAL_RX_WAIT_START          ; Loop until low

    ; --- 1.5 bit delay to sample mid-bit ---
    MOVE.L #35,D0             ; ~750 cycles (1.5 bit)
SERIAL_RX_START_DELAY:
    SUBQ.L #1,D0
    BNE.S SERIAL_RX_START_DELAY

    ; --- Read 8 data bits ---
    MOVEQ #8,D3               ; Bit counter
    MOVEQ #0,D1               ; Clear result
SERIAL_RX_LOOP:
    MOVE.W ($200000),D4       ; Read RX pin
;    MOVE.W D4,($200000)       ; Echo to TX pin for debugging. Replace by 4 NOPs if disabled
    NOP
    NOP
    NOP
    NOP
    AND.W #1,D4
    LSL.B #7,D4
    LSR.B #1,D1               ; Shift result right
    OR.B D4,D1                ; Add new bit (LSB to MSB)
    MOVE.L #23,D0             ; Full bit delay (~500 cycles)
SERIAL_RX_BIT_DELAY:
    SUBQ.L #1,D0
    BNE.S SERIAL_RX_BIT_DELAY
    SUBQ.B #1,D3
    BNE.S SERIAL_RX_LOOP

    ; --- Check stop bit (1 = high) ---
    MOVE.W ($200000),D4
;    MOVE.W D4,($200000)       ; Echo to TX pin for debugging. Replace by 4 NOPs if disabled
    NOP
    NOP
    NOP
    NOP
    AND.W #1,D4
    CMP.W #1,D4               ; Verify stop bit
    BNE.S SERIAL_RX_WAIT_START          ; Invalid, restart
    MOVEM.L (SP)+,D0/D2-D4    ; Restore registers
    RTS

; ========================================
; RECEIVE STRING (9600 baud, 8N1)
; Input: A0 = buffer address, D0.L = max buffer size
; Output: Buffer filled with null-terminated string
; Uses: $200000 bit 0 (RX), calls SERIAL_RX
; ========================================
SERIAL_RX_STRING:
    MOVEM.L D0-D1/A0,-(SP)    ; Save registers
    TST.L D0                  ; Check buffer size
    BEQ.S SRS_DONE            ; Exit if zero
SRS_LOOP:
    JSR SERIAL_RX             ; Get byte in D1
    CMP.B #-1,D1              ; Check for timeout (if implemented)
    BEQ.S SRS_LOOP            ; Retry on timeout
    CMP.B #$0A,D1             ; Check for newline
    BEQ.S SRS_TERMINATE
    CMP.B #$00,D1             ; Check for null
    BEQ.S SRS_TERMINATE
    MOVE.B D1,(A0)+           ; Store byte, increment
    SUBQ.L #1,D0              ; Decrement buffer size
    BNE.S SRS_LOOP            ; Continue if space remains
SRS_TERMINATE:
    CLR.B (A0)                ; Null-terminate
SRS_DONE:
    MOVEM.L (SP)+,D0-D1/A0
    RTS

; ========================================
; SERIAL OUTPUT (9600 baud, 8N1)
; Input: D1.B = byte to send
; Uses: $200000 bit 0
; ========================================
SERIAL_TX:
    MOVEM.L D0/D2-D4,-(SP)       ; Save registers
    MOVE.L D1,D2
    MOVEQ #8,D3
    MOVE.W #0,$200000         ; Start bit
    MOVE.L #26,D0
SERIAL_TX_START_DELAY:
    SUBQ.L #1,D0
    BNE.S SERIAL_TX_START_DELAY
SERIAL_TX_LOOP:
    MOVE.W D2,D4
    AND.W #1,D4
    MOVE.W D4,$200000
    LSR.B #1,D2
    MOVE.L #26,D0
SERIAL_TX_BIT_DELAY:
    SUBQ.L #1,D0
    BNE.S SERIAL_TX_BIT_DELAY
    SUBQ.B #1,D3
    BNE.S SERIAL_TX_LOOP
    MOVE.W #1,$200000         ; Stop bit
    MOVE.L #26,D0
SERIAL_TX_STOP_DELAY:
    SUBQ.L #1,D0
    BNE.S SERIAL_TX_STOP_DELAY
    MOVEM.L (SP)+,D0/D2-D4       ; Restore registers
    RTS

; ========================================
; STRING OUTPUT (9600 baud, 8N1)
; Input: A0 = address of null-terminated string
; Uses: $200000 bit 0 (via SERIAL_TX)
; ========================================
SERIAL_STRING:
    MOVEM.L D0-D1/A0,-(SP)    ; Save registers
STRING_LOOP:
    MOVE.B (A0)+,D1           ; Load byte, increment A0
    BEQ.S STRING_DONE         ; Null terminator, exit
    JSR SERIAL_TX             ; Send byte
    BRA STRING_LOOP
STRING_DONE:
    MOVEM.L (SP)+,D0-D1/A0
    RTS

; ========================================
; PRINT ASCII or '.' for NON-PRINTABLE
; Input: D1.B = byte
; ========================================
PRINT_CHAR:
    CMP.B #32,D1
    BLT.S PC_DOT
    CMP.B #127,D1
    BGT.S PC_DOT
    BRA.S PC_SEND
PC_DOT:
    MOVE.B #'.',D1
PC_SEND:
    JSR SERIAL_TX
    RTS

; ========================================
; CONVERT BYTE to HEX (2 chars)
; Input: D1.B = byte
; ========================================
HEX_BYTE:
    MOVEM.L D2,-(SP)
    MOVE.B D1,D2
    LSR.B #4,D1
    JSR HEX_NIBBLE
    MOVE.B D2,D1
    AND.B #$0F,D1
    JSR HEX_NIBBLE
    MOVEM.L (SP)+,D2
    RTS

; ========================================
; CONVERT NIBBLE to HEX CHAR
; Input: D1.B = 4-bit value (0-F)
; ========================================
HEX_NIBBLE:
    CMP.B #10,D1
    BLT.S HN_NUM
    ADD.B #'A'-10,D1
    BRA.S HN_SEND
HN_NUM:
    ADD.B #'0',D1
HN_SEND:
    JSR SERIAL_TX
    RTS

; =========================================
; CONVERT LWORD to HEX (8 chars)
; Input: D1.L
; =========================================
HEX_LWORD:
    MOVEM.L D1,-(SP) ; Save registers
    ROL.L #8,D1
    JSR HEX_BYTE
    ROL.L #8,D1
    JSR HEX_BYTE
    ROL.L #8,D1
    JSR HEX_BYTE
    ROL.L #8,D1
    JSR HEX_BYTE
    MOVEM.L (SP)+,D1
    RTS

;============================================
; OUTPUT CR LF
;============================================
NEW_LINE:
    MOVEM.L D1,-(SP) ; Save registers
;    MOVE.B #13,D1             ; CR
;    JSR SERIAL_TX
    MOVE.B #10,D1             ; LF
    JSR SERIAL_TX
    MOVEM.L (SP)+,D1
    RTS

; ========================================
; HEX DUMP and STRING OUTPUT to SERIAL (9600 baud, 8N1)
; HEX_DUMP: Input: A0 = start address, D0.L = byte count
; Uses: $200000 bit 0 for serial output
; Stack: Used for subroutines
; 4.8 MHz CPU clock
; Hex Dump Format: "00001000: 48 65 6C 6C 6F 00 ...  Hello..  "
; vasm: No spaces after commas, m68000 mode
; ========================================

HEX_DUMP:
    MOVEM.L D0-D4/A0-A1,-(SP) ; Save registers
    TST.L D0
    BEQ HD_DONE

HD_LINE:
    ; --- Print address (8 hex digits) ---
    MOVE.L A0,D1
    ROL.L #8,D1
    JSR HEX_BYTE
    ROL.L #8,D1
    JSR HEX_BYTE
    ROL.L #8,D1
    JSR HEX_BYTE
    ROL.L #8,D1
    JSR HEX_BYTE

    ; --- Print ": " ---
    MOVE.B #':',D1
    JSR SERIAL_TX
    MOVE.B #' ',D1
    JSR SERIAL_TX

    ; --- Print 16 hex bytes ---
    MOVEQ #16,D3
    MOVE.L A0,A1
    MOVE.L D0,D4
HD_HEX_LOOP:
    CMP.L #0,D0               ; Check bytes remaining
    BEQ.S HD_PAD              ; Pad if no bytes left
    MOVE.B (A1)+,D1           ; Read byte, increment
    JSR HEX_BYTE              ; Convert to hex
    MOVE.B #' ',D1
    JSR SERIAL_TX             ; Space
    SUBQ.L #1,D0
    SUBQ.B #1,D3
    BNE.S HD_HEX_LOOP

    ; --- Pad with spaces if <16 bytes ---
HD_PAD:
    TST.B D3
    BEQ.S HD_ASCII
    MOVE.B #' ',D1
    JSR SERIAL_TX
    JSR SERIAL_TX
    MOVE.B #' ',D1
    JSR SERIAL_TX
    SUBQ.B #1,D3
    BNE.S HD_PAD

    ; --- Print ASCII chars ---
HD_ASCII:
    MOVE.B #' ',D1
    JSR SERIAL_TX
    JSR SERIAL_TX
    MOVEQ #16,D3
    MOVE.L A0,A1
    MOVE.L D4,D0
HD_ASCII_LOOP:
    CMP.L #0,D0
    BEQ.S HD_ASCII_END
    MOVE.B (A1)+,D1
    JSR PRINT_CHAR
    SUBQ.L #1,D0
    SUBQ.B #1,D3
    BNE.S HD_ASCII_LOOP

HD_ASCII_END:
    TST.B D3
    BEQ.S HD_LINE_END
    MOVE.B #'.',D1
    JSR SERIAL_TX
    SUBQ.B #1,D3
    BNE.S HD_ASCII_END

HD_LINE_END:
;    MOVE.B #13,D1             ; CR
;    JSR SERIAL_TX
    MOVE.B #10,D1             ; LF
    JSR SERIAL_TX

    ADD.L #16,A0              ; Next 16 bytes
    TST.L D0
    BNE HD_LINE

HD_DONE:
    MOVEM.L (SP)+,D0-D4/A0-A1
    RTS


;=============================
; WAIT FOR ONE SECOND
;=============================
DELAY1S:
    MOVEM.L D0,-(SP) ; Save registers
    MOVE.L  #266664,D0
DELAY1S_LOOP:
    SUBQ.L  #1,D0
    BNE.S   DELAY1S_LOOP
    MOVEM.L (SP)+,D0
    RTS


; ========================================
; CHECKSUM
; Input: A0 = start address, D0.L = byte count
; ========================================
CHECKSUM:
    MOVEM.L D0-D2/A0,-(SP) ; Save registers

    CLR.L D1                         ; Sum accumulator
    CLR.L D2                         ; Also clear the whole D2 so we can use MOVE.B

    TST.L D0
    BEQ CHECKSUM_DONE

CHECKSUM_LOOP:
    MOVE.B (A0)+,D2                  ; Read byte
    ADD.L D2,D1                      ; Add to sum
    DBRA D0,CHECKSUM_LOOP

CHECKSUM_DONE:

    JSR HEX_LWORD       ; print sum in D1
    JSR NEW_LINE

    MOVEM.L (SP)+,D0-D2/A0
    RTS

;===============================
; MONITOR
;===============================

; ========================================
; MONITOR, HEX DUMP, STRING OUTPUT/INPUT (9600 baud, 8N1)
; Commands: dump <addr> <count>, write <addr> <byte>, read <addr>, jump <addr>
; Input (HEX_DUMP): A0 = start address, D0.L = byte count
; Input (SERIAL_STRING): A0 = null-terminated string address
; Input (SERIAL_RECV_STRING): A0 = buffer address, D0.L = max buffer size
; Output (SERIAL_RX): D1.B = received byte
; Uses: $200000 bit 0 (TX and RX)
; Stack: Used for subroutines
; 4.8 MHz CPU clock
; Hex Dump Format: "001006C0: 28 43 29 20 ...  (C) Matthias..."
; vasm: No spaces after commas, m68000 mode
; ========================================

BUFFER    SET $2000

MONITOR_START:
    LEA WELCOME,A0
    JSR SERIAL_STRING

; --- Monitor Loop ---
MONITOR:
    LEA PROMPT,A0
    JSR SERIAL_STRING        ; Print "> "
    LEA BUFFER,A0
    MOVE.L #32,D0            ; 32-byte buffer
    JSR SERIAL_RX_STRING   ; Get command
    LEA BUFFER,A0
;    JSR SERIAL_STRING
;    JSR NEW_LINE
    CMP.B #0,(A0)
    BEQ.W MONITOR           ; Empty input, go back to prompt
    CMP.B #10,(A0)
    BEQ.W MONITOR           ; Only newline in input, go back to prompt
    ; Check command
    MOVE.L A0,A1
    CMP.B #'d',(A1)+
    BNE.S NOT_DUMP
    CMP.B #'u',(A1)+
    BNE.S NOT_DUMP
    CMP.B #'m',(A1)+
    BNE.S NOT_DUMP
    CMP.B #'p',(A1)+
    BNE.S NOT_DUMP
    CMP.B #' ',(A1)+
    BNE.S NOT_DUMP
    JSR PARSE_DUMP
    BRA MONITOR
NOT_DUMP:
    MOVE.L A0,A1
    CMP.B #'w',(A1)+
    BNE.S NOT_WRITE
    CMP.B #'r',(A1)+
    BNE.S NOT_WRITE
    CMP.B #'i',(A1)+
    BNE.S NOT_WRITE
    CMP.B #'t',(A1)+
    BNE.S NOT_WRITE
    CMP.B #'e',(A1)+
    BNE.S NOT_WRITE
    CMP.B #' ',(A1)+
    BNE.S NOT_WRITE
    JSR PARSE_WRITE
    BRA MONITOR
NOT_WRITE:
    MOVE.L A0,A1
    CMP.B #'r',(A1)+
    BNE.S NOT_READ
    CMP.B #'e',(A1)+
    BNE.S NOT_READ
    CMP.B #'a',(A1)+
    BNE.S NOT_READ
    CMP.B #'d',(A1)+
    BNE.S NOT_READ
    CMP.B #' ',(A1)+
    BNE.S NOT_READ
    JSR PARSE_READ
    BRA MONITOR
NOT_READ:
    MOVE.L A0,A1
    CMP.B #'j',(A1)+
    BNE.S NOT_JUMP
    CMP.B #'u',(A1)+
    BNE.S NOT_JUMP
    CMP.B #'m',(A1)+
    BNE.S NOT_JUMP
    CMP.B #'p',(A1)+
    BNE.S NOT_JUMP
    CMP.B #' ',(A1)+
    BNE.S NOT_JUMP
    JSR PARSE_JUMP
    BRA MONITOR
NOT_JUMP:
    MOVE.L A0,A1
    CMP.B #'c',(A1)+
    BNE.S NOT_CHECKSUM
    CMP.B #'h',(A1)+
    BNE.S NOT_CHECKSUM
    CMP.B #'e',(A1)+
    BNE.S NOT_CHECKSUM
    CMP.B #'c',(A1)+
    BNE.S NOT_CHECKSUM
    CMP.B #'k',(A1)+
    BNE.S NOT_CHECKSUM
    CMP.B #'s',(A1)+
    BNE.S NOT_CHECKSUM
    CMP.B #'u',(A1)+
    BNE.S NOT_CHECKSUM
    CMP.B #'m',(A1)+
    BNE.S NOT_CHECKSUM
    CMP.B #' ',(A1)+
    BNE.S NOT_CHECKSUM
    JSR PARSE_CHECKSUM
    BRA MONITOR
NOT_CHECKSUM:
    MOVE.L A0,A1
    CMP.B #'h',(A1)+
    BNE.S NOT_HELP
    CMP.B #'e',(A1)+
    BNE.S NOT_HELP
    CMP.B #'l',(A1)+
    BNE.S NOT_HELP
    CMP.B #'p',(A1)+
    BNE.S NOT_HELP
    CMP.B #00,(A1)+
    BNE.S NOT_HELP
    JSR PRINT_HELP
    BRA MONITOR
NOT_HELP:
    LEA ERROR,A0
    JSR SERIAL_STRING        ; Print "Error"
    BRA MONITOR
;EMPTY_INPUT:
;    JSR NEW_LINE
;    BRA MONITOR

; --- Parse "dump <addr> <count>" ---
PARSE_DUMP:
    MOVEM.L D0-D1/A0-A1,-(SP)
    JSR PARSE_ADDRN           ; Get address in A0
    CMP.B #',',D1
    BNE.S PD_ERROR
    JSR PARSE_COUNTN          ; Get count in D0
    CMP.B #$00,D1
    BNE.S PD_ERROR
    JSR HEX_DUMP
    BRA.S PD_DONE
PD_ERROR:
    LEA ERROR,A0
    JSR SERIAL_STRING
PD_DONE:
    MOVEM.L (SP)+,D0-D1/A0-A1
    RTS

; --- Parse "write <addr> <byte>" ---
PARSE_WRITE:
    MOVEM.L D0-D1/A0-A1,-(SP)
    JSR PARSE_ADDRN           ; Get address in A0
    CMP.B #',',D1
    BNE.S PW_ERROR
    JSR PARSE_BYTE           ; Get byte in D0
    CMP.B #$00,D1
    BNE.S PW_ERROR
    MOVE.B D0,(A0)           ; Write byte
    BRA.S PW_DONE
PW_ERROR:
    LEA ERROR,A0
    JSR SERIAL_STRING
    BRA.S PW_END
PW_DONE:
    LEA OK,A0
    JSR SERIAL_STRING
PW_END:
    MOVEM.L (SP)+,D0-D1/A0-A1
    RTS

; --- Parse "read <addr>" ---
PARSE_READ:
    MOVEM.L D0-D1/A0-A1,-(SP)
    JSR PARSE_ADDRN           ; Get address in A0
    CMP.B #$00,D1
    BNE.S PR_ERROR
    MOVE.B (A0),D1           ; Read byte
    JSR HEX_BYTE
;    MOVE.B #13,D1
;    JSR SERIAL_TX
    MOVE.B #10,D1
    JSR SERIAL_TX
    BRA.S PR_DONE
PR_ERROR:
    LEA ERROR,A0
    JSR SERIAL_STRING
PR_DONE:
    MOVEM.L (SP)+,D0-D1/A0-A1
    RTS

; --- Parse "jump <addr>" ---
PARSE_JUMP:
    MOVEM.L D0-D1/A0,-(SP)
    JSR PARSE_ADDRN           ; Get address in A0
    CMP.B #$0,D1
    BNE.S PJ_ERROR
;    MOVEM.L (SP)+,D0-D1/A0
    MOVE.L A0,A1
    LEA OK,A0
    JSR SERIAL_STRING
    JSR (A1)                 ; Jump to address

    MOVE.W #1,($200000)     ; prepare TX
    JSR DELAY1S
    JSR NEW_LINE
    LEA OK,A0
    JSR SERIAL_STRING
    BRA.S PJ_END
PJ_ERROR:
    LEA ERROR,A0
    JSR SERIAL_STRING
PJ_END:
    MOVEM.L (SP)+,D0-D1/A0
    RTS

; --- Parse "checksum <addr> <count>" ---
PARSE_CHECKSUM:
    MOVEM.L D0-D1/A0-A1,-(SP)
    JSR PARSE_ADDRN           ; Get address in A0
    CMP.B #',',D1
    BNE.S PC_ERROR
    JSR PARSE_COUNTN          ; Get count in D0
    CMP.B #$00,D1
    BNE.S PC_ERROR
    JSR CHECKSUM
    BRA.S PC_DONE
PC_ERROR:
    LEA ERROR,A0
    JSR SERIAL_STRING
PC_DONE:
    MOVEM.L (SP)+,D0-D1/A0-A1
    RTS

; --- Print "help" ---
PRINT_HELP:
    MOVEM.L D1/A0,-(SP)
    LEA HELP,A0
    JSR SERIAL_STRING

    LEA HELP_MONITOR_START,A0
    JSR SERIAL_STRING

    MOVE.L #MONITOR_START,D1
    JSR HEX_LWORD
    JSR NEW_LINE

    LEA HELP_STACK_POINTER,A0
    JSR SERIAL_STRING

    MOVE.L SP,D1
    JSR HEX_LWORD
    JSR NEW_LINE

    MOVEM.L (SP)+,D1/A0
    RTS

; --- Parse 6-digit hex address at A1 into A0 ---
PARSE_ADDR6:
    MOVEQ #0,D0
    MOVEQ #6,D2
PA6_LOOP:
    MOVE.B (A1)+,D1
    JSR HEX_TO_NIBBLE
    CMP.B #-1,D1
    BEQ.S PA6_INVALID
    LSL.L #4,D0
    OR.B D1,D0
    SUBQ.B #1,D2
    BNE.S PA6_LOOP
    MOVE.L D0,A0
    MOVE.B (A1)+,D1          ; Next char (comma or LF)
    RTS
PA6_INVALID:
    MOVE.L #0,A0
    MOVE.B (A1)+,D1
    RTS

; --- Parse 1-8 digit hex address at A1 into A0, return next char in D1 ---
PARSE_ADDRN:
    MOVEQ #0,D0               ; Clear result
    MOVEQ #8,D2               ; Max 8 digits
    MOVEQ #0,D3               ; Digit counter
PAN_LOOP:
    MOVE.B (A1),D1            ; Get char (don’t increment yet)
    JSR HEX_TO_NIBBLE         ; Convert to nibble
    CMP.B #-1,D1              ; Check invalid
    BEQ.S PAN_END
    LSL.L #4,D0               ; Shift result left
    OR.B D1,D0                ; Add nibble
    ADDQ.B #1,D3              ; Increment digit count
    ADDQ.L #1,A1              ; Move to next char
    SUBQ.B #1,D2              ; Decrement max digits
    BNE.S PAN_LOOP             ; Continue if < 8 digits
PAN_END:
    TST.B D3                  ; Check if any digits
    BEQ.S PAN_INVALID
    MOVE.L D0,A0              ; Set address
    MOVE.B (A1)+,D1           ; Get next char
    RTS
PAN_INVALID:
    MOVE.L #0,A0                  ; No valid digits
    MOVE.B (A1)+,D1           ; Get next char
    RTS

; --- Parse 2-digit hex count at A1 into D0, next char in D1 ---
PARSE_COUNT2:
    MOVEQ #0,D0
    MOVEQ #2,D2
PC2_LOOP:
    MOVE.B (A1)+,D1
    JSR HEX_TO_NIBBLE
    CMP.B #-1,D1
    BEQ.S PC2_INVALID
    LSL.W #4,D0
    OR.B D1,D0
    SUBQ.B #1,D2
    BNE.S PC2_LOOP
    MOVE.B (A1)+,D1          ; Next char (LF)
    RTS
PC2_INVALID:
    MOVEQ #0,D0
    MOVE.B (A1)+,D1
    RTS

; --- Parse 1-4 digit hex count at A1 into D0, return next char in D1 ---
PARSE_COUNTN:
    MOVEQ #0,D0               ; Clear result
    MOVEQ #4,D2               ; Max 4 digits
    MOVEQ #0,D3               ; Digit counter
PCN_LOOP:
    MOVE.B (A1),D1            ; Get char (don’t increment yet)
    JSR HEX_TO_NIBBLE         ; Convert to nibble
    CMP.B #-1,D1              ; Check invalid
    BEQ.S PCN_END
    LSL.L #4,D0               ; Shift result left
    OR.B D1,D0                ; Add nibble
    ADDQ.B #1,D3              ; Increment digit count
    ADDQ.L #1,A1              ; Move to next char
    SUBQ.B #1,D2              ; Decrement max digits
    BNE.S PCN_LOOP             ; Continue if < 8 digits
PCN_END:
    TST.B D3                  ; Check if any digits
    BEQ.S PCN_INVALID
;    MOVE.L D0,A0              ; Set address
    MOVE.B (A1)+,D1           ; Get next char
    RTS
PCN_INVALID:
    MOVE.L #0,D0              ; No valid digits
    MOVE.B (A1)+,D1           ; Get next char
    RTS


; --- Parse 2-digit hex byte at A1 into D0, next char in D1 ---
PARSE_BYTE:
    MOVEQ #0,D0
    MOVEQ #2,D2
PB_LOOP:
    MOVE.B (A1)+,D1
    JSR HEX_TO_NIBBLE
    CMP.B #-1,D1
    BEQ.S PB_INVALID
    LSL.B #4,D0
    OR.B D1,D0
    SUBQ.B #1,D2
    BNE.S PB_LOOP
    MOVE.B (A1)+,D1          ; Next char (LF)
    RTS
PB_INVALID:
    MOVEQ #0,D0
    MOVE.B (A1)+,D1
    RTS

; --- Convert ASCII hex in D1 to nibble (0-F, case insensitiv) ---
HEX_TO_NIBBLE:
    CMP.B #'0',D1
    BLT.S HTN_INVALID
    CMP.B #'9',D1
    BLE.S HTN_NUM
    CMP.B #'A',D1
    BLT.S HTN_INVALID
    CMP.B #'F',D1
    BLE.S HTN_UPPER
    CMP.B #'a',D1
    BLT.S HTN_INVALID
    CMP.B #'f',D1
    BLE.S HTN_LOWER
HTN_INVALID:
    MOVEQ #-1,D1
    RTS
HTN_NUM:
    SUB.B #'0',D1
    RTS
HTN_UPPER:
    SUB.B #'A'-10,D1
    RTS
HTN_LOWER:
    SUB.B #'a'-10,D1
    RTS

POST_RAM_55:
    DC.B    "RAM TEST 55", 10, 0
POST_RAM_AA:
    DC.B    "RAM TEST AA", 10, 0
POST_RAM_WALKING_1S:
    DC.B    "RAM TEST WALKING 1S", 10, 0
POST_RAM_ADDRESS_IN_DATA:
    DC.B    "RAM TEST ADDRESS IN DATA", 10, 0

POST_RAM_ERROR:
    DC.B    "RAM ERROR", 10, 0
POST_RAM_OK:
    DC.B    "RAM OK", 10, 0
POST_ROM_ERROR:
    DC.B    "ROM ERROR", 10, 0
POST_ROM_OK:
    DC.B    "ROM OK", 10, 0

WELCOME:
    DC.B "Simple 68000 Monitor. Type 'help' for help.",10,0
PROMPT:
    DC.B "> ",0
ERROR:
    DC.B "Error",10,0
OK:
    DC.B "Ok",10,0
HELP:
    DC.B "Help:",10
    DC.B " - checksum <addr>,<count>",10
    DC.B " - dump <addr>,<count>",10
    DC.B " - jump <addr>",10
    DC.B " - read <addr>",10
    DC.B " - write <addr>,<byte>",10
    DC.B 0
HELP_MONITOR_START:
    DC.B "MONITOR START: ",0
HELP_STACK_POINTER:
    DC.B "STACK POINTER: ",0

MY_STRING2:
    DC.B    "(C) Matthias Berger 13.9.1996 - 2025 * ",0

MY_STRING:
    DC.B    "HELLO WORLD!", 10, 0        ; Null-terminated string

    END
