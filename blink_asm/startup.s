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
    JSR     MAIN              ; Call main routine
HALT:
    STOP    #$2700            ; Stop CPU
    BRA     HALT              ; Infinite loop

; Exception handlers
DEFAULT_HANDLER:
BUS_ERROR:
ADDRESS_ERROR:
ILLEGAL_INSTR:
    RTE                       ; Return from exception

; Main program: Toggle LED on bit 0 of IO port at 1 Hz
MAIN:
    LEA     $200000,A0        ; IO port address (74LS374 latch)
    MOVE.W  #0,D1             ; Initial state (LED off)
TOGGLE_LOOP:
    MOVE.W  D1,(A0)           ; Write to latch (bit 0 controls LED)
    JSR     DELAY_05S         ; Delay ~0.5 seconds
    EOR.W   #1,D1             ; Toggle bit 0
    BRA     TOGGLE_LOOP       ; Continue toggling

; Delay ~0.5s at 4.8 MHz (~2,400,000 cycles)
DELAY_05S:
    MOVE.L  #133333,D0        ; Loop counter
DELAY_LOOP:
    SUBQ.L  #1,D0             ; Decrement
    BNE.S   DELAY_LOOP        ; Branch if not zero
    RTS                       ; Return

    DC.B    "(C) Matthias Berger 13.9.1996 * ",0

    END
