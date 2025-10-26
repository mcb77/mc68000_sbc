    ; File: startup.asm
    ; ~/src/vasm/vasmm68k_mot -Fbin -m68000 startup.s -o startup.bin
    ORG     $3000        ; Start in RAM at 0x3000

; Main program: Toggle LED on bit 0 of IO port at 1 Hz
MAIN:
    LEA     $200000,A0        ; IO port address (74LS374 latch)
    MOVE.W  #0,D1             ; Initial state (LED off)
TOGGLE_LOOP:
    MOVE.W  D1,(A0)           ; Write to latch (bit 0 controls LED)
    JSR     DELAY_05S         ; Delay ~0.5 seconds
    EOR.W   #1,D1             ; Toggle bit 0
;    BRA     TOGGLE_LOOP       ; Continue toggling
    RTS

; Delay ~0.5s at 4.8 MHz (~2,400,000 cycles)
DELAY_05S:
    MOVE.L  #133333,D0        ; Loop counter
DELAY_LOOP:
    SUBQ.L  #1,D0             ; Decrement
    BNE.S   DELAY_LOOP        ; Branch if not zero
    RTS                       ; Return

    END
