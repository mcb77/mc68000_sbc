# Makefile for Motorola 68000 bare-metal project
# Builds assembly code, generates binary, and splits for 2764 EPROMs

# Toolchain
AS = ~/src/vasm/vasmm68k_mot
PYTHON = python3
MINIPRO = minipro  # Command-line tool for T48

# Source files
ASM_SRC = startup.s
SPLIT_SCRIPT = ../split_eprom.py
CHECKSUM_SCRIPT = ./patch_checksum.py

# Output files
BIN = startup.bin
HIGH_BIN = high.bin
LOW_BIN = low.bin

# Tool flags
ASFLAGS = -Fbin -m68000
#MINIPROFLAGS = -p "2816" -b  # Blank check before write
MINIPROFLAGS = -p "W27C512@DIP28" -b -s # Blank check before write


# Default target
all: $(HIGH_BIN) $(LOW_BIN)

# Program high-byte EPROM with minipro (T48)
program-high: $(HIGH_BIN)
	@echo "Please insert high-byte EPROM into T48 and press Enter:"
	@read dummy
	$(MINIPRO) $(MINIPROFLAGS) -w $(HIGH_BIN)

# Program low-byte EPROM with minipro (T48)
program-low: $(LOW_BIN)
	@echo "Please insert low-byte EPROM into T48 and press Enter:"
	@read dummy
	$(MINIPRO) $(MINIPROFLAGS) -w $(LOW_BIN)

# Program both EPROMs
program: program-high program-low

# Split binary for EPROMs
$(HIGH_BIN) $(LOW_BIN): $(BIN) $(SPLIT_SCRIPT)
	$(PYTHON) $(CHECKSUM_SCRIPT) $<
	$(PYTHON) $(SPLIT_SCRIPT) $<

# Assemble assembly files
%.bin: %.s
	$(AS) $(ASFLAGS) $< -o $@

# Clean build artifacts
clean:
	rm -f $(BIN) $(HIGH_BIN) $(LOW_BIN)

.PHONY: all clean
