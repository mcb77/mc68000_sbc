#!/usr/bin/env python3
import sys
import struct

# Constants
EPROM_SIZE = 16384  # Total size of ROM (bytes, for two 2764 EPROMs)
CHECKSUM_SIZE = 4   # Size of checksum (bytes)

# Read input binary
with open(sys.argv[1], 'rb') as f:
    data = f.read()

# Ensure input is exactly EPROM_SIZE bytes
if len(data) != EPROM_SIZE:
    print(f"Error: Input file must be exactly {EPROM_SIZE} bytes, got {len(data)}")
    sys.exit(1)

# Sum first EPROM_SIZE - CHECKSUM_SIZE bytes (modulo 2^32)
sum_bytes = sum(data[i] for i in range(EPROM_SIZE - CHECKSUM_SIZE)) & 0xffffffff

# Read stored checksum (last 4 bytes) as big-endian 32-bit value
stored_checksum = struct.unpack('>I', data[EPROM_SIZE - CHECKSUM_SIZE:])[0]

# Verify checksum
if stored_checksum != sum_bytes:
    print(f"Checksum mismatch: stored 0x{stored_checksum:08x}, expected 0x{sum_bytes:08x}")
    sys.exit(1)
print(f"Checksum verified: 0x{stored_checksum:08x} matches expected 0x{sum_bytes:08x}")
