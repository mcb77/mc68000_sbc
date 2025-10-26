#!/usr/bin/env python3
import sys

# Constants
EPROM_SIZE = 16384  # Total size of ROM (bytes, for two 2764 EPROMs)
CHECKSUM_SIZE = 4   # Size of checksum (bytes)

# Read input binary
with open(sys.argv[1], 'rb') as f:
    data = f.read()

# Pad to EPROM_SIZE bytes with 0xFF
pad_length = EPROM_SIZE - len(data)
data += b'\xFF' * pad_length

# Compute sum of first EPROM_SIZE - CHECKSUM_SIZE bytes (modulo 2^32)
sum_bytes = sum(data[i] for i in range(EPROM_SIZE - CHECKSUM_SIZE)) & 0xffffffff

# Store sum as big-endian 32-bit checksum in last 4 bytes
data = data[:EPROM_SIZE - CHECKSUM_SIZE] + bytes([
    (sum_bytes >> 24) & 0xff,  # b0: MSB
    (sum_bytes >> 16) & 0xff,  # b1
    (sum_bytes >> 8) & 0xff,   # b2
    sum_bytes & 0xff           # b3: LSB
])

# Write back to input file
with open(sys.argv[1], 'wb') as f:
    f.write(data)

print(f"Patched 32-bit checksum 0x{sum_bytes:08x} into {sys.argv[1]} at 0x{EPROM_SIZE - CHECKSUM_SIZE:06x}")
