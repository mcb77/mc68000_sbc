#!/usr/bin/env python3
import sys
with open(sys.argv[1], 'rb') as f:
    data = f.read()
if len(data) % 2:
    data += b'\x00'
high_bytes = data[0::2][:8192] + b'\xFF' * (8192 - len(data[0::2]))
low_bytes = data[1::2][:8192] + b'\xFF' * (8192 - len(data[1::2]))
with open('high.bin', 'wb') as f:
    f.write(high_bytes)
with open('low.bin', 'wb') as f:
    f.write(low_bytes)
print("Generated high.bin and low.bin for 8kb EPROMs.")
