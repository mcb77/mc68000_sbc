import serial
import sys
import time

def open_serial_port(port, baud=9600):
    """Open the Arduino bridge without pulsing DTR or RTS."""
    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.bytesize = serial.EIGHTBITS
        ser.parity = serial.PARITY_NONE
        ser.stopbits = serial.STOPBITS_ONE
        ser.timeout = 0
        ser.write_timeout = 5
        ser.dtr = False
        ser.rts = False
        ser.open()
        ser.dtr = False
        ser.rts = False
        time.sleep(2.0)
        ser.reset_input_buffer()
        print(f"Opened serial port {port}")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port {port}: {e}")
        sys.exit(1)

def read_binary_file(filename):
    """Read binary file into bytes."""
    try:
        with open(filename, 'rb') as f:
            data = f.read()
        print(f"Read {len(data)} bytes from {filename}")
        return data
    except FileNotFoundError:
        print(f"Error: File {filename} not found")
        sys.exit(1)
    except Exception as e:
        print(f"Error reading file {filename}: {e}")
        sys.exit(1)

def read_line(ser, timeout=1):
    """Read a line until newline or timeout."""
    start_time = time.time()
    line = b""
    while time.time() - start_time < timeout:
        char = ser.read(1)
        if not char:  # Timeout on read
            continue
        line += char
        if char == b'\n':
            try:
                return line.decode('ascii', errors='ignore').strip('\r\n')
            except UnicodeDecodeError:
                return ""
        time.sleep(0.001)  # Small delay to avoid CPU hogging
    return ""  # Timeout, return empty string

def send_command(ser, command, timeout=0.1):
    """Send command with LF terminator and read response line."""
    ser.write((command + '\n').encode('ascii'))
    time.sleep(0.01)  # Wait for SBC response
    response = read_line(ser, timeout)
    return response

def read_until(ser, needle, timeout):
    """Read until needle is present or the timeout expires."""
    buf = b""
    end = time.time() + timeout
    while time.time() < end and needle not in buf:
        waiting = ser.in_waiting
        if waiting:
            buf += ser.read(waiting)
        else:
            time.sleep(0.01)
    return buf

def read_reply(ser, timeout):
    """Read through Ok and the prompt line that follows it.

    The monitor prints eight hex digits, LF, Ok, LF, then "> " and LF.
    Stopping at Ok leaves the prompt line in the UART buffer, and the next
    read_line takes that line instead of the next command's reply.
    """
    response = read_until(ser, b"Ok\n", timeout)
    if b"Ok\n" not in response:
        return response
    tail = response.split(b"Ok\n", 1)[1]
    if b"\n" not in tail:
        response += read_until(ser, b"\n", 1.0)
    return response

def wait_prompt(ser):
    """Require the monitor prompt before a load. A stuck load stays silent."""
    ser.write(b"\n")
    ser.flush()
    buf = read_until(ser, b"\n", 2.0)
    if b">" not in buf:
        print(f"No prompt, got {buf!r}")
        return False
    return True

def upload_binary(ser, filename, start_addr,execute):
    """Upload binary file with load and jump to the start address."""
    data = read_binary_file(filename)
    if not data:
        print("Error: file is empty")
        return False

    # The monitor stack occupies $00F000 and up. load refuses that range.
    if start_addr < 0 or start_addr + len(data) > 0xF000:
        print("Error: image would run into the monitor stack at $00F000")
        return False

    # Each load has to finish and print Ok. A 256-byte write overflows the
    # Uno's serial buffer, the monitor stays in SERIAL_RX, and the next
    # command is eaten as payload. 32 bytes fits that buffer.
    if not wait_prompt(ser):
        return False

    offset = 0
    while offset < len(data):
        chunk = data[offset:offset + 0x20]
        addr = start_addr + offset
        command = f"load {addr:06X},{len(chunk):04X}"
        print(f"Sending: {command} ({len(chunk)} bytes)")
        ser.write((command + "\n").encode("ascii"))
        ser.flush()
        # The header is parsed from the line buffer before SERIAL_RX runs.
        # A gap here is required: the bytes are still moving through the
        # Uno after flush returns.
        time.sleep(0.2)
        early = ser.read(ser.in_waiting or 0)
        if b"Error" in early:
            print(f"Response: {early!r}")
            print("Error from SBC")
            return False
        # One byte at a time. A burst fills the Uno while the monitor
        # replies, and SoftwareSerial then stops delivering bytes.
        for byte in chunk:
            ser.write(bytes([byte]))
            ser.flush()
            time.sleep(0.003)
        # 8 hex digits, LF, Ok, LF, then the prompt line. About 1 ms per payload byte.
        timeout = 5 + len(chunk) / 800
        response = read_reply(ser, timeout)
        text = response.decode("latin1", "replace")
        print(f"Response: {text!r}")
        if b"Error" in response or b"Ok\n" not in response:
            print("Error from SBC")
            return False
        expect = f"{sum(chunk) & 0xFFFFFFFF:08X}"
        if expect.encode("ascii") not in response:
            print(f"Checksum mismatch, expected {expect}")
            return False
        offset += len(chunk)

    # checksum's count is the byte count. Eight digits cover the whole image.
    command = f"checksum {start_addr:08X},{len(data):08X}"
    print(f"Sending: {command}")
    ser.write((command + "\n").encode("ascii"))
    ser.flush()
    response = read_reply(ser, 5)
    text = response.decode("latin1", "replace")
    print(f"Response: {text!r}")
    expect = f"{sum(data) & 0xFFFFFFFF:08X}"
    if b"Error" in response or b"Ok\n" not in response or expect.encode("ascii") not in response:
        print(f"Checksum mismatch, expected {expect}")
        return False

    print("Upload successful")

    if execute == 1:
        # Send jump command
        command = f"jump {start_addr:06X}"
        print(f"Sending: {command}")
        response = send_command(ser, command)
        print(f"Response: {response}")
        if "Error" in response:
            print(f"Error from SBC: {response}")
            return False

    print("Execute successful")
    return True

def main():
    if len(sys.argv) != 5:
        print("Usage: python upload_binary.py <port> <filename> <start_addr> <execute_flag>")
        print("Example: python upload_binary.py /dev/ttyUSB0 program.bin 003000 1")
        sys.exit(1)

    port = sys.argv[1]
    filename = sys.argv[2]
    try:
        start_addr = int(sys.argv[3], 16)
    except ValueError:
        print("Error: Start address must be a hexadecimal number")
        sys.exit(1)

    try:
        execute = int(sys.argv[4])
    except ValueError:
        print("Error: Execute flag must be a 0 or 1")
        sys.exit(1)


    # Open serial port
    ser = open_serial_port(port)

    try:
        # Flush input buffer
        ser.reset_input_buffer()

        # Upload binary
        if upload_binary(ser, filename, start_addr, execute):
            print("Program uploaded and executed")
            # Read any output (e.g., program output)
            time.sleep(1)
            response = read_line(ser, timeout=2)
            while response:
                print("SBC output:", response)
                response = read_line(ser, timeout=2)
        else:
            print("Upload failed")
            sys.exit(1)

    finally:
        ser.close()
        print("Serial port closed")

if __name__ == "__main__":
    main()