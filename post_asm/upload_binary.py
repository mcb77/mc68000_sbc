import serial
import sys
import time

def open_serial_port(port, baud=9600, timeout=0.1):
    """Open serial port with specified settings."""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
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

def upload_binary(ser, filename, start_addr,execute):
    """Upload binary file to SBC and jump to start address."""
    data = read_binary_file(filename)

    # Send write commands
    addr = start_addr
    for byte in data:
        command = f"write {addr:06X},{byte:02X}"
        print(f"Sending: {command}")
        response = send_command(ser, command)
        print(f"Response: {response}")
        if "Error" in response:
            print(f"Error from SBC: {response}")
            return False
        addr += 1

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

    finally:
        ser.close()
        print("Serial port closed")

if __name__ == "__main__":
    main()