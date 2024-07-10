import serial
import time

def initialize_uart(port, baudrate=9600):
    return serial.Serial(port, baudrate, timeout=1)

def read_from_uart(uart):
    bytes_read = uart.read(100)  # Adjust the number of bytes as needed
    return bytes_read

def write_to_uart(uart, data):
    uart.write(data)
    print("Bytes written to UART:", repr(data))

def debug_uart(uart):
    bytes_read = read_from_uart(uart)
    if bytes_read:
        print("Bytes read from UART:", repr(bytes_read))

# Example usage
uart = initialize_uart('COM5')  # Replace with your actual serial port

while True:
    debug_uart(uart)
    data_to_send = b'\x01'  # Example byte data
    write_to_uart(uart, data_to_send)
    time.sleep(1)
