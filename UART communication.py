import time
from machine import Pin, UART

# updateUART，RX = 5，TX = 4 (for example)
uart = UART(0, baudrate=9600, tx=Pin(4), rx=Pin(5))

def send_data(data):
    uart.write(data)
    print(f"Sent: {data}")

def receive_data():
    if uart.any():
        data = uart.read()
        print(f"Received: {data}")
        return data
    return None

while True:
    # export
    send_data("Hello Wixel!")
    
    # import
    received = receive_data()
    
    # waiting time
    time.sleep(1)
