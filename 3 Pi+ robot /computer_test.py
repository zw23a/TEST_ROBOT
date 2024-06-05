# pip install pyserial   (if you dont have this lib)
import serial
import threading
import time

# Configure the serial port
ser = serial.Serial('COM4', 9600)  # Replace 'COM1' with the correct port for your Wixel module
# ser = serial.Serial('/dev/ttyACM0', 9600)

# Function to send data
def send_data():
    while True:
        message = input("Enter message to send: ")
        ser.write(message.encode())
        print(f"Sent: {message}")
        time.sleep(1)

# Function to read data
def read_data():
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode().strip()
            print(f"Received: {data}")

# Create threads for sending and reading data
send_thread = threading.Thread(target=send_data)
read_thread = threading.Thread(target=read_data)

send_thread.start()
read_thread.start()

send_thread.join()
read_thread.join()
