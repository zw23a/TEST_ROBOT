# pip install keyboard
import serial
import keyboard

ser = serial.Serial('COM3', 57600) 

def send_command(command):
    ser.write(command.encode())
    print(f"Sent: {command}")

def main():
    print("Control the robot with WASD keys. Press 'q' to quit.")
    while True:
        if keyboard.is_pressed('w'):
            send_command('FORWARD')
        elif keyboard.is_pressed('s'):
            send_command('BACKWARD')
        elif keyboard.is_pressed('a'):
            send_command('LEFT')
        elif keyboard.is_pressed('d'):
            send_command('RIGHT')
        elif keyboard.is_pressed('q'):
            print("Exiting...")
            break
        else:
            send_command('STOP')
        keyboard.wait('any', suppress=False)  # Wait for any key press


