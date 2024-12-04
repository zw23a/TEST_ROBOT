from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
from machine import UART, Pin
import time

# Set robot ID (1, 2, or 3)
ROBOT_ID = 1  # Change this for each robot

# Determine the next robot's ID
if ROBOT_ID == 1:
    NEXT_ROBOT_ID = 2
elif ROBOT_ID == 2:
    NEXT_ROBOT_ID = 3
elif ROBOT_ID == 3:
    NEXT_ROBOT_ID = 1

display = robot.Display()
motors = robot.Motors()
yellow_led = robot.YellowLED()

edition = editions.select()

if edition == "Standard":
    max_speed = 1500
elif edition == "Turtle":
    max_speed = 3000
elif edition == "Hyper":
    max_speed = 1125
    motors.flip_left(True)
    motors.flip_right(True)

def initialize_uart():
    uart = UART(0, baudrate=9600, tx=Pin(28), rx=Pin(29))
    return uart

def send_command(uart, target_id, command):
    packet = bytes([target_id, command])
    uart.write(packet)

def read_command(uart, buffer):
    while uart.any():
        byte = uart.read(1)
        if byte:
            buffer.append(byte[0])
    while len(buffer) >= 2:
        target_id = buffer[0]
        cmd = buffer[1]
        del buffer[:2]
        if target_id == ROBOT_ID:
            return cmd
        else:
            continue
    return None

uart = initialize_uart()
buffer = []  # Use list for buffer

display.fill(0)
display.show()
time.sleep(1)

cmd1 = 0x00  # Move forward command
cmd2 = 0x01  # Move backward command

# Robot 1 starts the cycle
if ROBOT_ID == 1:
    time.sleep(1)
    
    # Move forward
    yellow_led.on()
    motors.set_speeds(max_speed, max_speed)
    time.sleep(1)
    motors.set_speeds(0, 0)
    time.sleep(1)
    # Send backward command to next robot
    send_command(uart, NEXT_ROBOT_ID, cmd2)

while True:
    command = read_command(uart, buffer)
    
    if command == cmd1:
        # Move forward
        yellow_led.on()
        motors.set_speeds(max_speed, max_speed)
        time.sleep(1)
        motors.set_speeds(0, 0)
        time.sleep(1)
        # Send backward command to next robot
        send_command(uart, NEXT_ROBOT_ID, cmd2)
        
    elif command == cmd2:
        # Move backward
        yellow_led.off()
        motors.set_speeds(-max_speed, -max_speed)
        time.sleep(1)
        motors.set_speeds(0, 0)
        time.sleep(1)
        # Send forward command to next robot
        send_command(uart, NEXT_ROBOT_ID, cmd1)