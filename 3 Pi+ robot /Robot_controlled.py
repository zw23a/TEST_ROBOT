import time
from machine import Pin, UART
from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions

bump_sensors = robot.BumpSensors()
motors = robot.Motors()
display = robot.Display()
edition = editions.select()

if edition == "Standard":
    max_speed = 1500
    turn_time = 250
elif edition == "Turtle":
    max_speed = 3000
    turn_time = 500
elif edition == "Hyper":
    max_speed = 1125
    turn_time = 150
    motors.flip_left(True)
    motors.flip_right(True)


#  UART，RX  5，TX  4 (adjust by real)
uart = UART(0, baudrate=9600, tx=Pin(4), rx=Pin(5))




def execute_command(command):
    if command == "FORWARD":
        motors.set_speeds(max_speed,max_speed)

    elif command == "BACKWARD":
        motors.set_speeds(-max_speed,max_speed)
    elif command == "LEFT":
        motors.set_speeds(0,max_speed/2)
    elif command == "RIGHT":
        motors.set_speeds(max_speed/2,0)
    elif command == "STOP":
        motors.set_speeds(0,0)
    else:
        print(f"Unknown command: {command}")

while True:
    if uart.any():
        command = uart.read().decode().strip()
        print(f"Received: {command}")
        execute_command(command)
    time.sleep(0.1)



