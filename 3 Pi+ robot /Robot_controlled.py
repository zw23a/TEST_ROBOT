import time
from machine import Pin, UART
from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions

bump_sensors = robot.BumpSensors()
motors = robot.Motors()
display = robot.Display()
edition = editions.select()
yellow_led = robot.YellowLED()
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
uart = UART(0, baudrate=9600, tx=Pin(28), rx=Pin(29))
yellow_led.on()
display.fill(0)
display.text("Ready", 80, 0)
display.show()

try:
    while True:
        if uart.any():
            command = uart.read(1).decode()
            if command == 'F':
                motors.set_speeds(max_speed,max_speed)
            elif command == 'B':
                motors.set_speeds(-max_speed,-max_speed)
            elif command == 'L':
                motors.set_speeds(0, max_speed/2)
            elif command == 'R':
                motors.set_speeds(max_speed/2, 0)
            elif command == 'S':
                motors.set_speeds(0, 0)  
except KeyboardInterrupt:
    print("Exiting...")
