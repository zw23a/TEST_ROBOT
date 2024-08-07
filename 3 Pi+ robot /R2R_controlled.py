from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
from machine import UART, Pin
import time

display = robot.Display()
motors = robot.Motors()
yellow_led = robot.YellowLED()

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


def initialize_uart():
    # Initialize UART with appropriate TX and RX pins
    uart = UART(0, baudrate=9600, tx=Pin(28), rx=Pin(29))  # Adjust TX and RX pins as necessary
    return uart

def read_command(uart):
    if uart.any():
        command = uart.read(100)  # Adjust the number of bytes as needed
        return command
    return None

uart = initialize_uart()
display.fill(0)
display.show()
time.sleep(1)
cmd1 = b'x00' #stright
cmd2 = b'x01' #right
cmd3 = b'x02' #stop
while True:
    command  = read_command(uart)
    if command == cmd1:

        #straight
        yellow_led.on()
        display.fill(0)
        display.text("Straight", 0, 0)
        display.show()
        motors.set_speeds(max_speed,max_speed)
        
        
    if command ==cmd3:

        #stop 
        yellow_led.off()
        motors.set_speeds(0, 0)
    if command ==cmd2:
        #turn
        display.fill(0)
        display.text("Right", 0, 0)
        display.show()
        motors.set_speeds(1000,-1000)
        

    

