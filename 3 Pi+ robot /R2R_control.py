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

def send_command(uart, command):
    uart.write(command)
    #print("Command sent:", repr(command))
    display.fill(0)
    display.text(repr(command),88,0)
    display.show()

uart = initialize_uart()
display.fill(0)

display.show()
time.sleep(1)
cmd1 = b'x01' #stright
cmd2 = b'x00' #right
cmd3 = b'x02' #stop
while True:
    #straight
    yellow_led.on()
    send_command(uart,cmd1)
    display.fill(0)
    display.text("Straight", 0, 0)
    display.show()
    motors.set_speeds(max_speed,max_speed)
    time.sleep(3)

    #stop and turn
    yellow_led.off()
    send_command(uart,cmd2)
    motors.set_speeds(0, 0)
    display.fill(0)
    display.text("Right", 0, 0)
    display.show()
    motors.set_speeds(1000,-1000)
    time.sleep(2.4)

    
