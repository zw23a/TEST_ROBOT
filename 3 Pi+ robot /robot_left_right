from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
import time

display = robot.Display()
bump_sensors = robot.BumpSensors()
motors = robot.Motors()
buzzer = robot.Buzzer()
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

display.fill(0)
display.show()
time.sleep(1)

while True:
    #straight
    yellow_led.on()
    display.fill(0)
    display.text("Straight", 88, 0)
    display.show()
    motors.set_speeds(max_speed,max_speed)
    time.sleep(turn_time/200)

    #stop and turn
    yellow_led.off()
    motors.set_speeds(0, 0)
    display.fill(0)
    display.text("Right", 88, 0)
    display.show()
    buzzer.play("e32")
    motors.set_speeds(1000,-1000)
    time.sleep(2)

    
