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
bump_sensors.calibrate()
time.sleep(1)

while True:
    motors.set_speeds(max_speed, max_speed)
    bump_sensors.read()

    left_pressed = bump_sensors.left_is_pressed()
    right_pressed = bump_sensors.right_is_pressed()

    if left_pressed and right_pressed:
        # Both bumpers are pressed
        yellow_led.on()
        motors.set_speeds(0, 0)
        display.fill(0)
        display.text("Both", 0, 0)
        display.show()
        buzzer.play("g32")
        # Perform specific action for both bumpers pressed
        motors.set_speeds(-max_speed / 2, -max_speed / 2)
        time.sleep(turn_time / 1000)
        motors.set_speeds(max_speed / 2, -max_speed / 2)
        time.sleep(turn_time / 1000)
        motors.set_speeds(0, 0)
        yellow_led.off()
        display.fill(0)
        display.show()
    elif left_pressed:
        # Left bumper is pressed
        yellow_led.on()
        motors.set_speeds(0, 0)
        display.fill(0)
        display.text("Left", 0, 0)
        display.show()
        buzzer.play("c32")
        # Perform specific action for left bumper pressed
        motors.set_speeds(-max_speed/5, -max_speed/5)
        time.sleep(turn_time / 1000)
        motors.set_speeds(max_speed/3, -max_speed/3)
        time.sleep(turn_time / 1000)
        motors.set_speeds(0, 0)
        yellow_led.off()
        display.fill(0)
        display.show()        

    elif right_pressed:
        # Right bumper is pressed
        yellow_led.on()
        motors.set_speeds(0, 0)
        display.fill(0)
        display.text("Right", 88, 0)
        display.show()
        buzzer.play("e32")
        # Perform specific action for right bumper pressed
        motors.set_speeds(-max_speed/5, -max_speed/5)
        time.sleep(turn_time / 1000)
        motors.set_speeds(-max_speed/3, max_speed/3)
        time.sleep(turn_time / 1000)
        motors.set_speeds(0, 0)
        yellow_led.off()
        display.fill(0)
        display.show()   
