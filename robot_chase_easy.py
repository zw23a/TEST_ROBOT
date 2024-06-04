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

current_speed = max_speed

while True:
    motors.set_speeds(current_speed, current_speed)
    bump_sensors.read()

    left_pressed = bump_sensors.left_is_pressed()
    right_pressed = bump_sensors.right_is_pressed()

    if left_pressed or right_pressed:
        yellow_led.on()
        display.fill(0)
        display.text("Bump", 0, 0)
        display.show()
        buzzer.play("g32")
        
        # Halve the speed
        current_speed = max_speed // 2
        motors.set_speeds(current_speed, current_speed)
        
        # Gradually increase speed back to max
        while current_speed < max_speed:
            time.sleep(1)  # Adjust the sleep time to control the speed increase rate
            current_speed += 50
            if current_speed > max_speed:
                current_speed = max_speed
            motors.set_speeds(current_speed, current_speed)
        
        yellow_led.off()
