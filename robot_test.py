from pololu_3pi_2040_robot import robot

from pololu_3pi_2040_robot.extras import editions

import time

 

display = robot.Display()

bump_sensors = robot.BumpSensors()

motors = robot.Motors()

buzzer = robot.Buzzer()

yellow_led = robot.YellowLED()

encoders = robot.Encoders()

 

edition = editions.select()

if edition == "Standard":

    max_speed = 1500

    turn_time = 0.25  # Converted to seconds

elif edition == "Turtle":

    max_speed = 3000

    turn_time = 0.5  # Converted to seconds

elif edition == "Hyper":

    max_speed = 1125

    turn_time = 0.15  # Converted to seconds

    motors.flip_left(True)

    motors.flip_right(True)

   

display.fill(0)

display.show()

bump_sensors.calibrate()

 

time.sleep(1000)  # Converted to seconds

while True:
    motors.set_speeds(max_speed, max_speed)
    bump_sensors.read()
     if bump_sensors.left_is_pressed():
     	if bump_sensors.left_is_pressed():
     	   yellow_led.on()
           motors.set_speeds(0, 0)
           buzzer.play("a32")
        display.fill(0)
        display.text("Left", 0, 0)
        display.show()
        
        motors.set_speeds(max_speed // 2, -max_speed // 2)

        time.sleep(turn_time)

 

        motors.set_speeds(0, 0)

        buzzer.play("b32")

        yellow_led.off()

 

        display.fill(0)

        display.show()

 

    if bump_sensors.right_is_pressed():

        single_bump_triggered = True

        yellow_led.on()

        motors.set_speeds(0, 0)

        buzzer.play("e32")

        display.fill(0)

        display.text("Right", 88, 0)

        display.show()

 

        motors.set_speeds(-max_speed // 2, max_speed // 2)

        time.sleep(turn_time)

 

        motors.set_speeds(0, 0)

        buzzer.play("f32")

        yellow_led.off()

 

        display.fill(0)

        display.show()

 

    if not single_bump_triggered and bump_sensors.right_is_pressed() and bump_sensors.left_is_pressed():

        yellow_led.on()

        motors.set_speeds(0, 0)

        buzzer.play("e32")

        display.fill(0)

        display.text("Both", 88, 0)

        display.show()

 

        motors.set_speeds(-max_speed // 2, -max_speed // 2)

        time.sleep(turn_time)

       

        motors.set_speeds(max_speed // 3, -max_speed // 3)

        time.sleep(turn_time)

 

        motors.set_speeds(0, 0)

        buzzer.play("f32")

        yellow_led.off()

        display.fill(0)

        display.show()
