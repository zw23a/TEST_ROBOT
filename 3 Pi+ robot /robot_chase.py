from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
import time
import _thread

display = robot.Display()
motors = robot.Motors()
line_sensors = robot.LineSensors()
bump_sensors = robot.BumpSensors()

button_a = robot.ButtonA()

edition = editions.select()
if edition == "Standard":
    max_speed = 6000
    calibration_speed = 1000
    calibration_count = 100
elif edition == "Turtle":
    max_speed = 6000
    calibration_speed = 3000
    calibration_count = 100
elif edition == "Hyper":
    max_speed = 2000
    calibration_speed = 1000
    calibration_count = 100
    motors.flip_left(True)
    motors.flip_right(True)

display.fill(0)
display.text("Line Follower", 0, 0)
display.text("Place on line", 0, 20)
display.text("and press A to", 0, 30)
display.text("calibrate.", 0, 40)
display.show()

while not button_a.check():
    pass

display.fill(0)
display.show()
time.sleep(0.5)

motors.set_speeds(calibration_speed, -calibration_speed)
for _ in range(calibration_count // 4):
    line_sensors.calibrate()

motors.off()
time.sleep(0.2)

motors.set_speeds(-calibration_speed, calibration_speed)
for _ in range(calibration_count // 2):
    line_sensors.calibrate()

motors.off()
time.sleep(0.2)

motors.set_speeds(calibration_speed, -calibration_speed)
for _ in range(calibration_count // 4):
    line_sensors.calibrate()

motors.off()

t1 = 0
t2 = time.ticks_us()
p = 0
line = []
starting = False
run_motors = False
last_update_ms = 0

def update_display():
    global line
    display.fill(0)
    display.text("Line Follower", 0, 0)
    if starting:
        display.text("Press A to stop", 0, 10)
    else:
        display.text("Press A to start", 0, 10)
    ms = (t2 - t1)/1000
    display.text(f"Main loop: {ms:.1f}ms", 0, 20)
    display.text('p = '+str(p), 0, 30)
    scale = 24/1000

    print(line)
    display.fill_rect(36, 64-int(line[0]*scale), 8, int(line[0]*scale), 1)
    display.fill_rect(48, 64-int(line[1]*scale), 8, int(line[1]*scale), 1)
    display.fill_rect(60, 64-int(line[2]*scale), 8, int(line[2]*scale), 1)
    display.fill_rect(72, 64-int(line[3]*scale), 8, int(line[3]*scale), 1)
    display.fill_rect(84, 64-int(line[4]*scale), 8, int(line[4]*scale), 1)

    display.show()

def follow_line():
    last_p = 0
    global p, ir, t1, t2, line, max_speed, run_motors
    while True:
        line = line_sensors.read_calibrated()[:]
        line_sensors.start_read()
        t1 = t2
        t2 = time.ticks_us()

        if line[1] < 700 and line[2] < 700 and line[3] < 700:
            if p < 0:
                l = 0
            else:
                l = 4000
        else:
            l = (1000 * line[1] + 2000 * line[2] + 3000 * line[3] + 4000 * line[4]) // sum(line)

        p = l - 2000
        d = p - last_p
        last_p = p
        pid = p * 90 + d * 2000

        min_speed = 0
        left = max(min_speed, min(max_speed, max_speed + pid))
        right = max(min_speed, min(max_speed, max_speed - pid))

        if run_motors:
            motors.set_speeds(left, right)
            
        else:
            motors.off()
            
def check_bumpers():
    global max_speed, run_motors
    max_speed_local = max_speed
    bump_sensor.read()
    if bump_sensors.left_is_pressed() or bump_sensors.right_is_pressed():
        # Reduce speed to half
        max_speed = max_speed // 2
        #motors.set_speeds(current_speed, current_speed)
        time.sleep(1)
        # Gradually increase speed back to max
        while max_speed < max_speed_local:
            #time.sleep(1)  # Adjust the sleep time to control the speed increase rate
            max_speed += 50
            if max_speed > max_speed_local:
                max_speed = max_speed_local
            #motors.set_speeds(current_speed, current_speed)
    

_thread.start_new_thread(follow_line, ())
time.sleep_ms(1)

while True:
    t = time.ticks_ms()
    #check_bumpers()  # Check bumpers in the main loop
    if time.ticks_diff(t, last_update_ms) > 100:
        last_update_ms = t
        update_display()

    if button_a.check():
        if not starting:
            starting = True
            start_ms = t
        else:
            starting = False
            run_motors = False

    if starting and time.ticks_diff(t, start_ms) > 1000:
        run_motors = True
    
    
