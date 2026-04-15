from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
import time

# Simple demo for students:
# 1. Press B to start.
# 2. The robot says hello on the screen.
# 3. It drives forward/backward, spins, and does a short dance.
# 4. The screen shows encoder counts so students can see that code
#    can both command motion and read sensors.
#
# Students can safely change these values to explore:
DRIVE_SPEED = 0.45
TURN_SPEED = 0.40
FORWARD_MS = 900
BACKWARD_MS = 500
TURN_MS = 500
DANCE_STEP_MS = 220
DANCE_REPEAT = 3

display = robot.Display()
motors = robot.Motors()
encoders = robot.Encoders()
yellow_led = robot.YellowLED()
button_b = robot.ButtonB()

edition = editions.select()
if edition == "Hyper":
    motors.flip_left(True)
    motors.flip_right(True)


def show_screen(title, line1="", line2="", line3=""):
    display.fill(0)
    display.text(title[:16], 0, 0)
    display.text(line1[:16], 0, 16)
    display.text(line2[:16], 0, 30)
    display.text(line3[:16], 0, 44)
    display.show()


def speed_from_percent(percent):
    return int(motors.MAX_SPEED * percent)


def set_motor_percent(left_percent, right_percent):
    left = speed_from_percent(left_percent)
    right = speed_from_percent(right_percent)
    motors.set_speeds(left, right)


def stop_drive():
    motors.set_speeds(0, 0)


def drive_for(left_percent, right_percent, duration_ms, title):
    start_ms = time.ticks_ms()
    set_motor_percent(left_percent, right_percent)

    while time.ticks_diff(time.ticks_ms(), start_ms) < duration_ms:
        left_count, right_count = encoders.get_counts(reset=False)
        show_screen(
            title,
            "L={}".format(left_count),
            "R={}".format(right_count),
            "B = next time",
        )
        time.sleep_ms(60)

    stop_drive()
    time.sleep_ms(180)


def blink_led(times, on_ms=140, off_ms=140):
    for _ in range(times):
        yellow_led.on()
        time.sleep_ms(on_ms)
        yellow_led.off()
        time.sleep_ms(off_ms)


def wait_for_start():
    last_blink = time.ticks_ms()
    led_on = False

    while True:
        now = time.ticks_ms()
        left_count, right_count = encoders.get_counts(reset=False)

        show_screen(
            "3pi+ Demo",
            "Press B",
            "L={} R={}".format(left_count, right_count),
            "Lets code robots",
        )

        if time.ticks_diff(now, last_blink) >= 250:
            if led_on:
                yellow_led.off()
            else:
                yellow_led.on()
            led_on = not led_on
            last_blink = now

        if button_b.check():
            yellow_led.off()
            return

        time.sleep_ms(25)


def countdown():
    for number in (3, 2, 1):
        show_screen("Starting", "in {}".format(number))
        blink_led(1, 160, 80)
        time.sleep_ms(760)


def hello_scene():
    show_screen("Hello!", "I am a robot", "I can move", "and measure")
    blink_led(3)
    time.sleep_ms(700)


def movement_scene():
    encoders.get_counts(reset=True)

    drive_for(DRIVE_SPEED, DRIVE_SPEED, FORWARD_MS, "Forward")
    drive_for(-DRIVE_SPEED, -DRIVE_SPEED, BACKWARD_MS, "Backward")
    drive_for(-TURN_SPEED, TURN_SPEED, TURN_MS, "Spin Left")
    drive_for(TURN_SPEED, -TURN_SPEED, TURN_MS, "Spin Right")


def dance_scene():
    show_screen("Dance Time", "Watch pattern")
    time.sleep_ms(500)

    for _ in range(DANCE_REPEAT):
        drive_for(0.45, 0.05, DANCE_STEP_MS, "Dance Left")
        drive_for(0.05, 0.45, DANCE_STEP_MS, "Dance Right")
        drive_for(-0.30, -0.30, 120, "Moonwalk")


def encoder_scene():
    left_count, right_count = encoders.get_counts(reset=False)
    average = (left_count + right_count) // 2

    show_screen(
        "Sensors",
        "L={}".format(left_count),
        "R={}".format(right_count),
        "AVG={}".format(average),
    )
    time.sleep_ms(1800)


def ending_scene():
    show_screen(
        "Your Turn",
        "Change speed",
        "Change time",
        "Try new moves",
    )
    blink_led(4, 90, 90)
    time.sleep_ms(1200)


def main():
    stop_drive()
    yellow_led.off()

    while True:
        wait_for_start()
        countdown()
        hello_scene()
        movement_scene()
        dance_scene()
        encoder_scene()
        ending_scene()


main()
