from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
import time

# Very simple classroom demo:
# Press B, then watch one short move repeat several times.
# This is designed to teach the idea of:
# "for i in range(...): do the same action again and again"

REPEAT_TIMES = 4
MOVE_SPEED = 0.40
MOVE_TIME_MS = 350
PAUSE_MS = 250

display = robot.Display()
bump_sensors = robot.BumpSensors()
motors = robot.Motors()
buzzer = robot.Buzzer()
button_b = robot.ButtonB()
yellow_led = robot.YellowLED()

edition = editions.select()
if edition == "Standard":
    bumper_max_speed = 1500
    bumper_turn_time_ms = 250
elif edition == "Turtle":
    bumper_max_speed = 3000
    bumper_turn_time_ms = 500
if edition == "Hyper":
    bumper_max_speed = 1125
    bumper_turn_time_ms = 150
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


def drive_straight(percent):
    speed = speed_from_percent(percent)
    motors.set_speeds(speed, speed)


def stop_drive():
    motors.set_speeds(0, 0)


def set_motor_speeds(left_speed, right_speed):
    motors.set_speeds(int(left_speed), int(right_speed))


def handle_bumper_hit():
    bump_sensors.read()
    left_pressed = bump_sensors.left_is_pressed()
    right_pressed = bump_sensors.right_is_pressed()

    if not left_pressed and not right_pressed:
        return False

    stop_drive()
    yellow_led.on()

    if left_pressed and right_pressed:
        show_screen("Bumper Hit", "Both", "Back + turn")
        buzzer.play("g32")
        set_motor_speeds(-bumper_max_speed / 2, -bumper_max_speed / 2)
        time.sleep_ms(bumper_turn_time_ms)
        set_motor_speeds(bumper_max_speed / 2, -bumper_max_speed / 2)
        time.sleep_ms(bumper_turn_time_ms)
    elif left_pressed:
        show_screen("Bumper Hit", "Left", "Back + right")
        buzzer.play("c32")
        set_motor_speeds(-bumper_max_speed / 5, -bumper_max_speed / 5)
        time.sleep_ms(bumper_turn_time_ms)
        set_motor_speeds(bumper_max_speed / 3, -bumper_max_speed / 3)
        time.sleep_ms(bumper_turn_time_ms)
    else:
        show_screen("Bumper Hit", "Right", "Back + left")
        buzzer.play("e32")
        set_motor_speeds(-bumper_max_speed / 5, -bumper_max_speed / 5)
        time.sleep_ms(bumper_turn_time_ms)
        set_motor_speeds(-bumper_max_speed / 3, bumper_max_speed / 3)
        time.sleep_ms(bumper_turn_time_ms)

    stop_drive()
    yellow_led.off()
    show_screen("Loop Resume", "Robot reacted", "to bumper hit")
    time.sleep_ms(350)
    return True


def wait_for_start():
    while True:
        show_screen(
            "For Loop Demo",
            "Press B",
            "Loop + bumper",
            "same move",
        )

        if button_b.check():
            return

        time.sleep_ms(30)


def countdown():
    for number in (3, 2, 1):
        show_screen("Starting", str(number))
        yellow_led.on()
        time.sleep_ms(200)
        yellow_led.off()
        time.sleep_ms(800)


def repeat_move_demo():
    # This is the main teaching idea:
    # the same block runs REPEAT_TIMES times.
    for step in range(REPEAT_TIMES):
        step_started_at = time.ticks_ms()
        yellow_led.on()

        while time.ticks_diff(time.ticks_ms(), step_started_at) < MOVE_TIME_MS:
            show_screen(
                "Loop Running",
                "step {}/{}".format(step + 1, REPEAT_TIMES),
                "Move forward",
                "check bumpers",
            )

            drive_straight(MOVE_SPEED)
            if handle_bumper_hit():
                break
            time.sleep_ms(25)

        stop_drive()
        yellow_led.off()

        show_screen(
            "Pause",
            "step {}/{}".format(step + 1, REPEAT_TIMES),
            "Loop finished",
            "this round",
        )
        time.sleep_ms(PAUSE_MS)


def ending_screen():
    show_screen(
        "Done",
        "Change",
        "REPEAT_TIMES",
        "and test again",
    )
    time.sleep_ms(1800)


def main():
    stop_drive()
    yellow_led.off()
    show_screen("Bumpers", "Calibrating...")
    bump_sensors.calibrate()
    time.sleep_ms(1000)

    while True:
        wait_for_start()
        countdown()
        repeat_move_demo()
        ending_screen()


main()
