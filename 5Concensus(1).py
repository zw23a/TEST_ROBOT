from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
from machine import UART, Pin
import random
import time

# Copy this file to each robot and only change ROBOT_ID.
ROBOT_ID = 1

DEFAULT_FORMATION_MODE = "min_time"

# Same UART assumption as the 3-robot version.
UART_ID = 0
UART_BAUD = 9600
UART_TX_PIN = 28
UART_RX_PIN = 29

RANDOM_MOVE_MAX_MS = 3000
MOTOR_PERCENT = 0.50
APPROACH_PERCENT = 0.30
FINE_PERCENT = 0.18
POSITION_TOLERANCE_COUNTS = 14
CONTROL_PERIOD_MS = 30
SEND_PERIOD_MS = 200
MODE_REPEAT_MS = 3000
TARGET_REPEAT_MS = 3000
DISPLAY_PERIOD_MS = 150
MOVE_COST_PER_SECOND = 5.0
SEND_OFFSET_MS = 40

NEIGHBORS = {
    1: (2,),
    2: (1, 3),
    3: (2, 4),
    4: (3, 5),
    5: (4,),
}

display = robot.Display()
motors = robot.Motors()
encoders = robot.Encoders()
yellow_led = robot.YellowLED()
button_a = robot.ButtonA()
button_b = robot.ButtonB()
button_c = robot.ButtonC()

edition = editions.select()
if edition == "Hyper":
    motors.flip_left(True)
    motors.flip_right(True)

uart = UART(UART_ID, baudrate=UART_BAUD, tx=Pin(UART_TX_PIN), rx=Pin(UART_RX_PIN))
rx_buffer = b""


def show_screen(title, line1="", line2="", line3="", line4=""):
    display.fill(0)
    display.text(title[:16], 0, 0)
    display.text(line1[:16], 0, 16)
    display.text(line2[:16], 0, 26)
    display.text(line3[:16], 0, 36)
    display.text(line4[:16], 0, 46)
    display.show()


def average_counts(reset=False):
    left, right = encoders.get_counts(reset=reset)
    return (left + right) // 2


def speed_from_percent(percent):
    return int(motors.MAX_SPEED * percent)


def drive_straight(speed):
    motors.set_speeds(speed, speed)


def stop_drive():
    motors.set_speeds(0, 0)


def rounded_div(numerator, denominator):
    if numerator >= 0:
        return (numerator + denominator // 2) // denominator
    return -((-numerator + denominator // 2) // denominator)


def clear_uart_buffer():
    global rx_buffer

    rx_buffer = b""
    while uart.any():
        uart.read()
        time.sleep_ms(5)


def wait_for_start():
    show_screen("Robot {}".format(ROBOT_ID), "Press B", "random move")
    while True:
        if button_b.check():
            break
        time.sleep_ms(20)

    for remaining in (3, 2, 1):
        show_screen("Robot {}".format(ROBOT_ID), "Start in {}".format(remaining))
        time.sleep_ms(1000)


def run_random_excursion():
    random.seed(time.ticks_ms() + ROBOT_ID * 997)
    signed_ms = random.randint(-RANDOM_MOVE_MAX_MS, RANDOM_MOVE_MAX_MS)
    travel_speed = speed_from_percent(MOTOR_PERCENT)
    motion_speed = travel_speed if signed_ms >= 0 else -travel_speed

    average_counts(reset=True)

    direction = "FWD" if signed_ms >= 0 else "REV"
    show_screen(
        "Random Move",
        "id={}".format(ROBOT_ID),
        "{} {}ms".format(direction, abs(signed_ms)),
    )

    yellow_led.on()
    drive_straight(motion_speed)
    time.sleep_ms(abs(signed_ms))
    stop_drive()
    yellow_led.off()
    time.sleep_ms(200)

    return signed_ms, average_counts(reset=False)


def send_message(kind, *fields):
    payload = ",".join([kind] + [str(v) for v in fields]) + "\n"
    uart.write(payload.encode())


def parse_message(raw_line, shared):
    try:
        text = raw_line.decode().strip()
    except Exception:
        return

    if not text:
        return

    parts = text.split(",")
    kind = parts[0]

    if kind == "MODE" and len(parts) == 4:
        try:
            sender = int(parts[1])
            origin = int(parts[2])
            mode_name = parts[3]
        except ValueError:
            return
        if sender in NEIGHBORS[ROBOT_ID]:
            shared["mode_votes"][origin] = mode_name
        return

    if kind == "MODE_SYNC" and len(parts) == 3:
        try:
            sender = int(parts[1])
            mode_name = parts[2]
        except ValueError:
            return
        if sender in NEIGHBORS[ROBOT_ID]:
            shared["selected_mode"] = mode_name
        return

    if kind == "POS" and len(parts) == 4:
        try:
            sender = int(parts[1])
            origin = int(parts[2])
            value = int(parts[3])
        except ValueError:
            return
        if sender in NEIGHBORS[ROBOT_ID] and 1 <= origin <= 5:
            shared["positions"][origin] = value
        return

    if kind == "TARGET" and len(parts) == 4:
        try:
            sender = int(parts[1])
            target_robot = int(parts[2])
            target_value = int(parts[3])
        except ValueError:
            return
        if sender not in NEIGHBORS[ROBOT_ID]:
            return

        if target_robot == ROBOT_ID:
            shared["target"] = target_value
            return

        if ROBOT_ID == 2 and sender == 3 and target_robot == 1:
            shared["relay_targets"][1] = target_value
            return

        if ROBOT_ID == 4 and sender == 3 and target_robot == 5:
            shared["relay_targets"][5] = target_value


def poll_messages(shared):
    global rx_buffer

    if not uart.any():
        return

    incoming = uart.read()
    if not incoming:
        return

    rx_buffer += incoming
    if len(rx_buffer) > 512:
        rx_buffer = b""

    while b"\n" in rx_buffer:
        raw_line, rx_buffer = rx_buffer.split(b"\n", 1)
        parse_message(raw_line, shared)


def send_mode_upstream(shared, local_mode):
    if ROBOT_ID == 1:
        if local_mode is not None:
            send_message("MODE", 1, 1, local_mode)
        return

    if ROBOT_ID == 2:
        if local_mode is not None:
            send_message("MODE", 2, 2, local_mode)
        relay_mode = shared["mode_votes"].get(1)
        if relay_mode is not None:
            send_message("MODE", 2, 1, relay_mode)
        return

    if ROBOT_ID == 4:
        if local_mode is not None:
            send_message("MODE", 4, 4, local_mode)
        relay_mode = shared["mode_votes"].get(5)
        if relay_mode is not None:
            send_message("MODE", 4, 5, relay_mode)
        return

    if ROBOT_ID == 5:
        if local_mode is not None:
            send_message("MODE", 5, 5, local_mode)


def choose_mode():
    shared = {
        "mode_votes": {},
        "selected_mode": None,
        "positions": {1: None, 2: None, 3: None, 4: None, 5: None},
        "relay_targets": {},
        "target": None,
    }
    local_mode = None
    last_mode_send = time.ticks_ms() - SEND_PERIOD_MS + ROBOT_ID * SEND_OFFSET_MS
    last_sync_send = time.ticks_ms() - SEND_PERIOD_MS + ROBOT_ID * SEND_OFFSET_MS
    sync_started_at = None

    while True:
        now = time.ticks_ms()
        poll_messages(shared)

        if local_mode is None:
            if button_a.check():
                local_mode = "min_time"
            elif button_b.check():
                local_mode = "least_squares"
            elif button_c.check():
                local_mode = DEFAULT_FORMATION_MODE

        if time.ticks_diff(now, last_mode_send) >= SEND_PERIOD_MS:
            send_mode_upstream(shared, local_mode)
            last_mode_send = now

        if ROBOT_ID == 3:
            if local_mode is not None:
                shared["selected_mode"] = local_mode
            else:
                for candidate_id in (2, 4, 1, 5):
                    candidate_mode = shared["mode_votes"].get(candidate_id)
                    if candidate_mode is not None:
                        shared["selected_mode"] = candidate_mode
                        break

            if shared["selected_mode"] is not None:
                if sync_started_at is None:
                    sync_started_at = now
                if time.ticks_diff(now, last_sync_send) >= SEND_PERIOD_MS:
                    send_message("MODE_SYNC", 3, shared["selected_mode"])
                    last_sync_send = now
                if time.ticks_diff(now, sync_started_at) >= MODE_REPEAT_MS:
                    return shared["selected_mode"]
        elif ROBOT_ID in (2, 4):
            if shared["selected_mode"] is not None:
                if sync_started_at is None:
                    sync_started_at = now
                if time.ticks_diff(now, last_sync_send) >= SEND_PERIOD_MS:
                    send_message("MODE_SYNC", ROBOT_ID, shared["selected_mode"])
                    last_sync_send = now
                if time.ticks_diff(now, sync_started_at) >= MODE_REPEAT_MS:
                    return shared["selected_mode"]
        else:
            if shared["selected_mode"] is not None:
                return shared["selected_mode"]

        chosen_text = local_mode if local_mode is not None else "--"
        show_screen(
            "Pick Mode",
            "A:min_time",
            "B:least_sq",
            "C:default",
            "mine={}".format(chosen_text[:10]),
        )
        time.sleep_ms(30)


def countdown_before_converge(formation_mode):
    label = "min_time" if formation_mode == "min_time" else "least_sq"
    for remaining in (3, 2, 1):
        show_screen(
            "Mode Locked",
            label,
            "start in {}".format(remaining),
        )
        time.sleep_ms(1000)


def compute_targets(x1, x2, x3, x4, x5, formation_mode):
    if formation_mode == "least_squares":
        xs = (x1, x2, x3, x4, x5)
        sum_x = x1 + x2 + x3 + x4 + x5
        mean_x_times_5 = sum_x
        centered_sum = (-2 * x1) + (-1 * x2) + (0 * x3) + (1 * x4) + (2 * x5)
        slope_num = centered_sum
        slope_den = 10

        targets = {}
        for robot_id in (1, 2, 3, 4, 5):
            centered_index = robot_id - 3
            numerator = mean_x_times_5 * slope_den + 5 * centered_index * slope_num
            denominator = 5 * slope_den
            targets[robot_id] = rounded_div(numerator, denominator)
        return targets

    diff = x5 - x1
    return {
        1: x1,
        2: x1 + rounded_div(diff, 4),
        3: x1 + rounded_div(diff, 2),
        4: x1 + rounded_div(3 * diff, 4),
        5: x5,
    }


def send_positions_upstream(shared, local_position):
    if ROBOT_ID == 1:
        send_message("POS", 1, 1, local_position)
        return

    if ROBOT_ID == 2:
        send_message("POS", 2, 2, local_position)
        if shared["positions"][1] is not None:
            send_message("POS", 2, 1, shared["positions"][1])
        return

    if ROBOT_ID == 4:
        send_message("POS", 4, 4, local_position)
        if shared["positions"][5] is not None:
            send_message("POS", 4, 5, shared["positions"][5])
        return

    if ROBOT_ID == 5:
        send_message("POS", 5, 5, local_position)


def send_targets_downstream(shared, targets):
    if ROBOT_ID == 3:
        send_message("TARGET", 3, 1, targets[1])
        send_message("TARGET", 3, 2, targets[2])
        send_message("TARGET", 3, 3, targets[3])
        send_message("TARGET", 3, 4, targets[4])
        send_message("TARGET", 3, 5, targets[5])
        return

    if ROBOT_ID == 2 and 1 in shared["relay_targets"]:
        send_message("TARGET", 2, 1, shared["relay_targets"][1])
        return

    if ROBOT_ID == 4 and 5 in shared["relay_targets"]:
        send_message("TARGET", 4, 5, shared["relay_targets"][5])


def exchange_positions_and_get_target(local_position, formation_mode):
    shared = {
        "positions": {1: None, 2: None, 3: None, 4: None, 5: None},
        "relay_targets": {},
        "target": None,
        "mode_votes": {},
        "selected_mode": None,
    }
    shared["positions"][ROBOT_ID] = local_position

    last_send = time.ticks_ms() - SEND_PERIOD_MS + ROBOT_ID * SEND_OFFSET_MS
    last_target_send = time.ticks_ms() - SEND_PERIOD_MS + ROBOT_ID * SEND_OFFSET_MS
    last_display = time.ticks_ms() - DISPLAY_PERIOD_MS
    target_ready_since = None

    while True:
        now = time.ticks_ms()
        poll_messages(shared)

        if time.ticks_diff(now, last_send) >= SEND_PERIOD_MS:
            send_positions_upstream(shared, local_position)
            last_send = now

        if ROBOT_ID == 3:
            shared["positions"][3] = local_position
            if (
                shared["positions"][1] is not None
                and shared["positions"][2] is not None
                and shared["positions"][4] is not None
                and shared["positions"][5] is not None
            ):
                targets = compute_targets(
                    shared["positions"][1],
                    shared["positions"][2],
                    shared["positions"][3],
                    shared["positions"][4],
                    shared["positions"][5],
                    formation_mode,
                )
                shared["target"] = targets[3]

                if time.ticks_diff(now, last_target_send) >= SEND_PERIOD_MS:
                    send_targets_downstream(shared, targets)
                    last_target_send = now
                    if target_ready_since is None:
                        target_ready_since = now

                if (
                    target_ready_since is not None
                    and time.ticks_diff(now, target_ready_since) >= TARGET_REPEAT_MS
                ):
                    return shared["target"]
        elif ROBOT_ID in (2, 4):
            if shared["target"] is not None and target_ready_since is None:
                target_ready_since = now

            if time.ticks_diff(now, last_target_send) >= SEND_PERIOD_MS:
                send_targets_downstream(shared, None)
                last_target_send = now

            if (
                shared["target"] is not None
                and target_ready_since is not None
                and time.ticks_diff(now, target_ready_since) >= TARGET_REPEAT_MS
            ):
                return shared["target"]
        else:
            if shared["target"] is not None:
                return shared["target"]

        if time.ticks_diff(now, last_display) >= DISPLAY_PERIOD_MS:
            if ROBOT_ID == 3:
                got_left = shared["positions"][1] is not None and shared["positions"][2] is not None
                got_right = shared["positions"][4] is not None and shared["positions"][5] is not None
                show_screen(
                    "Sync Target",
                    "x={}".format(local_position),
                    "L={}".format("ok" if got_left else "--"),
                    "R={}".format("ok" if got_right else "--"),
                )
            elif ROBOT_ID == 2:
                show_screen(
                    "Relay Left",
                    "x={}".format(local_position),
                    "1={}".format("--" if shared["positions"][1] is None else "ok"),
                    "t1={}".format("--" if 1 not in shared["relay_targets"] else "ok"),
                )
            elif ROBOT_ID == 4:
                show_screen(
                    "Relay Right",
                    "x={}".format(local_position),
                    "5={}".format("--" if shared["positions"][5] is None else "ok"),
                    "t5={}".format("--" if 5 not in shared["relay_targets"] else "ok"),
                )
            else:
                show_screen(
                    "Wait Target",
                    "x={}".format(local_position),
                    "from center",
                )
            last_display = now

        time.sleep_ms(20)


def move_to_position(target_position):
    cruise_speed = speed_from_percent(MOTOR_PERCENT)
    approach_speed = speed_from_percent(APPROACH_PERCENT)
    fine_speed = speed_from_percent(FINE_PERCENT)

    start_ms = time.ticks_ms()
    last_display = start_ms - DISPLAY_PERIOD_MS

    while True:
        current_position = average_counts(reset=False)
        error = target_position - current_position
        abs_error = abs(error)

        if abs_error <= POSITION_TOLERANCE_COUNTS:
            stop_drive()
            elapsed_ms = time.ticks_diff(time.ticks_ms(), start_ms)
            return current_position, elapsed_ms

        if abs_error > 220:
            speed = cruise_speed
        elif abs_error > 80:
            speed = approach_speed
        else:
            speed = fine_speed

        drive_straight(speed if error > 0 else -speed)

        now = time.ticks_ms()
        if time.ticks_diff(now, last_display) >= DISPLAY_PERIOD_MS:
            show_screen(
                "Converging",
                "cur={}".format(current_position),
                "tar={}".format(target_position),
                "err={}".format(error),
            )
            last_display = now

        time.sleep_ms(CONTROL_PERIOD_MS)


def main():
    clear_uart_buffer()
    wait_for_start()

    signed_ms, local_position = run_random_excursion()
    clear_uart_buffer()

    formation_mode = choose_mode()
    show_screen(
        "Mode Chosen",
        "id={}".format(ROBOT_ID),
        formation_mode[:16],
        "cmd={}ms".format(signed_ms),
    )
    time.sleep_ms(700)

    countdown_before_converge(formation_mode)
    clear_uart_buffer()

    target_position = exchange_positions_and_get_target(local_position, formation_mode)
    show_screen(
        "Target Ready",
        "id={}".format(ROBOT_ID),
        formation_mode[:16],
        "target={}".format(target_position),
    )
    time.sleep_ms(700)

    final_position, travel_ms = move_to_position(target_position)
    travel_cost = (travel_ms / 1000.0) * MOVE_COST_PER_SECOND

    yellow_led.on()
    show_screen(
        "Done",
        "final={}".format(final_position),
        "t={:.2f}s".format(travel_ms / 1000.0),
        "cost={:.2f}".format(travel_cost),
    )

    while True:
        time.sleep_ms(100)


main()
