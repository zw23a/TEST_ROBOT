from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
from machine import UART, Pin
import random
import time

# Copy this file to each robot and only change ROBOT_ID.
ROBOT_ID = 1

DEFAULT_FORMATION_MODE = "min_time"

# This program assumes all three robots share the same UART bus.
# Messages are broadcast, and each robot only uses the messages that are
# allowed by the chain graph 1-2-3.
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
TARGET_REPEAT_MS = 1500
DISPLAY_PERIOD_MS = 150
MOVE_COST_PER_SECOND = 5.0

NEIGHBORS = {
    1: (2,),
    2: (1, 3),
    3: (2,),
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


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def speed_from_percent(percent):
    return int(motors.MAX_SPEED * percent)


def drive_straight(speed):
    motors.set_speeds(speed, speed)


def stop_drive():
    motors.set_speeds(0, 0)


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

    position = average_counts(reset=False)
    return signed_ms, position


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

    if kind == "POS" and len(parts) == 3:
        sender = int(parts[1])
        value = int(parts[2])
        if sender in NEIGHBORS[ROBOT_ID]:
            shared["positions"][sender] = value
        return

    if kind == "TARGET" and len(parts) == 4:
        sender = int(parts[1])
        target_robot = int(parts[2])
        target_value = int(parts[3])
        if sender == 2 and target_robot == ROBOT_ID:
            shared["target"] = target_value
        return

    if kind == "DONE" and len(parts) == 4:
        sender = int(parts[1])
        if sender in NEIGHBORS[ROBOT_ID]:
            shared["done"][sender] = True
        return

    if kind == "MODE" and len(parts) == 3:
        sender = int(parts[1])
        mode_name = parts[2]
        if sender in NEIGHBORS[ROBOT_ID]:
            shared["mode_votes"][sender] = mode_name
        return

    if kind == "MODE_SYNC" and len(parts) == 3:
        sender = int(parts[1])
        mode_name = parts[2]
        if sender == 2 or sender in NEIGHBORS[ROBOT_ID]:
            shared["selected_mode"] = mode_name


def poll_messages(shared):
    global rx_buffer

    if not uart.any():
        return

    incoming = uart.read()
    if not incoming:
        return

    rx_buffer += incoming
    while b"\n" in rx_buffer:
        raw_line, rx_buffer = rx_buffer.split(b"\n", 1)
        parse_message(raw_line, shared)


def compute_targets(x1, x2, x3, formation_mode):
    if formation_mode == "least_squares":
        # Orthogonal projection onto the affine-line subspace for 3 equally
        # spaced robots.
        t1 = int(round((5 * x1 + 2 * x2 - x3) / 6.0))
        t2 = int(round((x1 + x2 + x3) / 3.0))
        t3 = int(round((-x1 + 2 * x2 + 5 * x3) / 6.0))
        return {1: t1, 2: t2, 3: t3}

    # Minimum-time / minimum-linear-cost solution.
    midpoint = (x1 + x3) // 2
    return {1: x1, 2: midpoint, 3: x3}


def choose_mode():
    shared = {
        "mode_votes": {},
        "selected_mode": None,
    }
    local_mode = None
    last_mode_send = time.ticks_ms() - SEND_PERIOD_MS
    last_sync_send = time.ticks_ms() - SEND_PERIOD_MS
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

        if local_mode is not None and time.ticks_diff(now, last_mode_send) >= SEND_PERIOD_MS:
            send_message("MODE", ROBOT_ID, local_mode)
            last_mode_send = now

        if ROBOT_ID == 2:
            if local_mode is not None:
                shared["selected_mode"] = local_mode
            else:
                left_vote = shared["mode_votes"].get(1)
                right_vote = shared["mode_votes"].get(3)
                if left_vote is not None:
                    shared["selected_mode"] = left_vote
                elif right_vote is not None:
                    shared["selected_mode"] = right_vote

            if shared["selected_mode"] is not None:
                if sync_started_at is None:
                    sync_started_at = now
                if time.ticks_diff(now, last_sync_send) >= SEND_PERIOD_MS:
                    send_message("MODE_SYNC", 2, shared["selected_mode"])
                    last_sync_send = now
                if time.ticks_diff(now, sync_started_at) >= 1200:
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


def exchange_positions_and_get_target(local_position, formation_mode):
    shared = {
        "positions": {1: None, 2: None, 3: None},
        "target": None,
        "done": {},
        "mode_votes": {},
        "selected_mode": None,
    }
    shared["positions"][ROBOT_ID] = local_position

    last_send = time.ticks_ms() - SEND_PERIOD_MS
    last_display = time.ticks_ms() - DISPLAY_PERIOD_MS
    last_target_send = time.ticks_ms() - SEND_PERIOD_MS
    target_ready_since = None

    while True:
        now = time.ticks_ms()
        poll_messages(shared)

        if time.ticks_diff(now, last_send) >= SEND_PERIOD_MS:
            send_message("POS", ROBOT_ID, local_position)
            last_send = now

        if ROBOT_ID == 2:
            shared["positions"][2] = local_position
            if shared["positions"][1] is not None and shared["positions"][3] is not None:
                targets = compute_targets(
                    shared["positions"][1],
                    shared["positions"][2],
                    shared["positions"][3],
                    formation_mode,
                )
                shared["target"] = targets[2]

                if time.ticks_diff(now, last_target_send) >= SEND_PERIOD_MS:
                    send_message("TARGET", 2, 1, targets[1])
                    send_message("TARGET", 2, 2, targets[2])
                    send_message("TARGET", 2, 3, targets[3])
                    last_target_send = now
                    if target_ready_since is None:
                        target_ready_since = now

                if (
                    target_ready_since is not None
                    and time.ticks_diff(now, target_ready_since) >= TARGET_REPEAT_MS
                ):
                    return shared["target"]
        else:
            if shared["target"] is not None:
                return shared["target"]

        if time.ticks_diff(now, last_display) >= DISPLAY_PERIOD_MS:
            if ROBOT_ID == 2:
                got_1 = shared["positions"][1] is not None
                got_3 = shared["positions"][3] is not None
                show_screen(
                    "Sync Target",
                    "x={}".format(local_position),
                    "1={}".format("ok" if got_1 else "--"),
                    "3={}".format("ok" if got_3 else "--"),
                )
            else:
                show_screen(
                    "Wait Target",
                    "x={}".format(local_position),
                    "from robot 2",
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
    wait_for_start()

    signed_ms, local_position = run_random_excursion()
    formation_mode = choose_mode()
    show_screen(
        "Mode Chosen",
        "id={}".format(ROBOT_ID),
        formation_mode[:16],
        "cmd={}ms".format(signed_ms),
    )
    time.sleep_ms(700)

    countdown_before_converge(formation_mode)

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
    send_message("DONE", ROBOT_ID, final_position, travel_ms)

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
from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
from machine import UART, Pin
import random
import time

# Copy this file to each robot and only change ROBOT_ID.
ROBOT_ID = 1

DEFAULT_FORMATION_MODE = "min_time"

# This program assumes all three robots share the same UART bus.
# Messages are broadcast, and each robot only uses the messages that are
# allowed by the chain graph 1-2-3.
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
TARGET_REPEAT_MS = 1500
DISPLAY_PERIOD_MS = 150
MOVE_COST_PER_SECOND = 5.0

NEIGHBORS = {
    1: (2,),
    2: (1, 3),
    3: (2,),
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


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def speed_from_percent(percent):
    return int(motors.MAX_SPEED * percent)


def drive_straight(speed):
    motors.set_speeds(speed, speed)


def stop_drive():
    motors.set_speeds(0, 0)


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

    position = average_counts(reset=False)
    return signed_ms, position


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

    if kind == "POS" and len(parts) == 3:
        sender = int(parts[1])
        value = int(parts[2])
        if sender in NEIGHBORS[ROBOT_ID]:
            shared["positions"][sender] = value
        return

    if kind == "TARGET" and len(parts) == 4:
        sender = int(parts[1])
        target_robot = int(parts[2])
        target_value = int(parts[3])
        if sender == 2 and target_robot == ROBOT_ID:
            shared["target"] = target_value
        return

    if kind == "DONE" and len(parts) == 4:
        sender = int(parts[1])
        if sender in NEIGHBORS[ROBOT_ID]:
            shared["done"][sender] = True
        return

    if kind == "MODE" and len(parts) == 3:
        sender = int(parts[1])
        mode_name = parts[2]
        if sender in NEIGHBORS[ROBOT_ID]:
            shared["mode_votes"][sender] = mode_name
        return

    if kind == "MODE_SYNC" and len(parts) == 3:
        sender = int(parts[1])
        mode_name = parts[2]
        if sender == 2 or sender in NEIGHBORS[ROBOT_ID]:
            shared["selected_mode"] = mode_name


def poll_messages(shared):
    global rx_buffer

    if not uart.any():
        return

    incoming = uart.read()
    if not incoming:
        return

    rx_buffer += incoming
    while b"\n" in rx_buffer:
        raw_line, rx_buffer = rx_buffer.split(b"\n", 1)
        parse_message(raw_line, shared)


def compute_targets(x1, x2, x3, formation_mode):
    if formation_mode == "least_squares":
        # Orthogonal projection onto the affine-line subspace for 3 equally
        # spaced robots.
        t1 = int(round((5 * x1 + 2 * x2 - x3) / 6.0))
        t2 = int(round((x1 + x2 + x3) / 3.0))
        t3 = int(round((-x1 + 2 * x2 + 5 * x3) / 6.0))
        return {1: t1, 2: t2, 3: t3}

    # Minimum-time / minimum-linear-cost solution.
    midpoint = (x1 + x3) // 2
    return {1: x1, 2: midpoint, 3: x3}


def choose_mode():
    shared = {
        "mode_votes": {},
        "selected_mode": None,
    }
    local_mode = None
    last_mode_send = time.ticks_ms() - SEND_PERIOD_MS
    last_sync_send = time.ticks_ms() - SEND_PERIOD_MS
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

        if local_mode is not None and time.ticks_diff(now, last_mode_send) >= SEND_PERIOD_MS:
            send_message("MODE", ROBOT_ID, local_mode)
            last_mode_send = now

        if ROBOT_ID == 2:
            if local_mode is not None:
                shared["selected_mode"] = local_mode
            else:
                left_vote = shared["mode_votes"].get(1)
                right_vote = shared["mode_votes"].get(3)
                if left_vote is not None:
                    shared["selected_mode"] = left_vote
                elif right_vote is not None:
                    shared["selected_mode"] = right_vote

            if shared["selected_mode"] is not None:
                if sync_started_at is None:
                    sync_started_at = now
                if time.ticks_diff(now, last_sync_send) >= SEND_PERIOD_MS:
                    send_message("MODE_SYNC", 2, shared["selected_mode"])
                    last_sync_send = now
                if time.ticks_diff(now, sync_started_at) >= 1200:
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


def exchange_positions_and_get_target(local_position, formation_mode):
    shared = {
        "positions": {1: None, 2: None, 3: None},
        "target": None,
        "done": {},
        "mode_votes": {},
        "selected_mode": None,
    }
    shared["positions"][ROBOT_ID] = local_position

    last_send = time.ticks_ms() - SEND_PERIOD_MS
    last_display = time.ticks_ms() - DISPLAY_PERIOD_MS
    last_target_send = time.ticks_ms() - SEND_PERIOD_MS
    target_ready_since = None

    while True:
        now = time.ticks_ms()
        poll_messages(shared)

        if time.ticks_diff(now, last_send) >= SEND_PERIOD_MS:
            send_message("POS", ROBOT_ID, local_position)
            last_send = now

        if ROBOT_ID == 2:
            shared["positions"][2] = local_position
            if shared["positions"][1] is not None and shared["positions"][3] is not None:
                targets = compute_targets(
                    shared["positions"][1],
                    shared["positions"][2],
                    shared["positions"][3],
                    formation_mode,
                )
                shared["target"] = targets[2]

                if time.ticks_diff(now, last_target_send) >= SEND_PERIOD_MS:
                    send_message("TARGET", 2, 1, targets[1])
                    send_message("TARGET", 2, 2, targets[2])
                    send_message("TARGET", 2, 3, targets[3])
                    last_target_send = now
                    if target_ready_since is None:
                        target_ready_since = now

                if (
                    target_ready_since is not None
                    and time.ticks_diff(now, target_ready_since) >= TARGET_REPEAT_MS
                ):
                    return shared["target"]
        else:
            if shared["target"] is not None:
                return shared["target"]

        if time.ticks_diff(now, last_display) >= DISPLAY_PERIOD_MS:
            if ROBOT_ID == 2:
                got_1 = shared["positions"][1] is not None
                got_3 = shared["positions"][3] is not None
                show_screen(
                    "Sync Target",
                    "x={}".format(local_position),
                    "1={}".format("ok" if got_1 else "--"),
                    "3={}".format("ok" if got_3 else "--"),
                )
            else:
                show_screen(
                    "Wait Target",
                    "x={}".format(local_position),
                    "from robot 2",
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
    wait_for_start()

    signed_ms, local_position = run_random_excursion()
    formation_mode = choose_mode()
    show_screen(
        "Mode Chosen",
        "id={}".format(ROBOT_ID),
        formation_mode[:16],
        "cmd={}ms".format(signed_ms),
    )
    time.sleep_ms(700)

    countdown_before_converge(formation_mode)

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
    send_message("DONE", ROBOT_ID, final_position, travel_ms)

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
