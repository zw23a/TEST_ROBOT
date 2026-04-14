from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
from machine import UART, Pin
import random
import time

# Copy this file to each robot and only change ROBOT_ID.
ROBOT_ID = 1

UART_ID = 0
UART_BAUD = 9600
UART_TX_PIN = 28
UART_RX_PIN = 29

RANDOM_MOVE_MAX_MS = 3000
POS_SEND_MS = 120
CTRL_SEND_MS = 160
READY_REPEAT_MS = 1800
STOP_REPEAT_MS = 2500
SETTLE_AFTER_MOVE_MS = 500
DISPLAY_PERIOD_MS = 150
CONTROL_PERIOD_MS = 30
SEND_OFFSET_MS = 35

MOTOR_PERCENT = 0.50
APPROACH_PERCENT = 0.30
FINE_PERCENT = 0.18
POSITION_TOLERANCE_COUNTS = 14
MOVE_SETTLE_TOLERANCE_COUNTS = 24
ALIGN_DONE_TOLERANCE_COUNTS = 40
ITERATION_TOLERANCE_COUNTS = ALIGN_DONE_TOLERANCE_COUNTS
MOVE_TIMEOUT_MS = 2600
STALL_IMPROVEMENT_COUNTS = 5
STALL_NEAR_TARGET_MS = 450
STALL_TIMEOUT_MS = 900
MOVE_COST_PER_SECOND = 5.0

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
button_b = robot.ButtonB()

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


def set_debug_rx(shared, text):
    shared["debug_rx"] = text[:16]


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


def send_message(kind, *fields):
    payload = ",".join([kind] + [str(v) for v in fields]) + "\n"
    uart.write(payload.encode())


def send_ack(target_robot, move_id):
    send_message("ACK", ROBOT_ID, target_robot, move_id)


def clear_uart_buffer():
    global rx_buffer

    rx_buffer = b""
    while uart.any():
        uart.read()
        time.sleep_ms(5)


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
        try:
            sender = int(parts[1])
            value = int(parts[2])
        except ValueError:
            return
        if sender in NEIGHBORS[ROBOT_ID]:
            shared["positions"][sender] = value
            set_debug_rx(shared, "rx P{}={}".format(sender, value))
        return

    if kind == "READYCHAIN" and len(parts) == 2:
        try:
            sender = int(parts[1])
        except ValueError:
            return
        if sender in NEIGHBORS[ROBOT_ID]:
            shared["ready_from_right"] = True
            set_debug_rx(shared, "rx R{}".format(sender))
        return

    if kind == "TOKEN" and len(parts) == 6:
        try:
            sender = int(parts[1])
            target = int(parts[2])
            move_id = int(parts[3])
            direction = int(parts[4])
            cycle_max = int(parts[5])
        except ValueError:
            return
        if sender in NEIGHBORS[ROBOT_ID]:
            shared["token"] = (target, move_id, direction, cycle_max)
            set_debug_rx(shared, "rx T{}>{}".format(sender, target))
            if target == ROBOT_ID:
                send_ack(sender, move_id)
        return

    if kind == "ACK" and len(parts) == 4:
        try:
            sender = int(parts[1])
            receiver = int(parts[2])
            move_id = int(parts[3])
        except ValueError:
            return
        if receiver == ROBOT_ID and sender in NEIGHBORS[ROBOT_ID]:
            shared["acks"][(sender, move_id)] = True
            set_debug_rx(shared, "rx A{}={}".format(sender, move_id))
        return

    if kind == "STOP" and len(parts) == 2:
        try:
            sender = int(parts[1])
        except ValueError:
            return
        if sender in NEIGHBORS[ROBOT_ID]:
            shared["stop"] = True
            set_debug_rx(shared, "rx STOP {}".format(sender))


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


def wait_for_start():
    last_display = time.ticks_ms() - DISPLAY_PERIOD_MS
    while True:
        now = time.ticks_ms()
        current_position = average_counts(reset=False)

        if time.ticks_diff(now, last_display) >= DISPLAY_PERIOD_MS:
            show_screen(
                "Robot {}".format(ROBOT_ID),
                "Press B start",
                "x={}".format(current_position),
                "rand then sweep",
            )
            last_display = now

        if button_b.check():
            break

        time.sleep_ms(20)

    for remaining in (3, 2, 1):
        show_screen("Robot {}".format(ROBOT_ID), "Start in {}".format(remaining))
        time.sleep_ms(1000)


def run_random_excursion():
    random.seed(time.ticks_ms() + ROBOT_ID * 991)
    signed_ms = random.randint(-RANDOM_MOVE_MAX_MS, RANDOM_MOVE_MAX_MS)

    average_counts(reset=True)
    motion_speed = speed_from_percent(MOTOR_PERCENT)
    motion_speed = motion_speed if signed_ms >= 0 else -motion_speed
    direction = "FWD" if signed_ms >= 0 else "REV"

    show_screen(
        "Random Init",
        "id={}".format(ROBOT_ID),
        "{} {}ms".format(direction, abs(signed_ms)),
    )

    yellow_led.on()
    drive_straight(motion_speed)
    time.sleep_ms(abs(signed_ms))
    stop_drive()
    yellow_led.off()
    time.sleep_ms(250)

    return signed_ms, average_counts(reset=False)


def move_to_position(target_position):
    cruise_speed = speed_from_percent(MOTOR_PERCENT)
    approach_speed = speed_from_percent(APPROACH_PERCENT)
    fine_speed = speed_from_percent(FINE_PERCENT)

    start_position = average_counts(reset=False)
    start_ms = time.ticks_ms()
    last_display = start_ms - DISPLAY_PERIOD_MS
    best_abs_error = abs(target_position - start_position)
    last_progress_ms = start_ms
    stable_hits = 0

    while True:
        current_position = average_counts(reset=False)
        error = target_position - current_position
        abs_error = abs(error)

        if abs_error <= POSITION_TOLERANCE_COUNTS:
            stable_hits += 1
        elif abs_error <= MOVE_SETTLE_TOLERANCE_COUNTS:
            stable_hits += 1
        else:
            stable_hits = 0

        if stable_hits >= 3:
            stop_drive()
            elapsed_ms = time.ticks_diff(time.ticks_ms(), start_ms)
            correction = abs(target_position - start_position)
            return current_position, elapsed_ms, correction

        now = time.ticks_ms()
        if abs_error + STALL_IMPROVEMENT_COUNTS < best_abs_error:
            best_abs_error = abs_error
            last_progress_ms = now

        stalled_ms = time.ticks_diff(now, last_progress_ms)
        if stalled_ms >= STALL_NEAR_TARGET_MS and abs_error <= ALIGN_DONE_TOLERANCE_COUNTS:
            stop_drive()
            elapsed_ms = time.ticks_diff(now, start_ms)
            correction = abs(target_position - start_position)
            return current_position, elapsed_ms, correction

        if stalled_ms >= STALL_TIMEOUT_MS or time.ticks_diff(now, start_ms) >= MOVE_TIMEOUT_MS:
            stop_drive()
            elapsed_ms = time.ticks_diff(now, start_ms)
            correction = abs(target_position - start_position)
            return current_position, elapsed_ms, correction

        if abs_error <= MOVE_SETTLE_TOLERANCE_COUNTS:
            stop_drive()
            if time.ticks_diff(now, last_display) >= DISPLAY_PERIOD_MS:
                show_screen(
                    "Settling",
                    "cur={}".format(current_position),
                    "tar={}".format(target_position),
                    "err={}".format(error),
                )
                last_display = now
            time.sleep_ms(CONTROL_PERIOD_MS)
            continue

        if abs_error > 220:
            speed = cruise_speed
        elif abs_error > 80:
            speed = approach_speed
        else:
            speed = fine_speed

        drive_straight(speed if error > 0 else -speed)

        if time.ticks_diff(now, last_display) >= DISPLAY_PERIOD_MS:
            show_screen(
                "Moving",
                "cur={}".format(current_position),
                "tar={}".format(target_position),
                "err={}".format(error),
            )
            last_display = now

        time.sleep_ms(CONTROL_PERIOD_MS)


def broadcast_position_periodic(shared, last_send):
    now = time.ticks_ms()
    current_position = average_counts(reset=False)
    if time.ticks_diff(now, last_send) >= POS_SEND_MS:
        send_message("POS", ROBOT_ID, current_position)
        return current_position, now
    return current_position, last_send


def force_broadcast_position():
    current_position = average_counts(reset=False)
    send_message("POS", ROBOT_ID, current_position)
    return current_position


def compute_target(local_position, positions):
    if ROBOT_ID == 1:
        if positions.get(2) is None:
            return None
        return rounded_div(local_position + positions[2], 2)

    if ROBOT_ID == 2:
        if positions.get(1) is None or positions.get(3) is None:
            return None
        return rounded_div(positions[1] + local_position + positions[3], 3)

    if ROBOT_ID == 3:
        if positions.get(2) is None or positions.get(4) is None:
            return None
        return rounded_div(positions[2] + local_position + positions[4], 3)

    if ROBOT_ID == 4:
        if positions.get(3) is None or positions.get(5) is None:
            return None
        return rounded_div(positions[3] + local_position + positions[5], 3)

    if positions.get(4) is None:
        return None
    return rounded_div(positions[4] + local_position, 2)


def target_debug(shared):
    local_position = average_counts(reset=False)
    target = compute_target(local_position, shared["positions"])
    if target is None:
        return "tar=--"
    return "tar={}".format(target)


def next_turn_after(current_robot, direction, cycle_max):
    if direction == 1:
        if current_robot < 5:
            return current_robot + 1, 1, cycle_max, False
        return 4, -1, cycle_max, False

    if current_robot > 1:
        return current_robot - 1, -1, cycle_max, False

    if cycle_max <= ITERATION_TOLERANCE_COUNTS:
        return 0, 0, cycle_max, True
    return 2, 1, 0, False


def wait_until_all_ready_and_maybe_start(shared):
    last_ready_send = time.ticks_ms() - CTRL_SEND_MS + ROBOT_ID * SEND_OFFSET_MS
    chain_started = False
    start_token_at = None
    last_display = time.ticks_ms() - DISPLAY_PERIOD_MS

    while True:
        now = time.ticks_ms()
        poll_messages(shared)
        current_position, last_pos = broadcast_position_periodic(shared, shared["last_pos_send"])
        shared["last_pos_send"] = last_pos

        if ROBOT_ID == 5:
            chain_started = True
        elif shared["ready_from_right"]:
            chain_started = True

        if chain_started and ROBOT_ID != 1 and time.ticks_diff(now, last_ready_send) >= CTRL_SEND_MS:
            send_message("READYCHAIN", ROBOT_ID)
            last_ready_send = now

        if ROBOT_ID == 1 and shared["ready_from_right"]:
            if start_token_at is None:
                start_token_at = time.ticks_add(now, SETTLE_AFTER_MOVE_MS)
            if time.ticks_diff(start_token_at, now) <= 0:
                shared["token"] = (1, 1, 1, 0)
                return

        if shared["token"] is not None:
            return

        if time.ticks_diff(now, last_display) >= DISPLAY_PERIOD_MS:
            if ROBOT_ID == 1:
                line2 = "all ready" if shared["ready_from_right"] else "wait 2..5"
            elif ROBOT_ID == 5:
                line2 = "ready chain"
            else:
                line2 = "relay ready" if chain_started else "wait right"

            show_screen(
                "Init Ready",
                shared["debug_rx"],
                line2,
                "x={}".format(current_position),
            )
            last_display = now

        time.sleep_ms(20)


def main_loop():
    shared = {
        "positions": {neighbor: None for neighbor in NEIGHBORS[ROBOT_ID]},
        "ready_from_right": False,
        "token": None,
        "stop": False,
        "acks": {},
        "last_pos_send": time.ticks_ms() - POS_SEND_MS + ROBOT_ID * SEND_OFFSET_MS,
        "debug_rx": "--",
    }
    last_handled_move_id = 0
    forward_token = None
    forward_token_ack_key = None
    last_token_send = time.ticks_ms() - CTRL_SEND_MS
    stop_started_at = None
    last_stop_send = time.ticks_ms() - CTRL_SEND_MS
    last_display = time.ticks_ms() - DISPLAY_PERIOD_MS

    wait_until_all_ready_and_maybe_start(shared)

    while True:
        now = time.ticks_ms()
        poll_messages(shared)
        local_position, last_pos = broadcast_position_periodic(shared, shared["last_pos_send"])
        shared["last_pos_send"] = last_pos

        if shared["stop"]:
            if stop_started_at is None:
                stop_started_at = now
            if time.ticks_diff(now, last_stop_send) >= CTRL_SEND_MS:
                send_message("STOP", ROBOT_ID)
                last_stop_send = now
            if time.ticks_diff(now, last_display) >= DISPLAY_PERIOD_MS:
                show_screen(
                    "DONE",
                    shared["debug_rx"],
                    "robot {}".format(ROBOT_ID),
                    "x={}".format(local_position),
                )
                last_display = now
            if time.ticks_diff(now, stop_started_at) >= STOP_REPEAT_MS:
                while True:
                    time.sleep_ms(100)
            time.sleep_ms(20)
            continue

        if forward_token is not None and forward_token_ack_key is not None:
            if shared["acks"].get(forward_token_ack_key):
                forward_token = None
                forward_token_ack_key = None
            elif time.ticks_diff(now, last_token_send) >= CTRL_SEND_MS:
                send_message(
                    "TOKEN",
                    ROBOT_ID,
                    forward_token[0],
                    forward_token[1],
                    forward_token[2],
                    forward_token[3],
                )
                last_token_send = now

        token = shared["token"]
        if token is not None:
            target_robot, move_id, direction, cycle_max = token

            if target_robot == ROBOT_ID and move_id > last_handled_move_id:
                target_position = compute_target(local_position, shared["positions"])

                if target_position is None:
                    if time.ticks_diff(now, last_display) >= DISPLAY_PERIOD_MS:
                        if ROBOT_ID == 1:
                            need_text = "2={}".format(shared["positions"].get(2))
                        elif ROBOT_ID == 2:
                            need_text = "1={} 3={}".format(shared["positions"].get(1), shared["positions"].get(3))
                        elif ROBOT_ID == 3:
                            need_text = "2={} 4={}".format(shared["positions"].get(2), shared["positions"].get(4))
                        elif ROBOT_ID == 4:
                            need_text = "3={} 5={}".format(shared["positions"].get(3), shared["positions"].get(5))
                        else:
                            need_text = "4={}".format(shared["positions"].get(4))
                        show_screen(
                            "Wait Pos",
                            shared["debug_rx"],
                            "mv={}".format(move_id),
                            need_text[:16],
                            "tar=--",
                        )
                        last_display = now
                    time.sleep_ms(20)
                    continue

                final_position, _, correction = move_to_position(target_position)
                local_position = final_position
                last_handled_move_id = move_id
                if correction > cycle_max:
                    cycle_max = correction

                settle_deadline = time.ticks_add(time.ticks_ms(), SETTLE_AFTER_MOVE_MS)
                settle_last_pos = time.ticks_ms() - POS_SEND_MS
                force_broadcast_position()
                while time.ticks_diff(settle_deadline, time.ticks_ms()) > 0:
                    poll_messages(shared)
                    current_position = average_counts(reset=False)
                    if time.ticks_diff(time.ticks_ms(), settle_last_pos) >= POS_SEND_MS:
                        send_message("POS", ROBOT_ID, current_position)
                        settle_last_pos = time.ticks_ms()
                    show_screen(
                        "Broadcast",
                        shared["debug_rx"],
                        "robot {}".format(ROBOT_ID),
                        "x={}".format(current_position),
                        "tar={}".format(target_position),
                    )
                    time.sleep_ms(20)

                next_robot, next_dir, next_cycle_max, should_stop = next_turn_after(
                    ROBOT_ID,
                    direction,
                    cycle_max,
                )

                if should_stop:
                    shared["stop"] = True
                    stop_started_at = time.ticks_ms()
                    send_message("STOP", ROBOT_ID)
                    last_stop_send = stop_started_at
                    continue

                forward_token = (next_robot, move_id + 1, next_dir, next_cycle_max)
                forward_token_ack_key = (next_robot, move_id + 1)
                shared["acks"].pop(forward_token_ack_key, None)
                last_token_send = time.ticks_ms() - CTRL_SEND_MS
                shared["token"] = forward_token
                continue

        if time.ticks_diff(now, last_display) >= DISPLAY_PERIOD_MS:
            if token is None:
                turn_text = "turn=--"
            else:
                turn_text = "t={}#{}".format(token[0], token[1])

            if forward_token is not None:
                ack_text = "ack->{}".format(forward_token[0])
            else:
                ack_text = "ack=--"

            show_screen(
                "Token Sweep",
                shared["debug_rx"],
                turn_text,
                target_debug(shared),
                ack_text,
            )
            last_display = now

        time.sleep_ms(20)


def main():
    clear_uart_buffer()
    wait_for_start()
    signed_ms, local_position = run_random_excursion()
    clear_uart_buffer()

    show_screen(
        "Init Done",
        "cmd={}ms".format(signed_ms),
        "x={}".format(local_position),
    )
    time.sleep_ms(700)
    main_loop()


main()
