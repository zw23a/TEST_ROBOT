# controller_motion_led_loop_leader.py
# 动作循环 + 仅 LED 动画（无音乐）+ UART cmd（leader）
# 循环结束后按 B 重新开始

from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
from machine import UART, Pin
import time, math

# --- Peripherals ---
display = robot.Display()
motors = robot.Motors()
yellow_led = robot.YellowLED()
rgb_leds = robot.RGBLEDs()
edition = editions.select()

# Buttons (defensive: support multiple APIs)
try:
    button_b = robot.ButtonB()
except Exception:
    button_b = None

def b_pressed():
    """Return True if B is currently pressed (best-effort across APIs)."""
    if button_b is None:
        return False
    try:
        return bool(button_b.is_pressed())
    except AttributeError:
        pass
    try:
        return bool(button_b.check())
    except AttributeError:
        pass
    try:
        # Some variants expose .value() with 0/1 logic; pressed might be False/0
        return not bool(button_b.value())  # invert if pull-up
    except Exception:
        return False

# --- Speed profile by edition ---
if edition == "Standard":
    max_speed, turn_time = 1500, 250
elif edition == "Turtle":
    max_speed, turn_time = 3000, 500
elif edition == "Hyper":
    max_speed, turn_time = 1125, 150
    motors.flip_left(True); motors.flip_right(True)

# 可调参数
turn_speed = 1000           # 原地转向速度（摆头/转圈用）
SWIVEL_HALF_PERIOD = 500    # 摆头每 0.4s 切换一次左右
POLL_DT = 50                # 轮询步长 50ms

# --- UART (leader broadcast) ---
def init_uart():
    return UART(0, baudrate=9600, tx=Pin(28), rx=Pin(29))
def send(uart, cmd: bytes):
    uart.write(cmd)
uart = init_uart()

# 每个动作一个专用 cmd（可按需与从机协商）
CMD_FWD_3S        = b'x01'
CMD_STOP_1S       = b'x02'
CMD_BACK_3S       = b'x03'
CMD_FWD_1P5S      = b'x04'
CMD_SWIVEL_5S     = b'x05'
CMD_SPIN_LEFT_2P5 = b'x06'
CMD_SPIN_RIGHT_2P5= b'x07'
CMD_BACK_1P5S     = b'x08'
CMD_LOOP_END      = b'x09'   # 可选：循环结束/将重启

# --- LED-only animation (no music) ---
rgb_leds.set_brightness(5)

def led_anim_step():
    """彩虹 + 呼吸亮度效果；需在循环中频繁调用。"""
    t = time.ticks_ms()
    base_hue = (t // 10) % 360
    breath = 0.5 + 0.5 * math.sin(t / 500.0 * 2 * math.pi)
    value = int(60 + 195 * breath)          # 60~255
    sat = 255
    for i in range(6):
        hue = (base_hue + i * 20) % 360
        rgb_leds.set_hsv(i, [hue, sat, value])
    rgb_leds.show()

def show(line1, line2=""):
    display.fill(0); display.text(line1, 0, 0)
    if line2: display.text(line2, 0, 12)
    display.show()

def run_for(ms):
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < ms:
        led_anim_step()
        time.sleep_ms(POLL_DT)

def swivel_for(ms):
    """左右摆头（原地交替左/右转）持续 ms 毫秒。"""
    t0 = time.ticks_ms()
    last = t0
    right = True
    motors.set_speeds(turn_speed, -turn_speed) if right else motors.set_speeds(-turn_speed, turn_speed)
    while time.ticks_diff(time.ticks_ms(), t0) < ms:
        now = time.ticks_ms()
        if time.ticks_diff(now, last) >= SWIVEL_HALF_PERIOD:
            right = not right
            if right:
                motors.set_speeds(turn_speed, -turn_speed)
            else:
                motors.set_speeds(-turn_speed, turn_speed)
            last = now
        led_anim_step()
        time.sleep_ms(POLL_DT)

def wait_for_button_b():
    """结束后提示并等待按下 B 以重启；包含简易消抖。"""
    show("Done", "Press B to restart")
    # 确保先松开
    while b_pressed():
        led_anim_step(); time.sleep_ms(20)
    # 等待按下
    while not b_pressed():
        led_anim_step(); time.sleep_ms(30)
    # 等待释放（消抖）
    time.sleep_ms(150)
    while b_pressed():
        led_anim_step(); time.sleep_ms(20)

def run_sequence():
    """原有的两次动作序列。"""
    for i in range(2):
        # 前进 3s
        yellow_led.on()
        motors.set_speeds(max_speed, max_speed)
        show("Forward 3s")
        send(uart, CMD_FWD_3S); time.sleep_ms(50)
        send(uart, CMD_FWD_3S); time.sleep_ms(50)
        send(uart, CMD_FWD_3S); time.sleep_ms(50)
        run_for(3000)

        # 停 1s
        yellow_led.off()
        motors.set_speeds(0, 0)
        show("Stop 1s")
        send(uart, CMD_STOP_1S); time.sleep_ms(50)
        send(uart, CMD_STOP_1S); time.sleep_ms(50)
        send(uart, CMD_STOP_1S); time.sleep_ms(50)
        run_for(1000)

        # 后退 3s
        motors.set_speeds(-max_speed, -max_speed)
        show("Back 3s")
        send(uart, CMD_BACK_3S); time.sleep_ms(50)
        send(uart, CMD_BACK_3S); time.sleep_ms(50)
        send(uart, CMD_BACK_3S); time.sleep_ms(50)
        run_for(3000)

        # 停 1s
        motors.set_speeds(0, 0)
        show("Stop 1s")
        send(uart, CMD_STOP_1S); time.sleep_ms(50)
        send(uart, CMD_STOP_1S); time.sleep_ms(50)
        send(uart, CMD_STOP_1S); time.sleep_ms(50)
        run_for(1000)

        # 前进 1.5s
        motors.set_speeds(max_speed, max_speed)
        show("Forward 1.5s")
        send(uart, CMD_FWD_1P5S); time.sleep_ms(50)
        send(uart, CMD_FWD_1P5S); time.sleep_ms(50)
        send(uart, CMD_FWD_1P5S); time.sleep_ms(50)
        run_for(1500)

        # 停 1s
        motors.set_speeds(0, 0)
        show("Stop 1s")
        send(uart, CMD_STOP_1S); time.sleep_ms(50)
        send(uart, CMD_STOP_1S); time.sleep_ms(50)
        send(uart, CMD_STOP_1S); time.sleep_ms(50)
        run_for(1000)

        # 左右摆头 5s（原地）
        show("Swivel 5s", "L<->R in place")
        send(uart, CMD_SWIVEL_5S); time.sleep_ms(10)
        send(uart, CMD_SWIVEL_5S); time.sleep_ms(10)
        send(uart, CMD_SWIVEL_5S); time.sleep_ms(10)
        swivel_for(5000)

        # 停 1s
        motors.set_speeds(0, 0)
        show("Stop 1s")
        send(uart, CMD_STOP_1S); time.sleep_ms(50)
        send(uart, CMD_STOP_1S); time.sleep_ms(50)
        send(uart, CMD_STOP_1S); time.sleep_ms(50)
        run_for(1000)

        # 后退 1.5s（回到原点）
        motors.set_speeds(-max_speed, -max_speed)
        show("Back 1.5s", "Return to origin")
        send(uart, CMD_BACK_1P5S); time.sleep_ms(50)
        send(uart, CMD_BACK_1P5S); time.sleep_ms(50)
        send(uart, CMD_BACK_1P5S); time.sleep_ms(50)
        run_for(1500)

        # 停 1s（循环间隔/标记）
        motors.set_speeds(0, 0)
        show("Stop 1s", "Loop again")
        send(uart, CMD_LOOP_END); time.sleep_ms(50)
        send(uart, CMD_LOOP_END); time.sleep_ms(50)
        send(uart, CMD_LOOP_END); time.sleep_ms(50)
        run_for(1000)

    # 保险：最终停下
    motors.set_speeds(0, 0)
    yellow_led.off()

# --- Boot UI ---
show("Starting...")
time.sleep_ms(300)

# --- Run once, then wait for B to restart indefinitely ---
while True:
    run_sequence()
    wait_for_button_b()   # 显示“Press B to restart”，按下 B 后再次运行
