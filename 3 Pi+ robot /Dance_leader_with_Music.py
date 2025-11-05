# controller_music_motion_loop_leader.py
# 动作循环 + 背景音乐不断 + 每个动作都有独立 UART cmd（leader）

from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
from machine import UART, Pin
import time

# --- Peripherals ---
display = robot.Display()
motors = robot.Motors()
yellow_led = robot.YellowLED()
rgb_leds = robot.RGBLEDs()
buzzer = robot.Buzzer()
edition = editions.select()

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
SWIVEL_HALF_PERIOD = 400    # 摆头每 0.4s 切换一次左右
POLL_DT = 50                # 轮询步长 50ms，保证音乐续播

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

# --- Music (same score) ---
rgb_leds.set_brightness(5)
intro = "t140 l8 ms v15 O4 rd+d+d+ O6 d+d+d+ " + \
        "O4 d+d+d+ t120 v13 O6 d+d+d+ " + \
        "O4 t100 v12 d+ O6 d+d+ t90 v10 O4 d+ O6 d+d+ rrr"
song = "t108 l16 ms v15 " + \
    "O5 ms l16 r4 dO5d+d+>d+ d+>d+c+>d+ " + \
    "v15 mlO3g+32>d+32 v12 O5ms>d+b>d+a+>d+ v15 mlO3b32>g+32 v12 msO5>d+g>d+g+>d+" + \
    "v15 mlO4 d+32a+32 v12 mso5>d+d+>d+d+>d+ v15 mlO4g32a+32 v12 msO5>d+d+>d+c+>d+" + \
    "v14 mlO4g+32b32 v11 msO5>d+b>d+a+>d+ v15 mlO3b32>f32 v11 msO5>d+g>d+g+>d+" + \
    "v14 mlO4d+32g32 v11 msO5>d+<d+d+d+>d+ v13 d+>d+d+>d+c+>d+" + \
    "v13 mlO3g+32>d+32 v10 msO5>d+b>d+a+>d+ v13 mlO3b32>g+32 v10 msO5 >d+g>d+g+>d+" + \
    "v13 mlO4d+32a+32 v10 msO5>d+d+>d+d+>d+ v13 mlO4g32a+32 v10 msO5>d+d+>d+c+>d+" + \
    "v12 mlO4g+32b32 v9 msO5>d+g+b>d+>d+ v12 mlO4d+32g32 v9 msO5>d+ga+>d+>d+" + \
    "v15 t100 ml O4 <g+d+>b>g+ t95 d+32r32>d+32r32 d+4"

def music_callback(i: int):
    value = min(255, buzzer.volumes[i]*2)
    hue = (buzzer.notes[i] - 48) * 360 // 36
    sat = 255 - int(buzzer.durations[i]/4)
    sat = 0 if sat < 0 else (255 if sat > 255 else sat)
    for led in range(6):
        rgb_leds.set_hsv(led, [hue, sat, value])
    rgb_leds.show()
buzzer.set_callback(music_callback)

_phase = "intro"
def start_music():
    global _phase
    _phase = "intro"
    buzzer.play_in_background(intro)

def keep_music_playing():
    global _phase
    if buzzer.is_playing():
        return
    if _phase == "intro":
        _phase = "song"
        buzzer.play_in_background(song)
    else:
        buzzer.play_in_background(song)

def show(line1, line2=""):
    display.fill(0); display.text(line1, 0, 0)
    if line2: display.text(line2, 0, 12)
    display.show()

def run_for(ms):
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < ms:
        keep_music_playing()
        time.sleep_ms(POLL_DT)

def swivel_for(ms):
    """左右摆头（原地交替左/右转）持续 ms 毫秒，仅在开始时发送高层 cmd。"""
    t0 = time.ticks_ms()
    last = t0
    right = True
    # 初始方向
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
        keep_music_playing()
        time.sleep_ms(POLL_DT)

# --- Boot UI + music ---
show("Starting...", "Music on")
time.sleep_ms(300)
start_music()
time.sleep_ms(100)

# --- Infinite motion loop ---
while True:
    # 前进 3s
    yellow_led.on()
    show("Forward 3s")
    send(uart, CMD_FWD_3S)                 # leader cmd
    motors.set_speeds(max_speed, max_speed)
    run_for(3000)

    # 停 1s
    yellow_led.off()
    show("Stop 1s")
    send(uart, CMD_STOP_1S)
    motors.set_speeds(0, 0)
    run_for(1000)

    # 后退 3s
    show("Back 3s")
    send(uart, CMD_BACK_3S)
    motors.set_speeds(-max_speed, -max_speed)
    run_for(3000)

    # 停 1s
    show("Stop 1s")
    send(uart, CMD_STOP_1S)
    motors.set_speeds(0, 0)
    run_for(1000)

    # 前进 1.5s
    show("Forward 1.5s")
    send(uart, CMD_FWD_1P5S)
    motors.set_speeds(max_speed, max_speed)
    run_for(1500)

    # 停 1s
    show("Stop 1s")
    send(uart, CMD_STOP_1S)
    motors.set_speeds(0, 0)
    run_for(1000)

    # 左右摆头 5s（原地）
    show("Swivel 5s", "L<->R in place")
    send(uart, CMD_SWIVEL_5S)
    swivel_for(5000)

    # 停 1s
    show("Stop 1s")
    send(uart, CMD_STOP_1S)
    motors.set_speeds(0, 0)
    run_for(1000)

    # 原地左转 2.5s（转圈）
    show("Spin Left 2.5s")
    send(uart, CMD_SPIN_LEFT_2P5)
    motors.set_speeds(-turn_speed, turn_speed)  # 左转圈
    run_for(2500)

    # 停 1s
    show("Stop 1s")
    send(uart, CMD_STOP_1S)
    motors.set_speeds(0, 0)
    run_for(1000)

    # 原地右转 2.5s（转圈）
    show("Spin Right 2.5s")
    send(uart, CMD_SPIN_RIGHT_2P5)
    motors.set_speeds(turn_speed, -turn_speed)  # 右转圈
    run_for(2500)

    # 停 1s
    show("Stop 1s")
    send(uart, CMD_STOP_1S)
    motors.set_speeds(0, 0)
    run_for(1000)

    # 后退 1.5s（回到原点）
    show("Back 1.5s", "Return to origin")
    send(uart, CMD_BACK_1P5S)
    motors.set_speeds(-max_speed, -max_speed)
    run_for(1500)

    # 停 1s（循环间隔/标记）
    show("Stop 1s", "Loop again")
    send(uart, CMD_LOOP_END)   # 可选：告知从机一次序列结束
    motors.set_speeds(0, 0)
    run_for(1000)
