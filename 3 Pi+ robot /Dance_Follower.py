from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
from machine import UART, Pin
import time

display = robot.Display()
motors = robot.Motors()
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

turn_speed = 1000           # 原地转向速度（摆头/转圈用）
SWIVEL_HALF_PERIOD = 400    # 摆头每 0.4s 切换一次左右
POLL_DT = 50  
def initialize_uart():
    # Leader TX -> RX(29), 共地；波特率需一致
    uart = UART(0, baudrate=9600, tx=Pin(28), rx=Pin(29))
    return uart



def read_command(uart):
    if uart.any():
        command = uart.read(100)  # Adjust the number of bytes as needed
        return command
    return None
uart = initialize_uart()
display.fill(0)
display.text("Receiver Ready", 0, 0)
display.show()
time.sleep(0.5)

def run_for(ms):
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < ms:
        
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
        
        time.sleep_ms(POLL_DT)

_last_cmd = None  # 防止重复刷屏
CMD_FWD_3S        = b'x01'
CMD_STOP_1S       = b'x02'
CMD_BACK_3S       = b'x03'
CMD_FWD_1P5S      = b'x04'
CMD_SWIVEL_5S     = b'x05'
CMD_SPIN_LEFT_2P5 = b'x06'
CMD_SPIN_RIGHT_2P5= b'x07'
CMD_BACK_1P5S     = b'x08'
CMD_LOOP_END      = b'x09'   # 可选：循环结束/将重启

while True:
    command = read_command(uart)

    if command is None:
        # 没有新命令就小睡一会，降低 CPU 占用
        time.sleep_ms(5)
        continue

    # 只有在命令变化时才更新 UI/动作，避免重复刷新
    if command == _last_cmd:
        continue
    _last_cmd = command

    if command == CMD_FWD_3S:
        # straight
        yellow_led.on()
        display.fill(0)
        display.text("Straight", 0, 0)
        display.show()
        motors.set_speeds(max_speed, max_speed)
        run_for(3000)

    elif command == CMD_STOP_1S:
        # stop
        yellow_led.off()
        display.fill(0)
        display.text("Stop", 0, 0)
        display.show()
        motors.set_speeds(0, 0)
        run_for(1000)

    elif command == CMD_BACK_3S:
        # right turn (in place)
        yellow_led.on()
        display.fill(0)
        display.text("Right", 0, 0)
        display.show()
        motors.set_speeds(-max_speed, -max_speed)
        run_for(3000)

    elif command == CMD_SWIVEL_5S:

        yellow_led.on()
        display.fill(0)
        display.text("Swivel",0,0)
        display.show()
        swivel_for(5000)
    
    elif command == CMD_SPIN_RIGHT_2P5:
        yellow_led.on()
        display.fill(0)
        display.text("Spin_Right",0,0)
        display.show()
        motors.set_speeds(turn_speed,-turn_speed)
        run_for(2500)
    
    elif command == CMD_SPIN_LEFT_2P5:
        yellow_led.on()
        display.fill(0)
        display.text("Spin_Left",0,0)
        display.show()
        motors.set_speeds(-turn_speed,turn_speed)
        run_for(2500)

    elif command == CMD_LOOP_END:
        motors.set_speeds(0,0)
        run_for(1000)

