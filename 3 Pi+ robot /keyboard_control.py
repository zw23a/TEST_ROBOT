# pip install keyboard
import serial
import time

# 配置串口
port = '/dev/ttyUSB0'  # 根据你的系统调整端口名称（Windows 通常是 COMx）
baud_rate = 9600

ser = serial.Serial(port, baud_rate)

# 发送控制命令函数
def send_command(command):
    ser.write(command.encode())
    print(f'Sent: {command}')

# 示例：发送一系列控制命令
try:
    while True:
        command = input("Enter command (F: Forward, B: Backward, L: Left, R: Right, S: Stop): ")
        if command in ['F', 'B', 'L', 'R', 'S']:
            send_command(command)
        else:
            print("Invalid command. Please enter F, B, L, R, or S.")
except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()


