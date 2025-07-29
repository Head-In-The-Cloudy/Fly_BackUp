import serial
import numpy as np

# 摄像头配置
frameWidth = 640
frameHeight = 480
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error opening video stream or file")
cap.set(3, frameWidth)
cap.set(4, frameHeight)
cap.set(10, 150)
ser = serial.Serial('/dev/ttyS0', 256000, timeout=0.01)

mode1 = 0x00

class UartReceiver:
    def __init__(self):
        self.uart_buf = []
        self._data_len = 0
        self._data_cnt = 0
        self.state = 0

    def parse_byte(self, buf):
        if self.state == 0 and buf == 0xFF:
            self.state = 1
            self.uart_buf = [buf]
        elif self.state == 1 and buf == 0xFE:
            self.state = 2
            self.uart_buf.append(buf)
        elif self.state == 2 and buf < 0xFF:
            self.state = 3
            self.uart_buf.append(buf)
        elif self.state == 3 and buf < 50:
            self.state = 4
            self._data_len = buf
            self._data_cnt = buf + 5
            self.uart_buf.append(buf)
        elif self.state == 4 and self._data_len > 0:
            self._data_len -= 1
            self.uart_buf.append(buf)
            if self._data_len == 0:
                self.state = 5
        elif self.state == 5:
            self.uart_buf.append(buf)
            self.state = 0
            self.analyze(self.uart_buf)
            self.uart_buf = []
        else:
            self.state = 0
            self.uart_buf = []

    def analyze(self, data_buf):
        global mode
        # 校验和
        if sum(data_buf[:-1]) % 256 != data_buf[-1]:
            print("[INFO]校验失败")
            return

        if data_buf[2] == 0xA0:
            mode = data_buf[4]
            print(f"[INFO]工作模式：{hex(mode)}")
            Camus_send_data_to_Ardian(ser, 0xA1, mode+1)

UART = UartReceiver()

def Camus_send_data_to_Ardian(ser, command: int, data: list):
    """
    向主控发送数据，帧格式为：
    帧头(0xFF, 0xFE) + 命令(1字节) + 长度(1字节) + 数据(n字节) + 校验和(1字节)
    
    参数:
        ser: 已打开的串口对象（如 serial.Serial(...)）
        command: 命令字节 (int 类型，0x00~0xFF)
        data: 要发送的数据（list 类型，元素为 int，0x00~0xFF）
    """
    if not (0 <= command <= 255):
        raise ValueError("Command must be a byte value (0~255)")
    if any(not (0 <= b <= 255) for b in data):
        raise ValueError("All data elements must be byte values (0~255)")

    header = [0xFF, 0xFE]
    length = len(data)
    packet = header + [command, length] + data

    checksum = sum(packet) & 0xFF  # 校验和为所有前面的字节和的低8位
    packet.append(checksum)

    ser.write(bytearray(packet))  # 发送字节串

# 主循环读取数据
while True:
    if ser.in_waiting:
        data = ser.read(ser.in_waiting)
        for b in data:
            UART.parse_byte(b)
    
    time.sleep(0.01)

cap.release()
cv2.destroyAllWindows() 