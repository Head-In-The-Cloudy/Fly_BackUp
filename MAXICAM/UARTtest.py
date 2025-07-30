import maix
from maix import uart, app, time

device = "/dev/ttyS0"
# ports = uart.list_devices() # 列出当前可用的串口

serial = uart.UART(device, 115200)

cnt = 0
while not app.need_exit():
    string__ = f"hello world {cnt}\n"
    print("send string: ", string__)
    serial.write_str(string__)
    #print("received:", serial.read())
    cnt += 1
    time.sleep_ms(10)