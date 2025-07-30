from maix import uart,camera, display, image,time
import math
import numpy as np
import time
from collections import deque

IMAGE_WIDTH=640
IMAGE_HEIGHT=480
cam=camera.Camera(IMAGE_WIDTH,IMAGE_HEIGHT)
cam.skip_frames(30)


devices=uart.list_devices()
myuart = uart.UART(devices[0],115200)
myuart.open()
IMAGE_DIS_MAX=(int)(math.sqrt(IMAGE_WIDTH*IMAGE_WIDTH+IMAGE_HEIGHT*IMAGE_HEIGHT)/2)

ROWS, COLS = 7, 9
START = (6, 8)  # 从(7,9)开始
DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 上 下 左 右
DIRECTION_CODE = {(-1, 0): 1, (0, 1): 2, (1, 0): 3, (0, -1): 4}

class Camus_Map:
    def __init__(self):
        self.no_fly = set()
        self.path = []

    def receive_no_fly(self, buf):
        count = buf[3] // 2 - 1
        self.no_fly = set()
        for i in range(count):
            x = buf[5 + 2 * i] - 1
            y = buf[6 + 2 * i] - 1
            self.no_fly.add((x, y))

    def bfs_path(self, start, end):
        visited = [[False] * COLS for _ in range(ROWS)]
        prev = dict()
        q = deque()
        q.append(start)
        visited[start[0]][start[1]] = True

        while q:
            cx, cy = q.popleft()
            if (cx, cy) == end:
                break
            for dx, dy in DIRS:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < ROWS and 0 <= ny < COLS and (nx, ny) not in self.no_fly and not visited[nx][ny]:
                    visited[nx][ny] = True
                    prev[(nx, ny)] = (cx, cy)
                    q.append((nx, ny))

        if end not in prev and start != end:
            return []

        path = []
        cur = end
        path.append(cur)
        while cur != start:
            cur = prev[cur]
            path.append(cur)
        path.reverse()
        return path

    def generate_snake_targets(self):
        targets = []
        for i in range(ROWS):
            rng = range(COLS) if i % 2 == 0 else reversed(range(COLS))
            for j in rng:
                if (i, j) not in self.no_fly:
                    targets.append((i, j))
        return targets

    def extract_turning_points(self, path):
        if len(path) < 2:
            return path
        result = [path[0]]
        prev_dir = (path[1][0] - path[0][0], path[1][1] - path[0][1])
        for i in range(2, len(path)):
            new_dir = (path[i][0] - path[i-1][0], path[i][1] - path[i-1][1])
            if new_dir != prev_dir:
                result.append(path[i-1])
            prev_dir = new_dir
        result.append(path[-1])
        return result

    def plan_and_send(self):
        global uart_flag
        targets = self.generate_snake_targets()
        current = START
        full_path = []
        for tgt in targets:
            path = self.bfs_path(current, tgt)
            if not path:
                continue
            if full_path and full_path[-1] == path[0]:
                path = path[1:]
            full_path.extend(path)
            current = tgt
        self.path = full_path
        points = self.extract_turning_points(full_path)
        for pt in points:
            target.reserved1 = pt[0] + 1
            target.reserved2 = pt[1] + 1
            target.state = 0
            print(target.reserved1,target.reserved2)
            uart_flag = 1
            while target.state != 1 and ctr.work_mode == 1:
                yield
            
            if ctr.work_mode != 1:
                break

class Camus_Way:
    def __init__(self, path):
        self.path = path
        self.detected = set()

    def run(self):
        global uart_flag
        print(len(self.path))
        for i in range(len(self.path)):
            pt = self.path[i]
            if i > 0:
                prev = self.path[i - 1]
                dx = pt[0] - prev[0]
                dy = pt[1] - prev[1]
                code = DIRECTION_CODE.get((dx, dy), 5)
                target.reserved3 = code
                print(target.reserved3)
                target.state = 0
                uart_flag = 1
                while target.state != 1 and ctr.work_mode == 2:
                    yield

            if pt not in self.detected:
                detect_function(pt[0], pt[1])
                self.detected.add(pt)

        target.reserved3 = 5
        target.state = 0
        while target.state != 1 and ctr.work_mode == 2:
            yield  # 等待主控回应


# 替代实际飞控检测函数
def detect_function(x, y):
    print(f"Detect @ ({x+1},{y+1})")

class target_check(object):
    x=0          #int16_t
    y=0          #int16_t
    pixel=0      #uint16_t
    flag=0       #uint8_t
    state=0      #uint8_t
    angle=0      #int16_t
    distance=0   #uint16_t
    apriltag_id=0#uint16_t
    img_width=0  #uint16_t
    img_height=0 #uint16_t
    reserved1=0  #uint8_t
    reserved2=0  #uint8_t
    reserved3=0  #uint8_t
    reserved4=0  #uint8_t
    fps=0        #uint8_t
    range_sensor1=0
    range_sensor2=0
    range_sensor3=0
    range_sensor4=0
    camera_id=0
    reserved1_u32=0
    reserved2_u32=0
    reserved3_u32=0
    reserved4_u32=0

class uart_buf_prase(object):
    uart_buf = []
    _data_len = 0
    _data_cnt = 0
    state = 0

class mode_ctrl(object):
    work_mode = 0x00 #工作模式.默认是点检测，可以通过串口设置成其他模式
    check_show = 1   #开显示，在线调试时可以打开，离线使用请关闭，可提高计算速度

ctr=mode_ctrl()
if ctr.check_show==1:
    dis=display.Display()
R=uart_buf_prase()
target=target_check()
target.camera_id=0x01
target.reserved1_u32=65536
target.reserved2_u32=105536
target.reserved3_u32=65537
target.reserved4_u32=105537
HEADER=[0xFF,0xFC]
MODE=[0xF1,0xF2,0xF3]
#__________________________________________________________________
def package_blobs_data(mode):
    #数据打包封装
    data=bytearray([HEADER[0],HEADER[1],(0xA0+mode)&0xff,0x00,
                   target.x>>8,target.x&0xff,        #将整形数据拆分成两个8位
                   target.y>>8,target.y&0xff,        #将整形数据拆分成两个8位
                   target.pixel>>8,target.pixel&0xff,#将整形数据拆分成两个8位
                   target.flag,                 #数据有效标志位
                   target.state,                #数据有效标志位
                   target.angle>>8,target.angle&0xff,#将整形数据拆分成两个8位
                   target.distance>>8,target.distance,#将整形数据拆分成两个8位
                   target.apriltag_id>>8,target.apriltag_id,#将整形数据拆分成两个8位
                   target.img_width>>8,target.img_width&0xff,    #将整形数据拆分成两个8位
                   target.img_height>>8,target.img_height&0xff,  #将整形数据拆分成两个8位
                   target.fps,      #数据有效标志位
                   target.reserved1,#数据有效标志位
                   target.reserved2,#数据有效标志位
                   target.reserved3,#数据有效标志位
                   target.reserved4,#数据有效标志位
                   target.range_sensor1>>8,target.range_sensor1&0xff,
                   target.range_sensor2>>8,target.range_sensor2&0xff,
                   target.range_sensor3>>8,target.range_sensor3&0xff,
                   target.range_sensor4>>8,target.range_sensor4&0xff,
                   target.camera_id,
                   target.reserved1_u32>>24&0xff,target.reserved1_u32>>16&0xff,
                   target.reserved1_u32>>8&0xff,target.reserved1_u32&0xff,
                   target.reserved2_u32>>24&0xff,target.reserved2_u32>>16&0xff,
                   target.reserved2_u32>>8&0xff,target.reserved2_u32&0xff,
                   target.reserved3_u32>>24&0xff,target.reserved3_u32>>16&0xff,
                   target.reserved3_u32>>8&0xff,target.reserved3_u32&0xff,
                   target.reserved4_u32>>24&0xff,target.reserved4_u32>>16&0xff,
                   target.reserved4_u32>>8&0xff,target.reserved4_u32&0xff,
                   0x00])
    #数据包的长度
    data_len=len(data)
    #print("serial data length is:",data_len)
    data[3]=data_len-5#有效数据的长度
    #和校验
    sum=0
    for i in range(0,data_len-1):
        sum=sum+data[i]
    data[data_len-1]=(sum&0xff)
    #返回打包好的数据
    return bytes(data)
#__________________________________________________________________
#串口数据解析
def Receive_Anl(data_buf,num):
    global Map_buf
    #和校验
    sum = 0
    i = 0
    while i<(num-1):
        sum = sum + data_buf[i]
        i = i + 1
    sum = sum%256 #求余
    if sum != data_buf[num-1]:
        return
    #和校验通过
    if data_buf[2]==0xA0:
        #设置模块工作模式
        ctr.work_mode = data_buf[4]
        target.state = data_buf[5]
        Map_buf = data_buf
        print(ctr.work_mode)
        print("Set work mode success!")

#__________________________________________________________________
def uart_data_prase(data):
    if data:
        for buf in data:
            if R.state==0 and buf==0xFF:#帧头1
                R.state=1
                R.uart_buf.append(buf)
            elif R.state==1 and buf==0xFC:#帧头2
                R.state=2
                R.uart_buf.append(buf)
            elif R.state==2 and buf<0xFF:#功能字
                R.state=3
                R.uart_buf.append(buf)
            elif R.state==3 and buf<50:#数据长度小于50
                R.state=4
                R._data_len=buf  #有效数据长度
                R._data_cnt=buf+5#总数据长度
                R.uart_buf.append(buf)
            elif R.state==4 and R._data_len>0:#存储对应长度数据
                R._data_len=R._data_len-1
                R.uart_buf.append(buf)
                if R._data_len==0:
                    R.state=5
            elif R.state==5:
                R.uart_buf.append(buf)
                R.state=0
                Receive_Anl(R.uart_buf,R.uart_buf[3]+5)
        #        print(R.uart_buf)
                R.uart_buf=[]#清空缓冲区，准备下次接收数据
            else:
                R.state=0
                R.uart_buf=[]#清空缓冲区，准备下次接收数据
#__________________________________________________________________

def uart_data_read():
    uart_data_prase(myuart.read())

def send_data_via_uart(data):
    for byte in data:
        myuart.write(byte)

camus_map = Camus_Map()
#camus_way = Camus_Way(camus_map.path)
#######生成器王朝了，有没有懂的#####
planner = None
camus_runner = None
##################################

def Handle_Camus_Map(buf):
    global camus_way,planner
    camus_map.receive_no_fly(buf)
    planner = camus_map.plan_and_send()
    #camus_map.plan_and_send()
    #camus_way = Camus_Way(camus_map.path)
    
def Run_Camus_Way():
    camus_way.run()

path_planned = False
uart_flag = 1
ctr.work_mode=0x00
Map_buf = []
print("[INFO]Waiting...")
while True:
    if ctr.work_mode==0x00:#待机模式
        img=cam.read()
    elif ctr.work_mode==0x01:#路线规划模式
        uart_flag = 0
        if not path_planned and Map_buf != b"":  # 只接受一次 buf
            print("Begin Map")
            Handle_Camus_Map(Map_buf)
            Map_buf = []
            path_planned = True
        if planner :
            try:
                next(planner)
            except StopIteration:
                planner = None
                print("Map Send End")
                camus_way = Camus_Way(camus_map.path)
        #Camus_Map(R.uart_buf)
    elif ctr.work_mode==0x02:#路线导航模式
        uart_flag = 0
        if camus_runner is None:
            camus_runner = camus_way.run()
            Run_Camus_Way()
        try:
            next(camus_runner)
        except StopIteration:
            print("[INFO] Navigation Finished.")
            camus_runner = None
        #Run_Camus_Way()
    elif ctr.work_mode==0x03:
        img=cam.read()

    elif ctr.work_mode==0x04:
        img=cam.read()

    elif ctr.work_mode==0x05:
        img=cam.read()

    elif ctr.work_mode==0x06:
        img=cam.read()
 
    elif ctr.work_mode==0x07:
        img=cam.read()

    elif ctr.work_mode==0x0B:
        img=cam.read()
    else:
        pass
    if uart_flag :
        myuart.write(bytes(package_blobs_data(ctr.work_mode)))
    uart_data_read()

