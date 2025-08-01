import cv2
import numpy as np
import serial
from collections import deque

# 地图参数
ROWS, COLS = 7, 9
START = (6, 8)  # (7,9)

DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)]

# ---------- 串口配置 ----------
def init_serial():
    return serial.Serial('COM22', 115200, timeout=1)

def send_direction(ser, direction):
    msg = f"{direction}\n"
    ser.write(msg.encode())

def wait_for_ok(ser):
    while True:
        line = ser.readline().decode().strip()
        if line == "OK":
            break

# ---------- 检测函数 ----------
def detect_function(x, y):
    print(f"检测：({x + 1}, {y + 1})")

# ---------- 画图 ----------
def draw_map(grid_size, path, current, detected, no_fly):
    cell_size = 60
    img = np.ones((grid_size[0]*cell_size, grid_size[1]*cell_size, 3), dtype=np.uint8) * 255

    for x, y in no_fly:
        cv2.rectangle(img, (y*cell_size, x*cell_size), ((y+1)*cell_size, (x+1)*cell_size), (0, 0, 0), -1)

    for x, y in detected:
        cv2.rectangle(img, (y*cell_size, x*cell_size), ((y+1)*cell_size, (x+1)*cell_size), (0, 255, 0), -1)

    for (x, y) in path:
        cv2.rectangle(img, (y*cell_size, x*cell_size), ((y+1)*cell_size, (x+1)*cell_size), (200, 200, 255), 1)

    x, y = current
    cv2.rectangle(img, (y*cell_size, x*cell_size), ((y+1)*cell_size, (x+1)*cell_size), (0, 0, 255), -1)

    cv2.imshow("Map", img)
    cv2.waitKey(100)

# ---------- 回形 DFS 路线生成 ----------
dif_up = (-1, 0)
dif_left = (0, -1)
dif_down = (1, 0)
dif_right = (0, 1)
Priority_Camus =[dif_right,dif_up,dif_left,dif_down]
print(Priority_Camus)
def generate_spiral_path(start, no_fly):
    visited = [[False]*(COLS+2) for _ in range(ROWS+2)]
    for x, y in no_fly:
        visited[x][y] = True

    # 设置边界为已访问
    for x in range(ROWS+2):
        visited[x][0] = visited[x][COLS+1] = True
    for y in range(COLS+2):
        visited[0][y] = visited[ROWS+1][y] = True

    total_to_visit = ROWS * COLS - len(no_fly)
    path = []

    def in_bounds(x, y):
        return 1 <= x <= ROWS and 1 <= y <= COLS and not visited[x][y]

    # 回形优先策略：每次从当前位置判断最合适的方向
    def get_priority(x, y):
        if visited[x-1][y] and visited[x][y+1]:
            return [dif_left, dif_up, dif_right, dif_down]  # 右上角
        if visited[x-1][y] and visited[x][y-1]:
            return [dif_left, dif_down, dif_up, dif_right]  # 左上角
        if visited[x+1][y] and visited[x][y-1]:
            return [dif_right, dif_down, dif_left, dif_up]  # 左下角
        if visited[x+1][y] and visited[x][y+1]:
            return [dif_right, dif_up, dif_down, dif_left]  # 右下角
        return Priority_Camus  # 默认优先级

    def dfs(x, y):
        if not in_bounds(x, y):
            return False
        visited[x][y] = True
        path.append((x, y))

        if len(path) == total_to_visit:
            return True  # 成功完成所有格点访问

        for dx, dy in get_priority(x, y):
            if dfs(x + dx, y + dy):
                return True  # 成功找到完整路径

        # 回溯
        visited[x][y] = False
        path.pop()
        return False

    success = dfs(start[0]+1, start[1]+1)
    if not success:
        return []  # 返回空路径表示失败
    return path

def get_direction(prev, curr):
    dx = curr[0] - prev[0]
    dy = curr[1] - prev[1]
    if dx == -1: return "UP"
    if dx == 1:  return "DOWN"
    if dy == -1: return "LEFT"
    if dy == 1:  return "RIGHT"
    return "STAY"

# ---------- 主流程 ----------
def main():
    print("禁飞点数量 n：")
    n = int(input())
    print("禁飞点坐标（格式：x y，每行一个）：")
    no_fly = set()
    for _ in range(n):
        x, y = map(int, input().split())
        no_fly.add((x, y))

    path = generate_spiral_path(START, no_fly)
    if not path:
        print("无可用路径")
        return

    # ser = init_serial()
    detected = set()

    for i in range(len(path)):
        pt = path[i]
        draw_map((ROWS+1, COLS+1), path, pt, detected, no_fly)

        if i > 0:
            prev = path[i - 1]
            direction = get_direction(prev, pt)
            print(f"飞控指令: {direction}")
            # send_direction(ser, direction)
            # wait_for_ok(ser)

        if pt not in detected:
            detect_function(pt[0], pt[1])
            detected.add(pt)

    print("所有点已检测完毕")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
