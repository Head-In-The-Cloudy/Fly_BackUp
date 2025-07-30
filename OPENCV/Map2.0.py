import cv2
import numpy as np
import serial
import time
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

    # 画禁飞点
    for x, y in no_fly:
        cv2.rectangle(img, (y*cell_size, x*cell_size), ((y+1)*cell_size, (x+1)*cell_size), (0, 0, 0), -1)

    # 画已检测点
    for x, y in detected:
        cv2.rectangle(img, (y*cell_size, x*cell_size), ((y+1)*cell_size, (x+1)*cell_size), (0, 255, 0), -1)

    # 画路径
    for (x, y) in path:
        cv2.rectangle(img, (y*cell_size, x*cell_size), ((y+1)*cell_size, (x+1)*cell_size), (200, 200, 255), 1)

    # 当前点
    x, y = current
    cv2.rectangle(img, (y*cell_size, x*cell_size), ((y+1)*cell_size, (x+1)*cell_size), (0, 0, 255), -1)

    cv2.imshow("Map", img)
    cv2.waitKey(100)

# ---------- BFS ----------
def bfs_path(start, end, no_fly):
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
            if 0 <= nx < ROWS and 0 <= ny < COLS and (nx, ny) not in no_fly and not visited[nx][ny]:
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

# ---------- 生成蛇形路径 ----------
def generate_snake_targets(no_fly):
    targets = []
    for i in range(ROWS):
        rng = range(COLS) if i % 2 == 0 else reversed(range(COLS))
        for j in rng:
            if (i, j) not in no_fly:
                targets.append((i, j))
    return targets

def get_direction(prev, curr):
    dx = curr[0] - prev[0]
    dy = curr[1] - prev[1]
    if dx == -1: return "UP"
    if dx == 1:  return "DOWN"
    if dy == -1: return "LEFT"
    if dy == 1:  return "RIGHT"
    return "STAY"  # 不动（一般不会出现）

# ---------- 主流程 ----------
def main():
    print("禁飞点数量 n：")
    n = int(input())
    print("禁飞点坐标（格式：x y，每行一个）：")
    no_fly = set()
    for _ in range(n):
        x, y = map(int, input().split())
        no_fly.add((x - 1, y - 1))

    targets = generate_snake_targets(no_fly)
    if not targets:
        print("无可用路径")
        return

    current = START
    full_path = []

    for target in targets:
        subpath = bfs_path(current, target, no_fly)
        if not subpath:
            print(f"无法从 {current} 到 {target}")
            return
        if full_path and full_path[-1] == subpath[0]:
            subpath = subpath[1:]
        full_path.extend(subpath)
        current = target
    # ser = init_serial()
    detected = set()

    for i in range(len(full_path)):
        pt = full_path[i]

        # 可视化
        draw_map((ROWS, COLS), full_path, pt, detected, no_fly)

        if i > 0:
            prev = full_path[i - 1]
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
