from collections import deque

ROWS, COLS = 7, 9
START = (6, 8)  # 0-indexed of (7,9)

# 四邻方向
DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)]

def input_no_fly_points():
    print("禁飞点数量 n：")
    n = int(input())
    print("禁飞点坐标（格式：x y，每行一个）：")
    no_fly = set()
    for _ in range(n):
        x, y = map(int, input().split())
        no_fly.add((x - 1, y - 1))
    return no_fly

def is_valid(x, y, no_fly, visited):
    return 0 <= x < ROWS and 0 <= y < COLS and (x, y) not in no_fly and not visited[x][y]

# 用 BFS 找两点之间一条合法路径
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
            if is_valid(nx, ny, no_fly, visited):
                visited[nx][ny] = True
                prev[(nx, ny)] = (cx, cy)
                q.append((nx, ny))

    # 回溯路径
    if end not in prev and start != end:
        return []  # 无法连接
    path = []
    cur = end
    path.append(cur)
    while cur != start:
        cur = prev[cur]
        path.append(cur)
    path.reverse()
    return path

# 构建蛇形目标顺序
def generate_snake_targets(no_fly):
    targets = []
    for i in range(ROWS):
        rng = range(COLS) if i % 2 == 0 else reversed(range(COLS))
        for j in rng:
            if (i, j) not in no_fly:
                targets.append((i, j))
    return targets

# 提取转角点
def extract_turning_points(path):
    if not path:
        return []

    turning_points = [path[0]]

    def direction(a, b):
        return (b[0] - a[0], b[1] - a[1])

    for i in range(1, len(path) - 1):
        dir1 = direction(path[i - 1], path[i])
        dir2 = direction(path[i], path[i + 1])
        if dir1 != dir2:
            turning_points.append(path[i])

    turning_points.append(path[-1])
    return turning_points

# 输出结果
def print_result(points):
    print("\n路线关键点（起点 + 转角点 + 终点）：")
    for x, y in points:
        print(f"{x + 1} {y + 1}")

# 主程序
def main():
    no_fly = input_no_fly_points()

    snake_targets = generate_snake_targets(no_fly)
    if not snake_targets:
        print("无可行目标点")
        return

    full_path = []
    current = START
    for target in snake_targets:
        subpath = bfs_path(current, target, no_fly)
        if not subpath:
            print(f"从 {current} 无法到达 {target}，路径中断")
            return
        if full_path and full_path[-1] == subpath[0]:
            subpath = subpath[1:]  # 避免重复点
        full_path.extend(subpath)
        current = target

    ##################################Detect#################################
    detected = set()
    for point in full_path:
        if point not in detected:
            print(f"detect {point[0] + 1} {point[1] + 1}")
            detected.add(point)

    ##########################################################################
    turning_points = extract_turning_points(full_path)
    print_result(turning_points)


if __name__ == "__main__":
    main()
