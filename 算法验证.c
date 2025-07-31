#include <stdio.h>
#include <stdbool.h>

#define ROWS 7 // 行数，B1-B7
#define COLS 9 // 列数，A1-A9

// 方向常量：上、右、下、左（行索引0为底部，向上增加）
const int dr[4] = {1, 0, -1, 0}; // 行偏移：上(+1)、下(-1)
const int dc[4] = {0, 1, 0, -1}; // 列偏移：右(+1)、左(-1)

// 计算从起点可达的方格数量（BFS）
int count_reachable(int grid[ROWS][COLS], int start_row, int start_col, bool marked[ROWS][COLS])
{
    bool visited[ROWS][COLS] = {{false}};
    int queue[ROWS * COLS][2]; // 队列存储(row, col)
    int front = 0, rear = 0;
    int count = 0;

    // 初始化起点
    queue[rear][0] = start_row;
    queue[rear][1] = start_col;
    rear++;
    visited[start_row][start_col] = true;
    marked[start_row][start_col] = true;
    count++;

    while (front < rear)
    {
        int r = queue[front][0];
        int c = queue[front][1];
        front++;

        // 检查四个方向
        for (int d = 0; d < 4; d++)
        {
            int nr = r + dr[d];
            int nc = c + dc[d];
            // 检查边界和状态
            if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS && !grid[nr][nc] && !visited[nr][nc])
            {
                visited[nr][nc] = true;
                marked[nr][nc] = true;
                queue[rear][0] = nr;
                queue[rear][1] = nc;
                rear++;
                count++;
            }
        }
    }
    return count;
}

// 主函数：找到遍历路径
void find_path(int grid[ROWS][COLS])
{
    bool visited[ROWS][COLS] = {{false}}; // 访问标记
    bool marked[ROWS][COLS] = {{false}};  // 可达标记
    int path[ROWS * COLS][2];             // 存储路径，每个元素为(row, col)
    int path_length = 0;

    // 起点 (B1, A9) = (0,8)
    int start_row = 0;
    int start_col = 8;
    int curr_row = start_row;
    int curr_col = start_col;
    int curr_dir = -1; // 当前方向：-1表示无方向

    // 计算所有可达方格
    int reachable_count = count_reachable(grid, start_row, start_col, marked);
    if (reachable_count == 0)
    {
        printf("No reachable cells from start point.\n");
        return;
    }

    // 初始化起点
    visited[curr_row][curr_col] = true;
    path[path_length][0] = curr_row;
    path[path_length][1] = curr_col;
    path_length++;

    // 终点集合 (B1,A7)=(0,6), (B3,A9)=(2,8), (B2,A8)=(1,7)
    int end_points[3][2] = {{0, 6}, {2, 8}, {1, 7}};
    bool at_end_point = false;

    // 主循环：遍历所有可达方格
    int steps = 0;
    int max_steps = reachable_count * 10; // 防止无限循环
    while (path_length < reachable_count && steps < max_steps)
    {
        steps++;
        at_end_point = false;

        // 检查当前位置是否为终点
        for (int i = 0; i < 3; i++)
        {
            if (curr_row == end_points[i][0] && curr_col == end_points[i][1])
            {
                at_end_point = true;
                break;
            }
        }

        // 如果当前方向有效，尝试继续直线移动
        if (curr_dir != -1)
        {
            int nr = curr_row + dr[curr_dir];
            int nc = curr_col + dc[curr_dir];
            // 如果新位置有效且未访问，则继续移动
            if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS && !grid[nr][nc] && !visited[nr][nc] && marked[nr][nc])
            {
                curr_row = nr;
                curr_col = nc;
                visited[curr_row][curr_col] = true;
                path[path_length][0] = curr_row;
                path[path_length][1] = curr_col;
                path_length++;
                continue; // 保持方向继续
            }
            else
            {
                curr_dir = -1; // 无法继续，重置方向
            }
        }

        // 无当前方向或无法继续：选择新方向
        if (curr_dir == -1)
        {
            int best_dir = -1;
            int best_steps = 0; // 最佳方向能移动的步数

            // 检查所有四个方向
            for (int d = 0; d < 4; d++)
            {
                int nr = curr_row + dr[d];
                int nc = curr_col + dc[d];
                if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS && !grid[nr][nc] && !visited[nr][nc] && marked[nr][nc])
                {
                    // 计算此方向能直线移动的步数
                    int steps_available = 0;
                    int cr = nr;
                    int cc = nc;
                    while (1)
                    {
                        if (cr >= 0 && cr < ROWS && cc >= 0 && cc < COLS && !grid[cr][cc] && !visited[cr][cc] && marked[cr][cc])
                        {
                            steps_available++;
                        }
                        else
                        {
                            break;
                        }
                        cr += dr[d];
                        cc += dc[d];
                    }
                    // 选择能移动步数最多的方向（增加直线遍历）
                    if (steps_available > best_steps)
                    {
                        best_steps = steps_available;
                        best_dir = d;
                    }
                }
            }

            // 找到有效方向，移动
            if (best_dir != -1)
            {
                curr_dir = best_dir;
                int nr = curr_row + dr[curr_dir];
                int nc = curr_col + dc[curr_dir];
                curr_row = nr;
                curr_col = nc;
                visited[curr_row][curr_col] = true;
                path[path_length][0] = curr_row;
                path[path_length][1] = curr_col;
                path_length++;
                continue;
            }
            else
            {
                // 无未访问邻居：回溯到最近有未访问邻居的路径点
                bool found = false;
                for (int i = path_length - 2; i >= 0; i--)
                {
                    int r = path[i][0];
                    int c = path[i][1];
                    for (int d = 0; d < 4; d++)
                    {
                        int nr = r + dr[d];
                        int nc = c + dc[d];
                        if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS && !grid[nr][nc] && !visited[nr][nc] && marked[nr][nc])
                        {
                            curr_row = r;
                            curr_col = c;
                            curr_dir = -1;
                            found = true;
                            break;
                        }
                    }
                    if (found)
                        break;
                }
                if (!found)
                    break; // 无法回溯，退出
            }
        }
    }

    // 确保停在终点：如果不在终点，移动到最近终点
    at_end_point = false;
    for (int i = 0; i < 3; i++)
    {
        if (curr_row == end_points[i][0] && curr_col == end_points[i][1])
        {
            at_end_point = true;
            break;
        }
    }
    if (!at_end_point)
    {
        // 选择最近终点（简化：移动到第一个可达终点）
        for (int i = 0; i < 3; i++)
        {
            int er = end_points[i][0];
            int ec = end_points[i][1];
            if (marked[er][ec])
            {
                // 直接移动到终点（可能引入重复访问，但优先结束）
                curr_row = er;
                curr_col = ec;
                if (!visited[curr_row][curr_col])
                {
                    visited[curr_row][curr_col] = true;
                    path[path_length][0] = curr_row;
                    path[path_length][1] = curr_col;
                    path_length++;
                }
                break;
            }
        }
    }
    else
    {
        // 已在终点，确保路径记录
        path[path_length - 1][0] = curr_row;
        path[path_length - 1][1] = curr_col;
    }

    // 输出路径（转换为用户坐标：行B1=1, B7=7；列A1=1, A9=9）
    printf("Traversal path (row B, column A):\n");
    for (int i = 0; i < path_length; i++)
    {
        int user_row = path[i][0] + 1; // 行索引0->B1, 所以+1
        int user_col = path[i][1] + 1; // 列索引0->A1, 所以+1
        printf("(B%d, A%d)", user_row, user_col);
        if (i < path_length - 1)
            printf(" -> ");
        if ((i + 1) % 5 == 0)
            printf("\n"); // 每5个换行
    }
    printf("\nEnd at (B%d, A%d)\n", curr_row + 1, curr_col + 1);
}

// 示例使用
int main()
{
    // 示例网格：0=允许, 1=禁止（需用户定义）
    int grid[ROWS][COLS] = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0}, // B1 行
        {0, 0, 0, 0, 0, 0, 0, 0, 0}, // B2 行
        {0, 0, 0, 0, 0, 0, 0, 0, 0}, // B3 行
        {0, 0, 0, 0, 0, 0, 0, 0, 0}, // B4 行
        {0, 0, 0, 0, 0, 0, 0, 0, 0}, // B5 行
        {0, 0, 0, 0, 0, 0, 0, 0, 0}, // B6 行
        {0, 0, 0, 0, 0, 0, 0, 0, 0}  // B7 行
    };
    // 设置一些连续禁止方格（示例：L形）
    grid[2][2] = 1; // 禁止
    grid[2][3] = 1;
    grid[3][2] = 1;

    find_path(grid);
    return 0;
}