#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#define ROWS 7
#define COLS 9

// 方格状态枚举
typedef enum
{
    FREE,      // 可自由遍历
    FORBIDDEN, // 禁止遍历
    VISITED    // 已遍历
} CellState;

// 位置结构体
typedef struct
{
    int row; // 行 (B1-B7对应0-6)
    int col; // 列 (A1-A9对应0-8)
} Position;

// 方向枚举
typedef enum
{
    UP,
    RIGHT,
    DOWN,
    LEFT
} Direction;

// 全局变量
CellState grid[ROWS][COLS];
Position currentPos;
int visitedCount = 0;
int totalFreeCells = 0;
int straightMoves = 0;
Position endPositions[3] = {{0, 6}, {2, 8}, {1, 7}}; // B1,A7; B3,A9; B2,A8

// 初始化网格
void initializeGrid()
{
    for (int i = 0; i < ROWS; i++)
    {
        for (int j = 0; j < COLS; j++)
        {
            grid[i][j] = FREE;
        }
    }
    totalFreeCells = ROWS * COLS;
}

// 设置禁止遍历的方格
void setForbiddenCells(Position forbidden[], int count)
{
    for (int i = 0; i < count; i++)
    {
        if (forbidden[i].row >= 0 && forbidden[i].row < ROWS &&
            forbidden[i].col >= 0 && forbidden[i].col < COLS)
        {
            grid[forbidden[i].row][forbidden[i].col] = FORBIDDEN;
            totalFreeCells--;
        }
    }
}

// 检查位置是否有效且可访问
bool isValidPosition(Position pos)
{
    return pos.row >= 0 && pos.row < ROWS &&
           pos.col >= 0 && pos.col < COLS &&
           grid[pos.row][pos.col] == FREE;
}

// 检查是否是终止位置
bool isEndPosition(Position pos)
{
    for (int i = 0; i < 3; i++)
    {
        if (pos.row == endPositions[i].row && pos.col == endPositions[i].col)
        {
            return true;
        }
    }
    return false;
}

// 获取下一个移动方向（优先直线移动）
Direction getNextDirection(Direction lastDir)
{
    // 优先保持直线方向
    if (lastDir != -1)
    {
        Position nextPos = currentPos;
        switch (lastDir)
        {
        case UP:
            nextPos.row--;
            break;
        case RIGHT:
            nextPos.col++;
            break;
        case DOWN:
            nextPos.row++;
            break;
        case LEFT:
            nextPos.col--;
            break;
        }
        if (isValidPosition(nextPos))
        {
            return lastDir;
        }
    }

    // 如果不能继续直线，尝试其他方向
    for (Direction dir = UP; dir <= LEFT; dir++)
    {
        Position nextPos = currentPos;
        switch (dir)
        {
        case UP:
            nextPos.row--;
            break;
        case RIGHT:
            nextPos.col++;
            break;
        case DOWN:
            nextPos.row++;
            break;
        case LEFT:
            nextPos.col--;
            break;
        }
        if (isValidPosition(nextPos))
        {
            return dir;
        }
    }

    return -1; // 无有效方向
}

// 执行移动
void move(Direction dir)
{
    Position oldPos = currentPos;

    switch (dir)
    {
    case UP:
        currentPos.row--;
        break;
    case RIGHT:
        currentPos.col++;
        break;
    case DOWN:
        currentPos.row++;
        break;
    case LEFT:
        currentPos.col--;
        break;
    }

    // 标记为已访问
    grid[currentPos.row][currentPos.col] = VISITED;
    visitedCount++;

    // 检查是否是直线移动
    static Direction lastDir = -1;
    if (dir == lastDir)
    {
        straightMoves++;
    }
    lastDir = dir;
}

// 打印网格状态
void printGrid()
{
    for (int i = 0; i < ROWS; i++)
    {
        for (int j = 0; j < COLS; j++)
        {
            if (i == currentPos.row && j == currentPos.col)
            {
                printf("C ");
            }
            else
            {
                switch (grid[i][j])
                {
                case FREE:
                    printf(". ");
                    break;
                case FORBIDDEN:
                    printf("X ");
                    break;
                case VISITED:
                    printf("o ");
                    break;
                }
            }
        }
        printf("\n");
    }
}

// 主遍历函数
bool traverseGrid()
{
    // 设置起始位置 (B1,A9) -> (0,8)
    currentPos.row = 0;
    currentPos.col = 8;

    // 检查起始位置是否合法
    if (!isValidPosition(currentPos))
    {
        printf("起始位置不可用或已被禁止访问！\n");
        return false;
    }

    grid[currentPos.row][currentPos.col] = VISITED;
    visitedCount++;

    Direction lastDir = -1;

    while (visitedCount < totalFreeCells)
    {
        Direction nextDir = getNextDirection(lastDir);

        if (nextDir == -1)
        {
            printf("无法继续移动！\n");
            return false;
        }

        move(nextDir);
        lastDir = nextDir;

        printf("移动后位置: B%d,A%d\n", currentPos.row + 1, currentPos.col + 1);
        printGrid();
        printf("\n");

        // 检查是否到达终止位置
        if (isEndPosition(currentPos))
        {
            if (visitedCount == totalFreeCells)
            {
                printf("成功遍历所有方格！\n");
                printf("直线移动次数: %d\n", straightMoves);
                return true;
            }
            else
            {
                printf("提前到达终止位置，但未遍历所有方格！\n");
                return false;
            }
        }
    }

    printf("遍历完成但未到达终止位置！\n");
    return false;
}

int main()
{
    initializeGrid();

    // 设置禁止遍历的方格（示例）
    Position forbidden[] = {
        {1, 1}, {1, 2}, {1, 3}, // 示例禁止区域
        {3, 4},
        {3, 5},
        {4, 5} // L形禁止区域
    };
    setForbiddenCells(forbidden, sizeof(forbidden) / sizeof(forbidden[0]));

    if (traverseGrid())
    {
        printf("遍历成功！\n");
    }
    else
    {
        printf("遍历失败！\n");
    }

    return 0;
}