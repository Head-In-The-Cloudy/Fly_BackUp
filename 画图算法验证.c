#include <stdio.h>
#include <stdint.h>

typedef struct
{
    uint8_t A; // 横坐标
    uint8_t B; // 纵坐标
} vector2f;

uint8_t FLY_PORT[100][2] = {0};             // 存储补齐的航点
static uint8_t Write_In_FLY_PORT_count = 0; // 索引计数器

// 输入当前点 和 上一个点，补齐航点
void Write_In_FLY_PORT(vector2f MAPing_XY, vector2f pre_MAPing_XY)
{
    // 第一次写入，直接存当前点
    if (pre_MAPing_XY.A == 0 && pre_MAPing_XY.B == 0)
    {
        FLY_PORT[Write_In_FLY_PORT_count][0] = MAPing_XY.A;
        FLY_PORT[Write_In_FLY_PORT_count][1] = MAPing_XY.B;
        Write_In_FLY_PORT_count++;
        return;
    }

    // 如果A相等，B变化 => 竖直方向补点
    if (MAPing_XY.A == pre_MAPing_XY.A)
    {
        if (MAPing_XY.B > pre_MAPing_XY.B) // 向上补
        {
            for (int b = pre_MAPing_XY.B - 1; b <= MAPing_XY.B; b++)
            {
                FLY_PORT[Write_In_FLY_PORT_count][0] = MAPing_XY.A;
                FLY_PORT[Write_In_FLY_PORT_count][1] = b;
                Write_In_FLY_PORT_count++;
            }
        }
        else if (MAPing_XY.B < pre_MAPing_XY.B) // 向下补
        {
            for (int b = pre_MAPing_XY.B - 1; b >= MAPing_XY.B; b--)
            {
                FLY_PORT[Write_In_FLY_PORT_count][0] = MAPing_XY.A;
                FLY_PORT[Write_In_FLY_PORT_count][1] = b;
                Write_In_FLY_PORT_count++;
            }
        }
    }
    // 如果B相等，A变化 => 水平方向补点
    else if (MAPing_XY.B == pre_MAPing_XY.B)
    {
        if (MAPing_XY.A > pre_MAPing_XY.A) // 向右补
        {
            for (int a = pre_MAPing_XY.A + 1; a <= MAPing_XY.A; a++)
            {
                FLY_PORT[Write_In_FLY_PORT_count][0] = a;
                FLY_PORT[Write_In_FLY_PORT_count][1] = MAPing_XY.B;
                Write_In_FLY_PORT_count++;
            }
        }
        else if (MAPing_XY.A < pre_MAPing_XY.A) // 向左补
        {
            for (int a = pre_MAPing_XY.A - 1; a >= MAPing_XY.A; a--)
            {
                FLY_PORT[Write_In_FLY_PORT_count][0] = a;
                FLY_PORT[Write_In_FLY_PORT_count][1] = MAPing_XY.B;
                Write_In_FLY_PORT_count++;
            }
        }
    }
}

// 打印补齐的航点
void print_fly_port()
{
    printf("FLY_PORT 内容：\n");
    for (int i = 0; i < Write_In_FLY_PORT_count; i++)
    {
        printf("{%d,%d}\n", FLY_PORT[i][0], FLY_PORT[i][1]);
    }
}

int main()
{
    vector2f pre_point = {0, 0};
    vector2f point;

    // 手动输入多个航点
    for (int i = 0; i < 3; i++)
    {
        printf("请输入第 %d 个航点(A B): ", i + 1);
        scanf("%hhu %hhu", &point.A, &point.B);

        Write_In_FLY_PORT(point, pre_point);

        pre_point = point; // 更新上一个点
    }

    // 打印补齐后的航点
    print_fly_port();

    return 0;
}
