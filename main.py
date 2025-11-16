import numpy as np
import square  # 导入正方形模块
import circle  # 导入圆形模块

# 设置打印格式
np.set_printoptions(precision=3, suppress=True)


def run_demo():
    # --- 公共参数 ---
    center = [0.5, 0, 0.5]
    normal = [0, 0, 1]
    plane_axis = [1, 0, 0]
    max_speed = 0.05  # 5 cm/s
    dt = 0.01  # 10 ms

    print("====== 1. 正方形轨迹演示 ======")
    side_length = 0.1

    # 1. 获取起始位姿
    T_start = square.calculate_start_pose(center, normal, side_length, plane_axis)
    print(f"起始位姿矩阵:\n{T_start}\n")

    # 2. 生成轨迹
    pos, vel = square.generate_trajectory(center, normal, side_length, plane_axis, max_speed, dt)
    print(f"正方形轨迹生成完毕: 共 {len(pos)} 步")
    print(f"第一步位置: {pos[0]}")
    print(f"第一步速度: {vel[0]}\n")

    print("====== 2. 圆形轨迹演示 ======")
    radius = 0.1

    # 1. 获取起始位姿
    T_start_c = circle.calculate_start_pose(center, normal, radius, plane_axis)
    print(f"起始位姿矩阵:\n{T_start_c}\n")

    # 2. 生成轨迹
    pos_c, vel_c = circle.generate_trajectory(center, normal, radius, plane_axis, max_speed, dt)
    print(f"圆形轨迹生成完毕: 共 {len(pos_c)} 步")
    print(f"中点位置 (Max速度): {pos_c[len(pos_c) // 2]}")
    print(f"中点速度: {vel_c[len(vel_c) // 2]}")


if __name__ == "__main__":
    run_demo()