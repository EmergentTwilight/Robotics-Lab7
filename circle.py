import numpy as np
from utils import normalize


def calculate_start_pose(center, normal, radius, start_axis_vec):
    """
    【接口 1】计算圆形轨迹的起始位姿 (0度位置)。

    参数:
        center (list): 圆心 [x, y, z]
        normal (list): 法向量
        radius (float): 半径 (米)
        start_axis_vec (list): 定义0度起始角的向量

    返回:
        np.array: 4x4 齐次变换矩阵 T_start
    """
    center = np.array(center)
    normal_vec = normalize(np.array(normal))
    start_axis_vec = np.array(start_axis_vec)

    # 构建局部坐标系
    u_vec_raw = start_axis_vec - np.dot(start_axis_vec, normal_vec) * normal_vec
    if np.linalg.norm(u_vec_raw) < 1e-6:
        raise ValueError("起始轴向量 start_axis_vec 不能与法向量 normal 平行")
    u_vec = normalize(u_vec_raw)  # 0度方向
    v_vec = normalize(np.cross(normal_vec, u_vec))  # 90度切线方向

    # 姿态 R (X=半径方向, Y=切线方向, Z=法线)
    start_orientation_matrix = np.stack([u_vec, v_vec, normal_vec], axis=1)

    # 位置 P (0度处)
    start_position = center + radius * u_vec

    T = np.eye(4)
    T[0:3, 0:3] = start_orientation_matrix
    T[0:3, 3] = start_position
    return T


def generate_trajectory(center, normal, radius, start_axis_vec, max_speed, dt):
    """
    【接口 2】生成圆形的完整轨迹 (位置 + 速度)。

    特点:
        从0度起步，180度达最大速度，360度减速至0。
        位置使用 sin/cos 精确计算，无积分误差。

    返回:
        pos_traj (np.array): N x 3 位置数组
        vel_traj (np.array): N x 3 速度数组
    """
    center = np.array(center)
    normal_vec = normalize(np.array(normal))
    start_axis_vec = np.array(start_axis_vec)

    u_vec_raw = start_axis_vec - np.dot(start_axis_vec, normal_vec) * normal_vec
    if np.linalg.norm(u_vec_raw) < 1e-6:
        raise ValueError("起始轴向量 start_axis_vec 不能与法向量 normal 平行")
    u_vec = normalize(u_vec_raw)
    v_vec = normalize(np.cross(normal_vec, u_vec))

    # 规划总步数
    distance = 2.0 * np.pi * radius
    total_time = (2.0 * distance) / max_speed
    total_steps = int(np.round(total_time / dt))

    if total_steps == 0:
        return np.empty((0, 3)), np.empty((0, 3))

    # 三角速度规划
    accel_steps = total_steps // 2
    decel_steps = total_steps - accel_steps

    accel_speeds = np.linspace(0, max_speed, num=accel_steps, endpoint=False)
    decel_speeds = np.linspace(max_speed, 0, num=decel_steps, endpoint=True)
    scalar_speed_profile = np.concatenate([accel_speeds, decel_speeds])

    pos_traj_list = []
    vel_traj_list = []
    current_angle = 0.0

    for k in range(total_steps):
        theta = current_angle
        speed = scalar_speed_profile[k]

        # 1. 位置 (解析解)
        pos = center + radius * (np.cos(theta) * u_vec + np.sin(theta) * v_vec)

        # 2. 速度 (切线方向)
        tangent = -np.sin(theta) * u_vec + np.cos(theta) * v_vec
        vel = speed * tangent

        pos_traj_list.append(pos)
        vel_traj_list.append(vel)

        # 更新角度
        current_angle += (speed * dt) / radius

    return np.array(pos_traj_list), np.array(vel_traj_list)