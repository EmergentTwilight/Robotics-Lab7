import numpy as np
from utils import normalize, generate_segment_accel_decel_with_pos


def calculate_start_pose(center, normal, side_length, plane_axis_vec):
    """
    【接口 1】计算正方形轨迹的起始位姿 (4x4 齐次矩阵)。

    参数:
        center (list): 正方形中心 [x, y, z]
        normal (list): 法向量 [nx, ny, nz]
        side_length (float): 边长 (米)
        plane_axis_vec (list): 定义平面朝向的参考向量

    返回:
        np.array: 4x4 齐次变换矩阵 T_start
    """
    center = np.array(center)
    normal_vec = normalize(np.array(normal))
    plane_axis_vec = np.array(plane_axis_vec)

    # 构建局部坐标系
    u_vec_raw = plane_axis_vec - np.dot(plane_axis_vec, normal_vec) * normal_vec
    if np.linalg.norm(u_vec_raw) < 1e-6:
        raise ValueError("参考向量 plane_axis_vec 不能与法向量 normal 平行")
    u_vec = normalize(u_vec_raw)
    v_vec = normalize(np.cross(normal_vec, u_vec))

    # 计算P1点位置
    s = side_length / 2.0
    start_position = center - s * u_vec - s * v_vec

    # 构建姿态矩阵 R
    start_orientation_matrix = np.stack([u_vec, v_vec, normal_vec], axis=1)

    # 组装 T 矩阵
    T = np.eye(4)
    T[0:3, 0:3] = start_orientation_matrix
    T[0:3, 3] = start_position
    return T


def generate_trajectory(center, normal, side_length, plane_axis_vec, max_speed, dt):
    """
    【接口 2】生成正方形的完整轨迹 (位置 + 速度)。

    特点:
        每条边独立规划三角速度，在拐角处速度减为0。

    返回:
        pos_traj (np.array): N x 3 位置数组
        vel_traj (np.array): N x 3 速度数组
    """
    center = np.array(center)
    normal_vec = normalize(np.array(normal))
    plane_axis_vec = np.array(plane_axis_vec)

    # 重复几何计算以确定顶点
    u_vec_raw = plane_axis_vec - np.dot(plane_axis_vec, normal_vec) * normal_vec
    if np.linalg.norm(u_vec_raw) < 1e-6:
        raise ValueError("参考向量 plane_axis_vec 不能与法向量 normal 平行")
    u_vec = normalize(u_vec_raw)
    v_vec = normalize(np.cross(normal_vec, u_vec))

    s = side_length / 2.0
    p1 = center - s * u_vec - s * v_vec
    p2 = center + s * u_vec - s * v_vec
    p3 = center + s * u_vec + s * v_vec
    p4 = center - s * u_vec + s * v_vec

    waypoints = [p1, p2, p3, p4, p1]  # 闭环点集

    all_pos = []
    all_vel = []

    for i in range(4):
        p_s = waypoints[i]
        p_e = waypoints[i + 1]
        # 调用 utils 中的通用生成器
        pos, vel = generate_segment_accel_decel_with_pos(p_s, p_e, max_speed, dt)
        all_pos.append(pos)
        all_vel.append(vel)

    return np.vstack(all_pos), np.vstack(all_vel)