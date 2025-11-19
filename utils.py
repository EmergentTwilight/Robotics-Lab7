import numpy as np
import sympy as sp
from params import T06, q

def normalize(v):
    """
    【通用数学工具】将向量单位化。

    参数:
        v (np.array): 输入向量 (如 [1, 2, 3])

    返回:
        np.array: 单位向量。如果模长为0则原样返回。
    """
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def generate_segment_accel_decel_with_pos(p_start, p_end, max_speed, dt):
    """
    【通用轨迹工具】生成单条直线的“三角速度”轨迹（加速->减速），并计算位置。

    逻辑:
        使用三角速度规划，确保起点速度为0 -> 中间达到峰值 -> 终点速度为0。

    参数:
        p_start (np.array): 起点坐标 [x, y, z]
        p_end (np.array): 终点坐标 [x, y, z]
        max_speed (float): 允许的最大速度 (m/s)
        dt (float): 仿真时间步长 (s)

    返回:
        tuple: (position_trajectory, velocity_trajectory)
            - position_trajectory (np.array): N x 3 的期望位置数组
            - velocity_trajectory (np.array): N x 3 的期望速度数组
    """
    p_start = np.array(p_start)
    p_end = np.array(p_end)

    vec = p_end - p_start
    distance = np.linalg.norm(vec)

    # 容错处理：如果起点终点几乎重合
    if distance < 1e-6:
        return np.empty((0, 3)), np.empty((0, 3))

    direction = normalize(vec)

    # 1. 规划时间: 总时间 = 2 * 距离 / 峰值速度
    total_time = (2.0 * distance) / max_speed
    t_accel = total_time / 2.0

    # 2. 计算步数
    n_accel_steps = int(np.round(t_accel / dt))
    n_decel_steps = int(np.round(t_accel / dt))

    if n_accel_steps == 0: n_accel_steps = 1
    if n_decel_steps == 0: n_decel_steps = 1

    # 3. 生成标量速度曲线 (0 -> max -> 0)
    accel_speeds = np.linspace(0, max_speed, num=n_accel_steps, endpoint=False)
    decel_speeds = np.linspace(max_speed, 0, num=n_decel_steps, endpoint=True)
    speed_profile = np.concatenate([accel_speeds, decel_speeds])

    # 4. 生成 3D 速度向量
    velocity_trajectory = speed_profile[:, np.newaxis] * direction[np.newaxis, :]

    # 5. 积分计算位置
    position_trajectory = np.zeros_like(velocity_trajectory)
    current_pos = p_start.copy()

    for k in range(len(velocity_trajectory)):
        position_trajectory[k] = current_pos
        current_pos = current_pos + velocity_trajectory[k] * dt

    return position_trajectory, velocity_trajectory


def forward_kinematics(q_vals):
    assert len(q_vals) == 6
    return T06.subs({q[i]: q_vals[i] for i in range(len(q_vals))})