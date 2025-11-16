"""
机器人参数模块

本模块定义了6自由度机器人的修改DH参数、变换矩阵和雅可比矩阵，
用于正运动学计算和速度映射关系计算。

Modified DH Parameters:
| $\alpha_{i-1}$ | $a_{i-1}$ | $d_i$ | $\theta_i$ |
| -------------- | --------- | ----- | ---------- |
| 0              | 0         | 0.23  | $q_1$      |
| $-\pi/2$       | 0         | -0.054| $q_2 - \pi/2$ |
| 0              | 0.185     | 0     | $q_3$      |
| 0              | 0.17      | 0.077 | $q_4 + \pi/2$ |
| $\pi/2$        | 0         | 0.077 | $q_5 + \pi/2$ |
| $\pi/2$        | 0         | 0.0855| $q_6$      |
"""

import sympy as sp

# Joint angles
q1, q2, q3, q4, q5, q6 = sp.symbols("q1:7")

# Modified DH parameters
alpha = [0, -sp.pi / 2, 0, 0, sp.pi / 2, sp.pi / 2]
a = [0, 0, 0.185, 0.17, 0, 0]
d = [0.23, -0.054, 0, 0.077, 0.077, 0.0855]
theta = [q1, q2 - sp.pi / 2, q3, q4 + sp.pi / 2, q5 + sp.pi / 2, q6]


# Transform matrices
def get_transform(alpha, a, d, theta):
    """
    计算相邻连杆间的齐次变换矩阵（使用修改DH参数）。
    
    参数:
        alpha (float): 连杆扭角，绕X_{i-1}轴从Z_{i-1}旋转到Z_i的角度
        a (float): 连杆长度，沿X_{i-1}轴从Z_{i-1}到Z_i的距离
        d (float): 连杆偏距，沿Z_i轴从X_{i-1}到X_i的距离
        theta (float): 关节角度，绕Z_i轴从X_{i-1}旋转到X_i的角度
        
    返回:
        sp.Matrix: 4x4齐次变换矩阵，表示从连杆i-1坐标系到连杆i坐标系的变换
    """
    return sp.simplify(
        sp.Matrix(
            [
                [sp.cos(theta), -sp.sin(theta), 0, a],
                [
                    sp.sin(theta) * sp.cos(alpha),
                    sp.cos(theta) * sp.cos(alpha),
                    -sp.sin(alpha),
                    -sp.sin(alpha) * d,
                ],
                [
                    sp.sin(theta) * sp.sin(alpha),
                    sp.cos(theta) * sp.sin(alpha),
                    sp.cos(alpha),
                    sp.cos(alpha) * d,
                ],
                [0, 0, 0, 1],
            ]
        )
    )


# 相邻连杆间的变换矩阵
T01 = get_transform(alpha[0], a[0], d[0], theta[0])  # 基座到关节1
T12 = get_transform(alpha[1], a[1], d[1], theta[1])  # 关节1到关节2
T23 = get_transform(alpha[2], a[2], d[2], theta[2])  # 关节2到关节3
T34 = get_transform(alpha[3], a[3], d[3], theta[3])  # 关节3到关节4
T45 = get_transform(alpha[4], a[4], d[4], theta[4])  # 关节4到关节5
T56 = get_transform(alpha[5], a[5], d[5], theta[5])  # 关节5到关节6

# 累积变换矩阵（从基座到各关节）
T02 = sp.simplify(T01 * T12)  # 基座到关节2
T03 = sp.simplify(T02 * T23)  # 基座到关节3
T04 = sp.simplify(T03 * T34)  # 基座到关节4
T05 = sp.simplify(T04 * T45)  # 基座到关节5
T06 = sp.simplify(T05 * T56)  # 基座到末端执行器

# Jacobian matrix
# 雅可比矩阵描述了末端执行器的线速度和角速度与关节速度之间的关系
# J = [Jv; Jw]，其中：
#   Jv (3x6): 线速度雅可比矩阵，将关节速度映射到末端执行器线速度
#   Jw (3x6): 角速度雅可比矩阵，将关节速度映射到末端执行器角速度
J = sp.Matrix(
    [
        # Row 1 (末端执行器线速度 Vx)
        [
            -0.185 * sp.sin(q1) * sp.sin(q2)
            - 0.17 * sp.sin(q1) * sp.sin(q2 + q3)
            - 0.077 * sp.sin(q1) * sp.sin(q2 + q3 + q4)
            - 0.0855 * sp.sin(q1) * sp.cos(q5) * sp.cos(q2 + q3 + q4)
            - 0.0855 * sp.sin(q5) * sp.cos(q1)
            - 0.023 * sp.cos(q1),
            (-0.0855 * sp.sin(q2 + q3 + q4) * sp.cos(q5) + 0.185 * sp.cos(q2) + 0.17 * sp.cos(q2 + q3) + 0.077 * sp.cos(q2 + q3 + q4))
            * sp.cos(q1),
            (-0.0855 * sp.sin(q2 + q3 + q4) * sp.cos(q5) + 0.185 * sp.cos(q2) + 0.17 * sp.cos(q2 + q3) + 0.077 * sp.cos(q2 + q3 + q4))
            * sp.cos(q1),
            (-0.0855 * sp.sin(q2 + q3 + q4) * sp.cos(q5) + 0.17 * sp.cos(q2 + q3) + 0.077 * sp.cos(q2 + q3 + q4)) * sp.cos(q1),
            -0.0855 * sp.sin(q1) * sp.cos(q5) - 0.0855 * sp.sin(q5) * sp.cos(q1) * sp.cos(q2 + q3 + q4),
            0,
        ],
        # Row 2 (末端执行器线速度 Vy)
        [
            -0.0855 * sp.sin(q1) * sp.sin(q5)
            - 0.023 * sp.sin(q1)
            + 0.185 * sp.sin(q2) * sp.cos(q1)
            + 0.17 * sp.sin(q2 + q3) * sp.cos(q1)
            + 0.077 * sp.sin(q2 + q3 + q4) * sp.cos(q1)
            + 0.0855 * sp.cos(q1) * sp.cos(q5) * sp.cos(q2 + q3 + q4),
            (-0.0855 * sp.sin(q2 + q3 + q4) * sp.cos(q5) + 0.185 * sp.cos(q2) + 0.17 * sp.cos(q2 + q3) + 0.077 * sp.cos(q2 + q3 + q4))
            * sp.sin(q1),
            (-0.0855 * sp.sin(q2 + q3 + q4) * sp.cos(q5) + 0.185 * sp.cos(q2) + 0.17 * sp.cos(q2 + q3) + 0.077 * sp.cos(q2 + q3 + q4))
            * sp.sin(q1),
            (-0.0855 * sp.sin(q2 + q3 + q4) * sp.cos(q5) + 0.17 * sp.cos(q2 + q3) + 0.077 * sp.cos(q2 + q3 + q4)) * sp.sin(q1),
            -0.0855 * sp.sin(q1) * sp.sin(q5) * sp.cos(q2 + q3 + q4) + 0.0855 * sp.cos(q1) * sp.cos(q5),
            0,
        ],
        # Row 3 (末端执行器线速度 Vz)
        [
            0,
            -0.185 * sp.sin(q2) - 0.17 * sp.sin(q2 + q3) - 0.077 * sp.sin(q2 + q3 + q4) - 0.0855 * sp.cos(q5) * sp.cos(q2 + q3 + q4),
            -0.185 * sp.sin(q2) - 0.17 * sp.sin(q2 + q3) - 0.077 * sp.sin(q2 + q3 + q4) - 0.0855 * sp.cos(q5) * sp.cos(q2 + q3 + q4),
            -0.17 * sp.sin(q2 + q3) - 0.077 * sp.sin(q2 + q3 + q4) - 0.0855 * sp.cos(q5) * sp.cos(q2 + q3 + q4),
            0.0855 * sp.sin(q5) * sp.sin(q2 + q3 + q4),
            0,
        ],
        # Row 4 (末端执行器角速度 ωx)
        [
            0,
            -sp.sin(q1),
            -sp.sin(q1),
            -sp.sin(q1),
            sp.sin(q2 + q3 + q4) * sp.cos(q1),
            -sp.sin(q1) * sp.sin(q5) + sp.cos(q1) * sp.cos(q5) * sp.cos(q2 + q3 + q4),
        ],
        # Row 5 (末端执行器角速度 ωy)
        [
            0,
            sp.cos(q1),
            sp.cos(q1),
            sp.cos(q1),
            sp.sin(q1) * sp.sin(q2 + q3 + q4),
            sp.sin(q1) * sp.cos(q5) * sp.cos(q2 + q3 + q4) + sp.sin(q5) * sp.cos(q1),
        ],
        # Row 6 (末端执行器角速度 ωz)
        [1, 0, 0, 0, sp.cos(q2 + q3 + q4), -sp.sin(q2 + q3 + q4) * sp.cos(q5)],
    ]
)
