# Robotics-Lab7

## Modified DH Parameters

| $\alpha_{i-1}$ | $a_{i-1}$ | $d_i$ | $\theta_i$ |
| -------------- | --------- | ----- | ---------- |
| 0              | 0         | 0.23  | $q_1$      |
| $-\pi/2$       | 0         | -0.054| $q_2 - \pi/2$ |
| 0              | 0.185     | 0     | $q_3$      |
| 0              | 0.17      | 0.077 | $q_4 + \pi/2$ |
| $\pi/2$        | 0         | 0.077 | $q_5 + \pi/2$ |
| $\pi/2$        | 0         | 0.0855| $q_6$      |

```python
alpha = [0, -sp.pi / 2, 0, 0, sp.pi / 2, sp.pi / 2]
a = [0, 0, 0.185, 0.17, 0, 0]
d = [0.23, -0.054, 0, 0.077, 0.077, 0.0855]
theta = [q1, q2 - sp.pi / 2, q3, q4 + sp.pi / 2, q5 + sp.pi / 2, q6]
```

## 文件结构

square.py: 正方形轨迹规划器

circle.py: 圆形轨迹规划器

utils.py: 底层数学工具库

main.py: 调用示例

1. 正方形规划模块 (square.py)

### 接口 1.1: 计算起始位姿

计算机器人开始画正方形前的初始位置和姿态。
```python
calculate_start_pose(center, normal, side_length, plane_axis_vec)
```

输入:

center (list/array): 正方形中心 [x, y, z]

normal (list/array): 法向量

side_length (float): 边长 (米)

plane_axis_vec (list/array): 参考向量 (例如 [1,0,0])，决定正方形在平面内的旋转角度

输出:

T (4x4 np.array): 齐次变换矩阵 (包含起始位置和旋转矩阵)

### 接口 1.2: 生成轨迹

生成完整的正方形运动轨迹。

```python
generate_trajectory(center, normal, side_length, plane_axis_vec, max_speed, dt)
```
输入:

max_speed (float): 运动峰值速度 (m/s)

dt (float): 仿真/控制时间步长 (s)

(其他几何参数同上)

输出:

pos_traj (N x 3 np.array): 期望位置序列

vel_traj (N x 3 np.array): 期望速度序列 (用于 RRMC)

2. 圆形规划模块 (circle.py)

### 接口 2.1: 计算起始位姿

计算机器人开始画圆前的初始位置 (0度角位置)。
```python
calculate_start_pose(center, normal, radius, start_axis_vec)
```

输入:

center (list/array): 圆心 [x, y, z]

normal (list/array): 法向量

radius (float): 半径 (米)

start_axis_vec (list/array): 定义 0 度起始方向的向量

输出:

T (4x4 np.array): 齐次变换矩阵

### 接口 2.2: 生成轨迹

生成完整的圆形运动轨迹 (0度 -> 360度)。
```python
generate_trajectory(center, normal, radius, start_axis_vec, max_speed, dt)
```

输入:

几何参数同上，max_speed 为圆周运动的最大线速度。

输出:

pos_traj, vel_traj: 位置和速度序列。

3. 使用注意事项

单位: 所有长度单位为米 (m)，时间单位为秒 (s)。

坐标系: 输出的速度 vel_traj 是笛卡尔空间的 [vx, vy, vz]。如果需要控制姿态不变，请在控制循环中将角速度设为 [0, 0, 0]。

闭环控制: 建议使用输出的 pos_traj 进行位置反馈控制，以消除纯速度控制带来的累积误差。