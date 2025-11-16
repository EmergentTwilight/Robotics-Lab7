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

- **square.py**: 正方形轨迹规划器
  - `calculate_start_pose()`: 计算起始位姿
  - `generate_trajectory()`: 生成正方形轨迹

- **circle.py**: 圆形轨迹规划器
  - `calculate_start_pose()`: 计算起始位姿
  - `generate_trajectory()`: 生成圆形轨迹

- **utils.py**: 底层数学工具库
  - `normalize()`: 向量单位化
  - `generate_segment_accel_decel_with_pos()`: 生成直线轨迹

- **utils/params.py**: 机器人DH参数和雅可比矩阵定义
  - `get_transform()`: 计算变换矩阵
  - 预定义的DH参数和变换矩阵
  - 雅可比矩阵

- **main.py**: 调用示例

## 使用说明

1. 运行 `main.py` 查看正方形和圆形轨迹的生成示例
2. 各模块的详细接口说明请参考对应代码文件中的文档字符串
3. 所有长度单位为米 (m)，时间单位为秒 (s)
4. 输出的速度 vel_traj 是笛卡尔空间的 [vx, vy, vz]
5. 建议使用输出的 pos_traj 进行位置反馈控制，以消除纯速度控制带来的累积误差
