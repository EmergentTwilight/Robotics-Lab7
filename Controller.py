import sympy as sp
import numpy as np
from utils import forward_kinematics
from params import J
from scipy.spatial.transform import Rotation

class Controller:
    def __init__(self, trajectory, dt, loop: bool=False):
        """
        轨迹跟踪控制器
        
        参数:
            trajectory: 4x4齐次变换矩阵列表
            dt: 时间步长 (秒)
            loop: 是否循环轨迹
        """
        self.trajectory = trajectory
        self.dt = dt
        self.loop = loop
        
        # 计算轨迹总时长
        if hasattr(trajectory, '__len__') and len(trajectory) > 0:
            if loop:
                self.duration = len(trajectory) * dt
            else:
                self.duration = (len(trajectory) - 1) * dt
        else:
            self.duration = 0
            
        # 预编译雅可比矩阵数值函数
        self._jacobian_func = sp.lambdify(
            [sp.symbols('q1:7')], J, 'numpy'
        )
    
    def step(self, current_q_vals, current_time):
        """
        计算当前时间步的关节角速度
        
        参数:
            current_q_vals: 当前关节角度 [q1, q2, q3, q4, q5, q6]
            current_time: 当前时间 (秒)
            
        返回:
            tuple: (关节角速度, 目标位姿)
                - 关节角速度: numpy array [dq1, dq2, dq3, dq4, dq5, dq6]
                - 目标位姿: 4x4 齐次变换矩阵
        """
        # 单点轨迹处理
        if self.duration == 0:
            return np.zeros(6), np.array(self.trajectory[0])
            
        # 时间管理
        if self.loop:
            current_time = current_time % self.duration
        elif current_time >= self.duration:
            return np.zeros(6), forward_kinematics(current_q_vals)
        
        # 轨迹索引计算
        current_idx = int(current_time / self.dt)
        if self.loop:
            next_idx = (current_idx + 1) % len(self.trajectory)
        else:
            next_idx = min(current_idx + 1, len(self.trajectory) - 1)
        
        # 获取目标位姿
        target_pose = np.array(self.trajectory[next_idx])
        
        # 计算剩余时间
        target_time = next_idx * self.dt
        remaining_time = target_time - current_time
        
        if remaining_time <= 1e-6:
            return np.zeros(6), target_pose
        
        # 提取当前和下一个位姿
        current_pose = np.array(self.trajectory[current_idx])
        next_pose = np.array(self.trajectory[next_idx])
        
        # 计算线速度
        current_pos = current_pose[0:3, 3]
        next_pos = next_pose[0:3, 3]
        target_linear_vel = (next_pos - current_pos) / remaining_time
        
        # 计算角速度 (使用旋转向量)
        current_rot = current_pose[0:3, 0:3]
        next_rot = next_pose[0:3, 0:3]
        R_delta = next_rot @ current_rot.T
        
        try:
            delta_angle_vec = R.from_matrix(R_delta).as_rotvec()
            target_angular_vel = delta_angle_vec / remaining_time
        except Exception as e:
            print(f"警告：旋转向量计算失败 ({e})，返回零角速度")
            target_angular_vel = np.zeros(3)
        
        # 组合速度向量 [vx, vy, vz, wx, wy, wz]
        target_velocity = np.concatenate([target_linear_vel, target_angular_vel])
        
        # 计算雅可比矩阵
        try:
            J_current = self._jacobian_func(*current_q_vals)
            J_current = np.array(J_current, dtype=np.float64)
        except Exception as e:
            print(f"雅可比矩阵计算错误: {e}")
            return np.zeros(6), target_pose
        
        # 求解关节角速度
        try:
            J_pinv = np.linalg.pinv(J_current)
            joint_velocities = J_pinv @ target_velocity
        except Exception as e:
            print(f"关节速度计算错误: {e}")
            return np.zeros(6), target_pose
        
        return joint_velocities, target_pose
