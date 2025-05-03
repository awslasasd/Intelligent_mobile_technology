from math import sqrt, sin, cos, atan2
import numpy as np
import copy
import random
import time

# 定义动态窗口方法（DWA）类
class DynamicWindowApproach:
    def __init__(self, goal_x, goal_y, v_min=-3500.0, v_max=3500.0,
                 a_min=-3000.0, a_max=3000.0, w_max=10, aw_max=10,
                 dt=0.01, sim_duration=0.3, res=10):
        
        """
        初始化 DWA 轨迹规划算法的参数
        :param goal_x, goal_y: 目标点坐标
        :param v_min, v_max: 速度的最小值和最大值
        :param a_min, a_max: 线性加速度的最小值和最大值
        :param w_max: 角速度的最大值
        :param aw_max: 角加速度的最大值
        :param dt: 时间步长
        :param sim_duration: 轨迹模拟时间
        :param res: 速度搜索的分辨率
        """

        self.goal_pos = np.array([goal_x, goal_y])  # 目标位置
        self.robot_state = np.array([0.0, 0.0, 0.0])  # 机器人状态（x, y, 方向）
        self.robot_speed = np.array([0.0, 0.0])  # 机器人速度（线速度，角速度）
        
        # 设置速度、加速度、角速度的约束范围
        self.speed_limits = np.array([v_min, v_max])
        self.acceleration_limits = np.array([a_min, a_max])
        self.angular_speed_limits = np.array([-w_max, w_max])
        self.angular_acceleration_limits = np.array([-aw_max, aw_max])
        
        # 初始化障碍物信息
        self.static_obstacles = []
        self.dynamic_obstacles = []
        
        self.arrival_tolerance = 500  # 目标到达的容忍范围
        self.safety_distance = 300  # 机器人与障碍物的最小安全距离
        self.dt = dt  # 时间步长
        self.sim_duration = sim_duration  # 轨迹模拟的时间
        self.resolution = res  # 速度搜索的分辨率
        
        # 记录时间戳，用于计算动态障碍物速度
        self.previous_time = time.time()
        self.current_time = self.previous_time

    def update_sensor_data(self, vision):
        """
        更新传感器数据，包括机器人的位置、方向以及周围障碍物的位置。
        :param vision: 机器人视觉系统
        """
        self.current_time = time.time()
        time_interval = self.current_time - self.previous_time  # 计算时间间隔
        self.previous_time = self.current_time
        
        # 更新机器人自身状态
        self.robot_state[:2] = np.array([vision.my_robot.x, vision.my_robot.y])
        self.robot_state[2] = vision.my_robot.orientation
        
        updated_obstacles = []  # 更新障碍物信息
        self.dynamic_obstacles = []

        # 遍历场上的所有机器人（蓝色和黄色），获取障碍物信息
        for robot in vision.blue_robot + vision.yellow_robot:
            if robot.id != vision.my_robot.id:  # 过滤掉自己
                obstacle_pos = [robot.x, robot.y, robot.orientation]
                updated_obstacles.append(obstacle_pos)

                # 如果时间间隔足够大，则计算障碍物的速度并预测未来位置
                if time_interval > 0 and self.static_obstacles:
                    prev_obstacle_pos = self.static_obstacles[-1]
                    vel_x = (robot.x - prev_obstacle_pos[0]) / time_interval
                    vel_y = (robot.y - prev_obstacle_pos[1]) / time_interval
                    for i in range(5):  # 预测未来 5 个时间步的位置
                        predicted_pos = [robot.x, robot.y + np.sign(vel_y) * 50 * (i - 1), robot.orientation]
                        self.dynamic_obstacles.append(predicted_pos)
        
        self.static_obstacles = updated_obstacles  # 更新静态障碍物信息

    def has_reached_goal(self, goal_x, goal_y):
        """
        判断机器人是否到达目标点
        :param goal_x, goal_y: 目标点坐标
        :return: 是否到达目标点（True/False）
        """
        goal = np.array([goal_x, goal_y])
        distance = min(
            np.linalg.norm(self.goal_pos - self.robot_state[:2]),
            np.linalg.norm(goal - self.robot_state[:2])
        )
        return distance < self.arrival_tolerance  # 如果距离小于容忍范围，则认为到达目标点

    def simulate_future_state(self, v_x, omega):
        """
        预测机器人在一段时间后的状态
        :param v_x: 线速度
        :param omega: 角速度
        :return: 预测的机器人状态（x, y, 方向）
        """
        simulated_state = copy.deepcopy(self.robot_state)
        for _ in range(int(self.sim_duration / self.dt)):
            simulated_state[0] += v_x * self.dt * cos(simulated_state[2])
            simulated_state[1] += v_x * self.dt * sin(simulated_state[2])
            simulated_state[2] += omega * self.dt
        return simulated_state

    def find_optimal_motion(self):
        """
        计算最优的速度和角速度
        :return: 最优的线速度 vx 和角速度 w
        """
        max_w = 4 if self.robot_speed[0] < 200 else self.angular_speed_limits[1]

        # 计算当前速度的可行范围
        v_min = max(self.speed_limits[0], self.robot_speed[0] - self.sim_duration * self.acceleration_limits[1])
        v_max = min(self.speed_limits[1], self.robot_speed[0] + self.sim_duration * self.acceleration_limits[1])
        w_min = max(-max_w, self.robot_speed[1] - self.sim_duration * self.angular_acceleration_limits[1])
        w_max = min(max_w, self.robot_speed[1] + self.sim_duration * self.angular_acceleration_limits[1])

        best_score = float('-inf')
        optimal_vx, optimal_w = 0, 0

        # 在可行范围内搜索最佳速度组合
        for v_x in np.linspace(v_min, v_max, self.resolution):
            for omega in np.linspace(w_min, w_max, self.resolution):
                predicted_state = self.simulate_future_state(v_x, omega)
                score = self.evaluate_trajectory(predicted_state, v_x, omega, v_max, w_max)

                if score > best_score:
                    best_score = score
                    optimal_vx, optimal_w = v_x, omega

        self.robot_speed = np.array([optimal_vx, optimal_w])
        return optimal_vx, optimal_w

    def evaluate_trajectory(self, future_pos, v_x, omega, v_max, w_max):
        """
        评估某条轨迹的得分
        :return: 轨迹得分
        """
        goal_dist = np.linalg.norm(future_pos[:2] - self.goal_pos)
        goal_score = 1 / (1 + goal_dist)

        min_dist = self.compute_nearest_obstacle_distance(future_pos)
        dist_score = -np.inf if min_dist < self.safety_distance else 1.0

        velocity_score = v_x / v_max if goal_dist > 100 else 1.0 - v_x / v_max

        return 50 * dist_score + 10 * velocity_score + 20 * goal_score

    def normalize_angle(self, theta):
        return atan2(sin(theta), cos(theta))

    def update_goal(self, goal_x, goal_y):
        self.goal_pos = np.array([goal_x, goal_y])

    def update_arrival_tolerance(self, tolerance):
        self.arrival_tolerance = tolerance

    def compute_nearest_obstacle_distance(self, future_pos):
        """
        计算机器人未来状态到最近障碍物的距离
        :return: 最小距离
        """
        if not self.static_obstacles and not self.dynamic_obstacles:
            return 1e6

        all_obstacles = np.array(self.static_obstacles + self.dynamic_obstacles)
        obstacle_positions = all_obstacles[:, :2]

        future_pos_expanded = np.tile(future_pos[:2], (len(obstacle_positions), 1))
        distances = np.linalg.norm(future_pos_expanded - obstacle_positions, axis=1)

        return np.min(distances) if distances.size else 1e6

    def if_stop(self, distance_threshold=1000.0):
        if np.linalg.norm(self.robot_speed) < 1e-4:
            return True

        if np.linalg.norm(self.robot_state[:2] - self.goal_pos) > distance_threshold:
            print(f"\nExceeds threshold {distance_threshold}, stopping.")
            return True

        return False