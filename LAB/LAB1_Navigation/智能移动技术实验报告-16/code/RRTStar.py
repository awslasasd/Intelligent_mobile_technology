from math import sqrt, sin, cos, atan2
import numpy as np
import copy
import random
import math
import time

# 定义树节点类
class Node(object):
    def __init__(self, x, y, cost, parent):
        """
        初始化 RRT* 树节点
        :param x, y: 该节点的坐标
        :param cost: 从起点到该节点的代价（路径长度）
        :param parent: 指向父节点的指针，用于路径回溯
        """
        self.x = x
        self.y = y # 节点的坐标
        self.cost = cost # 从起点到该节点的代价（距离）
        self.parent = parent # 指向父节点的指针，用于回溯路径

# 定义 RRT* 路径规划算法类
class RRTStar:
    def __init__(self, max_edge_length=10000, sample_count=200, neighbor_count=10, 
                 goal_sample_prob=0.25, step_size=70, goal_threshold=150):
        """
        初始化 RRT* 规划器
        :param max_edge_length: 生成的最大边长
        :param sample_count: 采样次数
        :param neighbor_count: 每个新节点考虑的最近邻节点数
        :param goal_sample_prob: 以目标点采样的概率
        :param step_size: 每次向目标方向前进的步长
        :param goal_threshold: 目标点的容忍距离
        """
        self.x_min, self.x_max = -4500, 4500  # 地图 x 轴边界
        self.y_min, self.y_max = -3000, 3000  # 地图 y 轴边界
        self.robot_radius = 250  # 机器人半径
        self.avoidance_margin = 250  # 机器人避障安全距离

        self.max_edge_length = max_edge_length
        self.sample_count = sample_count  # 采样点数量
        self.neighbor_count = neighbor_count  # 近邻节点个数
        self.goal_sample_prob = goal_sample_prob  # 采样目标点的概率
        self.step_size = step_size  # 每次前进步长
        self.goal_threshold = goal_threshold  # 目标点的可接受误差范围

        self.node_list = []  # 存储路径树中的所有节点

    def euclidean_distance(self, a, b):
        """ 计算两个点之间的欧几里得距离 """
        return sqrt((a - b) ** 2)

    def is_valid_location(self, vision, x, y):
        """
        判断某个点是否是一个有效的机器人可行位置
        :param vision: 机器人视觉信息
        :param x, y: 需要检查的坐标点
        :return: 该位置是否可行（True/False）
        """
        for robot in vision.blue_robot + vision.yellow_robot:
            if robot.visible and robot.id > 0:
                # 计算该点与障碍物（其他机器人）的距离
                dist_sq = (self.euclidean_distance(robot.x, x))**2 + (self.euclidean_distance(robot.y, y))**2
                if dist_sq < (self.robot_radius / 1.8 + self.avoidance_margin / 1.8) ** 2:
                    return False  # 位置无效
        return True  # 位置有效

    def generate_random_point(self, vision):
        """ 在地图范围内生成一个随机点，并确保它是有效的（无碰撞） """
        while True:
            rand_x = random.uniform(self.x_min, self.x_max)
            rand_y = random.uniform(self.y_min, self.y_max)
            if self.is_valid_location(vision, rand_x, rand_y):
                return rand_x, rand_y

    def select_sample_point(self, vision, goal_x, goal_y):
        """ 以一定概率选择目标点作为采样点，否则随机生成一个点 """
        return (goal_x, goal_y) if random.random() < self.goal_sample_prob else self.generate_random_point(vision)

    def find_nearest_node(self, nodes, sample_point):
        """ 查找离采样点最近的节点 """
        min_dist = float('inf')
        nearest = None
        for node in nodes:
            dist = self.euclidean_distance(node.x, sample_point[0]) + self.euclidean_distance(node.y, sample_point[1])
            if dist < min_dist:
                min_dist = dist
                nearest = node
        return nearest

    def find_k_nearest_nodes(self, nodes, target_node):
        """ 查找目标节点附近的 k 个最近邻节点 """
        distances = [self.euclidean_distance(target_node.x, node.x) + self.euclidean_distance(target_node.y, node.y)
                     for node in nodes]
        nearest_indices = np.argsort(distances)
        return [nodes[i] for i in nearest_indices[:self.neighbor_count]]

    def steer(self, parent_node, target_point):
        """ 计算从父节点朝目标点前进一个步长后的新坐标 """
        theta = atan2(target_point[1] - parent_node.y, target_point[0] - parent_node.x)
        new_x = parent_node.x + cos(theta) * self.step_size
        new_y = parent_node.y + sin(theta) * self.step_size
        return new_x, new_y

    def rewire(self, new_node, neighbors):
        """ 重新连接新节点及其邻居节点，优化路径 """
        for node in neighbors:
            new_cost = new_node.cost + self.euclidean_distance(new_node.x, node.x) + self.euclidean_distance(new_node.y, node.y)
            if new_cost < node.cost:
                node.parent = new_node
                node.cost = new_cost

    def execute_rrt_star(self, goal_x, goal_y, start_x, start_y, vision):
        """
        执行 RRT* 规划，生成从起点到目标点的路径
        :param goal_x, goal_y: 目标点坐标
        :param start_x, start_y: 起始点坐标
        :param vision: 机器人视觉信息
        :return: 规划出的路径点及树结构信息
        """
        self.node_list = []  # 清空节点列表
        start_node = Node(start_x, start_y, 0.0, None)
        self.node_list.append(start_node)

        iteration = 0
        while True:
            iteration += 1
            sample = self.select_sample_point(vision, goal_x, goal_y)
            nearest = self.find_nearest_node(self.node_list, sample)
            new_x, new_y = self.steer(nearest, sample)

            new_node = Node(new_x, new_y, float('inf'), None)
            if self.is_valid_location(vision, new_x, new_y):
                neighbors = self.find_k_nearest_nodes(self.node_list, new_node)
                min_parent = nearest
                min_cost = nearest.cost + self.euclidean_distance(nearest.x, new_x) + self.euclidean_distance(nearest.y, new_y)

                for neighbor in neighbors:
                    cost = neighbor.cost + self.euclidean_distance(neighbor.x, new_x) + self.euclidean_distance(neighbor.y, new_y)
                    if cost < min_cost:
                        min_parent = neighbor
                        min_cost = cost

                new_node.cost = min_cost
                new_node.parent = min_parent
                self.node_list.append(new_node)
                self.rewire(new_node, neighbors)

            last_node = self.find_nearest_node(self.node_list, (goal_x, goal_y))
            if self.euclidean_distance(last_node.x, goal_x) ** 2 + self.euclidean_distance(last_node.y, goal_y) ** 2 < 10000 and iteration >= self.sample_count:
                break

        path_x, path_y = [goal_x], [goal_y]
        while last_node:
            path_x.append(last_node.x)
            path_y.append(last_node.y)
            last_node = last_node.parent

        edge_x1, edge_y1, edge_x2, edge_y2 = [], [], [], []
        for node in self.node_list:
            if node.parent:
                edge_x1.append(node.parent.x)
                edge_y1.append(node.parent.y)
                edge_x2.append(node.x)
                edge_y2.append(node.y)

        return path_x, path_y, edge_x1, edge_y1, edge_x2, edge_y2
