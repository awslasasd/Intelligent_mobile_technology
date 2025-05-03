from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs
import time
from RRTStar import RRTStar
from DWA import DynamicWindowApproach
import math

if __name__ == '__main__':
    vision = Vision()  # 初始化视觉感知模块
    action = Action()  # 初始化机器人动作控制模块
    debugger = Debugger()  # 初始化调试工具
    time.sleep(0.01)  # 稍作延时，确保初始化完成
    planner = RRTStar()  # 创建 RRT* 规划器实例
    time.sleep(0.01)  # 再次延时，避免冲突

    # 设置目标点坐标
    goal_x, goal_y = -2400, -1500
    print("start")  # 打印开始标志

    flag = 0  # 计数器，用于控制循环次数
    again = True  # 控制是否需要重新规划路径

    while flag < 10:  # 机器人往返 10 次

        while True:  # 持续进行路径规划，直到找到可行路径
            # 获取当前机器人位置
            start_x, start_y = vision.my_robot.x, vision.my_robot.y
            
            # 使用 RRT* 进行路径规划，获取路径点
            path_x, path_y, draw_x1, draw_y1, draw_x2, draw_y2 = planner.execute_rrt_star(start_x=start_x, start_y=start_y, goal_x=goal_x, goal_y=goal_y,vision=vision)

            # 如果成功找到路径（路径点列表不为空），则跳出循环
            if len(path_x) > 0 and len(path_y) > 0:
                again = True
                break

        # 存储机器人执行路径的轨迹点
        traj_x = []  # 存储该轮规划的轨迹点
        traj_y = []  
            
        # 创建 DWA 轨迹跟踪实例，并设置目标点
        dwa = DynamicWindowApproach(goal_x, goal_y, v_max=1500, v_min = 0)
        dwa.update_arrival_tolerance(500) # 设置 DWA 的目标阈值
        
        # 遍历路径点，逆序执行（从终点开始往回走）
        for target_x, target_y in zip(reversed(path_x), reversed(path_y)):  # visit target points in correct order   

            dwa.update_goal(target_x, target_y)  # 更新目标点
            pre_x,pre_y = target_x, target_y  # 记录前一个目标点

            while True:
                # 更新传感器数据（检测周围环境）
                dwa.update_sensor_data(vision)

                # 控制障碍物移动
                action.controlObs(vision)

                # 计算最优速度和角速度
                vx, vw= dwa.find_optimal_motion()

                # 如果目标点接近路径的第六个点，减少到达容忍度
                if target_x == path_x[5] and target_y == path_y[5]:
                    dwa.update_arrival_tolerance(200) # 500

                # 检查是否已经到达最终目标点
                if dwa.has_reached_goal(goal_x,goal_y):
                    break  # 退出循环，进入下一个路径点导航

                # 检测机器人是否停滞
                if dwa.if_stop():
                    print("Replanning due to stop condition...")  # 发生停滞，重新规划路径
                    again = False
                    break  # 退出循环，重新进行路径规划

                # 发送运动控制指令
                action.sendCommand(vx=vx, vy=0, vw=vw)

                if again == False:
                    break  # 重新规划路径

                # 绘制调试信息
                package = Debug_Msgs()
                debugger.draw_points(package, path_x, path_y)  # 绘制路径点
                debugger.draw_circle(package, target_x, target_y, radius=100)  # 标记当前目标点
                traj_x.append(vision.my_robot.x)  # 记录轨迹点
                traj_y.append(vision.my_robot.y)
                debugger.draw_points2(package,traj_x, traj_y)  # 绘制轨迹
                debugger.send(package)  # 发送调试信息

                # 记录当前机器人位置
                pre_x,pre_y = vision.my_robot.x, vision.my_robot.y
                
            # 机器人到达当前路径点后停止
            action.sendCommand(vx=0, vy=0, vw=0)
        
        # 如果机器人顺利到达目标点，切换目标位置
        if again:

            goal_x=-goal_x  # 翻转目标点的 x 坐标
            goal_y=-goal_y  # 翻转目标点的 y 坐标
            path_x=[]  # 清空路径数据
            path_y=[]
            
            # 获取当前机器人位置
            start_x, start_y = vision.my_robot.x, vision.my_robot.y

            # 计算目标方向
            theta = math.atan2((goal_y - start_y), (goal_x - start_x))
            origen = vision.my_robot.orientation  # 获取当前机器人朝向
            w_error = abs(origen - theta)  # 计算当前朝向与目标方向的误差

            # 选择旋转方向
            if w_error > math.pi :
                w2 = -3.0  # 逆时针旋转
            else :
                w2 = 3.0  # 顺时针旋转

            # 发送旋转指令
            action.sendCommand(vx=0, vy=0, vw=w2)

            # 等待旋转到合适方向
            while abs(origen - theta) > 0.2:
                origen = vision.my_robot.orientation  # 更新朝向

            # 停止旋转
            action.sendCommand(vx=0, vy=0, vw=0)
            time.sleep(0.1)  # 稍作延时

            # 增加循环计数
            flag = flag + 1