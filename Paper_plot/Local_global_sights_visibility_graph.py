"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
"""
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

import matplotlib.pyplot as plt
try:
    from Robot_main import *
except ImportError:
    raise


if __name__ == '__main__':
    # get user input
    # scenario 0: local sight -n 1 -sx 50 -sy 50 -gx 90 -gy 90
    # scenario 1: local sight -n 2 -sx 50 -sy 50 -gx 90 -gy 90
    # scenario 2: global_sight -n 6 -gx 90 -gy 90
    # scenario 3: 5_robots_demonstration -n 1 -gx 20 -gy 50
    # scenario 4: 5_robots_demonstration_result -n 0 -gx 20 -gy 50

    scenario = 4

    num_iter = 1
    map_name = "_map.csv"
    world_name = None
    # get start point and goal point
    start = 0, 0
    goal = 90, 90
    robot_radius = 0.3
    robot_vision = 20
    robot_type=RobotType.circle
    if scenario == 0:
        start = 50, 50
        num_iter = 1
    elif scenario == 1:
        start = 50, 50
        num_iter = 2
    elif scenario == 2:
        num_iter = 6
    elif scenario == 3:
        goal = 20, 50
        num_iter = 1
    elif scenario == 4:
        goal = 20, 50
        num_iter = 0
    # run robot
    robot_main(start, goal, map_name, world_name, num_iter, robot_vision, robot_type, robot_radius)