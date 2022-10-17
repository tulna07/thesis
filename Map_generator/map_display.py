'''
AutonomousRobot
This project simulates our path-planning for an autonomous robot that tries to find a way to reach a goal (target)
in certainly environment 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
'''

import matplotlib.pyplot as plt
import sys
import os
import argparse
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

try:
    from Robot_map_lib import *
    from Obstacles import *
except ImportError:
    raise

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
            description="map (obstacle) display",)
    parser.add_argument("-m", metavar="map name", default="_map_temp.csv", help="display map from map_name data file")
    args = parser.parse_args()

    map_name = args.m
    
    map = Map()
    obstacles = Obstacles()
    
    plt.axis("equal")
    plt.grid(True)
    
    # read data obstacles
    obstacles.read_csv(map_name)
    # draw map obstacles 
    map.display(plt, map_name, obstacles.obstacles)


    plt.show()