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
    from Robot_paths_lib import *
    from Robot_draw_lib import *
    from Robot_sight_lib import *
    from Robot import Robot
    from Obstacles import Obstacles
    from Robot_ranking import Ranker
    from Robot_map_lib import Map
except ImportError:
    raise


def main():
    map_name = "_map.csv"
    robot_vision = 20
    ranker = Ranker()
    plotter = Plot_robot(title="The_ASP_for_Autonomous_Robot_2_node")
    # display map
    obstacles = Obstacles()
    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles.read(None, map_name)
    Map().display(plt, map_name, obstacles.data())

    start = (0,0)
    goal = (90,90)
    
    center_points = np.array([[0, 0],
                       [-5, 90],
                       [50, 50.0],
                       [90, 10.0]
                       ])
    vision_list = []
    robot = Robot(start=start, vision_range=robot_vision)

    for center in center_points:
        robot.coordinate = center
        csight, osight  = scan_around(robot, obstacles.data(), goal)
        osight = np.array(osight)
        open_local_pts = osight[:, 2]    # open_local_pts
        
        ranks_new = np.array([ranker.rank(center, pt, goal) for pt in open_local_pts])
        ao_local = np.concatenate((open_local_pts, ranks_new), axis=1)
        next_pt, _  = robot.pick_next_point(ao_local)
        vision_list.append([osight, csight, open_local_pts, next_pt])

    plotter.goal(goal)            
    
    for i in range(len(center_points)):
        center = center_points[i]
        osight = vision_list[i][0]
        csight = vision_list[i][1]
        open_local_pts = vision_list[i][2]
        next_pt = vision_list[i][3]
        # display vision
        robot.coordinate = center
        plotter.vision(robot.coordinate, robot.vision_range, csight, osight)
        
        plotter.points(open_local_pts, ls_aopt)
        if i == 0:
            textpoint = (center[0] + 1 , center[1] )
        elif i == 1:
            textpoint = (center[0] -15, center[1] - 5)
        elif i == 2:
            textpoint = (center[0] +2 , center[1] + 2)
        elif i == 3:
            textpoint = (center[0] -17, center[1])

        plt.text(textpoint[0], textpoint[1], "center_{0}".format(i))
                    
        # display next point if existing
        if len(next_pt) > 0:
            plotter.point(next_pt, "or")
        plotter.robot(robot)

        plt.axis("equal")
    plt.show()

if __name__ == '__main__':
    main()