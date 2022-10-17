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
except ImportError:
    raise

pt_offset = np.zeros((100,2))  # 100 offsets of point
plotter = Plot_robot(title="The_ASP_for_Autonomous_Robot_2_node")
def main():
    goal = [10,10]
    robot_vision = 2
    center_points = []
    center_points.append([1,0.5])
    center_points.append([2,2.2])
    center_points.append([4,2.2])
    # read map 
    
    obstacles_data = Obstacles()
    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles_data.read(None, "_paper_map.csv")
    obstacles = np.array(obstacles_data.data())
    
    start_line = np.array([
                    [0.5, 1.5],
                    [0.5, 4]
                    ])
    
    end_line = np.array([
                    [4.7, 0],
                    [4.7, 2]
                    ])
    spts = intersection(center_points[0][0], center_points[0][1], robot_vision, start_line)
    epts = intersection(center_points[-1][0], center_points[-1][1], robot_vision, end_line) 
    
    if inside_line_segment(spts[0], start_line):
        start = spts[0]
    else:
        start = spts[1]
        
    if inside_line_segment(epts[0], end_line):
        end = epts[0]
    else:
        end = epts[1]
        
    skeleton_path = []
    skeleton_path.append(start)
    skeleton_path.append(center_points[0])
    skeleton_path.append(center_points[1])
    skeleton_path.append(center_points[2])
    skeleton_path.append(end)

    csights = []
    osights = []
    robot = Robot(start=start, vision_range=robot_vision)

    for center_point in center_points:
        robot.coordinate = center_point
        cs, os = scan_around(robot, obstacles, goal)
        csights.append(cs)
        osights.append(os)
    
    # draw circle range
    # image__1
    draw_critical_edge_temp = 0
    draw_circle_range_temp = 1
    skeleton_path_temp = 1
    strange_line_temp = 0
    collision_free_area_temp = 0
    final_approximate_shortest_path_temp = 0
    crosspoint_temp = 0

    # image__2
    draw_critical_edge_temp = 1
    draw_circle_range_temp = 1
    skeleton_path_temp = 1
    strange_line_temp = 0
    collision_free_area_temp = 1
    final_approximate_shortest_path_temp = 1
    crosspoint_temp = 0

    # image__3
    # draw_critical_edge_temp = 1
    # draw_circle_range_temp = 1
    # skeleton_path_temp = 1
    # strange_line_temp = 0
    # collision_free_area_temp = 1
    # final_approximate_shortest_path_temp = 1
    # crosspoint_temp = 1

    if draw_circle_range_temp:
        for center_point in center_points:
            plotter.vision_area(center_point, robot_vision)
            #plotter.vision_area(center_point, robot_vision/2)
  
    for ob_part in obstacles:
        plt.fill(ob_part[:,0], ob_part[:,1], color = 'k', alpha = 0.4, hatch='//')
    
    # text points
    plotter.point(start, ".b")
    plotter.point(end, ".b")
    for pt in center_points:
        plotter.point(pt, ".b")
    plt.text(center_points[0][0] , center_points[0][1] - 0.2, "a")
    plt.text(center_points[1][0] +0.1, center_points[1][1] +0.2, "b")
    plt.text(center_points[2][0] +0.1, center_points[2][1] +0.2, "c")
        
    plt.text(start[0], start[1]+0.2, "s")
    plt.text(end[0], end[1]-0.2, "e")

    
    
    traversal_sight = []
    i = 0
    for sight in osights: 
        traversal_sight.append([center_points[i], csights[i], osights[i]])
        i = i + 1

    asp, critical_ls = approximately_shortest_path(skeleton_path, traversal_sight, robot_vision)

    if draw_critical_edge_temp: # draw critical edge
        # draw plot_critical_line_segments
        i = 0
        j = 0
        for ls in critical_ls:
            pt = ls[1]
            plotter.line_segment( ls[1:3], "-b")
            if 0:
                if i < 3:
                    plt.text(pt[0], pt[1] +0.1, "CE_a{0}".format(j))
                elif i < 7:
                    plt.text(pt[0] , pt[1] - 0.1, "CE_b{0}".format(j))
                else:
                    plt.text(pt[0] , pt[1] + 0.1, "CE_c{0}".format(j))                
                i += 1
                if i == 2 or i == 6:
                    j = 0
                else:
                    j += 1
                
            
    if skeleton_path_temp:    # skeleton path
        plotter.path(  skeleton_path, ls="--.b")
    
    if strange_line_temp: # strange line
        plotter.line_segment(  (start,end), ls="-..r")
    
    if collision_free_area_temp: # collision-free area
        pt = []
        pt.append(start)
        for ls in critical_ls:
            if i == 3:
                pt.append(center_points[1])
                pt.append(center_points[2])
                pt.append(end)
                break
            pt.append(ls[1])
            i += 1
        len_ce = len(critical_ls)
        for i in range(len_ce):
            ls = critical_ls[len_ce-i-1]
            pt.append(ls[1])
            if i == 6:
                break
        pt.append(center_points[0])
        pt = np.array(pt)
        
        plt.fill(pt[:,0], pt[:,1], color = "g", alpha = 0.3, ls="-")
        
    # final approximate shortest path
    if final_approximate_shortest_path_temp:
        plotter.path(  asp, "-r")
        i = 0
        pt_offset[0] = [ -0.2 ,  -0.2]
        pt_offset[1] = [ 0    ,  0.15]
        pt_offset[2] = [ 0    ,  0.05]
        pt_offset[3] = [ 0    ,  -0.1]
        pt_offset[4] = [ 0.05 ,  -0.1]
        pt_offset[5] = [ -0.1 ,   0.1]
        pt_offset[6] = [ 0    ,  0.15]
        pt_offset[7] = [ 0.05 ,  0.05]
        pt_offset[8] = [ 0    ,   0.1]
        pt_offset[9] = [ 0.1  , -0.05]
        pt_offset[10] = [0.1  ,     0]
        pt_offset[11] = [0.1  ,     0]
        pt_offset[12] = [0.05 ,   0.1]
        for pt in asp:
            textpt = pt + pt_offset[i]
            plt.text(textpt[0], textpt[1], "p{0}".format(i))
            i += 1
            plotter.point(  pt, ".k")
    pt3 = asp[3]
    pt4 = asp[4]
    ptA = center_points[0]
    ptB = center_points[1]
    print (ptA, ptB, pt3, pt4)
    crosspoint = line_intersection((ptA,ptB), (pt3,pt4))
    
    if crosspoint_temp: # crosspoint
        plotter.point(  crosspoint, ".r")
        plt.text(crosspoint[0],crosspoint[1]-0.2,"m")
        plotter.line_segment(  (start,crosspoint), ls=":k")
        plotter.line_segment(  (crosspoint,end), ls=":k")
    plt.axis("equal")
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main()
