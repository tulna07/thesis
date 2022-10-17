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
except ImportError:
    raise


pt_offset = np.zeros((100,2))  # 100 offsets of point
plotter = Plot_robot(title="The_ASP_for_Autonomous_Robot_1_node")

def main():
    
    robot_vision = 3
    ob = np.array([
                [4, 0],
                [4, 1],
                [3.8, 0.3],
                [3.2, 0.3],
                [2.8, 2.2],
                [2, 1.5],
                [1.5, 2.5],
                [0,2],
                [0,0]
                 ])
    center = [3, 3.5]
    center1 = [4, 4]
    obstacles = []
    obstacles.append(ob)
    start_line = np.array([
                    [3.7, 0],
                    [3.7, 2]
                    ])
    
    end_line = np.array([
                    [0.05, 0],
                    [0.05, 4]
                    ])
    spts = intersection(center[0], center[1], robot_vision, start_line)
    epts = intersection(center[0], center[1], robot_vision, end_line) 
    
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
    skeleton_path.append(center)
    skeleton_path.append(end)

    robot = Robot(start=start, vision_range=robot_vision)
    robot.coordinate = center
    csights_0, osights_0 = scan_around(robot, obstacles, end)
    robot.coordinate = center1
    csights_1, osights_1 = scan_around(robot, obstacles, end)
    
    plotter.vision_area(center, robot_vision)
    
    # map drawing
    plt.fill(ob[:,0], ob[:,1], color = 'k', alpha = 0.4, hatch='//')
    
    traversal_sight = []
    traversal_sight.append([center, csights_0, osights_0])
    traversal_sight.append([center1, csights_1, osights_1])
    asp, critical_ls = approximately_shortest_path(skeleton_path, traversal_sight, robot_vision)

    if 1: # approximate path and critical edge
        # draw plot_critical_line_segments
        i = 0
        pt_offset[0] = [0.05, -0.1]
        pt_offset[1] = [-0.3, -0.4]
        pt_offset[2] = [-0.1, -0.1]
        pt_offset[3] = [-0.3, -0.2]
        pt_offset[4] = [ 0.1, -0.1]
        pt_offset[5] = [-0.1, -0.2]
        for ls in critical_ls:
            pt = ls[1]
            plotter.line_segment(ls[1:3], "-b")
            textpt = pt + pt_offset[i]
            plotter.plt.text(textpt[0], textpt[1], "CE_c{0}".format(i))
            i += 1
        # show_approximately_shortest_path
        if 0:
            plotter.path( asp, "-r")
            print ("________________")
            print (asp)
            print ("________________")
            i = 0
            for pt in asp:
                plotter.point( pt, ".r")
                plotter.plt.text(pt[0] , pt[1] , "P{0}".format(i))
                i = i + 1
    
    if 1: # collision-free area
        pt = []
        pt.append(start)
        i = 0
        for ls in critical_ls:
            pt.append(ls[1])
        pt.append(end)
        pt.append(center)
        pt = np.array(pt)
        
        plotter.plt.fill(pt[:,0], pt[:,1], color = "g", alpha = 0.3, ls="-")
        
    bound_pts = []
    for ls in critical_ls:
        bound_pts.append(ls[1])
 
    # get middle points
    mid_pts = []
    
    for pt in bound_pts:
        mid_pts.append(mid_point(center, pt))
    

        
    #plot_line(plt, (start,end), ls="-..r")
    
    plotter.plt.text(start[0] + 0.1, start[1] + 0.1, "s")
    plotter.plt.text(end[0] + 0.1, end[1] + 0.1, "e")
    plotter.plt.text(center[0] + 0.1, center[1] + 0.1, "c")
    
    # draw midpoint 
    if 0:
        i = 0
        for pt in mid_pts:
            plotter.point( pt, ls=".k")
            plt.text(pt[0] + 0.1, pt[1] + 0.1, "p{0}".format(i+1))
            i = i + 1
        plt.text(start[0] + 0.1, start[1] + 0.1, "p0")
        plt.text(end[0] + 0.1, end[1] + 0.1, "p{0}".format(i+1))
        
        
    # skeleton path
    plotter.path(skeleton_path, ls="--.b")
    

    # mid path
    if 0:
        plotter.line_segment((start,mid_pts[0]), "-r")
        plotter.line_segment( (end,  mid_pts[-1]), "-r")
        for i in range (len(mid_pts)-1):
            plotter.line_segment( (mid_pts[i],mid_pts[i+1]), ":r")
       
     
    # new path
    if 0:
        #new mid point
        new_mid_pts = []
        new_mid_pts.append(mid_point(mid_pts[1], mid_pts[0]))
        new_mid_pts.append(mid_point(mid_pts[1], mid_pts[2]))
        p_new = line_intersection(new_mid_pts, (center, bound_pts[1]))
        
        # draw new p1
        plotter.point( p_new, ls=".r")
        plt.text(p_new[0]+0.1, p_new[1] , "p2_new")
        mid_pts[1] = p_new
        for i in range (len(mid_pts)-1):
            plotter.line_segment( (mid_pts[i],mid_pts[i+1]), "-r")
        
    # final approximate shortest path
    if 1:
        plotter.path( asp, "-r")
        i = 0
        pt_offset[0] = [0, -0.1]
        pt_offset[1] = [+0.1, + 0.1]
        pt_offset[2] = [+0.1, + 0.1]
        pt_offset[3] = [-0.1, + 0.1]
        pt_offset[4] = [ +0.1, 0]
        pt_offset[5] = [-0.1, + 0.1]
        pt_offset[6] = [-0.2, -0.2]
        for pt in asp:

            textpt = pt + pt_offset[i]
            plt.text(textpt[0], textpt[1], "p{0}".format(i))
            i = i + 1
            plotter.point(pt, ".k")
        
    plt.axis("equal")
    #plt.grid(True)
    
    
    plt.show()

if __name__ == '__main__':
    main()
