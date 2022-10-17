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
    from Robot_draw_lib import *
except ImportError:
    raise




def main():
    start = [0,1]
    
    ob_d = np.array([
                [1, 0],
                [1, 2],
                [2, 1],
                [3.5, 2],
                [3,0]
                 ])
    ob_u = np.array([
                [1, 4],
                [0.8, 3],
                [2, 1.5],
                [3, 3],
                [3, 4]
                 ])
    #line segment
    ls  = np.array( [[ob_d[1],ob_u[1]],
                     [ob_d[2],ob_u[2]],
                     [ob_d[3],ob_u[3]]
                    ])
                    
    end = [4,3]

    plotter = Plot_robot(title="The_approximate_shortest_path_illustration")
    plotter.plt.axis("off")   # turns off axes
    plotter.plt.axis("tight")  # gets rid of white border
    #plotter.plt.axis("image")  # square up the image instead of filling the "figure" space    

    plotter.point(start, "or")
    plotter.point(end, "or")
    
    plt.fill(ob_d[:,0], ob_d[:,1], color = 'k', alpha = 0.2, hatch='/')
    plt.fill(ob_u[:,0], ob_u[:,1], color = 'k', alpha = 0.3, hatch='/')
    plt.axis("equal")
    #plt.grid(True)

    plt.text(start[0] - 0.1, start[1] - 0.2, "A({0},{1})".format(start[0], start[1]))
    plt.text(end[0] - 0.2, end[1] + 0.1, "B({0},{1})".format(end[0], end[1]))
    #plt.text(start[0] + 0.1, start[1] + 0.2, "P0")
    #plt.text(end[0] - 0.2, end[1]- 0.4, "P4")    
    p1 = mid_point(ls[0][0], ls[0][1])
    p2 = mid_point(ls[1][0], ls[1][1])
    p3 = mid_point(ls[2][0], ls[2][1])
    
    config_space = np.array([
                    start, 
                    ob_d[1], ob_d[2], ob_d[3],
                    end,
                    ob_u[3], ob_u[2], ob_u[1],
                    ])
    imageID = 2
    capture_file_extension = "svg"
    capture_file_extension = "eps"
    capture_file_extension = "pdf"
    capture_file_extension = "png"

    if imageID == 0:
        draw_config_space = 0
        draw_line_segment = 0
        shortest_path_step_1 = 0
        shortest_path_final_step  = 0
        shortest_path = 0
        capture_file_name = "The_approximate_shortest_path_illustration_0.{0}".format(capture_file_extension) 
    elif imageID == 1:
        draw_config_space = 1
        draw_line_segment = 1
        shortest_path_step_1 = 1
        shortest_path_final_step  = 0
        shortest_path = 0
        capture_file_name = "The_approximate_shortest_path_illustration_1.{0}".format(capture_file_extension) 
    else:
        draw_config_space = 1
        draw_line_segment = 1
        shortest_path_step_1 = 0
        shortest_path_final_step  = 0
        shortest_path = 1
        capture_file_name = "The_approximate_shortest_path_illustration_2.{0}".format(capture_file_extension) 
    
    

    # draw config space
    if draw_config_space:
        plt.fill(config_space[:,0], config_space[:,1], color = "g", alpha = 0.4)
        
    #draw line segment
    if draw_line_segment:
        #
        i = 0
        for line in ls:
            plotter.line_segment(line, ls="-b")
            spt = line [0]
            ept = line [1]
            if i == 0:
                plt.text(ept[0] - 0.2, ept[1] + 0.1, "ce{0}".format(i))
            elif i == 1:
                plt.text(ept[0] - 0.15 , ept[1] + 0.2, "ce{0}".format(i))
            elif i == 2:
                plt.text(ept[0] + 0.1 , ept[1] + 0.1, "ce{0}".format(i))
            i = i + 1
        
    #shortest path step 1
    if shortest_path_step_1:

        plt.text(p1[0] , p1[1]+0.1, "P1") 
        plt.text(p2[0] + 0.1 , p2[1]-0.1, "P2") 
        plt.text(p3[0] , p3[1]+0.1, "P3") 
        
        plotter.line_segment((start,p1), ls="--k")
        plotter.line_segment((p1, p2), ls="--k")

        plotter.line_segment( (start,ls[0][0]), ls="-r")
        plotter.line_segment( (ls[0][0], p2), ls="-r")
        plotter.line_segment( (p3, p2), ls="-r")
        plotter.line_segment( (p3, end), ls="-r")
        
        plotter.point(p1, ls = ".b")
        plotter.point(p2, ls = ".b")
        plotter.point(p3, ls = ".b")
        plotter.point( ls[0][0], ls = "or")
        plt.text(ls[0][0][0]- 0.2, ls[0][0][1]+ 0.085, "P1_new")
 
    #shortest path final step 
    if shortest_path_final_step:
        p3_final = line_intersection((ls[1][1],end), ls[2])
        plotter.line_segment( (start,ls[0][0]), ls="-r")
        plotter.line_segment( (ls[0][0], ls[1][1]), ls="-r")
        plotter.line_segment( (ls[1][1], end), ls="-r")
        plotter.line_segment( ls[0][0], ls = ".r")
        plotter.line_segment( ls[1][1], ls = ".r")
        plotter.line_segment( p3_final, ls = ".r")
        plt.text(ls[0][0][0], ls[0][0][1] + 0.1, "P1")
        plt.text(ls[1][1][0] + 0.1, ls[1][1][1] - 0.1, "P2")
        plt.text(p3_final[0], p3_final[1] + 0.1, "P3")
        
    #shortest path
    if shortest_path:
        plotter.line_segment( (start,end), ls="-..r")
        plotter.line_segment( (start,ls[0][0]), ls="-r")
        plotter.line_segment( (ls[0][0], ls[1][1]), ls="-r")
        plotter.line_segment( (ls[1][1], end), ls="-r")
    
    #plt.show()
    plt.savefig(capture_file_name, 
            bbox_inches ="tight",
            dpi=150)

if __name__ == '__main__':
    main()
