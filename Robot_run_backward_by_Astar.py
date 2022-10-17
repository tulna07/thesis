"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""

from Robot_math_lib import *
from Robot_paths_lib import *
from Robot_draw_lib import Plot_robot
from Robot_sight_lib import *
from Obstacles import *
from Program_config import *
from Robot_ranking import Ranker, Ranking_function
from Robot_class import Robot
from Robot_base import RobotType
import argparse
from a_star import main as A_star_planner
from Result_log import Result_Log
import time
from datetime import datetime

def robot_main( start, goal, map_name, world_name, num_iter, 
                robot_vision, robot_type, robot_radius, 
                ranking_function =Ranking_function.Angular_similarity):
    
    robot = Robot(start=start, goal=goal, vision_range= robot_vision, \
        robot_type=robot_type, robot_radius=robot_radius)
    ranker = Ranker(alpha=0.9, beta= 0.1, ranking_function=ranking_function)


    # declare potter
    plotter = Plot_robot(title="Path Planning for Autonomous Robot: {0}".format(map_name))
    
    # result records 
    experiment_results = Result_Log(header_csv=["asp_time","asp_path_cost","Astar_time","Astar_path_cost"])

    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles = Obstacles()
    obstacles.read(world_name, map_name)
    #obstacles.find_configuration_space(robot.radius)

    # for display information
    iter_count = 0
    if not easy_experiment: # skip printing if running easy experiment
        print("\nRobot is reaching to goal: {0} from start {1}".format(goal, start))
    # Astar _Paths
    Astar_paths = []
    while True:
        iter_count += 1
        robot.update_coordinate(robot.next_coordinate)
    
        if not easy_experiment: # skip printing if running easy experiment
            print("\n_number of iteration: {0}, current robot coordinate {1}".format(iter_count, robot.coordinate))

        # clean old data
        next_point = []
        robot.clear_local()

        # scan to get sights at local
        closed_sights, open_sights = scan_around(robot, obstacles, goal)
        
        # check whether the robot saw or reach the given goal
        robot.check_goal(goal, closed_sights)
        
        #robot.show_status()
        if not robot.saw_goal and not robot.reach_goal:
            # get local active point and its ranking
            robot.get_local_active_open_ranking_points(open_sights, ranker, goal)

            # stack local active open point to global set
            robot.expand_global_open_ranking_points(robot.local_active_open_rank_pts)
            
            # add new active open points to graph_insert
            robot.visibility_graph.add_local_open_points(robot.coordinate, robot.local_active_open_pts)
        
        # pick next point to make a move
        next_point, next_pt_idx = robot.pick_next_point(robot.global_active_open_rank_pts, goal)

        if next_point is not None:
            # find the shortest skeleton path from current position (center) to next point
            if tuple(next_point) == tuple(goal):
                skeleton_path = [robot.coordinate, goal]
            else:
                skeleton_path = robot.visibility_graph.BFS_skeleton_path(robot.coordinate, tuple(next_point))

                # then remove picked point from active global open point
                robot.remove_global_active_pts_by_index(next_pt_idx)
        else:
            robot.is_no_way_to_goal(True)

        # record the path and sight
        robot.expand_traversal_sights(closed_sights, open_sights)

        Astar_asp= []
        # compare a start
        if len(skeleton_path) > 2:
            print ("____backward path:")
            obstacles_Astar = []
            #robot.visibility_graph.get_all_non_leaf()
            visited_area = robot.visibility_graph.get_all_non_leaf()
            for x,y in visited_area:
                for i in range (int(x- robot_vision -3), int(x + robot_vision +3)):
                    for j in range (int(y- robot_vision -3),int(y + robot_vision +3)):
                        pt = (i,j)
                        if not inside_global_true_sight(pt, robot_vision+1, robot.traversal_sights):
                            if pt not in obstacles_Astar:
                                obstacles_Astar.append(pt)
            #for x,y in range ([center_area[0]-robot_v)
            obstacles_x_Astar = []
            obstacles_y_Astar = []
            for ob in obstacles_Astar:
                obstacles_x_Astar.append(ob[0])
                obstacles_y_Astar.append(ob[1])
            Astar_start_time = time.time()
            Astar_asp = A_star_planner(start=skeleton_path[0], goal=skeleton_path[-1], ox=obstacles_x_Astar, 
                oy=obstacles_y_Astar, robot_radius=robot.radius, plt= plotter.plt)
            Astar_path_cost = path_cost(Astar_asp)
            #print ("ASTART _PATHCOST= ", path_cost(Astar_asp))
            Astar_end_time = time.time()
        asp_start_time = time.time()
        asp, critical_ls = approximately_shortest_path(skeleton_path, robot.traversal_sights, robot.vision_range)
        asp_end_time = time.time()
        asp_path_cost = path_cost(asp)
        #print ("Autonomous Robot _PATHCOST= ", path_cost(asp))
        
        if len(skeleton_path) == 2:
            Astar_asp = skeleton_path
        else:
            Astar_time = Astar_end_time-Astar_start_time
            asp_time = asp_end_time-asp_start_time
            experiment_results.add_result([asp_time,asp_path_cost,Astar_time,Astar_path_cost])
        # mark visited path
        robot.expand_visited_path(asp)
        Astar_paths.append(Astar_asp)
        # make a move from current position
        if not robot.no_way_to_goal:
            robot.next_coordinate = motion(robot.coordinate, next_point)  # simulate robot

        if show_animation:
            plotter.show_animation(robot, world_name, iter_count, obstacles , goal, 
                    closed_sights, open_sights, skeleton_path, asp , critical_ls, next_point)
            if len(Astar_asp)>0:
                plotter.path(Astar_asp, "-.b")
        
        astart_cost = 0.0
        for astar_path in Astar_paths:
            astart_cost += path_cost(astar_path)
        print ("COMPARE: our algorithms [path cost]", robot.calculate_traveled_path_cost())
        print ("COMPARE: our algorithms (within Astar backward path)[path cost]", astart_cost)
        robot.print_infomation()
        # Run n times for debugging
        if  iter_count == num_iter:
            break
        
        if robot.finish():
            break
        
    if not easy_experiment: # skip printing and showing animation if running easy experiment
        print("Done")
        if show_animation:
            plotter.show()

    elif save_image:
        # showing the final result (for save image and display as well)
        plotter.show_animation(robot, world_name, iter_count, obstacles , goal, 
                    closed_sights, open_sights, skeleton_path, asp , critical_ls, next_point)
        fig_name = set_figure_name(range=robot.vision_range, start=start, goal=goal, picking_strategy=g_strategy)
        plotter.save_figure(fig_name, file_extension=".png")
        plotter.save_figure(fig_name, file_extension=".pdf")
        print ("Saved: {0}.pdf".format(fig_name))

    # log time and past cost
    if len(experiment_results.results_data) > 0:
        result_file= "result_Astar_ASP_{0}.csv".format(datetime.now().strftime("%m_%d_%H_%M_%S") )
        experiment_results.write_csv(file_name=result_file)
        print ("saved: {0}".format(result_file))
        print ("\nTo visualize the result run:\n" +
              "python Robot_main_backward_by_Astar_ASP_plot.py -r {0}".format(result_file))
    return robot
    



if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='Code for Autonomous Robot.')
    parser.add_argument('-n', metavar="number of iteration", type=int, help='number of iteration', default=1)
    parser.add_argument('-m', metavar="data_map", help='map data', default='_map.csv')
    parser.add_argument('-w', metavar="world_image", help='world model')
    parser.add_argument('-r', metavar="vision_range", type=float, help='vision range', default=20.0)
    parser.add_argument('-radius', metavar="robot radius", type=float, help='robot radius', default=0.5)
    parser.add_argument('-sx', metavar="start_x", type=float, help='start point x', default=0.0)
    parser.add_argument('-sy', metavar="start_y", type=float, help='start point y', default=0.0)
    parser.add_argument('-gx', metavar="goal_x", type=float, help='goal point x', default=50.0)
    parser.add_argument('-gy', metavar="goal_y", type=float, help='goal point y', default=50.0)
    menu_result = parser.parse_args()

    # get user input
    num_iter = menu_result.n
    map_name = menu_result.m
    #map_name = r"Map_generator\_map_temp.csv"
    world_name = menu_result.w

    # get start point and goal point
    start = menu_result.sx, menu_result.sy
    goal = menu_result.gx, menu_result.gy
    robot_radius = menu_result.radius
    robot_vision = menu_result.r
    robot_type = RobotType.circle

    # run robot
    robot_main(start=start, goal=goal, map_name=map_name, world_name=world_name,\
        num_iter=num_iter, robot_vision=robot_vision, robot_type=robot_type, robot_radius=robot_radius)
