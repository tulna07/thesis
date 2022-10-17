"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""

import numpy as np
from sklearn.metrics import top_k_accuracy_score

from Robot_math_lib import *
from Robot_paths_lib import *
from Robot_draw_lib import Plot_robot
from Robot_sight_lib import *
from Robot_map_lib import Map
from Obstacles import *
from Program_config import *
from Robot_class import Robot, RobotType
from Robot_ranking import Ranker
import argparse

def robot_main(start, goal, map_name, world_name, num_iter, robot_vision, robot_type, robot_radius):
    
    robot = Robot(start, robot_vision, robot_type, robot_radius)
    ranker = Ranker(alpha=0.9, beta= 0.1)

    # declare potter within window size
    plotter = Plot_robot(title="Path Planning for Autonomous Robot: {0}".format(map_name))
    
    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles = Obstacles()
    obstacles.read(world_name, map_name)

    # for display information
    iter_count = 0

    print("\nRobot is reaching to goal: {0} from start {1}".format(goal, start))

    while True:
        iter_count += 1
        print("\n_number of iteration:{0}, current robot coordinate{1}".format(iter_count, robot.coordinate))
        
        robot.update_coordinate(robot.next_coordinate)

        # clean old data
        next_point = []

        # scan to get sights at local
        #closed_sights, open_sights = scan_around(robot, obstacles.data(), goal)
        # get boundary points, which is intersection between circle and linesegments
        boundary_pts = get_boudary_points_theory(robot.coordinate, robot.vision_range, obstacles.obstacles, goal)

        # get arc points in active arc which is limited by parent_arc
        arc_pts, parent_arc = is_inside_active_arc(boundary_pts, robot.vision_range, robot.coordinate, goal)
        
        # get local open sights
        open_sights, arc_pts = get_open_sights_in_active_arc_theory(arc_pts, parent_arc, robot.vision_range, obstacles.obstacles, robot.coordinate)
        # closed sights
        closed_sights = []

        # check whether the robot saw or reach the given goal
        robot.check_goal(goal, closed_sights)
        #Robot.show_status()

        if not robot.saw_goal and not robot.reach_goal:
            # get local active point and its ranking
            robot.get_local_active_open_ranking_points(open_sights, ranker, goal)

            # stack local active open point to global set
            robot.expand_global_open_ranking_points(robot.local_active_open_rank_pts)
            

            # add new active open points to graph_insert
            # add new active open points to graph_insert
            robot.visibility_graph.add_local_open_points(robot.coordinate, robot.local_active_open_pts)
            #graph_add_lOpenPts(robot.visibility_graph, robot.coordinate, robot.local_active_open_pts)

            # pick next point to make a move
            next_point, next_pt_idx = robot.pick_next_point(robot.global_active_open_rank_pts, goal)

            if next_point is not None:
                # find the shortest skeleton path from current position (center) to next point
                skeleton_path = robot.visibility_graph.BFS_skeleton_path(robot.coordinate, tuple(next_point))
                #skeleton_path = BFS_skeleton_path(robot.visibility_graph, robot.coordinate, tuple(next_point))

                # then remove picked point from active global open point
                robot.remove_global_active_pts_by_index(next_pt_idx)
            else:
                print("No way to reach the goal!")
                robot.is_no_way_to_goal(True)

        else:
            next_point = goal
            # find the shortest path from center to next point
            skeleton_path = [robot.coordinate, goal]

        # record the path and sight
        robot.expand_traversal_sights(closed_sights, open_sights)

        if print_traversalSights:
            robot.print_traversal_sights()

        asp, critical_ls = approximately_shortest_path(skeleton_path, robot.traversal_sights, robot.vision_range)

        # mark visited path
        robot.expand_visited_path(asp)

        # make a move from current position
        if not robot.no_way_to_goal:
            robot.next_coordinate = motion(robot.coordinate, next_point)  # simulate robot

        if show_animation:
            # clear plot
            plotter.clear()
            
            # for stopping simulation with the esc key.
            plotter.plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
                      
            '''draw map obstacles/world '''            
            # prepare title
            cost = robot.calculate_traveled_path_cost()
            status_title = plotter.prepare_title(iter_count, cost)
            plotter.show_map(world_name=None, obstacles=obstacles, plot_title=status_title)
            
            ########################################################
            # active point (Assumptions of Hoai_An's theory)
            #########################################################
            if show_active_openpt_HA_Assumption:
                if show_circleGoal_HA_Assumption:
                    pdist_cg = point_dist(robot.coordinate, goal)
                    plotter.vision_area(goal, pdist_cg)
                if show_boundaryPts_HA_Assumption and len(boundary_pts) > 0:
                    plotter.points(boundary_pts, ls = ls_bp_HA)
                if show_parentArc_HA_Assumption and parent_arc is not None:
                    plotter.points(parent_arc, ls = ls_pap_HA)
                if show_activeArc_HA_Assumption and arc_pts is not None:
                    plotter.points(arc_pts, ls = ls_aap_HA)
                    if show_activeArcPts_order_HA_Assumption:
                        for i in range(len(arc_pts)):                            
                            plotter.text(arc_pts[i], "{0}".format(i))

            # show_traversalSights
            if show_traversalSights:
                plotter.show_traversal_sights(robot.traversal_sights, robot.vision_range)
            
            if show_robot:
                plotter.robot(robot)
            
            if show_goal:
                plotter.goal(goal, robot.reach_goal, robot.saw_goal)
            
            # plot robot's vision at local (center)
            plotter.vision(robot.coordinate, robot.vision_range, closed_sights, open_sights)
            
            if show_local_openpt and len(robot.local_open_pts) > 0:
                plotter.points(robot.local_open_pts, ls_lopt)
            
            if show_active_openpt and len(robot.global_active_open_rank_pts) > 0:
                plotter.points(robot.global_active_open_rank_pts, ls_aopt)
            
            if show_visibilityGraph:
                plotter.visibility_graph(robot.visibility_graph, ls_vg)
            
            if show_visitedPath:
                plotter.paths(robot.visited_paths, ls_vp, ls_goingp)
            
            if show_sketelonPath:
                plotter.path(skeleton_path, ls_sp)
            
            if show_approximately_shortest_path:
                plotter.path(asp, ls_asp)
            
            if show_critical_line_segments:
                plotter.critical_line_segments(critical_ls, ls_cls)
            
                # display next point if existing
            if show_next_point:
                if len(next_point) > 0:
                    plotter.point(next_point, ls_nextpt)
            
            # to set equal make sure x y axises are same resolution 
            plotter.set_equal()
            plotter.show_grid()
            plotter.plt.pause(1)


        robot.print_infomation()

        # Run n times for debugging
        if  iter_count == num_iter:
            break
        
        if robot.finish():
            break

    print("Done")
    plotter.show()


if __name__ == '__main__':
    
    
    parser = argparse.ArgumentParser(description='Code for Autonomous Robot.')
    parser.add_argument('-n', metavar="number of iteration", type=int, help='number of iteration', default=0)
    parser.add_argument('-m', metavar="data_map", help='map data', default='_forest.csv')
    parser.add_argument('-w', metavar="world_image", help='world model')
    parser.add_argument('-r', metavar="vision_range", type=float, help='vision range', default=100.0)
    parser.add_argument('-radius', metavar="robot radius", type=float, help='robot radius', default=0.2)
    parser.add_argument('-sx', metavar="start_x", type=float, help='start point x', default=0.0)
    parser.add_argument('-sy', metavar="start_y", type=float, help='start point y', default=0.0)
    parser.add_argument('-gx', metavar="goal_x", type=float, help='goal point x', default=499.0)
    parser.add_argument('-gy', metavar="goal_y", type=float, help='goal point y', default=499.0)
    menu_result = parser.parse_args()

    # get user input
    num_iter = menu_result.n
    map_name = menu_result.m
    world_name = menu_result.w
    # get start point and goal point
    start = menu_result.sx, menu_result.sy
    goal = menu_result.gx, menu_result.gy
    robot_radius = menu_result.radius
    robot_vision = menu_result.r 
    robot_type=RobotType.circle

    # run robot
    robot_main(start, goal, map_name, world_name, num_iter, robot_vision, robot_type, robot_radius)
