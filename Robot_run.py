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
from Robot_base import Picking_strategy, Ranking_type, RobotType
from logging_ranking import Logging_ranking
from Tree import Node
from RRTree_star import RRTree_star
from Robot_user_input import menu_Robot

def robot_main( start, goal, map_name, world_name, num_iter, 
                robot_vision, robot_type, robot_radius, 
                ranking_type = Ranking_type.Distance_Angle,
                ranking_function =Ranking_function.Angular_similarity,
                picking_strategy= Picking_strategy.local_first):
    
    robot = Robot(start=start, goal=goal, vision_range= robot_vision, \
                    robot_type=robot_type, robot_radius=robot_radius)
    
    # set alpha and beta only for distance and angle formula
    if ranking_type == Ranking_type.Distance_Angle and ranking_function == Ranking_function.RHS_RRT_base:
        ranking_function = Ranking_function.Angular_similarity
    ranker = Ranker(alpha=0.9, beta= 0.1, ranking_function=ranking_function)

    # declare plotter
    title = "Autonomous Robot Path-Planning: {0}".format(map_name)
    plotter = Plot_robot(title="Path Planning for Autonomous Robot: {0}".format(map_name))
    
    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles = Obstacles()
    obstacles.read(world_name, map_name)
    #obstacles.find_configuration_space(robot.radius)
 
    ''' generate a RRTree_star if ranking type is RRTree_Star_ranking '''
    if ranking_type == Ranking_type.RRTstar:
            # RRT start for ranking scores
        step_size = robot.vision_range
        sample_size = 2000
        # logging ranking
        rank_logger = Logging_ranking()
        r_logger_filename = rank_logger.set_logging_name(map_name=map_name, goal=goal,
            radius=robot_vision, step_size=step_size, sample_size=sample_size)

        # find working space boundary
        boundary_area = robot.find_working_space_boundaries(obstacles=obstacles)
        start_node = Node(goal, cost=0)            # initial root node, cost to root = 0
        if rank_logger.is_existed_log_file(r_logger_filename):
            print ("load existed-RRTreeStart_rank: ", r_logger_filename)
            RRT_star = rank_logger.load(r_logger_filename)
        else:
            print ("Generating new RRTree star then save to: ", r_logger_filename)
            RRT_star = RRTree_star(root=start_node, step_size=step_size, radius=robot.vision_range, 
                            random_area=boundary_area, sample_size=sample_size)
            RRT_star.build(goal_coordinate=start, plotter=plotter, obstacles=obstacles)

            # save ranking tree
            rank_logger.save_tree(RRTree_star=RRT_star, file_name=r_logger_filename)
    else:
        RRT_star = None


    iter_count = 0

    print("\nRobot is reaching to goal: {0} from start {1}".format(goal, start))
    print("Ranking type: {0}, picking strategy: {1}".format(ranking_type, picking_strategy))

    while True:
        iter_count += 1
        robot.update_coordinate(robot.next_coordinate)
    
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
            robot.get_local_active_open_ranking_points(open_sights=open_sights, ranker=ranker, goal=goal,\
                                                        RRT_star=RRT_star, ranking_type=ranking_type)
            # stack local active open point to global set
            robot.expand_global_open_ranking_points(robot.local_active_open_rank_pts)
            
            # add new active open points to graph_insert
            robot.visibility_graph.add_local_open_points(robot.coordinate, robot.local_active_open_pts)
        # pick next point to make a move
        next_point = robot.pick_next_point(goal, picking_strategy=picking_strategy)
        if next_point is not None:
            # find the shortest skeleton path from current position (center) to next point
            if tuple(next_point) == tuple(goal):
                skeleton_path = [robot.coordinate, goal]
            else:
                skeleton_path = robot.visibility_graph.BFS_skeleton_path(robot.coordinate, tuple(next_point))
                print("\nskeleton path {0}".format(skeleton_path))
        else:
            skeleton_path = []
            robot.is_no_way_to_goal(True)

        # record the path and sight
        robot.expand_traversal_sights(closed_sights, open_sights)

        asp, critical_ls = approximately_shortest_path(skeleton_path, robot.traversal_sights, robot.vision_range)

        # mark visited path
        robot.expand_visited_path(asp)

        # make a move from current position
        if not robot.no_way_to_goal:
            #robot.next_coordinate = motion(robot.coordinate, next_point)  # simulate robot
            robot.next_coordinate = tuple(next_point)# motion(robot.coordinate, next_point)  # simulate robot

        if show_animation:
            plotter.show_animation(robot, world_name, iter_count, obstacles , goal, 
                    closed_sights, open_sights, skeleton_path, asp , critical_ls, next_point)
            #plotter.tree_all_nodes(RRTx)
            if ranking_type == Ranking_type.RRTstar:
                plotter.tree(RRT_star,color_mode=TreeColor.by_cost)

        
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
        fig_name = set_figure_name(map_name=map_name, range=robot.vision_range, start=start, 
            goal=goal, picking_strategy=picking_strategy, ranking_function=ranking_function)
        plotter.save_figure(fig_name, file_extension=".png")
        #plotter.save_figure(fig_name, file_extension=".pdf")
        print ("Saved: {0}.pdf".format(fig_name))

    return robot

if __name__ == '__main__':
    
    # get user input
    menu_result = menu_Robot()
    num_iter = menu_result.n
    map_name = menu_result.m
    world_name = menu_result.w
    start = menu_result.sx, menu_result.sy
    goal = menu_result.gx, menu_result.gy
    robot_radius = menu_result.radius
    robot_vision = menu_result.r
    
    ranking_type = menu_result.rank_type
    if 'da' in ranking_type:
        ranking_type = Ranking_type.Distance_Angle
    elif 'r' in ranking_type:
        ranking_type = Ranking_type.RRTstar

    picking_strategy = menu_result.p
    if 'g' in picking_strategy:
        picking_strategy = Picking_strategy.global_first
    elif 'l' in picking_strategy:
        picking_strategy = Picking_strategy.local_first

    robot_type = RobotType.circle

    ranking_function =Ranking_function.Angular_similarity
    # run robot
    robot_main( start=start, goal=goal, map_name=map_name, world_name=world_name, num_iter=num_iter, 
                robot_vision=robot_vision, robot_type=robot_type, robot_radius=robot_radius, 
                ranking_type = ranking_type, ranking_function =ranking_function, picking_strategy= picking_strategy)
