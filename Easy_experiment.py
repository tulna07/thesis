"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""
import argparse
from datetime import datetime

from Robot_base import Picking_strategy, Ranking_type, RobotType
from Robot_ranking import Ranking_function

from Robot_run import robot_main
from Robot_run_RRTstar_ranking import robot_main as robot_RRTstar_ranking
from RRTree_X import robot_main as robot_RRTX
from RRT_user_input import *
from Program_config import save_image
from Easy_experiment_lib import Experimental_Result, Experiment_type

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Code for Autonomous Robot.')
    parser.add_argument('-n', metavar="number of iteration", type=int, help='number of iteration', default=100)
    parser.add_argument('-m', metavar="data_map", help='map data', default='_map.csv')
    parser.add_argument('-w', metavar="world_image", help='world model')
    parser.add_argument('-r', metavar="vision_range", type=float, help='vision range', default=5.0)
    parser.add_argument('-radius', metavar="robot radius", type=float, help='robot radius', default=0.5)

    menu_result = parser.parse_args()

    # get user input
    num_iter = menu_result.n
    map_name = menu_result.m
    world_name = menu_result.w
    robot_radius = menu_result.radius
    range_begin = menu_result.r
    robot_type = RobotType.circle

    # get start point and goal point
    start_list = []

    goal_list = []
    # much more fun map
    start_list.append ((0,0))
    #start_list.append ((20,62))

    # for much_more_fun.csv
    #goal_list.append ((50,50))
    #goal_list.append ((45,65))
    #goal_list.append ((100,100))

    # for map.csv
    # run robot
    #goal_list.append ((60, 90))
    #goal_list.append ((20, 60))
    #goal_list.append ((60, 70))
    #goal_list.append ((70, 40))
    #goal_list.append ((30, 50))

    # for map block_1.csv
    goal_list.append ((99, 99))

    # for map block_1.csv
    #goal_list.append ((250, 320))
    ''' 
    NOTE: set show_animation = False and easy_experiment = True in prog_config file to save your time :)) 
    '''

    result = Experimental_Result()
    
    #experiment_type= Experiment_type.COMPARE_LOCAL_GLOBAL
    #experiment_type= Experiment_type.COMPARE_RANKING_FUNCTION
    #experiment_type= Experiment_type.COMPARE_RRT_RANKING_FUNCTION
    experiment_type= Experiment_type.COMPARE_OUR_VS_RRTX_ALGORITHM

    robotA_ranking_function = Ranking_function.Angular_similarity
    #robotA_ranking_function = Ranking_function.Angular_similarity
    #robotA_ranking_function = Ranking_function.RHS_RRT_base
    
    #robotB_ranking_function = Ranking_function.Cosine_similarity
    #robotB_ranking_function = Ranking_function.Angular_similarity
    robotB_ranking_function = Ranking_function.RHS_RRT_base
    
    # picking strategies
    pickingA_strategy = Picking_strategy.global_first
    pickingB_strategy = Picking_strategy.local_first
    pickingC_strategy = Picking_strategy.local_first

    range_step = 5
    range_max = 81
    range_begin = 10

    print ("\n{0}, RobotA: {1}, RobotB {2}".format(experiment_type, 
                    robotA_ranking_function, robotB_ranking_function ))
    for s in start_list:
        start = s
        for g in goal_list:
            goal = g

            range_experiment_list = []

            for i in range (int ((range_max-range_begin)/range_step)):
                vision_range = range_begin + range_step*i
                range_experiment_list.append(vision_range)

                print("\nStart: {0} --> goal: {1}, range: {2}".format(start, goal, vision_range))
                # compare ranking function
                if experiment_type == Experiment_type.COMPARE_RANKING_FUNCTION:
                    robotA = robot_main(start, goal, map_name, world_name, num_iter, vision_range,
                        robot_type, robot_radius, robotA_ranking_function, picking_strategy=pickingA_strategy)
                    robotB = robot_main(start, goal, map_name, world_name, num_iter, vision_range, 
                        robot_type, robot_radius, robotB_ranking_function, picking_strategy=pickingA_strategy)
                
                elif experiment_type == Experiment_type.COMPARE_LOCAL_GLOBAL:   # compare local vs global pick strategy is default
                    robotA = robot_main(start, goal, map_name, world_name, num_iter, vision_range,
                        robot_type, robot_radius, robotA_ranking_function, picking_strategy=pickingA_strategy)
                    robotB = robot_main(start, goal, map_name, world_name, num_iter, vision_range, 
                        robot_type, robot_radius, robotB_ranking_function, picking_strategy=pickingB_strategy)

                elif experiment_type == Experiment_type.COMPARE_RRT_RANKING_FUNCTION:   # compare local vs global pick strategy is default
                    robotA = robot_main(start, goal, map_name, world_name, 
                            num_iter, vision_range, robot_type, robot_radius, robotA_ranking_function)
                    robotB = robot_main(start, goal, map_name, world_name, 
                            num_iter, vision_range, robot_type, robot_radius, robotA_ranking_function)
                    robotC = robot_RRTstar_ranking(start, goal, map_name, world_name, 
                            num_iter, vision_range, robot_type, robot_radius, robotB_ranking_function)

                elif experiment_type == Experiment_type.COMPARE_OUR_VS_RRTX_ALGORITHM:   # compare our vs RRtree X
                    # get user input
                    menu_result = menu_RRT()
                    # get start_cooridinate and goal_coordinate
                    step_size = menu_result.step_size
                    radius = menu_result.radius
                    sample_size = menu_result.ss

                    robotA = robot_RRTstar_ranking( start=start, goal=goal, map_name=map_name, world_name=world_name, num_iter=num_iter, 
                                        robot_vision=vision_range, robot_type=robot_type, robot_radius=robot_radius, 
                                        ranking_type = Ranking_type.RRTstar, ranking_function =robotA_ranking_function,
                                        picking_strategy= Picking_strategy.global_first)
                    robotB = robot_RRTstar_ranking( start=start, goal=goal, map_name=map_name, world_name=world_name, num_iter=num_iter, 
                                        robot_vision=vision_range, robot_type=robot_type, robot_radius=robot_radius, 
                                        ranking_type = Ranking_type.RRTstar, ranking_function =robotA_ranking_function,
                                        picking_strategy= Picking_strategy.local_first)
                    robotC = robot_RRTX(start, goal, map_name, world_name,\
                            num_iter, vision_range, robot_type, robot_radius, robotB_ranking_function, radius, \
                                step_size, sample_size)
                # Log the result, careful with the data order (start, goal, vision....)
                result.add_result([start, goal, vision_range, 
                        robotA.reach_goal, robotA.cost, 
                        robotB.reach_goal, robotB.cost, 
                        robotC.reach_goal, robotC.cost ])
            
            # composite images for easy to analyze
            if save_image:
                result.compare_imgs(map_name= map_name, start=start, goal=goal, 
                    range_list= range_experiment_list, experiment_type= experiment_type, 
                    robotA_ranking_function= robotA_ranking_function, 
                    robotB_ranking_function= robotB_ranking_function,
                    pickingA_strategy= pickingA_strategy,
                    pickingB_strategy= pickingB_strategy,
                    pickingC_strategy= pickingC_strategy)

    # write log file
    if experiment_type == Experiment_type.COMPARE_LOCAL_GLOBAL:
        result.set_header(["start","goal", "range",
                "global_reached_goal","global_cost","local_reached_goal","local_cost"])
        result_file= "result_{0}_local_global_{1}.csv".format(map_name, datetime.now().strftime("%m_%d_%H_%M_%S") )

    elif experiment_type == Experiment_type.COMPARE_RANKING_FUNCTION:
        result.set_header(["start","goal", "range",
            "reached_goal_ranking_function_1","cost_ranking_function_1",
            "reached_goal_ranking_function_2","cost_ranking_function_2"])
        result_file= "result_{0}_ranking_{1}.csv".format(map_name, datetime.now().strftime("%m_%d_%H_%M_%S") )
    elif experiment_type == Experiment_type.COMPARE_RRT_RANKING_FUNCTION:
        result.set_header(["start","goal", "range",
            "reached_goal_ranking_function_1","cost_ranking_function_1",
            "reached_goal_ranking_function_RRT","cost_ranking_function_RRT"])
        result_file= "result_{0}_ranking_RRT_{1}.csv".format(map_name, datetime.now().strftime("%m_%d_%H_%M_%S") )

    elif experiment_type == Experiment_type.COMPARE_OUR_VS_RRTX_ALGORITHM:
        result.set_header(["start","goal", "range",
            "our_global_reached","our_global_cost",
            "our_lobal_reached","our_lobal_cost",
            "rrtreeX_reached","rrtreeX__cost",])
        result_file= "result_{0}_our_RRTx_{1}.csv".format(map_name, datetime.now().strftime("%m_%d_%H_%M_%S") )
    else:
        result_file= "result_{0}_{1}.csv".format(map_name, datetime.now().strftime("%m_%d_%H_%M_%S") )

    result.write_csv(file_name=result_file)
    
    # DONE
    print ("\nTo visualize the result run:\n" +
              "python Easy_experiment_lib.py -r {0}".format(result_file))
    print ("\nDONE!  easy experiment....")