'''
autonomousRobot
reimplementing an algorithm which has been written from the following paper:
https://pdfs.semanticscholar.org/0cac/84962d0f1176c43b3319d379a6bf478d50fd.pdf
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
'''

from matplotlib.pyplot import title
import numpy as np

from Tree import Node
from RRTree import RRTree
from RRT_draw_lib import Plot_RRT
from Robot_math_lib import *

from RRT_user_input import menu_RRT
from Program_config import *
from Obstacles import Obstacles
from Queue_class import Priority_queue
from Robot_class import Robot, RobotType

class Debug_data:
    def __init__(self) -> None:
        self.p = False
    def set_p(self, number):
        self.p = True
        self.queue_number = number
    def queue_at_start(self, number):
        self.start_queue = number

    def queue_at_first_obstacle(self, number):
        self.first_obstacle = number

    def print (self):
        print ("start at t0: ", self.start_queue)
        print ("start first obstacle: ", self.first_obstacle)
        print ("end first obstacle: ", self.queue_number)

class RRTree_x(RRTree):

    ''' RRTree class from Tree class '''
    def __init__(self, root:Node, step_size = 5, radius=5, random_area=([0, 100],[0,100]), sample_size = 100):
        super().__init__(root, step_size, radius, random_area, sample_size)

    ''' add node to RRTreeX , return new_node and its neighbour_node(s), update lmc, weight'''
    def add_node_RRTx(self, accepted_coordinate):
        
        # find all neighbours of node_coordinate
        neighbour_nodes = self.neighbour_nodes(accepted_coordinate, self.radius)
        # pick nearest neighbour as new_node's parent
        nearest_neighbour_node = self.neighbours_smallest_lmc(accepted_coordinate, neighbour_nodes)
        if nearest_neighbour_node is not None: # found a neighbour to link to 
            # allocate new node then link to RRTree
            new_node = Node(accepted_coordinate)
            new_node.add_neighbours(neighbours=neighbour_nodes)
            self.add_node(new_node=new_node)
            self.add_edge_RRTx(parent_node=nearest_neighbour_node, node=new_node)
            return new_node, neighbour_nodes, nearest_neighbour_node
        return None, None, None

    def build(self,  goal_coordinate, plotter: Plot_RRT=None, obstacles=None, rrt_queue=Priority_queue):
        print ("builing Graph (adjacent graph and RRTreex) with {0} sampples".format(self.sampling_size))
        first_saw_goal = False

        for i in range(1, self.sampling_size):

            # generate random coordinate in sampling area = [min, max]
            rand_coordinate = self.random_coordinate()
            
            # orient to goal sometime :))
            if i %100 == 0 and not self.reach_goal: # bias to goal sometime
                rand_coordinate = np.array(goal_coordinate)

            # bring closer random coordinate to tree 
            accepted_coordinate = self.bring_closer(rand_coordinate=rand_coordinate)

            # if tree first saw given goal , instead of adding new random , add goal
            if not first_saw_goal:
                nn_goal = self.saw_goal(goal_coordinate)    # return nearst neighbour node of goal
                if nn_goal is not None: # existing a node nearby goal
                    first_saw_goal = True
                    self.reach_goal = True
                    accepted_coordinate = goal_coordinate

            # add and link node to tree
            new_node, neighbour_nodes, nearest_neighbour_node  = self.add_node_RRTx(accepted_coordinate)
            
            # rewire and inconsistency for RRTX
            self.rewire_RRTx(node=new_node, rrt_queue=rrt_queue)
            self.reduce_inconsistency(rrt_queue=rrt_queue)

            if self.reach_goal:
                goal_node = self.get_node_by_coords(goal_coordinate)
                self.path_to_goal = self.path_to_root(goal_node)
                self.total_goal_cost = goal_node.cost
               
            
            show_animation = False
            ''' for display '''
            if show_animation:
                plotter.build_tree_animation(num_iter= i, Tree= self, obstacles=obstacles,  goal_coords= self.root.coords,\
                    start_coords=goal_coordinate, rand_coordinate= rand_coordinate, rand_node=new_node, \
                        neighbour_nodes=neighbour_nodes, nearest_neighbour_node=nearest_neighbour_node)


    def local_obstacle_nodes(self, nodes, obstacles: Obstacles):
        obstacle_nodes = []
        for node in nodes:
            conllision = obstacles.check_point_collision(point=node.coords,\
                            obstacles_line_segments=obstacles.obstacles_line_segments)
            if conllision:
                obstacle_nodes.append(node)
                node.set_inactive()
            else:
                node.set_visited()
        return obstacle_nodes

    def propogate_descendants(self, discovered_obstacle_nodes, obstacle_nodes, rrtx_queue):
        discovered_obstacle_nodes_children = self.all_subtree_nodes_at_node(discovered_obstacle_nodes)
        all_children = discovered_obstacle_nodes.copy()
        all_children.extend(discovered_obstacle_nodes_children)
        sql_nodes = []
        for orphan_node in all_children:
            n_orphan_nodes = orphan_node.neighbours
            
            orphan_node.set_cost(float("inf"))
            orphan_node.set_lmc(float("inf"))
            for n_orphan_node in n_orphan_nodes:
                if n_orphan_node not in all_children and n_orphan_node not in obstacle_nodes:
                    n_orphan_node.set_cost(float("inf"))
                    rrtx_queue.verify_queue(n_orphan_node)
                    sql_nodes.append(n_orphan_node)
        for orphan_node in all_children:
            orphan_parent = orphan_node.parent
            if orphan_parent is not None:
                childrens = orphan_parent.children
                for child in childrens:
                    if child in all_children:
                        orphan_parent.remove_child(child)
            orphan_node.remove_parent()

        #for orphan_node in all_children:
        #    orphan_node_parent = orphan_node.parent
        #    if orphan_node_parent is not None:
        #        self.remove_edge(parent_node=orphan_node_parent, node=orphan_node)
        return all_children, sql_nodes

    def update_obstacles(self, neighbour_nodes, currnode, obstacles=Obstacles, rrtx_queue=Priority_queue):
        # obstacle_nodes which are nodes and belong to obstacles ares at local circle range
        # discovered_obstacle_nodes (aka orphan nodes) whose parent are dead
        # all_children are nodes, containing all discovered_obstacle_nodes and theirs discovered_obstacle_nodes's chilrend
        discovered_obstacle_nodes = []
        all_children = []
        sql_nodes = []
        obstacle_nodes = self.local_obstacle_nodes(nodes=neighbour_nodes, obstacles=obstacles)
        if len(obstacle_nodes) > 0: # found obstacle ahead, then rerouting tree
            # get discovered_obstacle_nodes witch are orphan nodes and theirs parent are obstacle_nodes
            discovered_obstacle_nodes = self.add_new_obstacle(obstacle_nodes, rrtx_queue=rrtx_queue)
            all_children, sql_nodes = self.propogate_descendants(discovered_obstacle_nodes, obstacle_nodes, rrtx_queue=rrtx_queue)

            rrtx_queue.verify_queue(node=currnode)
            self.reduce_inconsistency_v2(rrtx_queue=rrtx_queue, currnode=currnode)
        return obstacle_nodes, discovered_obstacle_nodes, all_children, sql_nodes

    def add_new_obstacle(self, obstacle_nodes, rrtx_queue:Priority_queue):
        discovered_obstacle_nodes = []
        for node in obstacle_nodes:
            orphan_nodes = node.neighbours
            for n_orphan_node in orphan_nodes:
                self.set_edge_weight(nodeA=node, nodeB=n_orphan_node, weight=float("inf"))
                if n_orphan_node.parent == node and n_orphan_node not in obstacle_nodes:
                    rrtx_queue.verify_orphan(n_orphan_node)
                    discovered_obstacle_nodes.append(n_orphan_node)
        return discovered_obstacle_nodes

def robot_main( start_cooridinate, goal_coordinate, map_name, world_name, num_iter, robot_vision, \
            robot_type, robot_radius, ranking_function, RRT_radius,\
                RRT_step_size, RRT_sample_size):
    
    ''' variable declaration '''
    robot = Robot(start=start_cooridinate, goal=goal_coordinate, vision_range=robot_vision)
    # set same window size to capture pictures
    wintitle = "Rapidly-exploring Random Tree X (RRT_X)"
    plotter = Plot_RRT(title=wintitle)
    plotter.set_equal()
    print (wintitle)

    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles = Obstacles()
    obstacles.read(world_name, map_name)
    obstacles.line_segments()
    rrt_queue = Priority_queue()

    # find working space boundary
    x_min = min(obstacles.x_lim[0], obstacles.y_lim[0], start_cooridinate[0], goal_coordinate[0]) - robot_vision
    x_max = max(obstacles.x_lim[1], obstacles.y_lim[1], start_cooridinate[1], goal_coordinate[1]) + robot_vision
    y_min = min(obstacles.x_lim[0], obstacles.y_lim[0], start_cooridinate[0], goal_coordinate[0]) - robot_vision
    y_max = max(obstacles.x_lim[1], obstacles.y_lim[1], start_cooridinate[1], goal_coordinate[1]) + robot_vision
    random_area = ([x_min, y_min], [x_max, y_max])

    ''' build tree '''
    goal_node = Node(goal_coordinate, lmc=0, cost=0)    # initial goal node, rsh to goal =0, cost to goal = 0
    RRTx = RRTree_x(root=goal_node, step_size=RRT_step_size, radius=RRT_radius, 
                    random_area=random_area, sample_size=RRT_sample_size)
    RRTx.build(goal_coordinate=start_cooridinate, plotter=plotter,obstacles=obstacles, rrt_queue=rrt_queue)

    #############################################################
    # initial variables
    #############################################################
    obstacle_nodes = None
    iter_count = 0
    if RRTx.reach_goal:     # generated tree reached to start from goal
        start_node = RRTx.dict[start_cooridinate]
        curr_node = start_node
        while curr_node is not goal_node:
            iter_count += 1
            # update new coordinate for robot
            robot.coordinate = curr_node.coords
            in_range_nodes = RRTx.neighbour_nodes(curr_node.coords, robot.vision_range)
            if in_range_nodes is not None:
                obstacle_nodes, discovered_obstacle_nodes, all_children, sql_nodes = \
                    RRTx.update_obstacles(neighbour_nodes=in_range_nodes, currnode=curr_node,\
                    obstacles=obstacles, rrtx_queue=rrt_queue)

            
            next_node = RRTx.pick_next(curr_node)
            if next_node is not None:
                path_look_ahead_to_goal = RRTx.path_to_root(curr_node)
                robot.set_look_ahead_to_goal(path=path_look_ahead_to_goal)            
            else:
                robot.set_look_ahead_to_goal(path=[]) 

            if next_node == goal_node:
                robot.reach_goal = True
            if next_node is not None:
                path = [curr_node.coords, next_node.coords]
                robot.expand_visited_path(path=path)
                curr_node = next_node
            else:
                print ("No parent found")
                break

            if len(robot.path_look_ahead_to_goal) == 0:
                print ("No look ahead found")
                break

            if show_animation and not easy_experiment:
                plotter.RRTX_animation(Tree=RRTx, obstacles=obstacles, robot=robot)
                #plotter.RRTX_animation(Tree=RRTx, obstacles=obstacles, robot=robot,\
                #    obstacle_nodes=obstacle_nodes, discovered_obstacle_nodes = discovered_obstacle_nodes,\
                #    all_children= all_children, rrt_queue=rrt_queue, sql_nodes=sql_nodes) 


            print ("from: {0} to {1}".format(robot.coordinate, curr_node.coords))

            # Run n times for debugging
            if  iter_count == num_iter:
                break

    else: # need rerun the tree
        print ("No path from goal to start belongs to generated tree")
        plotter.tree(RRTx, color_mode=TreeColor.by_lmc)

    if not easy_experiment: # skip printing and showing animation if running easy experiment
        print("Done")
        if show_animation:
            plotter.show()

    elif save_image:
        # showing the final result (for save image and display as well)
        plotter.RRTX_animation(Tree=RRTx, obstacles=obstacles, robot=robot)
        fig_name = set_figure_name(map_name=map_name, range=robot.vision_range, start=start_cooridinate, 
            goal=goal_coordinate, picking_strategy=g_strategy, ranking_function=ranking_function, RRTx=True)
        
        plotter.save_figure(fig_name, file_extension=".png")
        #plotter.save_figure(fig_name, file_extension=".pdf")
        print ("Saved: {0}.pdf".format(fig_name))

    return robot
            

if __name__ == '__main__':
    # get user input
    menu_result = menu_RRT()
    # get start_cooridinate and goal_coordinate
    start_cooridinate = menu_result.sx, menu_result.sy
    goal_coordinate = menu_result.gx, menu_result.gy

    step_size = menu_result.step_size
    radius = menu_result.radius
    vision_range = menu_result.r
    sample_size = menu_result.ss
    map_name = menu_result.m
    num_iter = menu_result.n
    world_name = menu_result.w

    robot_main(start_cooridinate = start_cooridinate, goal_coordinate= goal_coordinate, map_name= map_name, world_name=world_name,\
        num_iter=num_iter, robot_vision=vision_range, robot_type=RobotType.circle, robot_radius=0.5, 
                ranking_function =None, RRT_radius=radius, RRT_step_size=step_size, RRT_sample_size=sample_size)

    
    