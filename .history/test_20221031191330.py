from xmlrpc.client import Boolean
import numpy as np

from Tree import Tree, Node
from RRTree import RRTree
from RRT_draw_lib import Plot_RRT
from Robot_math_lib import *
from Robot_class import Robot, RobotType
from RRT_user_input import menu_RRT
from Program_config import *
from Obstacles import Obstacles
import pickle
from Tree import Node, Tree
import sys

sys.setrecursionlimit(12000)

class RRTree_star(RRTree):

    """ RRTree class from Tree class """
    def __init__(self, root, step_size = 5, radius=5, random_area=(0, 100), sample_size = 100):
        super().__init__(root, step_size, radius, random_area, sample_size)

    ''' add node to RRTree , return new_node and its neighbour_node(s)'''
    def add_node_RRTstar(self, accepted_coordinate):
        # find all neighbours of node_coordinate
        neighbour_nodes = self.neighbour_nodes(accepted_coordinate, self.radius)
        # pick nearest neighbour as new_node's parent
        nearest_neighbour_node = self.neighbours_smallest_cost(accepted_coordinate, neighbour_nodes)


        if nearest_neighbour_node is not None: # found a neighbour to link to 
            # allocate new node then link to RRTree
            new_node = Node(accepted_coordinate)
            new_node.add_neighbours(neighbours=neighbour_nodes)
            self.add_node(new_node=new_node)
            self.add_edge(parent_node=nearest_neighbour_node, node=new_node)
            return new_node, neighbour_nodes, nearest_neighbour_node
        return None, None, None        

    def build(self,  goal_coordinate, plotter: Plot_RRT=None, obstacles=None, show_animation=False):
        first_saw_goal = False

        for i in range(1, self.sampling_size):
            
            # generate random coordinate in sampling area = [min, max]
            rand_coordinate = self.random_coordinate()
            
            # orient to goal sometime :))
            if i % 5 == 0 and not self.reach_goal: # bias to goal sometime
                rand_coordinate = np.array(goal_coordinate)

            # bring closer random coordinate to tree
            accepted_coordinate_in_float = self.bring_closer(rand_coordinate=rand_coordinate)
            accepted_coordinate = tuple(int(x) for x in accepted_coordinate_in_float)

            
            # if tree first saw given goal , instead of adding new random , add goal
            if not first_saw_goal:
                nn_goal = self.saw_goal(goal_coordinate)    # return nearst neighbour node of goal
                if nn_goal is not None: # existing a node nearby goal
                    first_saw_goal = True
                    self.reach_goal = True
                    accepted_coordinate = goal_coordinate
            
            # add and link node to tree
            new_node, neighbour_nodes, nearest_neighbour_node  = self.add_node_RRTstar(accepted_coordinate)

            ''' rewire for RRT* '''
            self.rewire(node=new_node, neighbour_nodes=neighbour_nodes)
                
            if self.reach_goal:
                goal_node = self.get_node_by_coords(goal_coordinate)
                self.path_to_goal = self.path_to_root(goal_node)
                self.total_goal_cost = goal_node.cost

            ''' for display '''
            if i == self.sampling_size - 1:
                plotter.build_tree_animation(num_iter= i, Tree= self, obstacles=obstacles,  goal_coords=goal_coordinate, \
                    start_coords=self.root.coords, rand_coordinate= rand_coordinate, rand_node=new_node, 
                    neighbour_nodes=neighbour_nodes, nearest_neighbour_node=nearest_neighbour_node, color_tree=TreeColor.by_cost)
    
    # @Tu
    def get_visited_neighbor_nodes(self, neighbour_nodes, obstacles: Obstacles):
        if neighbour_nodes is None:
            return

        visited_neighbor_nodes = []
        for node in neighbour_nodes:
            collision = obstacles.check_point_collision(point=node.coords,\
                            obstacles_line_segments=obstacles.obstacles_line_segments)
            if not collision:
                visited_neighbor_nodes.append(node)
                node.set_visited()
            else:
                node.set_inactive()
        return visited_neighbor_nodes
    
    def draw_RRT_star(self,  goal_coordinate, start_coordinate, plotter: Plot_RRT=None, obstacles=None):
        plotter.build_RRT_star(num_iter= self.sampling_size - 1, Tree=self, obstacles=obstacles, goal_coords=goal_coordinate, \
                    start_coords=start_coordinate, color_tree=TreeColor.by_cost)

# @ Tu
HM_EPISODES = 5000
GOAL_REWARD = 1000
EPS_DECAY = 0.99  # Every episode will be epsilon*EPS_DECAY
LEARNING_RATE = 0.1
DISCOUNT = 0.95
epsilon = 0.9



def handle_q_table(save=Boolean, save_q_table={}):
    #save q_table
    if save:
        with open("qtable.pickle", "wb") as f:
            pickle.dump(save_q_table, f)
        return

    # initialize the q-table#
    q_table = {}
    try:
        with open("qtable.pickle", "rb") as f:
            q_table = pickle.load(f)
    except:
        q_table = {}
    return q_table

def distance_compare(Tree = Tree, node_1 = Node, node_2 = Node):
    nodes_list = []
    nodes_list.append(node_1)
    nodes_list.append(node_2)
    root_node = Tree.find_root(node_1)
    nodes_distance = Tree.distances(root_node,nodes_list)
    # return nodes_distance
    
    

def evaluate_reward(Tree = Tree, current_node = Node, next_node = Node):
    reward = 0
    current_node_degree = len(Tree.path_to_root(current_node)) - 1
    next_node_degree = len(Tree.path_to_root(next_node)) - 1
    degree = current_node_degree - next_node_degree
    if next_node.checkin:
        reward -= 500
    if degree >= 1: # next node belongs to parent degree of current node
        reward += degree*10
    elif degree <= -1: # next node belongs to children degree of current node
        reward -= abs(degree)*10
    elif degree == 0: # next node has the same degree of current node
        reward += 5            
    return reward   
    
def run_by_reinforcement_learning(goal, vision_range, robot, Tree, obstacles, q_table):
    robot_action = 0 
    robot_action_idx = 0
    action_take = ""
    
    robot_state = robot.get_robot_coords()
    current_node = Tree.get_node_by_coords(robot_state)       
    neighbor_nodes = Tree.neighbour_nodes(robot_state, vision_range)
    visited_neighbor_nodes = Tree.get_visited_neighbor_nodes(neighbor_nodes, obstacles)
            
    if not robot_state in q_table:
        q_table[robot_state] = [0 for i in range(len(visited_neighbor_nodes))]

    #take move base on highest q-value
    random_number = np.random.random() 
    if random_number > epsilon: 
    # if True:
        action_take = "q_value"
        robot_action_idx = np.argmax(q_table[robot_state])
        chosen_node_coords = visited_neighbor_nodes[robot_action_idx].coords
        for idx in range(len(robot.grid_coordinates)):
            if chosen_node_coords == robot.grid_coordinates[idx]:
                robot_action = idx
                break
        
    # take random move
    
    else:
        action_take = "random"
        if visited_neighbor_nodes:
            robot_action_idx = np.random.randint(len(visited_neighbor_nodes))
            for idx in range(len(robot.grid_coordinates)):
                if visited_neighbor_nodes[robot_action_idx].coords == robot.grid_coordinates[idx]:
                    robot_action = idx
                    break
        # else:
        #     return   
        
             
    # Take the action!
    robot.action(robot_action)              
    robot.is_reach_goal(goal)
    robot_next_state = robot.get_robot_coords()
    next_node = Tree.get_node_by_coords(robot_next_state)
    if robot.reach_goal:
        reward = GOAL_REWARD
    else:
        reward = evaluate_reward(Tree, current_node, next_node)
    
    next_node.set_checkin() #checkin node
    
    next_neighbor_nodes = Tree.neighbour_nodes(robot_next_state, vision_range)
    next_visited_neighbor_nodes = Tree.get_visited_neighbor_nodes(next_neighbor_nodes, obstacles)
    if not robot_next_state in q_table:
        q_table[robot_next_state] = [0 for i in range(len(next_visited_neighbor_nodes))]
    max_future_q = np.max(q_table[robot_next_state])
    current_q = np.max(q_table[robot_state][robot_action_idx])
    if reward == GOAL_REWARD:
        new_q = GOAL_REWARD
    else:
        new_q = (1 - LEARNING_RATE) * current_q + LEARNING_RATE * (reward + DISCOUNT * max_future_q)
        q_table[robot_state][robot_action_idx] = new_q
    
    return action_take , reward   
        
def run_by_rrtstar(robot=Robot,Tree=Tree, path_to_goal=[], vision_range=int):
    robot_current_node = Tree.get_node_by_coords(robot.get_robot_coords())
    # Get robot path from current node to goal based on RRT*
    Tree.path_to_goal = Tree.path_to_root(robot_current_node)
    
    # Move along RRT* path until seeing obstacles
    for node in Tree.path_to_goal:
        node.set_checkin()  # checkin node that already go through
        robot.coordinate = node.coords
        # visualize
        path_to_goal.append(node)

        neighbor_nodes = Tree.neighbour_nodes(robot.coordinate, vision_range)
        is_see_obstacles = robot.scan_obstacles(neighbor_nodes, obstacles)
        if is_see_obstacles:
            robot.generate_grid_coordinates()
            return     

def reach_goal(goal, robot=Robot):
    if robot.coordinate == goal:
        print("reach goal")
        return True
    # else :
    #     print("not reach goal yet")

    return False

def train(start, goal, obstacles=Obstacles(), vision_range=5, Tree=Tree):
    save_q_table = True
    q_table = handle_q_table(not save_q_table)
    for episode in range(HM_EPISODES):
        episode_reward = 0
        robot = Robot(start=start, goal=goal, vision_range=vision_range)
        
        # path to goal each episode
        path_to_goal = []

        while True:
            run_by_rrtstar(robot, Tree, path_to_goal, vision_range)

            # reach goal
            if reach_goal(goal, robot):
                break
           
            action_take, reward  = run_by_reinforcement_learning(goal, vision_range, robot, Tree, obstacles,q_table)
            episode_reward += reward
            
        Tree.path_to_goal = path_to_goal
        print("episode:", episode+1 , ", action:", action_take , ", total nodes:", len(Tree.path_to_goal), ", episode reward:", episode_reward)
        handle_q_table(save_q_table, q_table)
        
        # return
        
        global epsilon 
        epsilon *= EPS_DECAY
        
if __name__ == '__main__':
    
    # read tree from rrt_star.pickle
    
    with open('rrt_star.pickle', 'rb') as f: 
        RRT_star = pickle.load(f)
    
    ''' initial parameters '''
    # get user input
    menu_result = menu_RRT()
    # get start_cooridinate and goal_coordinate
    start_cooridinate = menu_result.sx, menu_result.sy
    goal_coordinate = menu_result.gx, menu_result.gy

    step_size = menu_result.step_size
    radius = menu_result.radius
    sample_size = menu_result.ss
    map_name = menu_result.m
    world_name = None

    ''' Running '''
    # set same window size to capture pictures
    plotter = Plot_RRT(title="Rapidly-exploring Random Tree Star (RRT*)")
    plotter.set_equal()

    obstacles = Obstacles()
    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles.read(world_name, map_name)
    # @Tu
    obstacles.line_segments()

    # find working space boundary
    x_min = min(obstacles.x_lim[0], obstacles.y_lim[0], start_cooridinate[0], goal_coordinate[0])
    x_max = max(obstacles.x_lim[1], obstacles.y_lim[1], start_cooridinate[1], goal_coordinate[1])
    y_min = min(obstacles.x_lim[0], obstacles.y_lim[0], start_cooridinate[0], goal_coordinate[0])
    y_max = max(obstacles.x_lim[1], obstacles.y_lim[1], start_cooridinate[1], goal_coordinate[1])
    random_area = ([x_min, y_min], [x_max, y_max])

    '''
        build tree
    '''
    # start_node = Node(start_cooridinate, cost=0)            # initial root node, cost to root = 0
    # RRT_star = RRTree_star(root=start_node, step_size=step_size, radius=5, 
    #                 random_area=random_area, sample_size=sample_size)
    # RRT_star.build(goal_coordinate=goal_coordinate, plotter=plotter, obstacles=obstacles, show_animation=True)
    
    ''' 
        save the tree
    '''
    # with open('rrt_star.pickle', 'wb') as f:
    #     pickle.dump(RRT_star, f)
    
    
    '''
        test start and goal exist RRT* path  
    ''' 
    # start_node = RRT_star.get_node_by_coords(start_cooridinate)
    # RRT_star.path_to_goal = RRT_star.path_to_root(start_node)

    ''' 
        train the robot , use the sample tree
    '''
    train(start=start_cooridinate, goal=goal_coordinate,\
          obstacles=obstacles, vision_range=5,\
          Tree=RRT_star)

    ''' 
        draw the result: obstacles + RRT* + robot path 
    ''' 
    RRT_star.draw_RRT_star(goal_coordinate=goal_coordinate, start_coordinate=start_cooridinate,\
                           plotter=plotter, obstacles=obstacles)
    plotter.show()

