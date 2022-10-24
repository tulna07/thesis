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

sys.setrecursionlimit(10000)

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
            # if i % 100 == 0 and not self.reach_goal: # bias to goal sometime
            #     rand_coordinate = np.array(goal_coordinate)

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
HM_EPISODES = 40000

MOVE_PENALTY = 1
WRONG_MOVE_PENALTY = 300 # Robot run into obstacles or in empty space
GOAL_REWARD = 1000

EPS_DECAY = 0.99  # Every episode will be epsilon*EPS_DECAY
SHOW_EVERY = 3000  # how often to play through env visually.

LEARNING_RATE = 0.1
DISCOUNT = 0.95

start_q_table = None

if start_q_table is None:
  # initialize the q-table#
  q_table = {}
else:
  with open(start_q_table, "rb") as f:
    q_table = pickle.load(f)

def robot_main(start, goal, obstacles=Obstacles(), vision_range=5, Tree=Tree):
    # Training
    epsilon = 1
    episode_rewards = []
    for episode in range(HM_EPISODES):
        episode_reward = 0
        robot = Robot(start=start, goal=goal, vision_range=vision_range)
        Tree.path_to_goal = []
        while True:
            robot_state = robot.get_robot_coords()
            # visualize
            node = Tree.get_node_by_coords(robot_state)
            Tree.path_to_goal.append(node)
            
            if not robot_state in q_table:
                q_table[robot_state] = [0 for i in range(80)]

            if np.random.random() > epsilon:
                # GET THE ACTION
                robot_action = np.argmax(q_table[robot_state])
            else:
                neighbor_nodes = Tree.neighbour_nodes(state, vision_range)
                visited_neighbor_nodes = Tree.get_visited_neighbor_nodes(neighbor_nodes, obstacles)
                if visited_neighbor_nodes:
                    chosen_neighbor_index = np.random.randint(0, len(visited_neighbor_nodes) - 1)
                    for idx in len(robot.grid_coordinates):
                        if visited_neighbor_nodes[chosen_neighbor_index].coords == robot.grid_coordinates[idx]:
                            robot_action = idx
                            break
                else:
                    break        
                

            # Take the action!
            robot.action(robot_action)
            
            robot.is_reach_goal(goal)
            if robot.reach_goal:
                reward = GOAL_REWARD
            # elif robot.check_wrong_move(obstacles):
            #     reward = -WRONG_MOVE_PENALTY
            else:
                reward = MOVE_PENALTY
            ## NOW WE KNOW THE REWARD, LET'S CALC YO
            # first we need to obs immediately after the move.
            robot_next_state = robot.get_robot_coords()
            if not robot_next_state in q_table:
                q_table[robot_next_state] = [0 for i in range(80)]
            max_future_q = np.max(q_table[robot_next_state])
            current_q = q_table[robot_next_state][robot_action]

            if reward == GOAL_REWARD:
                new_q = GOAL_REWARD
            else:
                new_q = (1 - LEARNING_RATE) * current_q + LEARNING_RATE * (reward + DISCOUNT * max_future_q)
            q_table[observation][robot_action] = new_q

            # if show:

            episode_reward += reward
            if reward == GOAL_REWARD:
                print("reach goal")
                break
            
        if reward == GOAL_REWARD:
            node = Tree.get_node_by_coords(robot.coordinate)
            Tree.path_to_goal.append(node)
            print("len path to goal", len(Tree.path_to_goal), start)
            with open("qtable.pickle", "wb") as f:
                pickle.dump(q_table, f)
                return
        if episode % 100 == 0:
            print(episode_reward)
        episode_rewards.append(episode_reward)
        epsilon *= EPS_DECAY
        
if __name__ == '__main__':
    
    #read tree from rrt_star.pickle
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

    ''' build tree '''
    #start_node = Node(start_cooridinate, cost=0)            # initial root node, cost to root = 0
    # RRT_star = RRTree_star(root=start_node, step_size=step_size, radius=5, 
    #                 random_area=random_area, sample_size=sample_size)
    # RRT_star.build(goal_coordinate=goal_coordinate, plotter=plotter, obstacles=obstacles, show_animation=True)
    
    #save the tree
    # with open('rrt_star.pickle', 'wb') as f:
    #     pickle.dump(RRT_star, f)
    
    
    # @Tu
    robot_main(start=start_cooridinate, goal=goal_coordinate, obstacles=obstacles, vision_range=5, Tree=RRT_star)
    RRT_star.draw_RRT_star(goal_coordinate=goal_coordinate, start_coordinate=start_cooridinate, plotter=plotter, obstacles=obstacles)
    plotter.show()

