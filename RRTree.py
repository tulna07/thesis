import numpy as np

from Tree import Tree, Node
from RRT_draw_lib import Plot_RRT
from Robot_math_lib import *

from RRT_user_input import menu_RRT
from Program_config import *
from Obstacles import Obstacles

class RRTree(Tree):

    """ RRTree class from Tree class """
    def __init__(self, root: Node, step_size = 5, radius=5, random_area=([0, 100],[0,100]), sample_size = 100):
        super().__init__(root)
        self.step_size = step_size
        self.radius = radius
        self.sampling_area = random_area
        self.sampling_size = sample_size
        self.reach_goal = False
        self.path_to_goal = []
        self.total_goal_cost = float('inf')

    def set_step_size(self, x): self.step_size = x
    def set_radius(self, x): self.radius = x

    ''' random node coords '''
    def random_coordinate(self):
        return np.random.random(2)*np.subtract(self.sampling_area[1],self.sampling_area[0]) + self.sampling_area[0]

    ''' add node to RRTree , return new_node and its neighbour_node(s)'''
    def add_node_RRT(self, picked_coordinate):
        # find the neareset node (in distance) to random coordinate
        _, nearest_node = self.nearest(picked_coordinate)

        # find all neighbours of node_coordinate
        neighbour_nodes = self.neighbour_nodes(picked_coordinate, self.radius)
        new_node = Node(picked_coordinate)
        new_node.add_neighbours(neighbours=neighbour_nodes)
        self.add_node(new_node=new_node)
        self.add_edge(parent_node=nearest_node, node=new_node)
        return new_node, neighbour_nodes, nearest_node

    def build(self,  goal_coordinate, plotter: Plot_RRT=None, obstacles=None):
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
            new_node, neighbour_nodes, nearest_neighbour_node  = self.add_node_RRT(accepted_coordinate)

                
            if self.reach_goal:
                goal_node = self.get_node_by_coords(goal_coordinate)
                self.path_to_goal = self.path_to_root(goal_node)
                self.total_goal_cost = goal_node.cost
            
            ''' for display '''
            if show_animation:
                plotter.build_tree_animation(num_iter= i, Tree= self, obstacles=None,  goal_coords=goal_coordinate, \
                    start_coords=self.root.coords, rand_coordinate= rand_coordinate, rand_node=new_node, 
                    neighbour_nodes=neighbour_nodes, nearest_neighbour_node=nearest_neighbour_node)

    ''' 
        check if the RRT contain nodes that are inside goal radius
        return None if do not see
        return nearest Node if existing a node (nearest) in goal radius 
    ''' 
    def saw_goal(self, goal_coords):
        # find nearest neighbours tree's nodes where are in circle of radius
        return self.nearest_neighbour(goal_coords, self.radius)

    '''check if the tree contains goal node'''
    #def is_reach_goal(self, goal_node):
    #    return goal_node in self      

    def path_to_root(self, node:Node):
        path = []
        path.append(node)
        while (node.parent): # loop whenever parent existed
            point_dist(node.coords, node.parent.coords)
            node = node.parent
            path.append(node)
        return path

if __name__ == '__main__':
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
    plotter = Plot_RRT(title="Rapidly-exploring Random Tree (RRT)")
    plotter.set_equal()

    obstacles = Obstacles()
    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles.read(world_name, map_name)

    # find working space boundary
    x_min = min(obstacles.x_lim[0], obstacles.y_lim[0], start_cooridinate[0], goal_coordinate[0])
    x_max = max(obstacles.x_lim[1], obstacles.y_lim[1], start_cooridinate[1], goal_coordinate[1])
    y_min = min(obstacles.x_lim[0], obstacles.y_lim[0], start_cooridinate[0], goal_coordinate[0])
    y_max = max(obstacles.x_lim[1], obstacles.y_lim[1], start_cooridinate[1], goal_coordinate[1])
    random_area = ([x_min, y_min], [x_max, y_max])

    ''' build tree '''
    start_node = Node(start_cooridinate, cost=0)            # initial root node, cost to root = 0
    RRT = RRTree(root=start_node, step_size=step_size, radius=radius, 
                    random_area=random_area, sample_size=sample_size)
    RRT.build(goal_coordinate=goal_coordinate, plotter=plotter, obstacles=obstacles)
    
    plotter.show()