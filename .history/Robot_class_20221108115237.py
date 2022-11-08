from types import new_class
from Robot_paths_lib import *
from Robot_math_lib import *
from Robot_sight_lib import inside_local_true_sight, inside_global_true_sight
from Robot_base import Picking_strategy, Ranking_type, Robot_base, RobotType
from Program_config import *
from Graph import Graph
from reinforcement_learning import *
import operator

#from RRTree_star import RRTree_star
class Robot(Robot_base):
    def __init__(self, start, goal, vision_range=20, robot_type= RobotType.circle, robot_radius= 0.2):

        super().__init__(vision_range=vision_range, robot_type=robot_type, robot_radius=robot_radius)

        self.start = start                  # start point
        self.goal = goal                    # goal point
        self.cost = 0                       # cost of visited path
        self.coordinate = tuple(start)      # hold current coordinate of robot
        self.next_coordinate = tuple(start) # hold next coordinate where robot moves to
        self.reach_goal = False             # True if robot reach goal
        self.saw_goal = False               # True if robot saw goal
        self.no_way_to_goal = False         # True if there is no path to goal
        self.path_look_ahead_to_goal = []   # path look a head to goal (estimate)

        #self.local_open_pts = []            # local open point
        self.local_active_open_pts = []     # local active open point
        self.local_active_open_rank_pts = []     # local active open point and its ranking
        self.global_active_open_rank_pts = []    # global active open points and its ranking

        self.traversal_sights = []          # hold traversal sights where robot visited
        self.visited_paths = []              # path where robot visited
        self.visited_path_directions = []    # status of visited subpath, true = forward, false = backward

        # visibility Graph containing information of visited places
        self.visibility_graph = Graph()

        # @Tu ----------------------------------------
        self.jump_amount_list_to_action = []
        self.generate_jump_amount_list_to_action()
        self.grid_coordinates = []
        self.generate_grid_coordinates()
        self.observation = ()

    ''' find working space boundaries'''
    def find_working_space_boundaries(self, obstacles):
        # find working space boundary
        x_min = min(obstacles.x_lim[0], obstacles.y_lim[0], self.start[0], self.goal[0]) - self.vision_range
        x_max = max(obstacles.x_lim[1], obstacles.y_lim[1], self.start[1], self.goal[1]) + self.vision_range
        y_min = min(obstacles.x_lim[0], obstacles.y_lim[0], self.start[0], self.goal[0]) - self.vision_range
        y_max = max(obstacles.x_lim[1], obstacles.y_lim[1], self.start[1], self.goal[1]) + self.vision_range
        return ([x_min, y_min], [x_max, y_max])

    def is_no_way_to_goal(self, noway):
        self.no_way_to_goal = noway

    def update_coordinate(self, coords):
        self.coordinate = coords

    def update_coordinate(self, point):
        self.coordinate = point

    def expand_traversal_sights(self, closed_sights, open_sights):
        self.traversal_sights.append([self.coordinate, closed_sights, open_sights])
   
    ''' expand visited path and its cost also '''
    def expand_visited_path(self, path):
        '''
        set direction: true means robot goes forward (len (asp) ==2)
                       false means robot back to point in explored area
        
        '''
        if len(path)> 0:
            self.visited_paths.append(path)

            direction = len(path) == 2
            self.visited_path_directions.append(direction)
            self.cost += path_cost(path)

    ''' Check if robot saw goal '''
    def is_saw_goal(self, goal, true_sight):
        self.saw_goal = inside_local_true_sight(goal, self.coordinate, self.vision_range, true_sight)

    ''' Check if robot reached goal '''
    def is_reach_goal(self, goal):
        self.reach_goal = point_dist(self.coordinate, goal) <= self.radius

    ''' Check if robot whether reached or saw goal '''
    def check_goal(self, goal, true_sight):
        
        self.is_reach_goal(goal)
        if not self.reach_goal:
            self.is_saw_goal(goal, true_sight)

    ''' print status and information of robot '''
    def print_infomation(self):
        if self.no_way_to_goal:
            print ("NO path to goal!")
        elif self.reach_goal:
            print ("Reached goal!")
        elif self.saw_goal:
            print ("Saw goal!")

        if print_traversalSights:
            print("traversal_sights:", self.traversal_sights)

        if print_visited_path:
            print("visited path:", self.visited_paths)
    ''' print global open points and its ranking '''
    def print_global_set(self):
        print ("Global open points and its ranking", self.global_active_open_rank_pts)

    ''' print gobal open points and its ranking '''
    def print_lobal_set(self):
        print ("Llobal open points and its ranking", self.local_active_open_rank_pts)

    def finish(self):
        return self.no_way_to_goal or self.reach_goal

    ''' clear local information '''
    def clear_local(self):
        self.local_open_pts = []
        self.local_active_open_pts = []
        self.local_active_open_rank_pts = []
        
    ''' get local open points '''
    def get_local_open_points_and_rank_by_RRTree_start(self, open_sights, nodes):
        self.local_open_pts = []
        if len(open_sights) > 0:
            open_sights = np.array(open_sights)
            self.local_open_pts = open_sights[:, 2]  # local_openPts
            for i in range(len(self.local_open_pts)):
                self.local_open_pts[i][0] = approximately_num(self.local_open_pts[i][0])
                self.local_open_pts[i][1] = approximately_num(self.local_open_pts[i][1])
        return self.local_open_pts

    ''' get local open points '''
    def get_local_open_points(self, open_sights):
        # open_sights is a list:

        self.local_open_pts = []
        if len(open_sights) > 0:
            for open_sight in open_sights:
                o_pt = approximately_num(open_sight[2][0]), approximately_num(open_sight[2][1])
                self.local_open_pts.append(o_pt)
        return self.local_open_pts
    
    ''' check whether local open_points are active '''
    def get_local_active_open_points(self):

        # get local active point for local points
        self.local_active_open_pts = []
        if len(self.local_open_pts):  # new local found
            self.local_active_open_pts = np.array(self.local_open_pts)

            # remove local_point which is inside explored area
            if len(self.traversal_sights) > 0:
                local_open_pts_status = [inside_global_true_sight(pt, self.vision_range, self.traversal_sights) for pt in self.local_active_open_pts]
                self.local_active_open_pts = self.local_active_open_pts[np.logical_not(local_open_pts_status)]
        

    ''' check if a point is inside explored area '''
    def inside_explored_area(self, pt):
        if len(self.traversal_sights) > 0:
            return inside_global_true_sight(pt, self.vision_range, self.traversal_sights)
        return False

    def ranking_active_open_point(self, ranker, goal):
        ranks_new = []
        self.local_active_open_rank_pts = []
        # Ranking new active openPts then stack to global set.
        if len(self.local_active_open_pts) > 0:
            ranks_new = np.array([ranker.rank(self.coordinate, pt, goal) for pt in self.local_active_open_pts])

            # store local open points and its ranking 
            self.local_active_open_rank_pts = np.concatenate((self.local_active_open_pts, ranks_new), axis=1)

    #def ranking_by_RRTree(self, open_sights, ranker, goal, RRT_star:RRTree_star):
    def ranking_by_RRTree(self, open_sights, ranker, goal, RRT_star):
        
        # clear old data
        self.local_active_open_pts  = []
        self.rank_score = []
        self.local_active_open_rank_pts = []

        # get all neighbour nodes in radius area
        neighbour_nodes = RRT_star.neighbour_nodes(node_coordinate=self.coordinate, radius=self.vision_range)
        if len(neighbour_nodes):
            current_node = RRT_star.get_node_by_coords(self.coordinate)

            ancentor, _ = RRT_star.find_next(start_node=current_node, nodes = neighbour_nodes)

            active_open_nodes = np.array(neighbour_nodes)
            # remove local_point which is inside explored area
            if len(self.traversal_sights) > 0:
                inside_status = [inside_global_true_sight(node.coords, self.vision_range, self.traversal_sights) for node in neighbour_nodes]
                active_open_nodes = active_open_nodes[np.logical_not(inside_status)]


            for open_sight in open_sights:
                inside_status = [inside_angle_area(ao_node.coords, self.coordinate, open_sight)[0] for ao_node in active_open_nodes]
                active_arc_nodes = active_open_nodes[inside_status]
                if len (active_arc_nodes) > 0:
                    if ancentor in active_arc_nodes:
                        self.local_active_open_pts.append(ancentor.coords)
                        self.rank_score.append(float('inf'))
                    else:   # select active arc_nodes
                        dist_c_n = [point_dist(self.coordinate, n_node.coords) for n_node in active_arc_nodes]
                        node_idx = np.argmax(dist_c_n)
                        self.local_active_open_pts.append(active_arc_nodes[node_idx].coords)
                        self.rank_score.append(1/active_arc_nodes[node_idx].cost)
            self.local_active_open_pts = np.array(self.local_active_open_pts)
            self.rank_score = np.reshape(self.rank_score, newshape=(len(self.rank_score), 1))
            if len(self.local_active_open_pts):
                self.local_active_open_rank_pts = np.concatenate((self.local_active_open_pts, self.rank_score), axis=1)

    ''' check whether local open_points are active '''
    def get_local_active_open_ranking_points(self, open_sights, ranker, goal, RRT_star=None, ranking_type= Ranking_type.Distance_Angle):
        if ranking_type == Ranking_type.Distance_Angle:
            # get all local points from open sights
            self.get_local_open_points(open_sights)

            # get only active open point
            self.get_local_active_open_points()

            # ranking and store it to local active open ranking points
            self.ranking_active_open_point(ranker=ranker, goal=goal)
        elif ranking_type == Ranking_type.RRTstar:
            self.ranking_by_RRTree(open_sights, ranker, goal, RRT_star)

    ''' the active_open_rank_pts has to not empty '''
    def pick_max_ranking(self, active_open_rank_pts):
        ranks = active_open_rank_pts[:, 2]
        next_pt_idx = np.argmax(ranks)
        next_point = active_open_rank_pts[next_pt_idx, 0:2]        
        return next_point, next_pt_idx

    ''' pick next point, where its ranking is heighest, in given list '''
    def pick_next_point(self, goal, picking_strategy = Picking_strategy.global_first):
        next_point, next_pt_idx = None, -1  # default (none, idx = -1)

        global_len = len(self.global_active_open_rank_pts)  # Note: global set already get current local
        local_len = len(self.local_active_open_rank_pts)

        # if already saw/reach goal, next point is goal
        if self.saw_goal or self.reach_goal:
            next_point = goal
            
        else:   # pick next one in waiting set.
            # pop the next point in global set first
            if picking_strategy == Picking_strategy.global_first:     # picking global first
                if global_len > 0:
                    next_point, next_pt_idx = self.pick_max_ranking(self.global_active_open_rank_pts)
            
            # pop the next point in local set first, if the next local point is not exist then pick
            # in global set.
            elif picking_strategy == Picking_strategy.local_first:     # picking local first
                if local_len > 0:
                    next_point, next_pt_idx = self.pick_max_ranking(self.local_active_open_rank_pts)
                    # global index 
                    next_pt_idx = global_len - local_len + next_pt_idx
                elif global_len > 0:
                    next_point, next_pt_idx = self.pick_max_ranking(self.global_active_open_rank_pts)

            # then remove picked point from active global open point
            if global_len > 0:
                self.remove_global_active_pts_by_index(next_pt_idx)

        return next_point

    ''' remove active point form active list '''
    def remove_global_active_pts_by_index(self, point_idx):
        self.global_active_open_rank_pts = np.delete(self.global_active_open_rank_pts, point_idx, axis=0)

    ''' remove active point form active list '''
    def remove_local_active_pts_by_index(self, point_idx):
        self.local_active_open_rank_pts = np.delete(self.local_active_open_rank_pts, point_idx, axis=0)

    ''' add local active and its ranking to global active points set '''
    def expand_global_open_ranking_points(self, local_active_open_pts):
        if len(local_active_open_pts) > 0:
            if len(self.global_active_open_rank_pts) == 0:
                self.global_active_open_rank_pts = np.array(local_active_open_pts)
            else:
                self.global_active_open_rank_pts = np.concatenate((self.global_active_open_rank_pts, local_active_open_pts), axis=0)

    ''' record look ahead path '''
    def set_look_ahead_to_goal(self, path):
        self.path_look_ahead_to_goal = path

    # @Tu ----------------------------------------
    def generate_jump_amount_list_to_action(self):
        R = self.vision_range
        X = int(R) # R is the radius
        for x in range(-X,X+1):
            Y = int((R*R-x*x)**0.5) # bound for y given x
            for y in range(-Y,Y+1):
                if x == 0 and y == 0:
                    continue
                self.jump_amount_list_to_action.append((x, y));

    def action(self, choice):
        x = self.jump_amount_list_to_action[choice][0]
        y = self.jump_amount_list_to_action[choice][1]
        self.move(x=x, y=y)

    def move(self, x, y):
        self.coordinate = tuple(map(operator.add, self.coordinate, (x, y)))

        # Generate new grid
        self.generate_grid_coordinates()
    
    def generate_grid_coordinates(self):
        self.grid_coordinates = []
        R = self.vision_range
        X = int(R) # R is the radius
        for x in range(-X,X+1):
            Y = int((R*R-x*x)**0.5) # bound for y given x
            for y in range(-Y,Y+1):
                if x == 0 and y == 0:
                    continue
                self.grid_coordinates.append((x + self.coordinate[0], y + self.coordinate[1]));

    #get coordinate of a node (miisie)
    def get_robot_coords(self):
        return self.coordinate
    
    ''' check rrt* path in vision range collides obstacles '''
    def check_path_collides_obstacles(self, path_line_segments, obstacles_line_segments):
        check = False
        obs_ls = []
        for obstracle_lss in obstacles_line_segments:
            print(obstracle_lss)
            for ls in obstracle_lss:
                pt_is = line_across(ls, path_line_segments)
                if pt_is:
                    check = True
                    break
            if check:
                obs_ls = obstracle_lss
                break 
        return check, obs_ls
    
    def check_intersection_obs(self, obstacles, node, next_node): 
        line_segment = ([node.coords, next_node.coords])
        return self.check_path_collides_obstacles(line_segment,obstacles.obstacles_line_segments)
       
    ''' check line segments between current node and its neighbors collide obstacles '''
    def check_neighbor_nodes_path(self, line_segments, obstacles, visited_neighbor_nodes):
        temp_filter=[]
        intersect = False
        for idx in range(len(line_segments)):
            for obstacle_ls in obstacles.obstacles_line_segments:
                if intersect:
                    break
                for ls in obstacle_ls:
                    pt_is = line_across(ls, line_segments[idx])
                    if pt_is:
                        intersect = True
                        break
                    
            if not intersect:
                temp_filter.append(visited_neighbor_nodes[idx])
            intersect = False
        return temp_filter
    
    def avg_distance_to_obs(self, current_node, obstacles):
        for obstracle_lss in obstacles.obstacles_line_segments:
            for ls in obstracle_lss:
                for idx in range len(ls):
                    point_dist(current_node.coords,ls[idx])
        return