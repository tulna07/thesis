from Obstacles import Obstacles

from Robot_math_lib import *
from Program_config import *
from matplotlib import patches
import matplotlib.image as mpimg
from matplotlib.collections import PatchCollection

from Plot_base_lib import Plot_base
from Graph import Graph

class Plot_robot(Plot_base):
    def __init__(self, size=(7,7), title="Path Planning Problem for an Autonomous Robot"):
        super().__init__(size, title)

    def sight(self, center, pair, cl="g", alpha=transparent, linestyle=":"):
        triangle = [center, pair[0], pair[1]]
        self.plot_triangle_fill(triangle, cl, alpha, linestyle)


    def closed_sights(self, center, true_sight, cl="g", ls_ts="-"):
        for pair in true_sight:
            self.sight(center, pair, cl, transparent, ls_ts)

    def open_sights_arc(self, center, open_sight, radius, cl="g", ls_ts="-"):
        arc_patches = []

        # center_ox: a point starts from center and follows X-axis direction 
        center_ox = np.add(center, [1,0] )
        for arc in open_sight:
            theta1radian = unsigned_angle(center, center_ox, arc[0])
            theta2radian = unsigned_angle(center, center_ox, arc[1])
            theta1 = math.degrees(theta1radian)
            theta2 = math.degrees(theta2radian)
            wedge = patches.Wedge(center, radius, theta1=theta1, theta2=theta2)
            arc_patches.append(wedge)
        collection = PatchCollection(arc_patches, facecolor=cl, linestyle='solid', edgecolor='r', alpha=transparent)
        self.ax.add_collection(collection)

    def vision_area(self, center, radius, ls=":"):
        """ draw a circle that limits the vision of robot """
        vision = self.plt.Circle(center, radius, color="red", linestyle=ls, fill=False)
        self.plt.gcf().gca().add_artist(vision)

    def vision(self, center, radius, csight, osight):
        if show_circleRange:
            self.vision_area(center, radius)

        if show_closedSight:
            self.closed_sights(center, csight, cl_ts, ls_ts)

        if show_openSight:
            self.open_sights_arc(center, osight, radius)

    def paths(self, paths, ls="-r", ls_next="-b"):
        for i in range(len(paths)):
            path = paths[i]
            if i == len(paths) - 1:
                self.path(path, ls_next)
            else:
                self.path(path, ls)

    def show_configuration_space(self, config_space: list):
        self.polygons(config_space, ls = ls_cspace)
        
    def visibility_graph(self, graph: Graph , ls_vg):
        visibility_graph = graph.graph
        for pnode in visibility_graph:
            for verteces in visibility_graph[pnode]:
                self.line_segment([pnode, verteces], ls_vg)

    def critical_line_segments(self, critical_ls, ls_cls):
        i = 0
        for ls in critical_ls:
            self.line_segment(ls[1:3], ls_cls)
            if show_cls_orderednumber:
                self.text(ls[1], i)
                i += 1

    def show_traversal_sights(self, traversal_sights, vision_range):
        for local in traversal_sights:
            local_center = local[0]  # center of robot at local
            local_closed_sights = local[1]  # closed sight at local
            local_open_sights = local[2]  # open sight at local
            self.vision(local_center, vision_range, local_closed_sights, local_open_sights)

    def show_animation(self, Robot, world_name, iter_count, obstacles:Obstacles , goal, 
                    closed_sights, open_sights, skeleton_path, asp , critical_ls, next_point):

        # clear plot
        self.clear()
            
        # for stopping simulation with the esc key.
        self.plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
                  
        ''' draw map obstacles/world '''            
        # prepare title
        status_title = self.prepare_title(iter_count, Robot.cost)
        self.show_map(world_name=world_name, obstacles=obstacles, plot_title=status_title)
        if obstacles.enable_config_space:
            self.show_configuration_space(obstacles.config_space)
        # show_traversalSights
        if show_traversalSights:
            self.show_traversal_sights(Robot.traversal_sights, Robot.vision_range)
        
        if show_robot:
            self.robot(Robot,yaw=0)
        
        if show_goal:
            self.goal(goal, Robot.reach_goal, Robot.saw_goal)
        
        # plot robot's vision at local (center)
        self.vision(Robot.coordinate, Robot.vision_range, closed_sights, open_sights)
        
        if show_local_openpt and len(Robot.local_open_pts) > 0:
            self.points(Robot.local_open_pts, ls_lopt)
        
        # ranking points
        if show_active_openpt and len(Robot.global_active_open_rank_pts) > 0:
            self.point_colors(Robot.global_active_open_rank_pts, Robot.global_active_open_rank_pts[:,2])
        
        if show_visibilityGraph:
            self.visibility_graph(Robot.visibility_graph, ls_vg)
        
        if show_visitedPath:
            #self.paths(Robot.visited_path, ls_vp, ls_goingp)
            self.paths_color(Robot.visited_paths, Robot.visited_path_directions)
        
        if show_sketelonPath and len(skeleton_path) > 0:
            self.path(skeleton_path, ls_sp)
        
        if show_approximately_shortest_path and len(asp)>0:
            self.path(asp, ls_asp)
        
        if show_critical_line_segments:
            self.critical_line_segments(critical_ls, ls_cls)
        
            # display next point if existing
        if show_next_point:
            if next_point is not None:
                self.point(next_point, ls_nextpt)
        
        # to set equal make sure x y axises are same resolution 
        self.set_equal()
        self.show_grid()
        if not easy_experiment:
            self.plt.pause(1)

    ''' plot connection between 2 nodes'''
    def connection(self, nodeA, nodeB, ls="-b"):
        self.line_segment( (nodeA.coords, nodeB.coords), ls=ls)
    
    ''' plot edges from node to its children '''
    def tree_edges(self, node, ls=ls_tree_edge):
        for node_children in node.children:
            self.line_segment( (node.coords, node_children.coords), ls)
    
    ''' plot a tree's node '''
    def tree_node(self, node, ls_active=ls_tree_node_active, ls_inactive= ls_tree_node_inactive,\
            ls_visited = ls_tree_node_visited):

        if node.visited:
            self.point(node.coords, ls=ls_visited)
        elif node.active:
            self.point(node.coords, ls=ls_active)
        else:
            self.point(node.coords, ls=ls_inactive)

    ''' plot tree (all edges and vertices) from given node as tree's root '''
    def tree(self, tree, node_en=True, edge_en=True, color_mode=TreeColor.no):
        nodes_coords  = []
        nodes_lmc = []
        nodes_cost = []
        for node in tree.all_nodes():   # get all nodes
            nodes_coords.append(node.coords)
            nodes_lmc.append(node.lmc)
            nodes_cost.append(node.cost)

            # draw edge
            if edge_en:
                self.tree_edges(node)      # plot all edges between node and its children            
            if node_en and color_mode==TreeColor.no:
                self.tree_node(node)   # plot nodes

        nodes_coords = np.array(nodes_coords)
        if node_en:
            if color_mode == TreeColor.by_lmc:
                self.point_colors(nodes_coords, nodes_lmc, colormap="Dark2")
            elif color_mode == TreeColor.by_cost:
                self.point_colors(nodes_coords, nodes_cost, colormap="Dark2")

    ''' plot all trees'node (ertices) from given node as tree's root '''
    def tree_all_nodes(self, tree):
        for node in tree.all_nodes():   # get all nodes
            self.tree_node(node)   # plot nodes
            #self.text(node.coords, "{0:.2f}".format(node.rhs))

    ''' plot all info ( number of iteration, cost, path, tree) '''
    def animation(self, num_iter, cost, path, Tree, obstacles,  start_coords, goal_coords):
        # prepare title
        reach_goal = (cost > 0 and cost != float("inf") )
        status_title = self.prepare_title(num_iter,cost)

        # clear old one
        self.clear()

        # plot new one
        self.show_map(world_name=None, obstacles=obstacles, plot_title=status_title)
        self.tree(Tree)
        self.goal(goal_coords, reach_goal, None)
        self.start(start_coords)
        self.pause(0.1)
