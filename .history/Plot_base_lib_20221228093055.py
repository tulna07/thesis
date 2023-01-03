import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import math

from Robot_map_lib import Map
from Robot_world_lib import World
from Robot_base import RobotType
from Program_config import *


class Plot_base:
    def __init__(self, size=(7,7), title="Autonomous Robot"):
        self.plt   = plt
        self.fig, self.ax = plt.subplots(figsize=size)
        #self.plt.figure(title)
        self.fig.canvas.manager.set_window_title(title)
        #self.fig.canvas.set_window_title(title)
        #self.plt.xlim(0, 100)
        #self.plt.ylim(0, 100)

    show = lambda self: self.plt.show()
    pause = lambda self, x: self.plt.pause(x)
    clear = lambda self: self.plt.cla()
    set_equal = lambda self: self.plt.axis("equal")
    show_grid = lambda self: self.plt.grid(True)
    
    ''' plot point(s)'''
    point  = lambda self, point,  ls=".r": self.plt.plot(point[0], point[1], ls)
    points = lambda self, points, ls=".r": self.plt.plot(points[:, 0], points[:, 1], ls)
    
    '''
    plot points with color map
    visit https://www.w3schools.com/python/matplotlib_scatter.asp to see more color map
    '''
    def point_colors(self, points, colors, colormap = 'winter_r', marker='.'):
        #max_range = int(max(colors)) + 1
        self.plt.scatter(points[:, 0], points[:, 1], c=colors, cmap=colormap, marker=marker)
        #self.plt.colorbar().remove() # remove old one
        #self.plt.colorbar()          # update new one
        #cbar = plt.colorbar(ticks=[])   # dont show ticks number
        #self.plt.clim(0, max_range)

    ''' plot text at near point coordinate '''
    text = lambda self, point, str: self.plt.text(point[0], point[1] + 2, str)

    ''' plot text and point'''
    def point_text(self, point, ls, text):
        self.point(point, ls)
        self.text(point, text)

    ''' plot line segment connecting 2 points'''
    line_segment = lambda self, line, ls="-.k", lw=1: self.plt.plot((line[0][0], line[1][0]), (line[0][1], line[1][1]),\
         ls,linewidth=lw)
    
    ''' plot path containing list of points '''
    def path(self, points, ls="-xr"):
        self.points(points=np.array(points), ls=ls)

    ''' plot path color containing list of points '''
    def path_color(self, points, color):
        new_point = np.array(points)
        plt.plot(new_point[:, 0], new_point[:, 1], c=color)

    ''' plot path with color '''
    def paths_color(self, paths, directions):
        cmap = plt.get_cmap('jet_r')
        c_forward = cmap(float(0.1))
        c_backward = cmap(float(0))
        # draw paths with 2 colors, red if direction = forward(true)
        # dark read if direction = backward (false)
        for path, direction in zip(paths, directions):
            if direction:   
                self.path_color(path, c_forward)
            else:
                self.path_color(path, c_backward)

    def polygon(self, polygon: list, ls= "-r"):
        new_points = polygon.copy()
        new_points.append(new_points[0])
        self.path(points= new_points, ls=ls)
    
    def polygons(self, polygons: list, ls= "-r"):   # polygons is list of polygon
        for polygon in polygons:
            self.polygon(polygon=polygon, ls = ls)

    ''' plot triangle (out fill) '''
    def plot_triangle(self, triangle, ls=":c"):
        self.line_segment((triangle[0], triangle[1]), ls)
        self.line_segment((triangle[1], triangle[2]), ls)
        self.line_segment((triangle[2], triangle[0]), ls)

    ''' plot triangle ( within fill) '''
    def plot_triangle_fill(self, triangle, cl="g", alpha=transparent, linestyle=":"):
        new_triangle = np.array(triangle)
        self.plt.fill(new_triangle[:, 0], new_triangle[:, 1], color=cl, alpha=alpha, linestyle=linestyle)
    
    ''' Goal'''
    def goal(self, goal, r_goal=False, s_goal=False):
        self.point(goal, ls_goal)
        if show_text_goal:
            if r_goal: 
                self.text(goal, "reached goal!")
            elif s_goal:
                self.text(goal, "saw goal!")
            else:
                self.text(goal, "goal")
    ''' Start '''
    def start(self, start):
        self.point_text(start, ls_start, "start!")

    ''' Obstacles'''
    def show_map(self, world_name=None, obstacles=None, plot_title=None):
        # draw world and map
        if show_world and world_name is not None:
            World().display(self.plt, mpimg, world_name)
            
        # draw map obstacles 
        if show_map:
            Map().display(self.plt, plot_title, obstacles.obstacles)
    
    ''' set plot's title'''
    title = lambda self, x: self.plt.title(x)
    
    ''' prepare title for display'''
    def prepare_title(self, iter_count, path_cost):
        plot_title = "Number of iteration: {0}".format(iter_count)
        if path_cost > 0 and path_cost != float('inf'):
            plot_title += ", path len: {:.2f}".format (path_cost)
        else:
            plot_title += ", not reached goal yet."
        return plot_title
    
    ''' prepare title for display'''
    def prepare_title_RRT_star(self, iter_count, tree):
        # plot_title = "Number of iteration: {0}".format(iter_count)
        plot_title= ""
        if tree.total_goal_cost > 0 and tree.total_goal_cost != float('inf'):
            plot_title += "Path length: {:.2f}".format (tree.total_goal_cost);
            plot_title += ", Waypoints: {}".format (len(tree.path_to_goal)-2);
        else:
            plot_title += ", not reached goal yet."
        return plot_title

    ''' vision libs'''

    ''' vision circle'''
    # @Tu
    def points_in_vision(self, center, vision_radius):
        R = vision_radius
        X = int(R) # R is the radius
        for x in range(-X,X+1):
            Y = int((R*R-x*x)**0.5) # bound for y given x
            for y in range(-Y,Y+1):
                yield (center[0]+ x, center[1] + y)
    # @Tu
    def vision_area(self, center, vision_radius, ls=":"):
        """ draw a circle that limits the vision of robot """
        for point in self.points_in_vision(center, vision_radius):
            plt.plot(point[0], point[1], 'bo', markersize=1, color="black")
        vision = plt.Circle(center, vision_radius, color="red", linestyle=ls, fill=False)
        plt.gcf().gca().add_artist(vision)

    ''' Robot '''
    def robot(self, robot, yaw=0):  # pragma: no cover
        x,y = robot.coordinate
        if robot.robot_type == RobotType.rectangle:
            outline = np.array([[-robot.length / 2, robot.length / 2,
                                (robot.length / 2), -robot.length / 2,
                                -robot.length / 2],
                                [robot.width / 2, robot.width / 2,
                                - robot.width / 2, -robot.width / 2,
                                robot.width / 2]])
            Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                            [-math.sin(yaw), math.cos(yaw)]])
            outline = (outline.T.dot(Rot1)).T
            outline[0, :] += x
            outline[1, :] += y
            plt.plot(np.array(outline[0, :]).flatten(),
                    np.array(outline[1, :]).flatten(), "-k")
        elif robot.robot_type == RobotType.circle:
            circle = plt.Circle((x, y), robot.radius, color="b")
            plt.gcf().gca().add_artist(circle)
            out_x, out_y = (np.array([x, y]) +
                            np.array([np.cos(yaw), np.sin(yaw)]) * robot.radius)
            plt.plot([x, out_x], [y, out_y], "-k")
    
    ''' save plot as image/pdf/svg/eps'''
    def save_figure(self, fig_name = "image",  file_extension = ".png", dpi=150):
        self.plt.savefig(fig_name + file_extension, bbox_inches ="tight", dpi=dpi)