import re
import numpy as np
import matplotlib.pyplot as plt
from ROS_read_yaml import read_yaml
from ROS_read_pgm import read_pgm

''' user input '''
import argparse



# declare filenames
#map_name = "turtlebot3_house"
#map_name = "turtlebot3_world"
#map_name = 'turtlebot3_world_full'
#yaml_file = map_name + ".yaml"
#map_file = map_name + ".pgm"

# using term coordinate or point (pt) to denote location in gazebo world
# using pixel (short px) to denote location in map image
def odom_to_pixel(origin_coordinate_px, resolution, odom_pose):
    ''' convert odometry to map pixel '''

    # calculate odom pose of robot in pixel
    pose_px = np.divide(odom_pose, resolution)
    print ("pose offset (in pixel):" , pose_px)

    return [origin_coordinate_px[0] + pose_px[0], origin_coordinate_px[1] - pose_px[1]]

def pixel_to_odom(origin_coordinate_px, resolution, next_pt_pixel):
    ''' convert map pixel to odometry '''
    odom_offset_coordinate = [next_pt_pixel[0] - origin_coordinate_px[0], origin_coordinate_px[1] - next_pt_pixel[1]]
    return np.multiply(odom_offset_coordinate, resolution)

def cal_origin_coordinate_pixel(img_central_pixel, origin):
    # convert ogirin coordinate of xyz ,where is at middle of gazebo world (0,0,0), to image pixel
    return [img_central_pixel[0] - origin[0], img_central_pixel[1] + origin[1]]

''' get user input '''
def menu():
    parser = argparse.ArgumentParser(description='odometry pose and image pixel converter.')
    parser.add_argument('-sm', help='slam map', default='turtlebot3_world_full')
    parser.add_argument('-ox', type=float, help='(odometry) pose: x',default=-2.0)
    parser.add_argument('-oy', type=float, help='(odometry) pose: y',default=-0.5)
    parser.add_argument('-px', type=float, help='(next point) pixel: x',default=200.0)
    parser.add_argument('-py', type=float, help='(next point) pixel: y',default=150.0)
    args = parser.parse_args()
    print ("___________________")
    print (args)

    return args
if __name__ == "__main__":
    ''' get user input'''
    userin = menu()
    slam_map = userin.sm
    yaml_file = slam_map + ".yaml"
    map_file = slam_map + ".pgm"

    ''' read map image '''
    image = read_pgm(map_file, byteorder='<')
    w, h = np.shape(image)
    #image central pixel (z = 0)
    imgc_px = [w/2, h/2]
    print ("image size" , (w, h))
    print ("image central pixel", imgc_px)


    ''' read map yaml info '''
    yaml_data = read_yaml(yaml_file)
    resolution = yaml_data["resolution"]
    origin = yaml_data["origin"]
    print ("resolution ", resolution)
    print ("origin ", origin)

    # calculate origin coordinate in image pixel
    origin_coordinate_px = cal_origin_coordinate_pixel(imgc_px, origin)
    
    # get odometry pose info (x = -2, y = - 0.5, z = 0)  then convert to pixel
    odom_pose = [userin.ox, userin.oy]   # get from odom topic (replace this info by /odom topic)
    print ("odom_pose ", odom_pose)

    # calculate robot position in pixel
    robot_px = odom_to_pixel(origin_coordinate_px, resolution, odom_pose)
    print ("robot_coordinate", robot_px)

    ''' move to next point in image pixel '''
    next_pt_px = [userin.px, userin.py]
    # calculate next odom pose in gazebo world
    nxt_odom_pose = pixel_to_odom(origin_coordinate_px, resolution, next_pt_px)


    # show map and robot position
    fig = plt.figure(figsize=(5, 5))
    fig.canvas.set_window_title('Odometry information and image pixel converter')
    plt.imshow(image, plt.cm.gray)
    points = np.array([ imgc_px,                 # middle pixel in image
                        origin_coordinate_px,    # origin cooridate (0,0,0) in image
                        robot_px,                # robot in image
                        next_pt_px,              # move to next point in image  
                        [0,0],
                        [w,0],
                        [0,h],
                        [w,h]
                       ])
    print (points)

    # for testing
    i = 0
    for pt in points:
        plt.plot(pt[0], pt[1], 'r.')
        if i == 0:  # middle pixel in image
            plt.text(pt[0], pt[1], "central px") 
        elif i == 1:  # origin cooridate (0,0,0) in image
            plt.text(pt[0], pt[1], "origin [0, 0, 0]")
        elif i == 2:  # robot in image
            plt.text(pt[0], pt[1], "robot {0}".format(odom_pose))
        elif i == 3:  # next point
            plt.text(pt[0], pt[1], "robot_next {0}".format(nxt_odom_pose))
        else:
            plt.text(pt[0], pt[1], "pt{0}".format(i))
        i = i + 1
    plt.show()

