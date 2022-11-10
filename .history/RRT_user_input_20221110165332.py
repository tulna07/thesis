import argparse

"""
get user inputs
"""

def menu_RRT():
    parser = argparse.ArgumentParser(description='RRT animation.')
    parser.add_argument('-n', metavar="number of iteration", type=int, help='number of iteration', default=0)
    parser.add_argument('-m', metavar="map name", help='map name', default='_map_blocks_1.csv')
    parser.add_argument('-w', metavar="world_image", help='world model')
    parser.add_argument('-step_size', metavar="step_size", type=float, help='step size', default=5.0)
    parser.add_argument('-ss', metavar="sample_size", type=int, help='sample size', default=2000)
    parser.add_argument('-radius', metavar="neighbour radius", type=float, help='radius', default=10.0)
    parser.add_argument('-r', metavar="vision range", type=float, help='vision range', default=10.0)
    parser.add_argument('-view', "--view", action='store_true', help='view map', default=False)
    # draw the tree
    
    # parser.add_argument('-sx', metavar="start_x", type=float, help='start point x', default=30.0)
    # parser.add_argument('-sy', metavar="start_y", type=float, help='start point y', default=94.0)
    # parser.add_argument('-gx', metavar="goal_x", type=float, help='goal point x', default=85.0)
    # parser.add_argument('-gy', metavar="goal_y", type=float, help='goal point y', default=20.0)
    
    # training
    
    parser.add_argument('-sx', metavar="start_x", type=float, help='start point x', default=85.0)
    parser.add_argument('-sy', metavar="start_y", type=float, help='start point y', default=20.0)
    parser.add_argument('-gx', metavar="goal_x", type=float, help='goal point x', default=30.0)
    parser.add_argument('-gy', metavar="goal_y", type=float, help='goal point y', default=94.0)
    
    # 33,57 83.03
    # 85,20 118
    # 99,60
    
    args = parser.parse_args()

    return args