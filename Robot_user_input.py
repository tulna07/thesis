import argparse

"""
get user inputs
"""

def menu_Robot():

    parser = argparse.ArgumentParser(description='Code for Autonomous Robot.')
    parser.add_argument('-n', metavar="number of iteration", type=int, help='number of iteration', default=1)
    parser.add_argument('-m', metavar="data_map", help='map data', default='_map.csv')
    parser.add_argument('-w', metavar="world_image", help='world model')
    parser.add_argument('-r', metavar="vision_range", type=float, help='vision range', default=20.0)
    parser.add_argument('-radius', metavar="robot radius", type=float, help='robot radius', default=0.5)
    parser.add_argument('-sx', metavar="start_x", type=float, help='start point x', default=0.0)
    parser.add_argument('-sy', metavar="start_y", type=float, help='start point y', default=0.0)
    parser.add_argument('-gx', metavar="goal_x", type=float, help='goal point x', default=70.0)
    parser.add_argument('-gy', metavar="goal_y", type=float, help='goal point y', default=90.0)

    parser.add_argument('-p', metavar="picking strategy", type=str, \
                help='input : (g) for global first, (l) for local first', default='l')

    parser.add_argument('-rank_type', metavar="ranking by tree or distance_and_angle formula", type=str,\
                help='input: (r) for ranking by  RRTreeStar; (da) for  ranking by distance_and_angle formula', default='r')
    args = parser.parse_args()
    return args