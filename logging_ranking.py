from operator import contains
from pathlib import Path
from RRTree_star import RRTree_star
import json
import csv
from Robot_math_lib import approximately_num
from Tree import Node

class Logging_ranking:
    def __init__(self) -> None:
        self.coord_key = "coordination"
        self.lmc_key = "lmc"
        self.cost_key = "cost"
        self.parent_key = "parent"
        self.children_key = "children"
        self.neighbours_key = "neighbours"
        self.neighbours_weight_key = "neighbours_weight"

    def set_logging_name(self, map_name, goal, radius=10, step_size=5, sample_size=10):
        return "log_ranking_tree_map{0}_goal_{1}_{2}_radius{3}_step_size{4}_sample_size{5}.csv".format(\
            map_name, int(goal[0]), int(goal[1]), radius, step_size, sample_size)
    
    def is_existed_log_file(self, file_name):
        path = Path(file_name)
        return path.is_file()

    def write_nodes_info(self, node:Node, file_writer):
        result_items = []
        result_items.append(node.coords[0])
        result_items.append(node.coords[1])
        result_items.append(node.cost)
        if node.parent is not None:
            result_items.append(node.parent.coords[0])
            result_items.append(node.parent.coords[1])
        else:
            result_items.append("")
            result_items.append("")
        file_writer.writerow(result_items)
        for childnode in node.children:
                self.write_nodes_info(childnode, file_writer=file_writer)
        
    ''' logging tree as json format '''
    def save_tree(self, RRTree_star:RRTree_star, file_name):
        #f = open(file_name, "w")
        #self.write_nodes_info(node=RRTree_star.root,file_writer=f)
        #f.close()
        f = open(file_name, 'w', newline='', encoding="utf-8")
        writer = csv.writer(f, delimiter=",")
        writer.writerow(["coordinate_x","coordinate_y","cost",\
            "parent_coordinate_x","parent_coordinate_y"])
        self.write_nodes_info(node=RRTree_star.root,file_writer=writer)
        #for result_items in self.results_data:
        #    writer.writerow(result_items)
        f.close()
    

    def load(self, file_name):
        first_line = True
        first_tree = True
        new_node = None
        RRtree_star = None
        with open(file_name, newline='') as f:
            reader = csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONE)
            for row in reader:
                if first_line:
                    first_line = False
                    continue
                else:
                    coordinate = tuple ((float(row[0]), float(row[1])))
                    cost = float(row[2])
                    new_node = Node(coords=coordinate,cost=cost)
                    if first_tree:
                        RRtree_star = RRTree_star(new_node)
                        first_tree = False
                    else:
                        parent_coordinate = float(row[3]), float(row[4])
                        parent_node = RRtree_star.get_node_by_coords((parent_coordinate))
                        RRtree_star.add_node(new_node)
                        RRtree_star.node_link(parent_node=parent_node, node=new_node)
        return RRtree_star
    