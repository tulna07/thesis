'''
autonomousRobot
reimplementing an algorithm which has been written from the following paper:
https://pdfs.semanticscholar.org/0cac/84962d0f1176c43b3319d379a6bf478d50fd.pdf
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
'''

from platform import node
from Robot_math_lib import approximately_num, point_dist
import numpy as np
from Queue_class import Priority_queue
delta_consistency = 0
''' Node class '''
class Node:
    def __init__(self, coords=None, cost=float('inf'), lmc=float('inf')):
        self.coords   = tuple(coords)   # coordinates of a node
        self.children = []
        self.parent  = None
        self.active = True          # if active = false (inactive), its belong to obstacle space
        self.lmc = lmc              # lmc is a measurement which look-ahead estimate of cost-to-goal
        self.cost = cost            # cost-to-goal
        #self.weight = weight        # weight from node to parent
        self.neighbours = []        # node's neighbours (for RTTx)
        self.neighbours_weight = [] # node's neighbours weight (for RRTx)
        self.visited = False        # status of visitting

    #get coordinate of a node (miisie)
    def get_coord(self):
        return self.coords 
     
    ''' set lmc '''
    def set_lmc(self, x):
        self.lmc = x

    ''' set cost '''
    def set_cost(self, x):
        self.cost = x   

    ''' update self lmc and all its children lmc (for rewiring) '''
    def update_lmcs(self):
        self.lmc = self.parent.lmc + self.weight
        [children_node.update_lmcs() for children_node in self.children]

    ''' update self cost and all its children costs (for rewiring) '''
    def update_costs(self):
        self.cost = self.parent.cost + self.get_neighbour_weight(self.parent)
        [children_node.update_costs() for children_node in self.children]

    ''' set visited node '''
    def set_visited(self):
        self.visited = True

    ''' set inactive node '''
    def set_inactive(self):
        self.active = False

    ''' bring random coordinate closer to nearest node '''
    def bring_closer_coordinate(self, random_node_coords, nearest_dist, step_size):
        min_dist = min(nearest_dist, step_size); # If the node is closer than epsilon, we have the same distance
        closer_coordinate = self.coords + (random_node_coords - self.coords)*min_dist/nearest_dist
        # to avoid error cause by the percise of floating point
        closer_coordinate = approximately_num(closer_coordinate[0]), approximately_num(closer_coordinate[1])
        return closer_coordinate

    add_child   = lambda self, x: self.children.append(x)
    remove_child   = lambda self, x: self.children.remove(x)

    ''' add parent '''
    def add_parent(self, parent):
        self.parent = parent
    ''' remove parent '''
    def remove_parent(self):
        self.parent = None
    
    ''' add a neighbour '''
    def add_neighbour(self, neighbour):
        weight = point_dist(self.coords, neighbour.coords)
        self.neighbours.append(neighbour)   # home's side
        self.neighbours_weight.append(weight)

        neighbour.neighbours.append(self)   # neighbours's side
        neighbour.neighbours_weight.append(weight)

    ''' add neighbours '''
    def add_neighbours(self, neighbours):
        for neighbour in neighbours:
            self.add_neighbour(neighbour)
    
    ''' set neighbour weight '''
    def set_neighbour_weight(self, node, weight):
        if node in self.neighbours:
            idx = self.neighbours.index(node)
            self.neighbours_weight[idx] = weight
            return True
        return False
    
    ''' get neighbour weight '''
    def get_neighbour_weight(self, node):
        if node in self.neighbours:
            idx = self.neighbours.index(node)
            return self.neighbours_weight[idx]
        return None

    ''' get all children '''
    def all_children(self):
        return self.children

    ''' print all node's neighbours'''
    def print_neighbours(self):
        neighbours_coords = []
        for nn in self.neighbours:
            neighbours_coords.append(nn.coords)
        print ("_neighbours: ", neighbours_coords)

    ''' print all node information '''
    def print (self):
        print ("\nNode coordinate:", self.coords)
        parent_coords = ''
        if self.parent is not None:
            parent_coords = self.parent.coords
        print ("_parents:", parent_coords)
        child_coords = []
        for child in self.children:
            child_coords.append(child.coords)
        print ("_children: ", child_coords)
        print ("_cost: ", str(self.cost))
        print ("_lmc: ", self.lmc)
        #self.print_neighbours()
        #print ("_neighbour: ", neighbours_coords)
        #print ("_neighbour weight: ", self.neighbours_weight)


''' Tree class '''
class Tree:
    def __init__(self, root: Node):
        self.root   = root
        self.dict   = {root.coords: self.root}

    ''' return node at give coords'''
    def get_node_by_coords(self, coordinate): # at_node
        return self.dict[coordinate]

    ''' add node to tree'''
    def add_node(self, new_node:Node):
        self.dict[new_node.coords]  = new_node
    
    ''' make an edge to tree via parent node; cost and weight also added'''
    def add_edge(self, parent_node: Node, node: Node):
        # calculate cost for new node
        cost = parent_node.cost + parent_node.get_neighbour_weight(node)
        node.set_cost(cost)
        
        # link to parent
        self.node_link(parent_node=parent_node, node=node)
    
    ''' FOR RRTX: make an edge to tree via parent node; lmc and weight also added'''
    def add_edge_RRTx(self, parent_node: Node, node: Node):
        # calculate lmc for new node
        lmc = parent_node.lmc + parent_node.get_neighbour_weight(node)
        node.set_lmc(lmc)       # lmc for RRTree_x

        # link to parent
        self.node_link(parent_node=parent_node, node=node)

    ''' create a link between parent node and node '''
    def node_link(self, parent_node: Node, node: Node):
        node.parent = parent_node
        parent_node.add_child(node)

    ''' remove a link between node and its parent '''
    def remove_edge(self, parent_node: Node, node: Node):
        # print(node)
        # if node in node.children:
        # print("debug", node, parent_node.children)
        if node in self.dict[parent_node.coords].children:
            self.dict[parent_node.coords].remove_child(node)
        node.parent = None

    __getitem__  = lambda self, x: self.dict[x]
    __contains__ = lambda self, x: x in self.dict

    ''' return a list of all tree's nodes'''
    def all_nodes(self):
        return list(self.dict.values())

    ''' return a list of all tree's nodes coordinate'''
    def all_nodes_coordinate(self):
        return list(self.dict.keys())

    ''' return all children of nodes in tree '''
    def all_subtree_nodes_at_node(self, nodes):
        all_children =[]
        for node in nodes:
            all_children.extend(node.all_children())

        for child in all_children:
            all_children.extend(child.all_children())
            
        return all_children
            
    ''' get cost of the given node '''
    def node_cost(self, node: Node):
        return node.cost

    ''' get costs of the given nodes '''
    def node_costs(self, nodes):
        return [node.cost for node in nodes]

    ''' get all costs of all tree's nodes'''
    def all_node_costs(self):
        return [node.cost for node in self.all_nodes()]

    ''' update all node's costs of subtree where subtree's root was given '''
    def update_subtree_costs(self, node: Node, cost= float('inf')):
        node.cost = cost
        for children_node in node.children:
            self.update_subtree_costs(node=children_node, cost=node.cost)

    ''' get weight of the given node '''
    def node_weight(self, node: Node):
        return node.weight

    ''' get lmc of given node '''
    def node_lmc(self, node: Node):
        return node.lmc

    ''' get lmcs of given nodes '''
    def node_lmcs(self, nodes):
        return [node.lmc for node in nodes]

    ''' calculate all lmcs of all tree's nodes '''
    def all_node_lmces(self):
        return [node.lmc for node in self.all_nodes()]

    ''' calcualte distances among node and given tree's nodes '''
    def distances(self, node_coords, tree_nodes):
        return [point_dist(node_coords, node.coords) for node in tree_nodes]

    ''' calculate all distances among node and all tree's nodes '''
    def all_distances(self, node_coords):
        return [point_dist(node_coords, n_coords) for n_coords in self.all_nodes_coordinate()]
    
    ''' bring random coordinate closer to nearest node in tree '''
    def bring_closer(self, rand_coordinate ):
        # find the neareset node (in distance) to random coordinate
        nearest_dist, nearest_node = self.nearest(rand_coordinate)

        # bring random coordinate closer to nearest node
        picked_coordinate = nearest_node.bring_closer_coordinate(rand_coordinate, nearest_dist, self.step_size)
        return picked_coordinate

    ''' update lmc of given node '''
    def update_LMC(self, node: Node):
        neighbour_nodes = node.neighbours
        active_neighbour_nodes = []

        for n_node in neighbour_nodes:
            if n_node.active:
                active_neighbour_nodes.append(n_node)
        neighours_smallest_lmc = self.neighbours_smallest_lmc(node.coords, active_neighbour_nodes)
        if neighours_smallest_lmc is not None:
            if node.parent is not None:
                self.remove_edge(parent_node=node.parent, node=node)    # remove old link
            self.add_edge_RRTx(parent_node=neighours_smallest_lmc, node=node)

    ''' find the nearest node to the given nodes, return its distance and index '''
    def nearest(self, node_coords):
        all_dist = self.all_distances(node_coords)
        nearest_idx = np.argmin(all_dist)
        nearest_distance = all_dist[nearest_idx]
        new_key = list(self.dict)
        nearest_key = new_key[nearest_idx]

        return nearest_distance, self.dict[nearest_key]
    
    '''  find nearest (in term of cost) neighbour node from list of nodes'''
    def neighbours_smallest_cost(self, node_coordinate, neighbour_nodes):
        # check if there is neighbour nearby
        if neighbour_nodes is None:
            return None
            
        # calculate all cost from random's neighbours tree's node to tree's root
        n_costs = np.array(self.node_costs(neighbour_nodes))
        # get distances from random node to all its neighbours
        n_dist = np.array(self.distances(node_coordinate, neighbour_nodes))
        # pick the closest neighbour
        n_idx = np.argmin(n_costs +  n_dist)
        nearest_neighbour_node = neighbour_nodes[n_idx]

        return nearest_neighbour_node

    '''  find nearest (in term of lmc) neighbour node from list of nodes'''
    def neighbours_smallest_lmc(self, node_coordinate, neighbour_nodes):
        # check if there is neighbour nearby
        nearest_neighbour_node = None
        if len(neighbour_nodes)> 0:
            # calculate all cost from random's neighbours tree's node to tree's root
            n_lmces = np.array(self.node_lmcs(neighbour_nodes))
            # get distances from random node to all its neighbours
            n_dist = np.array(self.distances(node_coordinate, neighbour_nodes))
            # pick the closest neighbour
            n_idx = np.argmin(n_lmces +  n_dist)
            nearest_neighbour_node = neighbour_nodes[n_idx]
        return nearest_neighbour_node

    '''  find nearest neighbour node to node_coordinate in circle of radius '''
    def nearest_neighbour(self, node_coordinate, radius):
        # find all neighbours of node_coordinate
        neighbour_nodes = self.neighbour_nodes(node_coordinate, radius)
        # pick nearest neighbour as new_node's parent
        return self.neighbours_smallest_cost(node_coordinate, neighbour_nodes)

    ''' find neighbour nodes at give coordinate in radius area '''
    def neighbour_nodes(self, node_coordinate, radius):
        # get array of all RRTree's node
        all_nodes = np.array(self.all_nodes())

        # calcualte all distances from random to tree nodes
        all_distances = np.array(self.all_distances(node_coordinate))

        # get all random's neighbours (represent n_) nodes which are inside radius
        n_node_in_range = all_distances < radius  
        n_node_valid = all_distances > 0
        n_node_indices = n_node_in_range & n_node_valid
        if np.sum(n_node_indices) == 0: # no-one lives nearby :)
            return None
        return all_nodes[n_node_indices]   
    
    ''' set edge weight between 2 nodes '''
    def set_edge_weight(self, nodeA:Node, nodeB:Node, weight=float("inf")):
        nodeA.set_neighbour_weight(node=nodeB, weight=float("inf"))
        nodeB.set_neighbour_weight(node=nodeA, weight=float("inf"))

    ''' print tree information '''
    def printTree(self, node, depth=0):
        if node is None: return
        print(" | "*depth, node)

        for child in node.children:
            self.printTree(child, depth+1)
    
    '''' find tree root , start from given node'''
    def find_root(self, node: Node):
        while(node.parent is not None):
            node = node.parent
        return node

    ''' pick next '''
    def pick_next(self, current_node:Node):
        if current_node.parent is not None:
            return current_node.parent
        return None
        
    ''' find next point in given nodes, start from give start_node '''
    def find_next(self, start_node: Node, nodes ):
        node = start_node
        path = []
        path.append(node)
        while (node.parent in nodes):
            node = node.parent
            path.append(node)
        return node, path

    ''' rewiring for RRT start '''
    def rewire(self, node:Node, neighbour_nodes):
        if node is None:
            return None
        node_cost = self.node_cost(node)
        neighbour_costs = self.node_costs(neighbour_nodes)
        to_neighbour_costs = self.distances(node.coords, neighbour_nodes)
        new_cost = np.array([node_cost]* len(neighbour_nodes)) + to_neighbour_costs

        is_rewire = new_cost < neighbour_costs
        for n_node in neighbour_nodes[is_rewire]:
            self.remove_edge(parent_node=n_node.parent, node=n_node) # remove old parent, ALWAYS COME FIRST
            self.add_edge(parent_node=node, node=n_node)   # connect to new parent
            n_node.update_costs()
    
    def is_NaN(self, num):
        return num != num

    ''' rewiring for RRT start '''
    def rewire_RRTx(self, node:Node, rrt_queue:Priority_queue):
        
        if node is None:
            return None
        node_lmc = self.node_lmc(node)
        node_cost = self.node_cost(node)
        if node_cost - node_lmc > delta_consistency:
            neighbour_nodes = np.array(node.neighbours)
            neighbour_weight = node.neighbours_weight
            neighbour_lmces = self.node_lmcs(neighbour_nodes)
            new_lmces = np.array([node_lmc]* len(neighbour_nodes)) + neighbour_weight
            is_rewire = new_lmces < neighbour_lmces

            for n_node in neighbour_nodes[is_rewire]:
                # do rewire
                if n_node.parent is not None:
                    self.remove_edge(parent_node=n_node.parent, node=n_node) # remove old parent, ALWAYS COME FIRST
                self.add_edge_RRTx(parent_node=node, node=n_node)   # connect to new parent

            for n_node in neighbour_nodes[is_rewire]:
                # add to queue for doing consistent (see algorithm paper)
                if n_node.cost - n_node.lmc > delta_consistency:
                    rrt_queue.verify_queue(n_node)

    ''' reduce inconsisitency (see algorithm paper) '''
    def reduce_inconsistency(self, rrt_queue: Priority_queue):
        while rrt_queue.size() > 0: 
            top_node = rrt_queue.pop()
            if top_node.cost - top_node.lmc > delta_consistency:
                self.update_LMC(node=top_node)
                self.rewire_RRTx(node=top_node, rrt_queue=rrt_queue); 
            top_node.cost = top_node.lmc

    ''' reduce inconsisitency (see algorithm paper) '''
    def reduce_inconsistency_v2(self, rrtx_queue: Priority_queue, currnode:Node):
        while (rrtx_queue.size() > 0 and rrtx_queue.key_less(current_node=currnode) ) or \
                currnode.cost == float("inf") or currnode.lmc != currnode.cost : 
            top_node = rrtx_queue.pop()
            if top_node.cost - top_node.lmc > delta_consistency:
                self.update_LMC(node=top_node)
                self.rewire_RRTx(node=top_node, rrt_queue=rrtx_queue); 
            top_node.cost = top_node.lmc      