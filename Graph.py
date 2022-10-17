from collections import defaultdict

class Graph:
    def __init__(self):
        self.graph = defaultdict(list)

    # Function to add local active point to graph
    def add_local_open_points(self, center, lActive_OpenPts):
        if len(lActive_OpenPts) > 0:
            self.graph_insert(center, lActive_OpenPts)

    # Function to insert edges into graph
    def graph_insert(self, pnode, leafs):
        # print ("__pnode:", pnode)
        # print ("__leafs:", leafs)
        if len(leafs) > 0:
            for leaf in leafs:
                self.graph[tuple(pnode)].append(tuple(leaf))
                self.graph[tuple(leaf)].append(tuple(pnode))

    def get_all_non_leaf(self):
        non_leaves = []
        for pnode in self.graph:
            if len(self.graph[pnode]) > 1:
                non_leaves.append(pnode)
        return non_leaves
    # path between two nodes of a graph
    def BFS_skeleton_path(self, start, goal):

        #print("BFS_skeleton_path: Current {0}, Next {1}".format(start, goal))
        explored = []

        # Queue for traversing the  
        # graph in the BFS 
        queue = [[start]]

        # If the desired node is  
        # reached 
        if start == goal:
            print("Same Node")
            return start

        # Loop to traverse the graph  
        # with the help of the queue 
        while queue:
            path = queue.pop(0)
            node = path[-1]

            # Condition to check if the 
            # current node is not visited 
            if node not in explored:
                neighbours = self.graph[node]
                # print ("_NODE:", node)
                # Loop to iterate over the  
                # neighbours of the node 
                for neighbour in neighbours:
                    # print ("___neighbour:", neighbour)
                    new_path = list(path)
                    new_path.append(neighbour)
                    queue.append(new_path)

                    # Condition to check if the  
                    # neighbour node is the goal 
                    if neighbour == goal:
                        #print("BFS_skeleton_path = ", new_path)
                        return new_path
                explored.append(node)

                # Condition when the nodes
        # are not connected 
        print("So sorry, but a connecting path doesn't exist :(")
        return []