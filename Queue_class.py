class Priority_queue:
    def __init__(self):
        self.dict = {}
    
    ''' get all values in the queue'''
    def get_all_values(self):
        return list (self.dict.values())

    ''' get all keys in the queue'''
    def get_all_keys(self):
        return list (self.dict.keys())

    ''' return size of the queue'''
    def size(self):
        return len(self.dict)

    ''' get key which has the highest priority in the queue 
        The smaller value, the higher priority.
    '''
    def get_highest_key(self):
        return min(list(self.dict.keys()))
    
    ''' pop the most priority key (smallest in value) form the queue'''
    def pop(self):
        top_key = self.get_highest_key()
        node = self.dict.pop(top_key)
        return node

    ''' remove existed node (if any) out of the queue '''
    def remove_node_from_queue(self, node):
        queue_key = self.get_key_in_queue(node=node)
        if queue_key is not None:
            self.dict.pop(queue_key)

    ''' add node to the queue '''
    def add_node_to_queue(self, node):
        node_key = self.get_node_key(node)
        self.dict[node_key] = node   # if existed update node; else insert node

    ''' verify opphan node in the queue '''
    def verify_orphan(self, node):
        self.remove_node_from_queue(node)

    ''' verify the queue '''
    def verify_queue(self, node):
        # if node exists in queue before
        # remove existed node then add new one
        self.remove_node_from_queue(node=node)
        self.add_node_to_queue(node=node)

    ''' check if the node existed in the queue '''    
    def get_key_in_queue(self, node):
        for queue_key, queue_node in self.dict.items():
            if node == queue_node:
                return queue_key
        return None

    ''' get a key of a given node '''
    def get_node_key(self, node):
        return (min(node.cost, node.lmc), node.cost)

    ''' check if the key of top_item is queue is less than the key of given current_node_item '''
    def key_less(self, current_node):
        top_key = self.get_highest_key()
        current_node_key = self.get_node_key(current_node)
        return (top_key[0] < current_node_key[0]) or \
            (top_key[0] == current_node_key[0] and top_key[1]<current_node_key[1])

    ''' print out'''
    def print(self):
        pkey, pnode = [], []
        for queue_key, queue_node in self.dict.items():
            pkey.append(queue_key)
            pnode.append(queue_node)
        print ("nodes in queue:", pnode)
        print ("keys in queue:", pkey)
