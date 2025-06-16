class Node:
    def __init__(self,pos,parent_id = -1):
       self.pos = pos
       self.parent_id = parent_id
       self.children_nodes = []
    
class Tree:
    def __init__(self):
        self.nodes = []
    
    def add_node(self,node):
        self.nodes.append(node)

    def get_path_to_root(self,node):
        path = []
        current_node = node
        while current_node.parent_id != -1:
            path.append(current_node.pos)
            current_node = self.nodes[current_node.parent_id]
        
        return path[::-1]

