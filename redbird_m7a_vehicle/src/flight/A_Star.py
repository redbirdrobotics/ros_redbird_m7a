
#total cost f = g + h
#g is the cost that it took to get to the current node from the start
#h is the estimated cost that it would take to reach the goal


class Node(object):
    #represents a single point(Node) in an array(map) of points

    nodes = 0

    #initializing properties of object Node
    def __init__(self, location):
        self._id = Node.nodes
        self._parent = -1
        self._location = location
        self._h = 0
        self._g = 0

        Node.nodes += 1

    def get_id(self):
        return self._id

    def set_parent(self, parent):
        self._parent = parent

    def get_parent(self):
        return self._parent

    def get_location(self):
        return self._location

    def set_h_cost(self, cost):
        self._h = cost

    def set_g_cost(self, cost):
        self._g = cost

    def get_h_cost(self):
        return self._h

    def get_g_cost(self):
        return self._g

    
    # calculating for the cost of movement and total cost
    def calculate_g_cost(self, goal):
        return 10

    def calculate_h_cost(self, goal):
        dx = abs(self._location[0] - goal.get_location()[0])
        dy = abs(self._location[1] - goal.get_location()[1])
        return (dx + dy) - 1 * min(dx, dy)

    def get_f_cost(self):
        return self._g + self._h


class Map(object):

    def __init__(self, cols, rows):
        self._cols = cols
        self._rows = rows

        self._nodes = []
        self._reference_map = []

        #Goes through each point on the map and places an object Node and adds values to the ref map
        for x in range(cols):
            for y in range(rows):
                self._nodes.append(Node((x,y)))
                self._reference_map.append((x,y))

    def print_map(self):
        print self._reference_map

    def get_nodes(self):
        return self._nodes

    def get_reference_map(self):
        return self._reference_map

    #for obstacles, node is removed from map
    def add_obstacles(self, locations):
        for location in locations:
            self._reference_map.remove(location)

    def get_node_at_location(self, location):
        for node in self._nodes:
            if node.get_location() == location:
                return node

    def get_node_by_id(self, id):
        for node in self._nodes:
            if node.get_id() == id:
                return node

    def get_adjacent_nodes(self, location):
        directions = [(1,0), (0,1), (-1,0), (0,-1)]
        children = []
        for direction in directions:
            child = (location[0] + direction[0], location[1] + direction[1])
            if child in self._reference_map:
                children.append(self.get_node_at_location(child))
        return children

    
class Utilities:

    def __init__(array):
        self._array = array

 #static because doesn't use the object Utilities itself
    @staticmethod
 # analyzes nearby nodes and returns node with the least cost
    def get_lowest_cost(list, goal):
        target_node = list[0]
        low_cost = target_node.get_f_cost()
        for i in range(len(list)):
            new_node = list[i]
            new_cost = new_node.get_f_cost()

            if new_cost < low_cost:
                low_cost = new_cost
                target_node = new_node
        return target_node

    @staticmethod
    def duplicates(list):
        output = []
        for pt in list:
            if pt not in output:
                output.append(pt)
        return output

    @staticmethod
    def boundary_eliminate(list, max_x, max_y):
        output = []
        for pt in list:

            if 0 <= pt[0] < max_x and 0 <= pt[1] < max_y:
                output.append(pt)

        return output


