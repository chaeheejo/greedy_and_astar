import sys
import time
import pandas as pd

bfsCost=0
astarCost=0

# This class represent a graph
class Graph:
    # Initialize the class
    def __init__(self, graph_dict=None):
        self.graph_dict = graph_dict or {}

    # Add a link from A and B of given distance, and also add the inverse link if the graph is undirected
    def connect(self, A, B, distance=1):
        self.graph_dict.setdefault(A, {})[B] = distance

    # Get neighbors or a neighbor
    def get(self, a, b=None):
        links = self.graph_dict.setdefault(a, {})
        if b is None:
            return links
        else:
            return links.get(b)

    # Return a list of nodes in the graph
    def nodes(self):
        s1 = set([k for k in self.graph_dict.keys()])
        s2 = set([k2 for v in self.graph_dict.values() for k2, v2 in v.items()])
        nodes = s1.union(s2)
        return list(nodes)

# This class represent a node
class Node:
    # Initialize the class
    def __init__(self, name: str, parent: str):
        self.name = name
        self.parent = parent
        self.g = 0  # Distance to start node
        self.h = 0  # Distance to goal node
        self.f = 0  # Total cost

    # Compare nodes
    def __eq__(self, other):
        return self.name == other.name

    # Sort nodes
    def __lt__(self, other):
        return self.f < other.f

    # Print node
    def __repr__(self):
        return ('({0},{1})'.format(self.position, self.f))

# Best-first search
def best_first_search(graph, heuristics, start, end):

    global bfsCost

    # Create lists for open nodes and closed nodes
    open = []
    closed = []
    # Create a start node and an goal node
    start_node = Node(start, None)
    goal_node = Node(end, None)
    # Add the start node
    open.append(start_node)

    # Loop until the open list is empty
    while len(open) > 0:
        # Sort the open list to get the node with the lowest cost first
        open.sort()
        # Get the node with the lowest cost
        current_node = open.pop(0)

        # Add the current node to the closed list
        closed.append(current_node)

        # Check if we have reached the goal, return the path
        if current_node == goal_node:
            bfsCost = current_node.g
            path = []
            while current_node != start_node:
                path.append(current_node.name)
                current_node = current_node.parent
            path.append(start_node.name)
            # Return reversed path
            return path[::-1]

        # Get neighbours
        neighbors = graph.get(current_node.name)
        # Loop neighbors
        for key, value in neighbors.items():
            # Create a neighbor node
            neighbor = Node(key, current_node)
            # Check if the neighbor is in the closed list
            if (neighbor in closed):
                continue
            # Calculate cost to goal
            neighbor.g = current_node.g + graph.get(current_node.name, neighbor.name)
            neighbor.h = heuristics.get(neighbor.name)
            neighbor.f = neighbor.h
            # Check if neighbor is in open list and if it has a lower f value
            if (add_to_open(open, neighbor) == True):
                # Everything is green, add neighbor to open list
                open.append(neighbor)
    # Return None, no path is found
    return "FAILURE: NO PATH FOUND"


# A* search
def astar_search(graph, heuristics, start, end):

    global astarCost

    # Create lists for open nodes and closed nodes
    open = []
    closed = []
    # Create a start node and an goal node
    start_node = Node(start, None)
    goal_node = Node(end, None)
    # Add the start node
    open.append(start_node)

    # Loop until the open list is empty
    while len(open) > 0:
        # Sort the open list to get the node with the lowest cost first
        open.sort()
        # Get the node with the lowest cost
        current_node = open.pop(0)
        # Add the current node to the closed list
        closed.append(current_node)

        # Check if we have reached the goal, return the path
        if current_node == goal_node:
            astarCost = current_node.g
            path = []
            while current_node != start_node:
                path.append(current_node.name)
                current_node = current_node.parent
            path.append(start_node.name)
            # Return reversed path
            return path[::-1]

        # Get neighbours
        neighbors = graph.get(current_node.name)
        # Loop neighbors
        for key, value in neighbors.items():
            # Create a neighbor node
            neighbor = Node(key, current_node)
            # Check if the neighbor is in the closed list
            if (neighbor in closed):
                continue
            # Calculate full path cost
            neighbor.g = current_node.g + graph.get(current_node.name, neighbor.name)
            neighbor.h = heuristics.get(neighbor.name)
            neighbor.f = neighbor.g + neighbor.h
            # Check if neighbor is in open list and if it has a lower f value
            if (add_to_open(open, neighbor) == True):
                # Everything is green, add neighbor to open list
                open.append(neighbor)
    # Return None, no path is found
    return None


# Check if a neighbor should be added to open list
def add_to_open(open, neighbor):
    for node in open:
        if (neighbor == node and neighbor.f >= node.f):
            return False
    return True

# The main entry point for this module
if __name__ == '__main__':
    # Create a graph
    graph = Graph()
    # Create graph connections (Actual distance)

    # read 'driving.csv'
    driving = pd.read_csv('driving.csv')

    state=[]
    for x in range(len(driving)):
        state.append(driving['STATE'][x])

    for x in range(len(driving)):
        for y in range(len(driving)):
            if driving[state[x]][y] != -1 :
                graph.connect(state[x], state[y], driving[state[x]][y])

    # read 'straightline.csv'
    straightline = pd.read_csv('straightline.csv')

    heuristics = {}
    for x in range(len(driving)):
            heuristics[state[x]]=straightline[sys.argv[2]][x]


    if len(sys.argv) != 3:
        print("\nERROR : Not enough or too many arguments.")
        sys.exit()

    print("Cho, Chaehui, A20497810 solution:")
    print("Initial state: {}".format(sys.argv[1]))
    print("Goal state: {}".format(sys.argv[2]))

    bfsTime = time.time()
    for x in range(10):
        # Run best first search algorithm
        print("\nGreedy Best First Search:")
        bfsPath = best_first_search(graph, heuristics, sys.argv[1], sys.argv[2])
        print("Solution path:", end=' ')
        for x in bfsPath:
            print(x, end=' ')
        print("\nNumber of states on a path:",len(bfsPath))
        print("Path cost:", bfsCost)
    print("Execution time: ", (time.time()-bfsTime)/10, "seconds")

    astarTime = time.time()
    for x in range(10):
        # Run A* search algorithm
        print("\nA* Search:")
        astarPath = astar_search(graph, heuristics, sys.argv[1], sys.argv[2])
        print("Solution path:", end=' ')
        for x in astarPath:
            print(x, end=' ')
        print("\nNumber of states on a path:",len(astarPath))
        print("Path cost:", astarCost)
    print("Execution time: ", (time.time()-astarTime)/10, "seconds")

