from collections import deque
from functools import partial
from problem import *
from server import *
import problem
import subprocess
import time
import heapq
import rospy
import random	


MOVE_ACTIONS = ["MoveF", "MoveB"]
TURN_ACTIONS = ["TurnCW","TurnCCW"]
ACTIONS = ["TurnCW","TurnCCW","MoveF","MoveB"]

BLOCK = "Block"
ALLOWED = "Allowed"
mapper = {}


class Queue:
    def __init__(self):
        self.vals = []

    def empty(self):
        return len(self.vals) == 0

    def put(self, ele, key):
        heapq.heappush(self.vals, (key, ele))

    def pop(self):
        ele = heapq.heappop(self.vals)
        return ele[1]

    def first_key(self):
        return heapq.nsmallest(1, self.vals)[0][0]

    def delete(self, given_node):
        self.vals = [ele for ele in self.vals if ele[1] != given_node]
        heapq.heapify(self.vals)

    def __iter__(self):
        for key, node in self.vals:
            yield node


class Map:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.blocked_nodes = set()

    def cost(self, from_node, to_node):
        if from_node in self.blocked_nodes or to_node in self.blocked_nodes:
            return float('inf')
        else:
            return 1

class BotMap(Map):

    def new_blocked_nodes(self, current_view):
        bnodes_viewed = {node for node, nodetype in current_view.items()
                        if nodetype == BLOCK}
        return bnodes_viewed - self.blocked_nodes

    def update_blocked_nodes(self, new_blocked_nodes):
        self.blocked_nodes.update(new_blocked_nodes)


		
class Search_D_star_lite(object):
    
    def __init__(self, graph, start, goal):
        
	self.bot_map = BotMap(graph.rows, graph.cols)
        self.original_map = graph
        
	self.node_adjs = {}
	self.nearest_neigh_map = {}
        self.g_map = {}
        self.rhs_map = {}
	self.parent_map = {}

        self.km = 0
	
	# current state of the hunter
        self.start_node = start
	# current state of the target
        self.goal_node = goal

	self.queue = Queue()
        self.queue.put(self.start_node, self.get_key(self.start_node))
        self.nearest_neigh_map[self.start_node] = None

    def compute_rhs_val(self, node):
        lowest_cost_neighbour = self.lowest_cost_neighbour(node)
        self.nearest_neigh_map[node] = lowest_cost_neighbour
        return self.lookahead_cost(node, lowest_cost_neighbour)

    def lookahead_cost(self, node, neighbour):
        return self.get_gval(neighbour) + self.bot_map.cost(neighbour, node)

    def lowest_cost_neighbour(self, node):
        cost = partial(self.lookahead_cost, node)
        return min(self.get_neighbors(node), key=cost)

    def get_gval(self, node):
        return self.g_map.get(node, float('inf'))

    def get_rhsval(self, node):
	if node == self.start_node:
		return 0
        return self.rhs_map.get(node, float('inf'))

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_key(self, node):
        return (min([self.get_gval(node), self.get_rhsval(node)]) + self.heuristic(node, self.goal_node) + self.km,
            min([self.get_gval(node), self.get_rhsval(node)]))


    def update_neighbour_node(self, node):
        #if node != self.goal_node:
        #    self.rhs_map[node] = self.compute_rhs_val(node)

	#print("Updating node: " + str(node))
	#print("G-value: " + str(self.g(node)))
	#print("RHS-value: " + str(self.rhs(node))
	
        self.queue.delete(node)
        if self.get_gval(node) != self.get_rhsval(node):
            self.queue.put(node, self.get_key(node))
	
	#print("Nodes in priority queue: \n")
	#for key, node in self.queue:
	#	print(key)
	#	print(node)
		 

    def get_best_path(self):
        last_nodes = deque(maxlen=10)
	
	while not self.queue.empty() and self.queue.first_key() < self.get_key(self.goal_node) or self.get_rhsval(self.goal_node) != self.get_gval(self.goal_node):
	    
	    k_old = self.queue.first_key()
	    node = self.queue.pop()
	    
	    print("Node removed from queue: " + str(node))
            print("Key of the node :" + str(k_old))
            
	    last_nodes.append(node)
            if len(last_nodes) == 10 and len(set(last_nodes)) < 3:
                raise Exception("Fail! Stuck in a loop")

            k_new = self.get_key(node)
            if k_old < k_new:
                self.queue.put(node, k_new)

            elif self.get_gval(node) > self.get_rhsval(node):
		#print("Updating the g value of the node :" + str(node))
                self.g_map[node] = self.get_rhsval(node)
		neighs = self.get_neighbors(node)
		for neigh in neighs:
			if self.start_node != neigh and self.get_rhsval(neigh) > self.get_gval(node) + 1:
				self.parent_map[neigh] = node
				self.rhs_map[neigh] = self.get_gval(node) + 1
				self.update_neighbour_node(neigh)

            else:
                self.g_map[node] = float('inf')
		neighs = self.get_neighbors(node) + [node]
		for neigh in neighs:
			if neigh != self.start_node and self.parent_map[neigh] == node:
				self.rhs_map[neigh] = self.compute_rhs_val(neigh)
				if self.rhs_map[neigh] == float('inf'):
					self.parent_map[neigh] = None
				else:
					self.parent_map[neigh] = self.lowest_cost_neighbour(node)		
			self.update_neighbour_node(neigh)

        return self.nearest_neigh_map.copy(), self.g_map.copy()



    # getting the neighbors
    def get_neighbors(self, node):
	tnode = mapper[node]
	if node not in self.node_adjs:
		neighs = []
		for i in range(4):
			tnode, tcost, tvalid = problem.get_successor(tnode, TURN_ACTIONS[0])
			next_state, gcost, isvalid = problem.get_successor(tnode, MOVE_ACTIONS[0])
			if isvalid == 1:
				if (next_state.x, next_state.y) not in mapper:
					mapper[(next_state.x, next_state.y)] = next_state
				neighs.append((next_state.x, next_state.y))
		self.node_adjs[node] = neighs
	return self.node_adjs[node]
	
	
    # robot observing the neighbors that are adjacent to the current position
    def sense_adjacent_nodes(self, position):
	adj_nodes = {}
	tnode = mapper[position]
	for num in range(4):
		tnode, tcost, tvalid = problem.get_successor(tnode, TURN_ACTIONS[0])
		next_state, gcost, isvalid = problem.get_successor(tnode, MOVE_ACTIONS[0])
		
		if (next_state.x, next_state.y) not in mapper:
			mapper[(next_state.x, next_state.y)] = next_state
		if isvalid == 0:
			adj_nodes[(next_state.x, next_state.y)] = BLOCK
		elif isvalid == 1:
			adj_nodes[(next_state.x, next_state.y)] = ALLOWED
	
	
	return adj_nodes  

    # Basic Deletion as per the algorithm in the paper
    def basic_deletion(self, old_start):
	self.parent_map[self.start_node] = None
	self.rhs_map[old_start] = self.compute_rhs_val(old_start)

	if self.rhs_map[old_start] == float('inf'):
		self.parent_map[old_start] = float('inf')
	else:
		self.parent_map[old_start] = self.lowest_cost_neighbour(old_start)
	self.update_neighbour_node(old_start)



    def sense_surroundings_and_move_to_destination(self):
	result_path = [self.goal_node]
	while self.start_node != self.goal_node:
		old_start = self.start_node
		old_goal = self.goal_node
		
		newadj_nodes = self.sense_adjacent_nodes(self.start_node)
		for node, nodetype in newadj_nodes.items():
			print("Node: "+ str(node) + "\n nodeType: " + nodetype)

        	new_bnodes = self.bot_map.new_blocked_nodes(newadj_nodes)
        	self.bot_map.update_blocked_nodes(new_bnodes)
        	self.get_best_path()
		
		if self.get_gval(self.goal_node) == float('inf'):
                	raise Exception("No path")

		temp_node = self.goal_node
		path_obtained = [temp_node]
		while self.parent_map[temp_node] != self.start_node:
			path_obtained.insert(0, self.parent_map[temp_node])
			temp_node = self.parent_map[temp_node]
		
		
		print("\n Path obtained for start to goal node: " + str(path_obtained))
		
		hunter_node = self.start_node
		next_index = 0
		
		# Hunter following the path obtained to catch the target
		while next_index < len(path_obtained):
			print("Hunter moving from node: " + str(hunter_node) + " to node: " + str(path_obtained[next_index]))
			hunter_node = path_obtained[next_index]
			result_path.append(hunter_node)
			next_index = next_index + 1
			
			change_in_target = raw_input("Did the target move from the position? Yes or No : ")
			if change_in_target == "Yes":
				moving_nodes = [(self.goal_node[0] - 0.5, self.goal_node[1]), (self.goal_node[0], self.goal_node[1] - 0.5)]
				self.goal_node = moving_nodes[random.randint(0, 1)]
				break
		print("\n\nGoal state moved from node: " + str(old_goal) + " to node: " + str(self.goal_node))
		
		if hunter_node == self.goal_node:
			return result_path
		
		# Updating the start node with current hunter node and goal node with updated goal node
		self.start_node = hunter_node
		self.km = self.km + self.heuristic(self.goal_node, old_goal)
		
		if old_start != self.start_node:
			self.basic_deletion(old_start)					
        
        return result_path


# generating grid from ROS environment by running server.py
def grid_from_environment():
    g = Map(5, 5)
    start = problem.get_initial_state()
    end = problem.get_goal_state()
    walls_list = bfs(start)
    tuple_list = []
    for w in walls_list:
	mapper[(w.x, w.y)] = w
	tuple_list.append((w.x, w.y))
    g.walls = tuple_list
    return g, start, end


def bfs(start):
    walls = set()
    queue = [start]
    visited_nodes = set()
    while queue:
        node = queue.pop(0)
	visited_nodes.add(node)
        for action in ACTIONS:
            next_node, next_cost, isvalid = problem.get_successor(node, action)
            if isvalid == 0 and next_node not in walls:
                visited_nodes.add(next_node)
		walls.add(next_node)
	    elif next_node not in visited_nodes:
                queue.append(next_node)
    return walls


# get refined actions to publish
def get_refined_actions_to_publish(path, start_node):
    curr_node = start_node
    refined_actions = []
    
    for index in range(1, len(path)):
	desired_state = path[index]
	print("\nDesired State: " + str(desired_state))
	for i in range(4):
	    # trying to move forward and validating the next node
	    next_node, gcost, isvalid = problem.get_successor(curr_node, MOVE_ACTIONS[0])
	    
	    if isvalid == 1 and next_node.x == desired_state[0] and next_node.y == desired_state[1]:
		refined_actions.append(MOVE_ACTIONS[0])
		curr_node = next_node
		print("action obtained at node: " + str(curr_node) + "\taction: " + str(MOVE_ACTIONS[0]))
	        break
	    else:
		curr_node, next_cost, isvalid = problem.get_successor(curr_node, TURN_ACTIONS[0])
		refined_actions.append(TURN_ACTIONS[0])
    return refined_actions


if __name__ == "__main__":

    ros_core = subprocess.Popen("roscore")
    ros_server = subprocess.Popen("rosrun search server.py -d 5 -n 2 -s 10", shell=True)
    #publisher = rospy.Publisher("/results", String, queue_size=10)
    #rospy.init_node('D-star-lite')

    time.sleep(5)
    grid, start, end = grid_from_environment()

    s = (start.x, start.y)
    e = (end.x, end.y)
    mapper[s] = start
    mapper[e] = end
    dstar = Search_D_star_lite(grid, s, e)
	
    #print("\n Initialized D star lite oject")
    #print("Move to goal has been called")
    path = dstar.sense_surroundings_and_move_to_destination()

    print("\n\nPath obtained using Basic Moving Target D-star lite algorithm: ")
    print(path)

    print("Start Node: " + str(start))
    refined_actions = get_refined_actions_to_publish(path, start)

    print("\nRefined actions to be published to the ros")
    print(refined_actions)

    ros_server.kill()
    ros_core.kill()
    subprocess.call(["pkill", "-f", "rosrun"])
    subprocess.call(["pkill", "roscore"])
    
