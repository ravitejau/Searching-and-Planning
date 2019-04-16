#!/usr/bin/env python

import heapq
import problem 
import rospy
from std_msgs.msg import String
import argparse
import math

publisher = rospy.Publisher("/actions",String,queue_size =10)
parser = argparse.ArgumentParser()
parser.add_argument('-a',help = "Please mention algorithm to use. Default is BFS", metavar = 'bfs', action='store', dest='algorithm', default="bfs", type=str)

class CustomState(problem.State):

    def __init__(self, x, y,orientation):
	problem.State.__init__(self, x, y, orientation)
    
    def __hash__(self):
	return hash((self.x, self.y, self.orientation))


def bfs():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions() 
    action_list = []

    queue_nodes = [(init_state, 0, [])]
    visited_nodes = set()
    while queue_nodes:
        node, cost, path = queue_nodes.pop(0)
	visited_nodes.add(node)
        if problem.is_goal_state(node):
	    action_list = path
	    break
        for action in possible_actions:
            possible_node, possible_cost = problem.get_successor(node, action)
	    if possible_node not in visited_nodes:
                queue_nodes.append((possible_node, possible_cost + cost, path + [action]))
    
    return action_list


def ucs():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions() 
    action_list = []
    
    initial_tuple = (0, init_state, [])
    priority_queue = [initial_tuple]
    visited = dict()
    visited[init_state] = (initial_tuple, False)

    while priority_queue:
	# Pop the element in the queue/min heap
        tuple_temp = heapq.heappop(priority_queue)
	if problem.is_goal_state(tuple_temp[1]):
	    action_list = tuple_temp[2]
	    break

	if visited[tuple_temp[1]][1] is True:
	    continue
	
	visited[tuple_temp[1]] = (tuple_temp, True)
	
	for action in possible_actions:
	    
	    possible_node, possible_cost = problem.get_successor(tuple_temp[1], action)
	    action_tuple = (tuple_temp[0] + possible_cost, possible_node, tuple_temp[2] + [action])
            
	    if possible_node not in visited:
	        heapq.heappush(priority_queue, action_tuple)
		visited[possible_node] = (action_tuple, False)

	    elif visited[possible_node][1] is False and visited[possible_node][0][0] > tuple_temp[0] + possible_cost:
		heapq.heappush(priority_queue, action_tuple)
		visited[possible_node] = (action_tuple, False)
	
    return action_list

def get_md_heuristic(initial_state, goal_state):
    #return abs(initial_state.x - goal_state.x) + abs(initial_state.y - goal_state.y)
    return math.sqrt((initial_state.x - goal_state.x) ** 2 + (initial_state.y - goal_state.y) ** 2) 

def gbfs():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions() 
    action_list = []

    #to get the next state, cost for an action on state_x use:
    #(nextstate, cost) = problem.get_successor(state, action)

    '''
    YOUR CODE HERE
    '''
    priority_queue = [(get_md_heuristic(init_state, goal_state), init_state, [])]
    visited_nodes = set()
    while priority_queue:
        cost, node, path = heapq.heappop(priority_queue)
	visited_nodes.add(node)
        if problem.is_goal_state(node):
	    action_list = path
	    break
        for action in possible_actions:
            possible_node, possible_cost = problem.get_successor(node, action)
	    if possible_node not in visited_nodes:
                heapq.heappush(priority_queue, (get_md_heuristic(possible_node, goal_state), possible_node, path + [action]))

    return action_list

def astar():
    i_state = problem.get_initial_state()
    init_state = CustomState(i_state.x, i_state.y, i_state.orientation)
    g_state = problem.get_goal_state()
    goal_state = CustomState(g_state.x, g_state.y, g_state.orientation)

    possible_actions = problem.get_actions() 
    action_list = []

    # first element in tuplerepresents h() + g() function
    # second element in tuple represents weight (g()) of the edge
    # third element in tuple represents state
    # fourth element in tuple is for action path to reach the state

    initial_tuple = (get_md_heuristic(init_state, goal_state), 0, init_state, [])
    priority_queue = [initial_tuple]
    visited = dict()
    visited[init_state] = (initial_tuple, False)

    while priority_queue:
	# Pop the element in the queue/min heap
        tuple_temp = heapq.heappop(priority_queue)
	if problem.is_goal_state(tuple_temp[2]):
	    action_list = tuple_temp[3]
	    break

	if visited[tuple_temp[2]][1] is True:
	    continue
	
	visited[tuple_temp[2]] = (tuple_temp, True)
	
	for action in possible_actions:
	    
	    p_node, possible_cost = problem.get_successor(tuple_temp[2], action)
	    children_node_cost = possible_cost + tuple_temp[1]
	    possible_node = CustomState(p_node.x, p_node.y, p_node.orientation)

	    total_cost = children_node_cost + get_md_heuristic(possible_node, goal_state) 
	    action_tuple = (total_cost, children_node_cost, possible_node, tuple_temp[3] + [action])
            
	    if possible_node not in visited:
	        heapq.heappush(priority_queue, action_tuple)
		visited[possible_node] = (action_tuple, False)

	    elif visited[possible_node][1] is False and visited[possible_node][0][0] > total_cost:
		heapq.heappush(priority_queue, action_tuple)
		visited[possible_node] = (action_tuple, False)
	
    return action_list


   

 # to execute a plan action_list = <list of actions>, use:
def exec_action_list(action_list):
    plan_str = '_'.join(action for action in action_list)
    publisher.publish(String(data = plan_str))

if __name__ == "__main__":
    rospy.init_node("custom_search_algorithm")
    args = parser.parse_args()
    algorithm = globals().get(args.algorithm)
    if algorithm is None:
        print "Incorrect Algorithm name."
        exit(1)
    actions = algorithm()
    print(actions)
    exec_action_list(actions)



