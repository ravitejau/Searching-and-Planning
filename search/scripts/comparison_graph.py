#!/usr/bin/env python

import rospy
import search_algorithm
import time

avg_time = [0 for i in range(4)]

#for i in range(0, 5):
#    start = time.time()
#    action_list = search_algorithm.bfs()
#    avg_time[0] += (time.time() - start)
    
#print("Time taken for bfs:"+ str(avg_time[0]))    

for i in range(0, 5):
    start = time.time()
    action_list = search_algorithm.ucs()
    avg_time[1] += (time.time() - start)
    
print("Time taken for ucs:"+ str(avg_time[1]))

for i in range(0, 5):
    start = time.time()
    action_list = search_algorithm.gbfs()
    avg_time[2] += (time.time() - start)
    
print("Time taken for gbfs:"+ str(avg_time[2]))

for i in range(0, 5):
    start = time.time()
    action_list = search_algorithm.astar()
    avg_time[3] += (time.time() - start)
    
print("Time taken for astar:"+ str(avg_time[3]))

avg_time = [i/5 for i in avg_time]
print(avg_time)
    

