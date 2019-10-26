#!/usr/bin/env python

import heapq
import problem 
import rospy
from std_msgs.msg import String
import argparse
import time
import os
import json

rospy.init_node("arrange_books")
publisher = rospy.Publisher("/actions",String,queue_size =10)
parser = argparse.ArgumentParser()
parser.add_argument('-d',help = "Please give the path to domain file ", action='store', dest="domain_ppdl", type=str)
parser.add_argument('-p',help = "Please give the path to problem file ", action='store', dest="problem_ppdl", type=str)

def calculate_heuristic(current, later):
    return (abs(current.x - later.x)+abs(current.y - later.y))

def gbfs(initial_state, goal_state):
    init_state = initial_state
    goal_state = goal_state
    possible_actions = problem.get_actions()
    action_list = []
    visited_list = []
    gbfs_queue = []
    intial_heuristic = calculate_heuristic(init_state, goal_state)
    # Storing intial configuration in queue
    heapq.heappush(gbfs_queue, (intial_heuristic,(init_state),[""]))
    while gbfs_queue:
        cummulative_cost, current_state, path_from_root = heapq.heappop(gbfs_queue)
        if (current_state in visited_list):
            continue
        visited_list.append(current_state)
        if (current_state == goal_state):
            action_list = path_from_root
            print "PATH FOUND"
            return action_list
        else:
            for action in possible_actions:
                #to get the next state, cost for an action on state_x use:
                next_possible_state,current_cost = problem.get_successor(current_state,action)
                if next_possible_state.x != -1 and next_possible_state not in visited_list:
                    next_heuristic = calculate_heuristic(next_possible_state, goal_state)
                    heapq.heappush(gbfs_queue, (next_heuristic,(next_possible_state),path_from_root+[action]))
    print "PATH NOT FOUND"
    return []


def json_parse_and_read_plan(domain_file, problem_file):
	start_time = time.time()
	os.system(("./scripts/FD/fast-downward.py {} {} --search \"lazy_greedy([ff()], preferred=[ff()])\"").format(domain_file, problem_file))
	with open('books.json') as book_parse:
		data = json.load(book_parse)

	next_state = problem.State(0, 0, "EAST")

	sas_file = open("sas_plan", "r")
	for line in sas_file:
		word = line.strip('(').split()
	
		if word[0] == "move":
			from_location = word[2]
			print from_location
			to_location = word[1][0:len(word[1])-5]
			print to_location
			if from_location == "tbot3_init_loc":
				init_state = problem.get_initial_state()
				if to_location[0:1] == 'b':
					curr_loc = data["books"][to_location]["load_loc"][0]
					curr_state = problem.State(curr_loc[0], curr_loc[1], "EAST")
				elif to_location[0:1] == 't':
					curr_loc = data["bins"][to_location]["load_loc"][0]
					curr_state = problem.State(curr_loc[0], curr_loc[1], "EAST")
				action_list = gbfs(init_state, curr_state)
				action_list.pop(0)
				if action_list:
					next_state = curr_state
					problem.execute_move_action(action_list)
					print(action_list)
			else:
				init_state = next_state
				if to_location[0:1] == 'b':
					curr_loc = data["books"][to_location]["load_loc"][0]
					curr_state = problem.State(curr_loc[0], curr_loc[1], "EAST")
				elif to_location[0:1] == 't':
					curr_loc = data["bins"][to_location]["load_loc"][0]
					curr_state = problem.State(curr_loc[0], curr_loc[1], "EAST")
				action_list = gbfs(init_state, curr_state)
				action_list.pop(0)
				if action_list:
					next_state = curr_state
					problem.execute_move_action(action_list)
					print(action_list)
		elif word[0] == "place":
			book_name = word[1]
			bin_name = word[6][0:len(word[6])-1]
			problem.execute_place_action(book_name, bin_name, next_state)
		elif word[0] == "pick":
			book_name = word[1]
			problem.execute_pick_action(book_name, next_state)
	end_time = time.time()
	#print "%s" %(end_time - start_time)


	
if __name__ == "__main__":
    args = parser.parse_args()
    domain_file = args.domain_ppdl
    problem_file = args.problem_ppdl
    json_parse_and_read_plan(domain_file, problem_file)
