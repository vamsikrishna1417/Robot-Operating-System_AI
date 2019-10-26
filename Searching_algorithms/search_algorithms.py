#!/usr/bin/env python

import heapq
import problem
import time
import rospy
from std_msgs.msg import String
import argparse

rospy.init_node("search_algorithm")
publisher = rospy.Publisher("/actions",String,queue_size =10)
parser = argparse.ArgumentParser()
parser.add_argument('-a',help = "Please mention algorithm to use. Default is BFS", metavar = 'bfs', action='store', dest='algorithm', default="bfs", type=str)



def bfs():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions()
    action_list = []
    visited_list = []
    bfs_queue = [(init_state,[""])]

    start = time.time()
    while (len(bfs_queue) != 0):
        current_state,current_action = bfs_queue.pop(0)
        if (current_state in visited_list):
            continue
        visited_list.append(current_state)
        for action in possible_actions:
            #to get the next state, cost for an action on state_x use:
            (nextstate, cost) = problem.get_successor(current_state, action)
            if (current_state == goal_state):
                action_list = current_action
                end = time.time()
                print "PATH FOUND"
                print "Time taken in seconds",(end - start)
                return action_list
            if nextstate.x != -1:
                bfs_queue.append((nextstate,current_action+[action]))
    print "PATH NOT FOUND"
    return []


def ucs():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions()
    action_list = []
    visited_list = []
    ucs_queue = []
    # Storing intial configuration in queue
    heapq.heappush(ucs_queue, (0,(init_state),[""]))
    start = time.time()
    while ucs_queue:
        cummulative_cost, current_state, path_from_root = heapq.heappop(ucs_queue)
        if (current_state in visited_list):
            continue
        visited_list.append(current_state)
        if (current_state == goal_state):
            action_list = path_from_root
            end = time.time()
            print "PATH FOUND"
            print "Time taken in seconds",(end - start)
            return action_list
        else:
            for action in possible_actions:
                #to get the next state, cost for an action on state_x use:
                next_possible_state,current_cost = problem.get_successor(current_state,action)
                if next_possible_state.x != -1 and next_possible_state not in visited_list:
                    heapq.heappush(ucs_queue, (cummulative_cost+current_cost,(next_possible_state),path_from_root+[action]))
    print "PATH NOT FOUND"
    return []


def calculate_heuristic(current, later):
    return (abs(current.x - later.x)+abs(current.y - later.y))

def gbfs():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions()
    action_list = []
    visited_list = []
    gbfs_queue = []
    intial_heuristic = calculate_heuristic(init_state, goal_state)
    # Storing intial configuration in queue
    heapq.heappush(gbfs_queue, (intial_heuristic,(init_state),[""]))
    start = time.time()
    while gbfs_queue:
        cummulative_cost, current_state, path_from_root = heapq.heappop(gbfs_queue)
        if (current_state in visited_list):
            continue
        visited_list.append(current_state)
        if (current_state == goal_state):
            action_list = path_from_root
            end = time.time()
            print "PATH FOUND"
            print "Time taken in seconds",(end - start)
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

def astar():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions()
    action_list = []
    visited_list = []
    astar_queue = []
    intial_heuristic = calculate_heuristic(init_state, goal_state)
    intial_cost = 0
    # Storing intial configuration in queue
    heapq.heappush(astar_queue, (intial_heuristic+intial_cost,(init_state),[""]))
    start = time.time()
    while astar_queue:
        cummulative_cost, current_state, path_from_root = heapq.heappop(astar_queue)
        if (current_state in visited_list):
            continue
        visited_list.append(current_state)
        if (current_state == goal_state):
            action_list = path_from_root
            end = time.time()
            print "PATH FOUND"
            print "Time taken in seconds",(end - start)
            return action_list
        else:
            for action in possible_actions:
                #to get the next state, cost for an action on state_x use:
                next_possible_state,current_cost = problem.get_successor(current_state,action)
                if next_possible_state.x != -1 and next_possible_state not in visited_list:
                    next_heuristic = calculate_heuristic(next_possible_state, goal_state)
                    heapq.heappush(astar_queue, (next_heuristic+cummulative_cost+current_cost,(next_possible_state),path_from_root+[action]))
    print "PATH NOT FOUND"
    return []

'''
Below two approaches are gbfs and astar search implementations with new heuristic function
heuristic *= (1 + p), where p is a random lesser value choosen as p < (minimum cost of taking one step)/(expected maximum path length)
'''
def gbfs_hs():
    #Tie_breaking_approach
    #In this we nudge the value of h so that f will increase as we move forward to goal
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions()
    action_list = []
    visited_list = []
    gbfs_queue = []
    intial_heuristic = calculate_heuristic(init_state, goal_state)
    intial_heuristic *= (1 + 0.03125)
    # Storing intial configuration in queue
    heapq.heappush(gbfs_queue, (intial_heuristic,(init_state),[""]))
    start = time.time()
    while gbfs_queue:
        cummulative_cost, current_state, path_from_root = heapq.heappop(gbfs_queue)
        if (current_state in visited_list):
            continue
        visited_list.append(current_state)
        if (current_state == goal_state):
            action_list = path_from_root
            end = time.time()
            print "PATH FOUND"
            print "Time taken in seconds",(end - start)
            return action_list
        else:
            for action in possible_actions:
                #to get the next state, cost for an action on state_x use:
                next_possible_state,current_cost = problem.get_successor(current_state,action)
                if next_possible_state.x != -1 and next_possible_state not in visited_list:
                    next_heuristic = calculate_heuristic(next_possible_state, goal_state)
                    next_heuristic *= (1 + 0.03125)
                    heapq.heappush(gbfs_queue, (next_heuristic,(next_possible_state),path_from_root+[action]))
    print "PATH NOT FOUND"
    return []

def astar_hs():
    #Tie_breaking_approach
    #In this we nudge the value of h so that f will increase as we move forward to goal
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions()
    action_list = []
    visited_list = []
    astar_queue = []
    intial_heuristic = calculate_heuristic(init_state, goal_state)
    intial_heuristic *= (1 + 0.03125)
    intial_cost = 0
    # Storing intial configuration in queue
    heapq.heappush(astar_queue, (intial_heuristic+intial_cost,(init_state),[""]))
    start = time.time()
    while astar_queue:
        cummulative_cost, current_state, path_from_root = heapq.heappop(astar_queue)
        if (current_state in visited_list):
            continue
        visited_list.append(current_state)
        if (current_state == goal_state):
            action_list = path_from_root
            end = time.time()
            print "PATH FOUND"
            print "Time taken in seconds",(end - start)
            return action_list
        else:
            for action in possible_actions:
                #to get the next state, cost for an action on state_x use:
                next_possible_state,current_cost = problem.get_successor(current_state,action)
                if next_possible_state.x != -1 and next_possible_state not in visited_list:
                    next_heuristic = calculate_heuristic(next_possible_state, goal_state)
                    next_heuristic *= (1 + 0.03125)
                    heapq.heappush(astar_queue, (next_heuristic+cummulative_cost+current_cost,(next_possible_state),path_from_root+[action]))
    print "PATH NOT FOUND"
    return []


# to execute a plan action_list = <list of actions>, use:
def exec_action_list(action_list):
    plan_str = '_'.join(action for action in action_list)
    publisher.publish(String(data = plan_str))

if __name__ == "__main__":
    args = parser.parse_args()
    algorithm = globals().get(args.algorithm)
    if algorithm is None:
        print "Incorrect Algorithm name."
        exit(1)
    actions = algorithm()
    exec_action_list(actions)



