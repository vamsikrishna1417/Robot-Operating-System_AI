#!/usr/bin/env python

from reinforcement.srv import *
import rospy
from mazeGenerator import *
import sys
import argparse
import time
from action_server import RobotActionsServer 
import pickle
import copy
from std_msgs.msg import String
import problem
import numpy as np
import copy
import subprocess

parser = argparse.ArgumentParser(description='AI-reinforcement learning')
parser.add_argument('-t1', action="store_true", default=False, dest='enable_t1', help='Enable to execute task1')
parser.add_argument('-t2', action="store_true", default=False, dest='enable_t2', help='Enable to execute task2')
parser.add_argument('-t3', action="store_true", default=False, dest='enable_t3', help='Enable to execute task3')
parser.add_argument('episodes', metavar='N', type=int, nargs='?', help='An integer specifying number of episodes')

rospy.init_node("reinforcement_server")
pub = rospy.Publisher('/results',String,queue_size=10)

def parse_trajectories_file():
    alpha = 0.3
    gamma = 0.9
    file = open("trajectories.txt")

    for line in file:
    	if (line[0] == '-'):
    		processid = subprocess.call('gnome-terminal -x bash -c "rosrun reinforcement server.py {}; bash"'.format(line), shell=True)
    		continue
    	elif (line[0] == '('):
    		init_data = line[0:len(line)]
    		data = init_data.split(',,')
    	else:
    		continue
    	action_list = problem.get_all_actions()
    	number_of_actions = len(action_list)
    	number_of_states = (len(data)/3)
    	q_table = np.zeros([number_of_states, number_of_actions])
    	count = 0
    	state_list = []
    	str_out = '"'
    	for i in range(0, len(data), 3):
    		if data[i] in state_list:
    			#If action is already present do not add it into state_list and update the present state row in the q_table
    			state = state_list.index(data[i])
    			count -= 1
    		else:
    			state_list.append(data[i])
    			state = count
    		data[i+1] = data[i+1].replace(',', '')
    		action_index = action_list.index(data[i+1])
    		if(count+1 < number_of_states):
    			next_state = count + 1
    			next_max = np.amax(q_table[next_state])
    		else:
    			next_max = 0
    		old_value = q_table[state, action_index]
    		new_value = (float(1 - alpha) * old_value) + (alpha * (float(data[i+2]) + (gamma * next_max)))
    		q_table[state, action_index] = new_value
    		#str_out = 'state_' + str(state)  + ':{' + 'action_' + str(action_index) + ':' + str(new_value)
    		#print str_out
    		count += 1

    	#Printing the string in the format as requested in the homework
    	for i in range(0, len(data), 3):
    		if data[i] in state_list:
    			str_out += data[i] + ':{'
    			index = state_list.index(data[i])
    			for j in range(0, number_of_actions):
    				if not (q_table[index][j] == 0):
    					str_out += action_list[j] + ':' + str(q_table[index][j]) + ',' 
    					str_out = str_out[:-1]
    					str_out += '}-'
    	str_out = str_out[:-1]
    	str_out += '"'
    	pub.publish(str_out)
    	print str_out

    		
    	#print(q_table)
    	#print '\n'

def q_learning_task2(num_episodes):
	action_list = problem.get_all_actions()
	epsilon = 0.99
	alpha = 0.3
	gamma = 0.9
	action_list = problem.get_all_actions()
	number_of_actions = len(action_list)
	q_table = [[0 for j in range(number_of_actions)] for i in range(1)]
	zero_list = [0 for i in range(number_of_actions)]
	outerloopcnt = 0

	for i in range(num_episodes):
		current_location = problem.get_current_state()
		cumulative_reward = 0
		state_list = []
		state_list.append(current_location)
		count = 0
		if(outerloopcnt%10 == 0):
			epsilon -= 0.1
		#Using Epsilon greedy approach
		# Run the episode till it reaches the terminal state
		while (problem.is_terminal_state() == 0):
			str_out = ""
			state_index = state_list.index(current_location)
			rand =  random.random()
			if (rand < epsilon):
				#selecting the action randomly
				action = random.choice(action_list)
				action_index = action_list.index(action)
			else:	
				#Taking the maximum valued action from the q_table for that state
				max_value = np.amax(q_table[state_index])
				action_index = q_table[state_index].index(max_value)
				action = action_list[action_index]


			#Executing the action corresponding to the action selected
			if ('normal_moveF' in action):
				success, next_state, reward = problem.execute_normal_moveF()
			elif ('normal_TurnCW' in action):
				success, next_state, reward = problem.execute_normal_TurnCW()
			elif ('normal_TurnCCW' in action):
				success, next_state, reward = problem.execute_normal_TurnCCW()
			elif ('careful_moveF' in action):
				success, next_state, reward = problem.execute_careful_moveF()
			elif ('careful_TurnCW' in action):
				success, next_state, reward = problem.execute_careful_TurnCW()
			elif ('careful_TurnCCW' in action):
				success, next_state, reward = problem.execute_careful_TurnCCW()
			elif ('normal_place' in action): 
				action = action.split(' ')
				book_name = action[1]
				bin_name = action[2][:-1]
				success, next_state, reward = problem.execute_normal_place(book_name, bin_name)
			elif ('careful_place' in action):
				action = action.split(' ')
				book_name = action[1]
				bin_name = action[2][:-1]
				success, next_state, reward = problem.execute_careful_place(book_name, bin_name)
			elif ('normal_pick' in action):
				action = action.split(' ')
				book_name = action[1][:-1]
				success, next_state, reward = problem.execute_normal_pick(book_name)
			elif ('careful_pick' in action):
				action = action.split(' ')
				book_name = action[1][:-1]
				success, next_state, reward = problem.execute_careful_pick(book_name)
			else:
				print "specified action is not in action list"

			if next_state in state_list:
				next_state_index = state_list.index(next_state)
			else:
				state_list.append(next_state)
				next_state_index = state_list.index(next_state)
				q_table.append(zero_list)

			old_value = q_table[state_index][action_index]
			next_max = np.amax(q_table[next_state_index])
			new_value = (float(1 - alpha) * old_value) + (alpha * ((reward) + (gamma * next_max)))
			#update the q_table
			q_table[state_index][action_index] = new_value
			str_out += '"(' + state_list[state_index] + ',' + action_list[action_index] + ',' + state_list[next_state_index] + ',' + str(reward) + ')'
			str_out += ':' + str(q_table[state_index][action_index]) + '"'
			pub.publish(str_out)


			#calculating the reward
			if (count == 0):
				cumulative_reward += reward
			else:
				cumulative_reward += (gamma ** count) * reward
			
			count += 1
			current_location = next_state

			#print current_location
		print (outerloopcnt, count, cumulative_reward)
		outerloopcnt += 1
		problem.reset_world()


def q_learning_task3(num_episodes):
	action_list = problem.get_all_actions()
	epsilon = 0.99
	alpha = 0.3
	gamma = 0.9
	action_list = problem.get_all_actions()
	number_of_actions = len(action_list)
	#print action_list
	q_table = [[0 for j in range(number_of_actions)] for i in range(1)]
	zero_list = [0 for i in range(number_of_actions)]
	outerloopcnt = 0
	book_loc_data = []
	trolley_loc_data = []
	new_action_list = []


	#parsing json file
	with open('books.json') as book_parse:
		data = json.load(book_parse)

	loc_data =  data["books"]
	#print len(loc_data)
	for i in range(0, len(loc_data)):
		book_name = "book_" + str(i+1)
		book_loc_data.append(loc_data[book_name]["load_loc"])
	#print book_loc_data

	bin_data =  data["bins"]
	#print len(bin_data)
	for i in range(0, len(bin_data)):
		trolley_name = "trolly_" + str(i+1)
		trolley_loc_data.append(bin_data[trolley_name]["load_loc"])
	#print trolley_loc_data

	for i in range(num_episodes):
		current_location = problem.get_current_state()
		cumulative_reward = 0
		state_list = []
		state_list.append(current_location)
		count = 0
		if(outerloopcnt%10 == 0):
			epsilon -= 0.1
		#Using Epsilon greedy approach
		# Run the episode till it reaches the terminal state

		while (problem.is_terminal_state() == 0):
			str_out = ""
			state_index = state_list.index(current_location)
			index = current_location.find('turtlebot3_burger')
			bot_loc = []
			if(index != -1):
				bot_location = current_location[index:]
				loc_list = bot_location.split(',')
				x_loc = loc_list[1]
				y_loc = loc_list[2]
				bot_loc.append(float(x_loc))
				bot_loc.append(float(y_loc))

			rand =  random.random()
			if (rand < epsilon):
				#selecting the action randomly
				new_action_list = copy.deepcopy(action_list)
				if (bot_loc not in book_loc_data[0]):
					i = 0
					while i < len(new_action_list):
						if "careful_pick" in new_action_list[i]:
							new_action_list.remove(new_action_list[i])
						i = i + 1
					i = 0
					while i < len(new_action_list):
						if "normal_pick" in new_action_list[i]:
							new_action_list.remove(new_action_list[i])
						i = i + 1
				if (bot_loc in trolley_loc_data[0]):
					i = 0
					while i < len(new_action_list):
						if "normal_place" in new_action_list[i]:
							new_action_list.remove(new_action_list[i])
						i = i + 1
					i = 0
					while i < len(new_action_list):
						if "careful_place" in new_action_list[i]:
							new_action_list.remove(new_action_list[i])
						i = i + 1
				action = random.choice(new_action_list)
				action_index = action_list.index(action)
			else:	
				#Taking the maximum valued action from the q_table for that state
				max_value = np.amax(q_table[state_index])
				action_index = q_table[state_index].index(max_value)
				action = action_list[action_index]
				#print bot_loc

			
			#Executing the action corresponding to the action selected
			if ('normal_moveF' in action):
				success, next_state, reward = problem.execute_normal_moveF()
			elif ('normal_TurnCW' in action):
				success, next_state, reward = problem.execute_normal_TurnCW()
			elif ('normal_TurnCCW' in action):
				success, next_state, reward = problem.execute_normal_TurnCCW()
			elif ('careful_moveF' in action):
				success, next_state, reward = problem.execute_careful_moveF()
			elif ('careful_TurnCW' in action):
				success, next_state, reward = problem.execute_careful_TurnCW()
			elif ('careful_TurnCCW' in action):
				success, next_state, reward = problem.execute_careful_TurnCCW()
			elif ('normal_place' in action): 
				action = action.split(' ')
				book_name = action[1]
				bin_name = action[2][:-1]
				success, next_state, reward = problem.execute_normal_place(book_name, bin_name)
			elif ('careful_place' in action):
				action = action.split(' ')
				book_name = action[1]
				bin_name = action[2][:-1]
				success, next_state, reward = problem.execute_careful_place(book_name, bin_name)
			elif ('normal_pick' in action):
				action = action.split(' ')
				book_name = action[1][:-1]
				success, next_state, reward = problem.execute_normal_pick(book_name)
			elif ('careful_pick' in action):
				action = action.split(' ')
				book_name = action[1][:-1]
				success, next_state, reward = problem.execute_careful_pick(book_name)
			else:
				print "specified action is not in action list"
					

			if next_state in state_list:
				next_state_index = state_list.index(next_state)
			else:
				state_list.append(next_state)
				next_state_index = state_list.index(next_state)
				q_table.append(zero_list)

			old_value = q_table[state_index][action_index]
			next_max = np.amax(q_table[next_state_index])
			new_value = (float(1 - alpha) * old_value) + (alpha * ((reward) + (gamma * next_max)))
			#update the q_table
			q_table[state_index][action_index] = new_value
			str_out += '"(' + state_list[state_index] + ',' + action_list[action_index] + ',' + state_list[next_state_index] + ',' + str(reward) + ')'
			str_out += ':' + str(q_table[state_index][action_index]) + '"'
			pub.publish(str_out)


			#calculating the reward
			if (count == 0):
				cumulative_reward += reward
			else:
				cumulative_reward += (gamma ** count) * reward
			
			count += 1
			current_location = next_state

			#print current_location
		print (outerloopcnt, count, cumulative_reward)
		outerloopcnt += 1
		problem.reset_world()



if __name__ == "__main__":
	args = parser.parse_args()
	if(args.enable_t1):
		print ("Executing Task 1")
		parse_trajectories_file()
	if(args.enable_t2):
		print ("Executing Task 2")
		q_learning_task2(args.episodes)
	if(args.enable_t3):
		print ("Executing Task 3")
		q_learning_task3(args.episodes)
