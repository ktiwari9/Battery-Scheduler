#! /usr/bin/env python

if __name__ == '__main__':

	# states = dict()
	# # state_file = '/home/milan/workspace/strands_ws/src/battery_scheduler/models/model2_rhc.sta'
	# state_file = '/home/milan/workspace/strands_ws/src/battery_scheduler/models/mini_bcth_model.sta'
	# with open(state_file, 'r') as state_file:
	# 	for line in state_file.readlines():
	# 	    if '_da' not in line and 'battery' not in line:
	# 	        state = line[:-2].split(':(')
	# 	        s = state[1].split(',')
	# 	        states.update({state[0] : [el.strip() for el in s]})


	filename = '/home/milan/prism/prism/bin/mini_bcth1.adv'
	# filename = '/home/milan/workspace/strands_ws/src/battery_scheduler/models/model2_rhcpre1.adv'
	time = []
	battery = []
	states_gr = []
	with open(filename, 'r') as f:
		for l in f:
			if 'gather_reward' in l:
				el = l[:-1].split(' ')
				states_gr.append(el[0].strip())
				print states[el[0]][0], states[el[0]][4] 

	print len(states_gr)
	
	# state_file = '/home/milan/workspace/strands_ws/src/battery_scheduler/models/model2_rhc.sta'
	
	# with open(state_file, 'r') as f:
	# 	for line in f:
	# 		if 't' not in line:
	# 			el = line[:-2].split(':(')
	# 			st = el[1].split(',')
	# 			if st[0] not in time:
	# 				time.append(st[0])
	# 			if st[4] not in battery:
	# 				battery.append(st[4])
	# 			# print (st[0], st[4])

	# print time
	# print battery

