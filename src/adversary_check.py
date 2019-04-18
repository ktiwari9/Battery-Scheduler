#! /usr/bin/env python

if __name__ == '__main__':
	# filename = '/home/milan/prism/prism/bin/advpre1.tra'
	filename = '/home/milan/workspace/strands_ws/src/battery_scheduler/models/model_ttpre1.adv'
	time = []
	battery = []
	states = []
	with open(filename, 'r') as f:
		for l in f:
			if 'gather_reward' in l:
				el = l[:-1].split(' ')
				states.append(el[0].strip())
				print el[1], el[-1]

	print len(states)
	state_file = '/home/milan/workspace/strands_ws/src/battery_scheduler/models/model_tt.sta'
	
	with open(state_file, 'r') as f:
		for line in f:
			if 't' not in line:
				el = line[:-2].split(':(')
				st = el[1].split(',')
				if st[0] not in time:
					time.append(st[0])
				if st[4] not in battery:
					battery.append(st[4])
				# print (st[0], st[4])

	print time
	print battery

