#! /usr/bin/env python

import battery_model
import numpy as np


if __name__ == '__main__':

	charge_model, discharge_model = battery_model.get_battery_model('/media/milan/DATA/battery_logs/real_battery')
	m = 0
	for model in [charge_model, discharge_model]:
		for b in model:
			# if m == 0 and b == 1:
			# 	model.update({b: model[0]})
			# 	# print b, sum(model[0].values())
			if m == 1 and b == 100:
				model.update({ b : model[99]})
				# print b, sum(model[99].values())
			else:
				b_next = model[b]
				total = float(sum(b_next.values()))
				for nb in b_next.keys():
					count = b_next[nb]
					b_next.update({ nb : count/total})
				model.update({b : b_next})
				# print b, sum(b_next.values())
		m = m+1


	start_time_int = 0
	end_time_int = 47

	file_path = '/home/milan/workspace/strands_ws/src/battery_scheduler/data/' + 'rhc_sep_pbr'

	rewards = []
	b_orig = []
	with open(file_path, 'r') as f:
		for line in f:
			if 'time' not in line:
				rewards.append(float(line.split(' ')[5]))
				b_orig.append(float(line.split(' ')[1]))
	# print rewards

	rew_ach = len(rewards)*[0]
	b_vals = len(rewards)*[100]
	no_days = len(rewards)/48
	battery = 100
	fully_charged = True
	for i in range(len(rewards)):
		if battery > 30 and fully_charged and rewards[i] != 0.0 and i%48 >= start_time_int and i%48 <= end_time_int:
			rew_ach[i] = rewards[i]
			b_vals[i] = battery
			battery = np.random.choice(discharge_model[battery].keys(), p=discharge_model[battery].values())
		elif battery > 30 and fully_charged and rewards[i] == 0.0 and i%48 >= start_time_int and i%48 <= end_time_int:
			b_vals[i] = battery
			battery = np.random.choice(charge_model[battery].keys(), p=charge_model[battery].values())
			if battery >= 100:
				fully_charged = True

		elif battery > 30 and not fully_charged and i%48 >= start_time_int and i%48 <= end_time_int:
			b_vals[i] = battery
			battery = np.random.choice(charge_model[battery].keys(), p=charge_model[battery].values())
			if battery >= 100:
				fully_charged = True
		elif battery <= 30 and fully_charged:
			b_vals[i] = battery
			battery = np.random.choice(charge_model[battery].keys(), p=charge_model[battery].values())
			fully_charged = False
		elif battery > 30 and i%48 not in range(start_time_int, end_time_int+1):
			battery = np.random.choice(charge_model[battery].keys(), p=charge_model[battery].values())
			if battery >= 100:
				fully_charged = True
			else:
				fully_charged = False


	print zip(b_vals, rew_ach)
	print zip(b_orig, rewards)
		
	print sum(rew_ach)




