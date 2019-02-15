#! /usr/bin/env python

import numpy as np
import pandas as pd

TOTAL_INT = 48

def read_file(fname):
	filename = '/home/milan/workspace/strands_ws/src/battery_scheduler/data/'+fname
	battery = []
	obtained_rewards = []
	actual_rewards = []
	actions = []
	actual_reward_present = True

	z = 0
	with open (filename, 'r') as f:
		for line in f:
			z+=1
			x = line[:-1].split(' ')
			if 'Pareto Point:' in x:
				pareto_point = float(x[1].strip())
			if 'actual_reward' in x:
				ind = x.index('actual_reward')
				actual_reward_present = True
			if z > 2:
				break
		for line in f:
			if 'battery' not in line:
				el = line[:-1].split(' ')
				battery.append(float(el[1].strip()))
				obtained_rewards.append(float(el[4].strip()))
				actions.append(el[3].strip())
				if actual_reward_present:
					actual_rewards.append(float(el[ind].strip()))

	return battery, actions, obtained_rewards, actual_rewards

def get_battery_statistics(battery):
	battery_mean = np.mean(battery)
	battery_std = np.std(battery)
	battery_min = min(battery)
	battery_max = max(battery)
	return battery_mean, battery_std, battery_max, battery_min

def get_reward_statistics(actual_rewards, obtained_rewards):
	total_rew = None
	if actual_rewards:
		total_rew = sum(actual_rewards)

	total_obtained = sum(obtained_rewards)
	hours_worked = sum([1 for r in obtained_rewards if r != 0])/float(2)
	hours_work_available = sum([1 for r in actual_rewards if r != 0])/float(2)

	if total_rew != None:
		return 100*float(total_obtained)/total_rew, total_rew, hours_worked, hours_work_available
	else:
		return total_obtained, None, hours_worked, hours_work_available

def get_continuous_action_time(actions):
	working_count = []
	charging_count = []
	count = 0
	for e,a in enumerate(actions):
		if a == 'go_charge' or a == 'stay_charging':
			count +=1 
		else:
			if count != 0:
				charging_count.append(count)
			count = 0

	count = 0
	for e,a in enumerate(actions):
		if a == 'gather_reward':
			count +=1 
		else:
			if count != 0:
				working_count.append(count)
			count = 0
	
	return np.mean(working_count), np.mean(charging_count)

if __name__ == '__main__':
	### write files directly to csv files for pareto files
	file_range = 86
	no_days = 1
	percentage_under_thresh = 31
	bmean_paretos = []
	b_std_paretos = []
	b_min_paretos = []
	b_max_paretos = []
	rew_percent_paretos = []
	percentage_under_30_paretos = []
	hours_percent_paretos = []
	avg_cont_work_paretos = []
	avg_cont_charge_paretos = []

	for f_no in range(file_range+1):
		fnames = ['bc_pareto'+str(f_no)+'_oct1_70b_1', 'bc_pareto'+str(f_no)+'_oct1_70b_2', 'bc_pareto'+str(f_no)+'_oct1_70b_3', 'bc_pareto'+str(f_no)+'_oct2_70b_1', 'bc_pareto'+str(f_no)+'_oct2_70b_2', 'bc_pareto'+str(f_no)+'_oct2_70b_3', 'bc_pareto'+str(f_no)+'_oct3_70b_1', 'bc_pareto'+str(f_no)+'_oct3_70b_2', 'bc_pareto'+str(f_no)+'_oct3_70b_3']
		
		b_total = []
		overall_totals_obt = []
		overall_total_rews = []
		overall_hours_worked = []
		overall_hours_available = []
		overall_cont_work = []
		overall_cont_charge = []
		percentage_under_30 = []

		for fname in fnames:		
			battery, actions, obtained_rewards, actual_rewards = read_file(fname)
			b_total.extend(battery)
			battery_mean, battery_std, battery_max, battery_min = get_battery_statistics(battery)
			print fname			
			total_obtained, total_rew, hours_worked, hours_work_available = get_reward_statistics(actual_rewards, obtained_rewards)
			overall_hours_available.append(hours_work_available)
			overall_hours_worked.append(hours_worked)
			if total_rew != None:
				overall_total_rews.append(total_rew)
			overall_totals_obt.append(total_obtained)

			avg_cont_work, avg_cont_charge = get_continuous_action_time(actions)
			overall_cont_charge.append(avg_cont_charge)
			overall_cont_work.append(avg_cont_work)
			ints_under_30 = [1 for x in battery if x < percentage_under_thresh]
			if ints_under_30:
				percentage_under_30.append((float(sum(ints_under_30))*100)/(no_days*48))
			else:
				percentage_under_30.append(0)

		battery_mean, battery_std, battery_max, battery_min = get_battery_statistics(b_total)
		bmean_paretos.append(battery_mean)
		b_std_paretos.append(battery_std)
		b_min_paretos.append(battery_min)
		b_max_paretos.append(battery_max)
		percentage_under_30_paretos.append(np.mean(percentage_under_30))

		print 'OVERALL STATS:'
		print 'Battery Stats:'
		print battery_mean, 'b_mean' 
		print battery_std, 'b_std'
		print battery_min, 'b_min'
		print battery_max, 'b_max'
		print '------------------------------------------------------------------'
		
		print 'Reward Stats'
		total_obtained = np.mean(overall_totals_obt)
		if overall_total_rews:
			total_rew = np.mean(overall_total_rews)
		else:
			total_rew = None
		hours_worked = np.mean(overall_hours_worked)
		hours_work_available = np.mean(overall_hours_available)

		rew_percent_paretos.append(total_obtained)
		hours_percent_paretos.append((hours_worked/hours_work_available)*100)

		print total_obtained, total_rew
		print hours_worked, hours_work_available, 'total_hours_worked'
		print '--------------------------------------------------------------------'
		
		avg_cont_work = np.mean(overall_cont_work)/2
		avg_cont_work_paretos.append(avg_cont_work)
		avg_cont_charge = np.mean(overall_cont_charge)/2
		avg_cont_charge_paretos.append(avg_cont_charge)
		print 'Cont Time Stats'
		print avg_cont_charge, 'charge cont'
		print avg_cont_work, 'work cont'
		print '-------------------------------------------------------------------'

	### reading output from prism to find policy file
	with open('/home/milan/workspace/strands_ws/src/battery_scheduler/data/'+'result_bc_fhc', 'r') as f:
		line_list = f.readlines()
		pareto_points = []
		for line in line_list:
		    if ': New point is (' in line:
		        el = line.split(' ')
		        prob30 = abs(float(el[4][1:-1]))
		        pareto_points.append(prob30)

	pareto_points_sorted = sorted(pareto_points, reverse=True)
	print pareto_points_sorted			


	df = pd.DataFrame(data = list(zip(bmean_paretos, b_std_paretos, b_min_paretos, b_max_paretos, rew_percent_paretos, hours_percent_paretos, avg_cont_charge_paretos, avg_cont_work_paretos, pareto_points_sorted, percentage_under_30_paretos)), columns=['bmean', 'bstd', 'bmin', 'bmax', 'percent_rew', 'percent_hours', 'cont_charge_hrs', 'cont_work_hrs', 'paretos', 'percentage_under_thresh'])
	df.to_csv('/home/milan/workspace/strands_ws/src/battery_scheduler/data/bc_pareto_csv', header=True, index=True)
