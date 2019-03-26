#! /usr/bin/env python
import numpy as np
import pandas as pd

TOTAL_INT = 48
THRESHOLD = 40

def read_file(fname):
	filename = '/home/milan/workspace/strands_ws/src/battery_scheduler/data/'+fname
	battery = []
	obtained_rewards = []
	actual_rewards = []
	actions = []
	actual_reward_present = False
	with open (filename, 'r') as f:
		for line in f:
			x = line[:-1].split(' ')
			if 'actual_reward' in x:
				ind = x.index('actual_reward')
				actual_reward_present = True
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
	b_end = np.mean([battery[((i+1)*TOTAL_INT)-1] for i in range(len(battery)/TOTAL_INT)])
	return battery_mean, battery_std, battery_max, battery_min, b_end

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
	# This will give csv file with all necessary values for plotting different kinds of stats for one particular form of control.

	# Set TOTAL_INT at the beginning of file
	fsets = [['p10rhct_bcth_oct123_70b_1', 'p10rhct_bcth_oct123_70b_2', 'p10rhct_bcth_oct123_70b_3'], ['p10rhct_bcth_sep242526_70b_1', 'p10rhct_bcth_sep242526_70b_2', 'p10rhct_bcth_sep242526_70b_3'], ['p10rhct_bcth_aug141516_70b_1', 'p10rhct_bcth_aug141516_70b_2', 'p10rhct_bcth_aug141516_70b_3'], ['p10rhct_bcth_aug202223_70b_1', 'p10rhct_bcth_aug202223_70b_2', 'p10rhct_bcth_aug202223_70b_3']] ## Can include sets of different days, different initial states.

	sets_percent_rew = []
	sets_percent_hours = []
	sets_percent_uth = []
	sets_battery = []
	sets_end_battery = []
	sets_cont_charge = []
	sets_cont_work = []

	for fnames in fsets:
		b_total = []
		overall_totals_obt = []
		overall_total_rews = []
		overall_hours_worked = []
		overall_hours_available = []
		overall_cont_work = []
		overall_cont_charge = []
		percentage_under_th = []

		for fname in fnames:		
			battery, actions, obtained_rewards, actual_rewards = read_file(fname)
			b_total.extend(battery)
			battery_mean, battery_std, battery_max, battery_min, b_end = get_battery_statistics(battery)
			print fname
			for i in range(len(obtained_rewards)/TOTAL_INT):
				print 'Day ', i+1, ': ', sum(obtained_rewards[i*TOTAL_INT:(i+1)*TOTAL_INT])  
			print 'Total across ', len(obtained_rewards)/TOTAL_INT, ' days: ', sum(obtained_rewards)
			print 'Battery Stats:'
			print battery_mean, 'b_mean' 
			print battery_std, 'b_std'
			print battery_min, 'b_min'
			print battery_max, 'b_max'
			print b_end, 'b_end'
			print '------------------------------------------------------------------'
			print 'Reward Stats'
			
			total_obtained, total_rew, hours_worked, hours_work_available = get_reward_statistics(actual_rewards, obtained_rewards)
			overall_hours_available.append(hours_work_available)
			overall_hours_worked.append(hours_worked)
			if total_rew != None:
				overall_total_rews.append(total_rew)
			overall_totals_obt.append(total_obtained)

			print total_obtained, total_rew
			print hours_worked, hours_work_available, 'total_hours_worked'
			ints_under_th = [1 for x in battery if x < THRESHOLD]
			no_days = len(obtained_rewards)/TOTAL_INT
			if ints_under_th:
				percentage_under_th.append((float(sum(ints_under_th))*100)/(no_days*48))
			else:
				percentage_under_th.append(0)
			print '--------------------------------------------------------------------'
			
			avg_cont_work, avg_cont_charge = get_continuous_action_time(actions)
			overall_cont_charge.append(avg_cont_charge)
			overall_cont_work.append(avg_cont_work)
			print 'Cont Time Stats'
			print avg_cont_charge, 'charge cont'
			print avg_cont_work, 'work cont'
			print '-------------------------------------------------------------------'

			sets_end_battery.append(b_end)
			sets_percent_rew.append(total_obtained)
			sets_percent_hours.append((hours_worked/hours_work_available)*100)
			if ints_under_th:
				sets_percent_uth.append((float(sum(ints_under_th))*100)/(no_days*48))
			else:
				sets_percent_uth.append(0)
			sets_cont_work.append(avg_cont_work)
			sets_cont_charge.append(avg_cont_charge)

		print 'OVERALL STATS:' ### TO BE EDITED 

		print 'Battery Stats:'
		battery_mean, battery_std, battery_max, battery_min, b_end = get_battery_statistics(b_total)
		sets_battery.extend(b_total)

		print battery_mean, 'b_mean' 
		print battery_std, 'b_std'
		print battery_min, 'b_min'
		print battery_max, 'b_max'
		print b_end, 'b_end'
		print '------------------------------------------------------------------'
		
		print 'Reward Stats'
		total_obtained = np.mean(overall_totals_obt)
		if overall_total_rews:
			total_rew = np.mean(overall_total_rews)
		else:
			total_rew = None
		hours_worked = np.mean(overall_hours_worked)
		hours_work_available = np.mean(overall_hours_available)
		# sets_percent_rew.extend(total_obtained)
		# sets_percent_hours.append((hours_worked/hours_work_available)*100)
		# sets_percent_uth.append(np.mean(percentage_under_th))
		
		print total_obtained, total_rew
		print hours_worked, hours_work_available, 'total_hours_worked'
		print np.mean(percentage_under_th), 'percentage_time_under_th'
		print '--------------------------------------------------------------------'
		
		avg_cont_work = np.mean(overall_cont_work)
		avg_cont_charge = np.mean(overall_cont_charge)
		# sets_cont_work.append(avg_cont_work)
		# sets_cont_charge.append(avg_cont_charge)
		print 'Cont Time Stats'
		print avg_cont_charge, 'charge cont'
		print avg_cont_work, 'work cont'
		print '-------------------------------------------------------------------'

	df_b = pd.DataFrame(data = list(zip(np.arange(len(sets_battery)), sets_battery)), columns=['nums', 'battery_life'])
	df_b.to_csv('/home/milan/workspace/strands_ws/src/battery_scheduler/data/csv_files/p10rhct_bcth_battery_csv.csv', header=False, index=False)
	df = pd.DataFrame(data = list(zip(sets_end_battery, sets_percent_rew, sets_percent_hours, sets_percent_uth, sets_cont_charge, sets_cont_work)), columns=['end_battery', 'percent_rew', 'percent_hours', 'percent_under_th', 'cont_charge_hrs', 'cont_work_hrs'])
	df.to_csv('/home/milan/workspace/strands_ws/src/battery_scheduler/data/csv_files/p10rhct_bcth_csv.csv', header=True, index=True)  ## - change file name according to control, one file name for one control