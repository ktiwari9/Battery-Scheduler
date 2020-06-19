#! /usr/bin/env python
import os
import yaml
import roslib
import numpy as np
import pandas as pd

TOTAL_INT = 48
THRESHOLD = 40


def get_battery_model():
    path = '/home/milan/workspace/strands_ws/src/battery_scheduler'
    # path = roslib.packages.get_pkg_dir('battery_scheduler')
    if os.path.isfile(path+'/models/battery_charge_model.yaml') and os.path.isfile(path+'/models/battery_discharge_model.yaml'):
        with open (path+'/models/battery_charge_model.yaml', 'r') as f_charge:
            charge_model = yaml.load(f_charge)
        with open (path+'/models/battery_discharge_model.yaml', 'r') as f_discharge:
            discharge_model = yaml.load(f_discharge)
        print ('Battery Models Found at: ' +path+'/models/battery_discharge_model.yaml'+', '+ path+'/models/battery_charge_model.yaml' )

        for model in [charge_model, discharge_model]:
            for b in model:
                bnext_dict = model[b]
                total = np.sum(np.array(bnext_dict.values()))
                for bn in bnext_dict:
                    bnext_dict[bn] = float(bnext_dict[bn])/total
        return charge_model, discharge_model
    else:
        raise ValueError('No models found. First create battery model with probabilistic_battery_model.py')


def read_file(fname):
	filename = '/home/milan/workspace/strands_ws/src/battery_scheduler/data/'+fname
	battery = []
	obtained_rewards = []
	actual_rewards = []
	actions = []
	paretos  = []  ## for pareto points
	actual_reward_present = False
	print filename
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
				# print el
				battery.append(float(el[1].strip()))
				obtained_rewards.append(float(el[4].strip()))
				actions.append(el[3].strip())
				paretos.append(float(el[8].strip()))
				if actual_reward_present:
					actual_rewards.append(float(el[ind].strip()))

	return battery, actions, obtained_rewards, actual_rewards, paretos

def get_battery_statistics(battery, actions):
	battery_mean = np.mean(battery)
	battery_std = np.std(battery)
	battery_min = min(battery)
	battery_max = max(battery)
	b_end = [battery[((i+1)*TOTAL_INT)] for i in range((len(battery)/TOTAL_INT)-1)]
	charge_model, discharge_model = get_battery_model()
	if actions[-1] == 'gather_reward':
		nb_vals = []
		prob = []
		for nb, p in discharge_model[int(battery[-1])].items():
			prob.append(p)
			nb_vals.append(nb)
	elif actions[-1] == 'stay_charging' or actions[-1] == 'go_charge':
		nb_vals = []
		prob = []
		for nb, p in charge_model[int(battery[-1])].items():
			prob.append(p)
			nb_vals.append(nb)

	b_end.append(np.random.choice(nb_vals, p=prob))

	return battery_mean, battery_std, battery_max, battery_min, b_end

def get_reward_statistics(actual_rewards, obtained_rewards):
	if actual_rewards:
		total_rew = np.array([sum(actual_rewards[i*TOTAL_INT:(i+1)*TOTAL_INT]) for i in range(len(actual_rewards)/TOTAL_INT)]
		)
	total_obtained = np.array([sum(obtained_rewards[i*TOTAL_INT:(i+1)*TOTAL_INT]) for i in range(len(obtained_rewards)/TOTAL_INT)])
	hours_worked_array = np.array(map(lambda x: 1.0 if x != 0 else 0, obtained_rewards))
	hours_work_available_array = np.array(map(lambda x: 1.0 if x != 0 else 0, actual_rewards))
	hours_worked = np.array([sum(hours_worked_array[i*TOTAL_INT:(i+1)*TOTAL_INT]) for i in range(len(hours_worked_array)/TOTAL_INT)])
	hours_work_available = np.array([sum(hours_work_available_array[i*TOTAL_INT:(i+1)*TOTAL_INT]) for i in range(len(hours_work_available_array)/TOTAL_INT)])

	if actual_rewards:
		return 100*(total_obtained/total_rew), total_rew, 100*(hours_worked/hours_work_available), hours_work_available
	else:
		return total_obtained, None, hours_worked, hours_work_available

def get_continuous_action_time(all_actions):
	avg_cont_work_int = []
	avg_cont_charge_int = []
	for i in range(len(all_actions)/TOTAL_INT):
		actions = all_actions[i*TOTAL_INT:(i+1)*TOTAL_INT]
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

		avg_cont_charge_int.append(np.mean(charging_count))
		avg_cont_work_int.append(np.mean(working_count))
		
	return np.array(avg_cont_work_int), np.array(avg_cont_charge_int)

if __name__ == '__main__':
	# This will give csv file with all necessary values for plotting different kinds of stats for one particular form of control.

	# Set TOTAL_INT at the beginning of file, change filename at end according to model
	# fsets = [['p6rhctt_dt_318110210_1', 'p6rhctt_dt_318110210_2', 'p6rhctt_dt_318110210_3'], ['p6rhctt_dt_188228248_1', 'p6rhctt_dt_188228248_2', 'p6rhctt_dt_188228248_3'],['p6rhctt_dt_309238289_1', 'p6rhctt_dt_309238289_2', 'p6rhctt_dt_309238289_3'],['p6rhctt_dt_249259269_1', 'p6rhctt_dt_249259269_2', 'p6rhctt_dt_249259269_3'], ['p6rhctt_dt_88310279_1', 'p6rhctt_dt_88310279_2', 'p6rhctt_dt_88310279_3'],['p6rhctt_dt_2113110811_1', 'p6rhctt_dt_2113110811_2', 'p6rhctt_dt_2113110811_3'] ]

	fsets = [['p6rhct_bcth_249259269_1', 'p6rhct_bcth_249259269_2', 'p6rhct_bcth_249259269_3'], ['p6rhct_bcth_271029103110_1', 'p6rhct_bcth_271029103110_2', 'p6rhct_bcth_271029103110_3'],['p6rhct_bcth_2103101211_1', 'p6rhct_bcth_2103101211_2', 'p6rhct_bcth_2103101211_3'], ['p6rhct_bcth_14815849_1', 'p6rhct_bcth_14815849_2', 'p6rhct_bcth_14815849_3'], ['p6rhct_bcth_10111910289_1', 'p6rhct_bcth_10111910289_70b_2', 'p6rhct_bcth_10111910289_70b_3'],['p6rhct_bcth_211022102310_70b_1', 'p6rhct_bcth_211022102310_70b_2', 'p6rhct_bcth_211022102310_70b_3']]
	
	# fsets = [['p6rhctt_dt_249259269_1'], ['p6rhctt_dt_88310279_1'],['p6rhctt_dt_2113110811_1'] ]
	# fsets = [['p6rhctt_dt_318110210_1'], ['p6rhctt_dt_188228248_1'],['p6rhctt_dt_309238289_1'] ]
	
	## Can include sets of different days, different initial states.

	sets_percent_rew = []                   ## for plotting values accross varied data
	sets_percent_hours = []
	sets_percent_uth = []
	sets_battery = []
	sets_end_battery = []
	sets_cont_charge = []
	sets_cont_work = []
	sets_exp_cost = []

	for fnames in fsets:
		b_total = []                          ## give array of expected daily values 
		overall_total_obt = []
		overall_total_rews = []
		overall_hours_worked = []
		overall_hours_available = []
		overall_cont_work = []
		overall_cont_charge = []
		overall_percent_under_th = []
		overall_end_battery = []
		overall_expected_cost = []

		for fname in fnames:		
			battery, actions, obtained_rewards, actual_rewards, paretos = read_file(fname)
			b_total.extend(battery)
			battery_mean, battery_std, battery_max, battery_min, b_end = get_battery_statistics(battery, actions)
			overall_end_battery.append(b_end) 
			
			total_obtained, total_rew, hours_worked, hours_work_available = get_reward_statistics(actual_rewards, obtained_rewards)
			overall_hours_available.append(hours_work_available)
			overall_hours_worked.append(hours_worked)
			if actual_rewards:
				overall_total_rews.append(total_rew)
			overall_total_obt.append(total_obtained)
			
			ints_under_th = np.array(map(lambda x: 1.0 if x < THRESHOLD else 0, battery))
			percentage_under_th = np.array([sum(ints_under_th[i*TOTAL_INT:(i+1)*TOTAL_INT]) for i in range(len(battery)/TOTAL_INT)])
			# no_days = len(obtained_rewards)/TOTAL_INT
			percentage_under_th = (percentage_under_th/TOTAL_INT)*100
			overall_percent_under_th.append(percentage_under_th)
			
			avg_cont_work, avg_cont_charge = get_continuous_action_time(actions)
			overall_cont_charge.append(avg_cont_charge/2)  ## cont charging in hours
			overall_cont_work.append(avg_cont_work/2)

			exp_cost = np.array([np.mean(paretos[i*TOTAL_INT:(i+1)*TOTAL_INT]) for i in range(len(battery)/TOTAL_INT)])
			overall_expected_cost.append(exp_cost)

		overall_end_battery = np.mean(np.array(overall_end_battery), axis=0)
		overall_total_obt = np.mean(np.array(overall_total_obt), axis=0)
		overall_hours_worked = np.mean(np.array(overall_hours_worked), axis=0)
		overall_percent_under_th = np.mean(np.array(overall_percent_under_th), axis=0)
		overall_cont_work = np.mean(np.array(overall_cont_work), axis=0)
		overall_cont_charge = np.mean(np.array(overall_cont_charge), axis=0)
		overall_expected_cost = np.mean(np.array(overall_expected_cost), axis=0)
	
		sets_end_battery.extend(list(overall_end_battery))
		sets_battery.extend(b_total)
		sets_percent_rew.extend(list(overall_total_obt))
		sets_percent_hours.extend(list(overall_hours_worked))
		sets_percent_uth.extend(list(overall_percent_under_th))
		sets_cont_work.extend(list(overall_cont_work))
		sets_cont_charge.extend(list(overall_cont_charge))
		sets_exp_cost.extend(list(overall_expected_cost))

	df_b = pd.DataFrame(data = list(zip(np.arange(len(sets_battery)), sets_battery)), columns=['nums', 'battery_life'])
	df_b.to_csv('/home/milan/workspace/strands_ws/src/battery_scheduler/data/csv_files/p6rhct_bcth_battery_csv.csv', header=False, index=False)
	df = pd.DataFrame(data = list(zip(sets_end_battery, sets_percent_rew, sets_percent_hours, sets_percent_uth, sets_cont_charge, sets_cont_work, sets_exp_cost)), columns=['end_battery', 'percent_rew', 'percent_hours', 'percent_under_th', 'cont_charge_hrs', 'cont_work_hrs', 'pareto_cost'])
	df.to_csv('/home/milan/workspace/strands_ws/src/battery_scheduler/data/csv_files/p6rhct_bcth_csv.csv', header=True, index=True) ## - change file name according to control, one file name for one control
