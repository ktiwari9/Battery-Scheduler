#! /usr/bin/env python

from datetime import date
# import generate_task_samples as generate_samples
import generate_samples
import numpy as np
import roslib
import yaml
import os



def get_battery_model():
    path = roslib.packages.get_pkg_dir('battery_scheduler')
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
 

class RuleBasedControl:

	def __init__(self, init_battery, init_charging):
		self.current_battery = init_battery
		self.current_charging = init_charging
		self.charge_model, self.discharge_model = get_battery_model()
		
		self.test_rewards = []
		with open(roslib.packages.get_pkg_dir('battery_scheduler')+'/data/rbc3_sample_rewards', 'r') as f:
			for line in f:
				self.test_rewards.append(float(line.split(' ')[2].strip()))

		self.battery = []
		self.charging = []
		self.action = []
		self.obtained_reward = []
		self.time = []

	def simulate(self):

		for i,r in enumerate(self.test_rewards):
			self.time.append(i)
			self.battery.append(self.current_battery)
			self.charging.append(self.current_charging)

			if (i%48 >= 8 and i%48 < 16) or (i%48 >= 24 and i%48 < 32) or (i%48 >= 40 and i%48 < 48):
				if self.current_charging == 1:
					self.action.append('stay_charging')
					self.obtained_reward.append(0)
					nb = []
					p = []
					for b,prob in self.charge_model[self.current_battery].items():
						nb.append(b)
						p.append(prob)
					self.current_battery = np.random.choice(nb, p=p)
					# self.current_battery = int(round(sum(np.array(nb)*np.array(p))))
					self.current_charging = 1

				else:
					self.action.append('go_charge')
					self.obtained_reward.append(0)
					nb = []
					p = []
					for b,prob in self.charge_model[self.current_battery].items():
						nb.append(b)
						p.append(prob)
					if self.current_battery != 99 or self.current_battery != 100:
						self.current_battery = np.random.choice(nb, p=p)-1
						# self.current_battery = int(round(sum(np.array(nb)*np.array(p))))-1
					else:
						self.current_battery = np.random.choice(nb, p=p)
						# self.current_battery = int(round(sum(np.array(nb)*np.array(p))))
					self.current_charging = 1
			
			elif (self.current_battery > 25 and self.current_battery < 70 and self.current_charging == 1) or (self.current_charging ==1 and r == 0):
				self.action.append('stay_charging')
				self.obtained_reward.append(0)
				nb = []
				p = []
				for b,prob in self.charge_model[self.current_battery].items():
					nb.append(b)
					p.append(prob)

				self.current_battery = np.random.choice(nb, p=p)
				# self.current_battery = int(round(sum(np.array(nb)*np.array(p))))
				self.current_charging = 1
			
			elif self.current_battery > 25 and r != 0:
				self.action.append('gather_reward')
				self.obtained_reward.append(r)
				nb = []
				p = []
				for b,prob in self.discharge_model[self.current_battery].items():
					nb.append(b)
					p.append(prob)
				self.current_battery = np.random.choice(nb, p=p)
				# self.current_battery = int(round(sum(np.array(nb)*np.array(p))))
				self.current_charging = 0

			else:
				self.action.append('go_charge')
				self.obtained_reward.append(0)	
				nb = []
				p = []
				for b,prob in self.charge_model[self.current_battery].items():
					nb.append(b)
					p.append(prob)

				if self.current_battery != 99 or self.current_battery != 100:
					self.current_battery = np.random.choice(nb, p=p)-1
					# self.current_battery = int(round(sum(np.array(nb)*np.array(p))))-1
				else:
					self.current_battery = np.random.choice(nb, p=p)
					# self.current_battery = int(round(sum(np.array(nb)*np.array(p))))
				self.current_charging = 1


	def get_plan(self, fname):
		print 'Writing plan..'
		file_name = roslib.packages.get_pkg_dir('battery_scheduler')+'/data/' + fname
		with open(file_name, 'w') as f:
			f.write('time battery charging action obtained_reward actual_reward\n')
			for t, b, ch, a, obr, ar in zip(self.time, self.battery, self.charging, self.action, self.obtained_reward, self.test_rewards):
				f.write('{0} {1} {2} {3} {4} {5}\n'.format(t, b, ch, a, obr, ar))

	def get_rewards_from_plan(self, fname):
		print 'Reading Plan...'
		file_name = roslib.packages.get_pkg_dir('battery_scheduler')+'/data/' + fname
		rewards_obtained = []
		with open(file_name, 'r') as f:
			for line in f:
				if 'time' not in line:
					s = line.split(' ')
					rewards_obtained.append(float(s[4]))
		return rewards_obtained



if __name__ == '__main__':
	no_int = 48

	sg = generate_samples.sample_generator(True, [date(2017, 11, 10), date(2017, 10, 19), date(2017, 9, 28)])     
	rewards = sg.rewards
	cl_id = sg.cl_ids
	act_rewards = sg.act_rewards
	path = roslib.packages.get_pkg_dir('battery_scheduler')+'/data/rbc3_sample_rewards'
	with open(path,'w') as f:
	    for r, c, a_r in zip(rewards, cl_id, act_rewards):
	        f.write('{0} {1} {2} '.format(c, r, a_r))
	        f.write('\n')

	np.random.seed(0)
	f_name = 'rbc3_10111910289_70b_1'
	rbc3 = RuleBasedControl(70,1)

	rbc3.simulate()
	rbc3.get_plan(f_name)
	rewards_obtained = rbc3.get_rewards_from_plan(f_name)
	no_days = len(rbc3.test_rewards)/no_int
	for i in range(no_days):
		print 'Day ', i+1, ' : ', sum(rewards_obtained[i*no_int:(i+1)*no_int]) 
	print 'Total : ', sum(rewards_obtained)

	np.random.seed(1)
	f_name = 'rbc3_10111910289_70b_2'
	rbc3 = RuleBasedControl(70,1)

	rbc3.simulate()
	rbc3.get_plan(f_name)
	rewards_obtained = rbc3.get_rewards_from_plan(f_name)
	no_days = len(rbc3.test_rewards)/no_int
	for i in range(no_days):
		print 'Day ', i+1, ' : ', sum(rewards_obtained[i*no_int:(i+1)*no_int]) 
	print 'Total : ', sum(rewards_obtained)


	np.random.seed(2)
	f_name = 'rbc3_10111910289_70b_3'
	rbc3 = RuleBasedControl(70,1)

	rbc3.simulate()
	rbc3.get_plan(f_name)
	rewards_obtained = rbc3.get_rewards_from_plan(f_name)
	no_days = len(rbc3.test_rewards)/no_int
	for i in range(no_days):
		print 'Day ', i+1, ' : ', sum(rewards_obtained[i*no_int:(i+1)*no_int]) 
	print 'Total : ', sum(rewards_obtained)




