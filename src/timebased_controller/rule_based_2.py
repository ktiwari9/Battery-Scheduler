#! /usr/bin/env python

from datetime import date
import generate_samples
# import generate_task_samples as generate_samples
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
		
		sg = generate_samples.sample_generator(True, test_days)
		self.test_rewards = sg.act_rewards
		
		self.battery = []
		self.charging = []
		self.action = []
		self.obtained_reward = []
		self.time = []

		self.simulate()

	def simulate(self):

		for i,r in enumerate(self.test_rewards):
			self.time.append(i)
			self.battery.append(self.current_battery)
			self.charging.append(self.current_charging)

			if i%48 >= 44 or i%48 < 16:
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
	
	rbc = RuleBasedControl(70, 1, [datetime(2019, 11,7), datetime(2019,11,8), datetime(2019,11,9)])
	rbc.get_plan('rbc2_711811911_1')

	np.random.seed(1)
	rbc = RuleBasedControl(70, 1, [datetime(2019, 11,7), datetime(2019,11,8), datetime(2019,11,9)])
	rbc.get_plan('rbc2_711811911_2')

	np.random.seed(2)
	rbc = RuleBasedControl(70, 1, [datetime(2019, 11,7), datetime(2019,11,8), datetime(2019,11,9)])
	rbc.get_plan('rbc2_711811911_3')

	rbc = RuleBasedControl(70, 1, [datetime(2019, 11, 10), datetime(2019,11, 11), datetime(2019,11,12)])
	rbc.get_plan('rbc2_101111111211_1')

	np.random.seed(1)
	rbc = RuleBasedControl(70, 1, [datetime(2019, 11, 10), datetime(2019,11, 11), datetime(2019,11,12)])
	rbc.get_plan('rbc2_101111111211_2')

	np.random.seed(2)
	rbc = RuleBasedControl(70, 1, [datetime(2019, 11, 10), datetime(2019,11, 11), datetime(2019,11,12)])
	rbc.get_plan('rbc2_101111111211_3')

	rbc = RuleBasedControl(70, 1, [datetime(2019, 11,13), datetime(2019,11,14), datetime(2019,11,15)])
	rbc.get_plan('rbc2_131114111511_1')

	np.random.seed(1)
	rbc = RuleBasedControl(70, 1, [datetime(2019, 11,13), datetime(2019,11,14), datetime(2019,11,15)])
	rbc.get_plan('rbc2_131114111511_2')

	np.random.seed(2)
	rbc = RuleBasedControl(70, 1, [datetime(2019, 11,13), datetime(2019,11,14), datetime(2019,11,15)])
	rbc.get_plan('rbc2_131114111511_3')
