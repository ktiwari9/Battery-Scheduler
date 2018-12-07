#! /usr/bin/env python

import numpy as np
import yaml
import os

def get_battery_model():
    ################ SPECIFY PATHS OF MODELS #######################
    path = '/home/milan/workspace/strands_ws/src/battery_scheduler'
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
		with open('/home/milan/workspace/strands_ws/src/battery_scheduler/data/sample_rewards', 'r') as f:
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
			
			if (self.current_battery > 30 and self.current_battery < 70 and self.current_charging == 1) or (self.current_charging ==1 and r == 0):
				self.action.append('stay_charging')
				self.obtained_reward.append(0)
				
				nb = []
				p = []
				for b,prob in self.charge_model[self.current_battery].items():
					nb.append(b)
					p.append(prob)

				self.current_battery = np.random.choice(nb, p=p)
				self.current_charging = 1

			
			elif self.current_battery > 30 and r != 0:
				self.action.append('gather_reward')
				self.obtained_reward.append(r)

				nb = []
				p = []
				for b,prob in self.discharge_model[self.current_battery].items():
					nb.append(b)
					p.append(prob)

				self.current_battery = np.random.choice(nb, p=p)
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
				else:
					self.current_battery = np.random.choice(nb, p=p)
				self.current_charging = 1


	def get_plan(self, fname):
		print 'Writing plan..'
		print self.time
		print self.charging
		print self.battery
		print self.action
		print self.obtained_reward

		file_name = '/home/milan/workspace/strands_ws/src/battery_scheduler/data/' + fname
		with open(file_name, 'w') as f:
			f.write('time battery charging  action  obtained_reward \n')
			for t, b, ch, a, obr in zip(self.time, self.battery, self.charging, self.action, self.obtained_reward):
				f.write('{0} {1} {2} {3} {4}\n'.format(t, b, ch, a, obr))


if __name__ == '__main__':
	rbc = RuleBasedControl(70,1)
	rbc.simulate()
	rbc.get_plan('rbc_aug1118_1')
