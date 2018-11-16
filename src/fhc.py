#! /usr/bin/env python
import probabilistic_rewards
import read_adversary
import prism_model
import numpy as np
import subprocess
import yaml
import sys
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
        return charge_model, discharge_model
    else:
        raise ValueError('No models found. First create battery model with probabilistic_battery_model.py')
 

class FiniteHorizonControl:

    def __init__(self, init_battery, init_charging, init_time):
        self.ur = probabilistic_rewards.uncertain_rewards()
        self.prob, self.clusters = ur.get_probabilistic_reward_model()
        self.charge_model, self.discharge_model = get_battery_model()
        self.cl_id = []
        self.sample_reward = []
        self.actual_reward = []
        # self.exp_reward = []
    
        #######################SPECIFY LOCATION ######################
        self.main_path = '/home/milan/workspace/strands_ws/src/battery_scheduler'
        self.path_rew = self.main_path + '/data/sample_rewards'
        self.path_mod = self.main_path + '/models/'
        self.path_data = self.main_path + '/data/'
    
        if not os.path.isfile(self.path_rew):
            raise ValueError('Sample Rewards Not Generated. Generate rewards with generate_samples.py')

        with open(self.path_rew,'r') as f:
            for line in f:
                x = line.split(' ')
                self.cl_id.append(int(float(x[0].strip())))
                self.sample_reward.append(float(x[1].strip()))
                self.actual_reward.append(float(x[2].strip()))
                # self.exp_reward.append(float(line.split(' ')[3]))        
    
        self.no_int = ur.no_int 
        self.no_days = len(ur.test_days)
        self.avg_totalreward = np.zeros((self.no_days))
        self.init_battery = init_battery
        self.init_charging = init_charging
        self.init_time = init_time
        self.no_simulations = 1

        
    def simulate(self):
        for k in range(self.no_days):
            self.pp = self.obatin_prism_model()
                

        ### from  current get to state which gathers rewards or charges
        ### match cluster according to rewards.
        ### pick battery ac to probability distribution     
        
            battery = no_simulations*[0]
            charging = no_simulations*[0]
            init_cluster= no_simulations*[0]
            tr_day = no_simulations*[0]
            for i in range(no_simulations):
                rewards, action, final_state = self.simulate_day(k,'real_dec16_wrhc')  #######################SPECIFY LOCATION ######################
                battery[i] = int(final_state[0])
                print battery[i]
                charging[i] = int(final_state[1])
                print charging[i]
                tr_day[i] = 0
                for r in range(len(rewards)):
                    if action[r] == 'gather_reward':
                        tr_day[i] = rewards[r] + tr_day[i]

            self.init_battery = int(np.average(np.array(battery)))
            self.init_charging = int(np.average(np.array(charging)))
            self.init_cluster = int(np.average(np.array(init_cluster)))
            self.avg_totalreward[k] = np.average(np.array(tr_day))
            print init_battery, ' end battery'
            print init_charging, ' end charging'
            print init_cluster, ' end cluster'
    
        for k in range(len(avg_totalreward)):
            print avg_totalreward[k], ' total_reward, day',k+1
        print np.sum(avg_totalreward), ' : Reward for Aug'

    def simulate_day(self, fname):
        current_state = self.pp.initial_state
        for i in range(self.no_int):
            actions = []
            while (('gather_reward' not in actions) or ('go_charge' not in actions) or ('stay_charging' not in actions)):
                nx_s, tp, actions = pp.get_possible_next_states(current_state)
                if all(a == 'observe' for a in actions):
                    for s in nx_s:
                        t, tp, o, e, b, ch, cl = self.pp.states[s]
                        if self.actual_reward[i] != 0:





    def obatin_prism_model(self):
        pm = prism_model.PrismModel('model_t.prism', self.init_time, self.init_battery, self.init_charging, self.clusters, self.prob, self.charge_model, self.discharge_model)
        #######################SPECIFY LOCATION ######################
        # running prism and saving output from prism
        with open(self.path_data+'result_fhc', 'w') as file:
            process = subprocess.Popen('./prism '+ self.path_mod + 'model_t.prism '+ self.path_mod +'model_prop.props -exportadv '+ self.path_mod+ 'model_t.adv -exportprodstates ' + self.path_mod +'model_t.sta -exporttarget '+self.path_mod+'model_t.lab',cwd='/home/milan/prism-svn/prism/bin', shell=True, stdout=subprocess.PIPE)
            for c in iter(lambda: process.stdout.read(1), ''):
                sys.stdout.write(c)
                file.write(c)
        ##reading output from prism to find policy file
        policy_file = []
        with open(self.path_data+'result_fhc', 'r') as f:
            line_list = f.readlines()
            for i in range(len(line_list)):
                if 'Computed point: ' in line_list[i]:
                    el = line_list[i].split(' ')
                    req_point = el[2][1:-1]
                    if abs(1.0-float(req_point)) < 0.001:
                        start_p = len('Adversary written to file "'+path_mod)
                        file_name = line_list[i-1][start_p:-3]
                        policy_file.append((req_point, file_name))

                if 'Result:' in line_list[i]:
                    s_policy_file = sorted(policy_file, key= lambda x: abs(1-float(x[0])))
                    
                    if len(s_policy_file) == 1:
                        policy_file_name = s_policy_file[0][1]
                    elif '('+s_policy_file[0][0]+',' in line_list[i]:
                        policy_file_name = s_policy_file[0][1]
                    else:
                        policy_file_name = s_policy_file[1][1]

        #######################SPECIFY LOCATION AS BEFORE ######################
        print 'Reading from ', policy_file_name
        pp = read_adversary.ParseAdversary([policy_file_name, 'model_t.sta', 'model_t.lab'])
        return pp

