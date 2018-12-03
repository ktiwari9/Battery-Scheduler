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

        for model in [charge_model, discharge_model]:
            for b in model:
                bnext_dict = model[b]
                total = np.sum(np.array(bnext_dict.values()))
                for bn in bnext_dict:
                    bnext_dict[bn] = float(bnext_dict[bn])/total
        return charge_model, discharge_model
    else:
        raise ValueError('No models found. First create battery model with probabilistic_battery_model.py')
 

class RecedingHorizonControl:
    
    def __init__(self, init_battery, init_charging):
        ur = probabilistic_rewards.uncertain_rewards()
        self.prob, self.clusters = ur.get_probabilistic_reward_model()
        self.charge_model, self.discharge_model = get_battery_model()
        self.cl_id = []
        self.sample_reward = []
        self.actual_reward = []
        self.exp_reward = []
        self.no_int = ur.no_int 
        self.no_days = len(ur.test_days)
        self.horizon = 48 ## in terms of intervals 
        self.no_simulations = 1
        for z in range(self.no_int*self.no_days):
            self.exp_reward.append(sum(self.prob[z%self.no_int]*self.clusters))
   
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

        self.totalreward = np.zeros((self.no_days))
        self.init_battery = init_battery
        self.init_charging = init_charging
        self.actions = []
        self.obtained_rewards = []
        self.battery = []
        self.charging = []
        self.time =[]
     
    def simulate(self):
        print 'Simulating...'
        for k in range(self.no_days*self.no_int):
            self.pp = self.obtain_prism_model(k)
            self.time.append(k)
            next_state = self.get_next_state(k) 
            t, tp, o, e, b, ch, cl = self.pp.get_state(next_state) 
            self.init_battery = int(b)
            self.init_charging = int(ch)
            
        for k in range(len(self.totalreward)):
            self.totalreward[k] = np.sum(self.obtained_rewards[k*self.no_int:(k+1)*self.no_int])
            print self.totalreward[k], ' total_reward, day',k+1
        print np.sum(self.totalreward), ' : Reward for Aug'

    def get_next_state(self,i):
        print i
        current_state = self.pp.initial_state
        actions = []
        while not(('gather_reward' in actions) or ('go_charge' in actions) or ('stay_charging' in actions)):
            nx_s, trans_prob, actions = self.pp.get_possible_next_states(current_state)
            print nx_s, trans_prob, actions
            if all(a == 'observe' for a in actions):
                for s in nx_s:
                    t, tp, o, e, b, ch, cl = self.pp.get_state(s)
                    if self.actual_reward[i] != 0 and tp == '1':
                        current_state = s
                    if self.actual_reward == 0 and tp == '0':
                        current_state = s 
                
            elif all(a == 'evaluate' for a in actions):
                min_cl = np.inf
                for s in nx_s:
                    t, tp, o, e, b, ch, cl = self.pp.get_state(s)
                    if abs(self.clusters[int(cl)] - self.actual_reward[i]) < min_cl:
                        current_state = s

            else:
                ct, ctp, co, ce, cb, cch, ccl = self.pp.get_state(current_state)
                self.charging.append(cch)
                self.battery.append(cb)
                    
                if all(a == 'stay_charging' for a in actions):
                    prob = []
                    for s in nx_s:
                        t, tp, o, e, b, ch, cl = self.pp.get_state(s)                  
                        prob.append(self.charge_model[int(cb)][int(b)])
                    current_state = np.random.choice(nx_s, p=np.array(prob))
                    req_a = actions[nx_s.index(current_state)]
                    self.obtained_rewards.append(0)

                elif all(a == 'go_charge' for a in actions):
                    prob = []
                    for s in nx_s:
                        t, tp, o, e, b, ch, cl = self.pp.get_state(s) 
                        if int(cb) == 100 or int(cb) == 99:                 
                            prob.append(self.charge_model[int(cb)][int(b)])
                        else:
                            prob.append(self.charge_model[int(cb)][int(b)+1])
                    current_state = np.random.choice(nx_s, p=np.array(prob))
                    req_a = actions[nx_s.index(current_state)]
                    self.obtained_rewards.append(0)
            
                elif all(a == 'gather_reward' for a in actions):
                    prob = []
                    for s in nx_s:
                        t, tp, o, e, b, ch, cl = self.pp.get_state(s)                  
                        prob.append(self.discharge_model[int(cb)][int(b)])
                    current_state = np.random.choice(nx_s, p=np.array(prob))
                    req_a = actions[nx_s.index(current_state)]
                    self.obtained_rewards.append(self.actual_reward[i])

                self.actions.append(req_a)
        
        return current_state


    def obtain_prism_model(self,t):
        prob_c = np.zeros((self.horizon, len(self.clusters)))
        for k in range(self.horizon):  
            prob_c[k] = self.prob[(t+k)%self.no_int]
        pm = prism_model.PrismModel('model_rhc.prism', self.init_battery, self.init_charging, self.clusters, prob_c, self.charge_model, self.discharge_model)
        #######################SPECIFY LOCATION ######################
        # running prism and saving output from prism
        with open(self.path_data+'result_rhc', 'w') as file:
            process = subprocess.Popen('./prism '+ self.path_mod + 'model_rhc.prism '+ self.path_mod +'model_prop.props -exportadv '+ self.path_mod+ 'model_rhc.adv -exportprodstates ' + self.path_mod +'model_rhc.sta -exporttarget '+self.path_mod+'model_rhc.lab',cwd='/home/milan/prism-svn/prism/bin', shell=True, stdout=subprocess.PIPE)
            for c in iter(lambda: process.stdout.read(1), ''):
                sys.stdout.write(c)
                file.write(c)
        ##reading output from prism to find policy file
        policy_file = []
        with open(self.path_data+'result_rhc', 'r') as f:
            line_list = f.readlines()
            for i in range(len(line_list)):
                if 'Computed point: ' in line_list[i]:
                    el = line_list[i].split(' ')
                    req_point = el[2][1:-1]
                    if abs(1.0-float(req_point)) < 0.001:
                        start_p = len('Adversary written to file "'+self.path_mod)
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
        pp = read_adversary.ParseAdversary([policy_file_name, 'model_rhc.sta', 'model_rhc.lab'])
        return pp

    def get_plan(self, fname):
        print 'Writing plan..'
        with open(self.path_data + fname, 'w') as f:
            f.write('time battery charging  action  obtained_reward match_reward actual_reward exp_reward\n')
            for t, b, ch, a, obr, mr, ar, er in zip(self.time, self.battery, self.charging, self.actions, self.obtained_rewards, self.sample_reward, self.actual_reward, self.exp_reward):
                f.write('{0} {1} {2} {3} {4} {5} {6} {7}\n'.format(t, b, ch, a, obr, mr, ar, er))

if __name__ == '__main__':
    fhc = RecedingHorizonControl(70, 1)
    fhc.simulate()
    fhc.get_plan('rhc_aug')