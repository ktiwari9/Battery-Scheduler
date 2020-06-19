#! /usr/bin/env python
import probabilistic_rewards
from datetime import datetime, date
import bc_read_adversary
import bcth_prism_model
import generate_samples
import numpy as np
import subprocess
import pymongo
import roslib
import copy
import yaml
import sys
import os


def get_battery_model():
    # path = '/home/milan/workspace/strands_ws/src/battery_scheduler'
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
        
        gocharge_model = dict ()
        for b, bdict in charge_model.items():
            g_bdict = dict()
            if b == 99 or b == 100:
                g_bdict = copy.deepcopy(bdict)
            else:
                for bn, count in bdict.items():
                    gbn = round(0.99*bn)
                    if gbn > b:
                        if gbn in g_bdict:
                            g_bdict[gbn] += count
                        else:
                            g_bdict.update({gbn : count})
                    else:
                        if bn in g_bdict:
                            g_bdict[bn] += count
                        else:
                            g_bdict.update({bn : count})
            gocharge_model.update({b:g_bdict})

        return charge_model, discharge_model, gocharge_model
    else:
        raise ValueError('No models found. First create battery model with probabilistic_battery_model.py')
 

class RecedingHorizonControl:
    
    def __init__(self, init_battery, init_charging, test_days, pareto_point):
        ur = probabilistic_rewards.uncertain_rewards(test_days)
        self.task_prob, self.prob, self.clusters = ur.get_probabilistic_reward_model()
        self.charge_model, self.discharge_model, self.gocharge_model = get_battery_model()
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
            print self.prob[z%self.no_int], self.clusters, sum(self.prob[z%self.no_int]*self.clusters)
        self.req_pareto_point = pareto_point
   
        self.main_path = roslib.packages.get_pkg_dir('battery_scheduler')
        self.path_mod = self.main_path + '/models/'
        self.path_data = self.main_path + '/data/'
    
        sg = generate_samples.sample_generator(True, test_days) 
        self.cl_id = []
        for clid in sg.cl_ids:
            if np.isnan(clid):
                cl = None
            else:
                cl = int(clid)
            self.cl_id.append(cl)
        self.sample_reward = sg.rewards
        self.actual_reward = sg.act_rewards

        self.totalreward = np.zeros((self.no_days))
        self.init_battery = init_battery
        self.init_charging = init_charging
        self.actions = []
        self.obtained_rewards = []
        self.battery = []
        self.charging = []
        self.time =[]
        self.pareto_point = []

        self.simulate()
     
    def simulate(self):
        print 'Simulating...'
        for k in range(self.no_days*self.no_int):
            print k, '----------------------------------------------------------------------------------'
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
        # print i
        current_state = self.pp.initial_state
        actions = []
        while not(('gather_reward' in actions) or ('go_charge' in actions) or ('stay_charging' in actions)):
            nx_s, trans_prob, actions = self.pp.get_possible_next_states(current_state)
            # print nx_s, trans_prob, actions
            if all(a == 'observe' for a in actions):
                if  len(nx_s) == 1:  ### only 1 state possible. 
                    current_state = nx_s[0]
                else:
                    for s in nx_s:
                        t, tp, o, e, b, ch, cl = self.pp.get_state(s)
                        if self.actual_reward[i] != 0 and tp == '1':
                            current_state = s
                        elif self.actual_reward[i] == 0 and tp == '0':
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
                    next_b = []
                    for s in nx_s:
                        t, tp, o, e, b, ch, cl = self.pp.get_state(s)                  
                        prob.append(self.charge_model[int(cb)][int(b)])
                        next_b.append(int(b))

                    current_state = np.random.choice(nx_s, p=np.array(prob))                   
                    req_a = actions[nx_s.index(current_state)]
                    self.obtained_rewards.append(0)

                elif all(a == 'go_charge' for a in actions):
                    prob = []
                    next_b = []
                    for s in nx_s:
                        t, tp, o, e, b, ch, cl = self.pp.get_state(s)         
                        prob.append(self.gocharge_model[int(cb)][int(b)])
                        next_b.append(int(b))
                    
                    current_state = np.random.choice(nx_s, p=np.array(prob))

                    req_a = actions[nx_s.index(current_state)]
                    self.obtained_rewards.append(0)
            
                elif all(a == 'gather_reward' for a in actions):
                    prob = []
                    next_b = []
                    for s in nx_s:
                        t, tp, o, e, b, ch, cl = self.pp.get_state(s)                  
                        prob.append(self.discharge_model[int(cb)][int(b)])
                        next_b.append(int(b))

                    current_state = np.random.choice(nx_s, p=np.array(prob))

                    req_a = actions[nx_s.index(current_state)]
                    self.obtained_rewards.append(self.actual_reward[i])

                self.actions.append(req_a)
        
        return current_state

    def obtain_prism_model(self,t):
        prob_c = np.zeros((self.horizon, len(self.clusters)))
        prob_t = np.zeros((self.horizon, 2))
        for k in range(self.horizon):  
            prob_c[k] = self.prob[(t+k)%self.no_int]
            prob_t[k] = self.task_prob[(t+k)%self.no_int]
        
        pm = bcth_prism_model.PrismModel('model_rhc.prism', self.horizon, self.init_battery, self.init_charging, self.cl_id[t], prob_t, self.clusters, prob_c, self.charge_model, self.discharge_model)
       
        #######################SPECIFY LOCATION ######################
        # running prism and saving output from prism
        with open(self.path_data+'result_rhc', 'w') as f:
            process = subprocess.call('./prism '+ self.path_mod + 'model_rhc.prism '+ self.path_mod +'batterycost_model_prop.props -paretoepsilon 0.1 -v -exportadv '+ self.path_mod+ 'model_rhc.adv -exportprodstates ' + self.path_mod +'model_rhc.sta -exporttarget '+self.path_mod+'model_rhc.lab',cwd='/home/milan/prism/prism/bin', shell=True, stdout=f)
        
        ##reading output from prism to find policy file
        ### for bcth
        policy_file = []
        pre1_point = None
        pre2_point = None
        with open(self.path_data+'result_rhc', 'r') as f:
            line_list = f.readlines()
            f_no_list = []
            pareto_points = []
            init = 0
            for e, line in enumerate(line_list):
                if 'pre1.adv' in line:
                    pre1_point = abs(float(line_list[e+1].split(',')[0].split('(')[1].strip()))

                if 'pre2.adv' in line:
                    pre2_point = abs(float(line_list[e+1].split(',')[0].split('(')[1].strip())) 

                if ': New point is (' in line:
                    el = line.split(' ')
                    if init == 0:
                        init_no = int(el[0][:-1])
                    cost40 = abs(float(el[4][1:-1]))
                    pareto_points.append(cost40)
                    f_no_list.append(str(int(el[0][:-1])-2))
                    init +=1 
               
        if 'pre1' == self.req_pareto_point or 'pre2' == self.req_pareto_point:
            f_no = self.req_pareto_point
            if self.req_pareto_point == 'pre1':
                self.pareto_point.append(pre1_point)
            elif self.req_pareto_point == 'pre2':
                self.pareto_point.append(pre2_point)
        else:
            if f_no_list:
                if self.req_pareto_point > 3 and self.req_pareto_point < 6:
                    approx_p_point = min(pareto_points) + ((max(pareto_points)-min(pareto_points))/3)*(float((self.req_pareto_point%3))/3)
                elif self.req_pareto_point == 6:
                    sorted_pareto_points = sorted(pareto_points)
                    if len(sorted_pareto_points) > 1:
                        approx_p_point = sorted_pareto_points[1]
                    else:
                        approx_p_point =  sorted_pareto_points[0]
                else:
                   approx_p_point = min(pareto_points) + ((max(pareto_points)-min(pareto_points)))*(float(self.req_pareto_point)/3) ## 3 -> no. of pareto points being considered
                p_point = min(pareto_points, key=lambda x: abs(x-approx_p_point))
                self.pareto_point.append(p_point)
                f_ind = pareto_points.index(p_point)
                f_no = f_no_list[f_ind]
            else:
                f_no = None 
        
        if f_no != None:
            print 'Reading from model_rhc'+f_no+'.adv'
            pp = bc_read_adversary.ParseAdversary(['model_rhc'+f_no+'.adv', 'model_rhc.sta', 'model_rhc.lab'])
            return pp
        else:
            raise ValueError('Adversary Not Found !!!')

    def get_plan(self, fname):
        if 'pre1' == self.req_pareto_point or 'pre2' == self.req_pareto_point:
            plan_path = self.path_data + self.req_pareto_point+ fname
        else:
            plan_path = self.path_data + 'p'+ str(self.req_pareto_point)+ fname
        print 'Writing plan to ', plan_path, ' ...'
        with open(plan_path, 'w') as f:
            f.write('time battery charging action obtained_reward match_reward actual_reward exp_reward pareto\n')
            for t, b, ch, a, obr, mr, ar, er, pp in zip(self.time, self.battery, self.charging, self.actions, self.obtained_rewards, self.sample_reward, self.actual_reward, self.exp_reward, self.pareto_point):
                f.write('{0} {1} {2} {3} {4} {5} {6} {7} {8}\n'.format(t, b, ch, a, obr, mr, ar, er, pp))


if __name__ == '__main__':
   
    np.random.seed(0)
    rhc = RecedingHorizonControl(70, 1, [datetime(2017,10,20)], 0)
    rhc.get_plan('rhctest_2010_1')

    # np.random.seed(1)
    # rhc = RecedingHorizonControl(70, 1, [datetime(2017,10,10), datetime(2017,10,11), datetime(2017,10,12)], 3)
    # rhc.get_plan('rhc_101011101210_2')

    # np.random.seed(2)
    # rhc = RecedingHorizonControl(70, 1, [datetime(2017,10,10), datetime(2017,10,11), datetime(2017,10,12)], 3)
    # rhc.get_plan('rhc_101011101210_3')
    
    # rhc = RecedingHorizonControl(70, 1, [datetime(2017,10,19), datetime(2017,10,20), datetime(2017,10,21)], 3)
    # rhc.get_plan('rhc_191020102110_1')

    # np.random.seed(1)
    # rhc = RecedingHorizonControl(70, 1, [datetime(2017,10,19), datetime(2017,10,20), datetime(2017,10,21)], 3)
    # rhc.get_plan('rhc_191020102110_2')

    # np.random.seed(2)
    # rhc = RecedingHorizonControl(70, 1, [datetime(2017,10,19), datetime(2017,10,20), datetime(2017,10,21)], 3)
    # rhc.get_plan('rhc_191020102110_3')

    # rhc = RecedingHorizonControl(70, 1,[datetime(2017,10,29), datetime(2017,10,30), datetime(2017,10,31)], 3)
    # rhc.get_plan('rhc_291030103110_1')

    # np.random.seed(1)
    # rhc = RecedingHorizonControl(70, 1, [datetime(2017,10,29), datetime(2017,10,30), datetime(2017,10,31)], 3)
    # rhc.get_plan('rhc_291030103110_2')

    # np.random.seed(2)
    # rhc = RecedingHorizonControl(70, 1, [datetime(2017,10,29), datetime(2017,10,30), datetime(2017,10,31)], 3)
    # rhc.get_plan('rhc_291030103110_3')

    # rhc = RecedingHorizonControl(70, 1, [datetime(2017,11,4), datetime(2017,11,5), datetime(2017,11,6)], 3)
    # rhc.get_plan('rhc_411511611_1')

    # np.random.seed(1)
    # rhc = RecedingHorizonControl(70, 1, [datetime(2017,11,4), datetime(2017,11,5), datetime(2017,11,6)], 3)
    # rhc.get_plan('rhc_411511611_2')

    # np.random.seed(2)
    # rhc = RecedingHorizonControl(70, 1, [datetime(2017,11,4), datetime(2017,11,5), datetime(2017,11,6)], 3)
    # rhc.get_plan('rhc_411511611_3')

    # rhc = RecedingHorizonControl(70, 1, [datetime(2017,11,12), datetime(2017,11,13), datetime(2017,11,14)], 3)
    # rhc.get_plan('rhc_121113111411_1')

    # np.random.seed(1)
    # rhc = RecedingHorizonControl(70, 1, [datetime(2017,11,12), datetime(2017,11,13), datetime(2017,11,14)], 3)
    # rhc.get_plan('rhc_121113111411_2')

    # np.random.seed(2)
    # rhc = RecedingHorizonControl(70, 1, [datetime(2017,11,12), datetime(2017,11,13), datetime(2017,11,14)], 3)
    # rhc.get_plan('rhc_121113111411_3')

    # rhc = RecedingHorizonControl(70, 1, [datetime(2017,12,1), datetime(2017,12,2), datetime(2017,12,3)], 3)
    # rhc.get_plan('rhc_112212312_1')

    # np.random.seed(1)
    # rhc = RecedingHorizonControl(70, 1, [datetime(2017,12,1), datetime(2017,12,2), datetime(2017,12,3)], 3)
    # rhc.get_plan('rhc_112212312_2')

    # np.random.seed(2)
    # rhc = RecedingHorizonControl(70, 1, [datetime(2017,12,1), datetime(2017,12,2), datetime(2017,12,3)], 3)
    # rhc.get_plan('rhc_112212312_3')
   
   