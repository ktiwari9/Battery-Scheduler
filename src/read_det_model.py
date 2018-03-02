#!/usr/bin/env python

import numpy as np
import sys
import os
import roslib
import plotly
import plotly
import plotly.plotly as py
import plotly.graph_objs as go


class parse_model:

    def __init__(self, filenames, actual_reward, exp_reward):
        #######################SPECIFY LOCATION ######################

        path = '/home/milan/workspace/strands_ws/src/battery_scheduler/models/'
        #path = roslib.packages.get_pkg_dir('battery_scheduler') + '/models/'
        for name in filenames:
            if name[-4:] == '.lab':
                with open(path+name, 'r') as label_file:
                    self.labels =[]
                    for line in label_file.readlines():
                        if 'init' not in line:
                            self.labels.append(line[:-1].split(': '))
                        
            elif name[-4:] == '.sta':
                with open(path+name, 'r') as state_file:
                    self.states = dict()
                    for line in state_file.readlines():
                        if '_da' not in line or 'battery' not in line:
                            state = line[:-2].split(':(')
                            s = state[1].split(',')
                            self.states.update({state[0] : (s[1], s[2], s[3], s[4])})
    
            elif name[-4:] == '.adv':
                with open(path+name, 'r') as adv_file: 
                    self.policy = dict()
                    for line in adv_file:
                        array = line[:-1].split(' ')
                        if len(array) > 2:
                            if array[0] not in self.policy:
                                l = []                
                            else:
                                l = self.policy[array[0]]
                            l.append(array[1:])
                            self.policy.update({array[0] : l})
            
        self.initial_state = self.get_initial_state()
        self.exp_reward = exp_reward
        self.actual_reward = actual_reward
        self.time_int = 48
        
    def get_initial_state(self):
        for element in self.labels:
            if int(element[1]) == 0:
                initial_state = element[0] 
        return initial_state
        
    
    # when no observed state
    def get_next_state(self, current_state):
        c_t = self.states[current_state][0]
        ns_p_a = self.policy[current_state]
        next_state = [ns_p_a[0][0], ns_p_a[0][2], self.actual_reward[int(c_t)], self.exp_reward[int(c_t)]]
        return  next_state   
            
    def simulate(self, name):
        in_state = self.get_initial_state()
        rewards = []
        states_plan = []
        action = []
        #######################SPECIFY LOCATION ######################
        
        path = '/home/milan/workspace/strands_ws/src/battery_scheduler/data/'
        #path = roslib.packages.get_pkg_dir('battery_scheduler') + '/data/'
        with open(path + name, 'w') as f:
            f.write('time battery charging  action  actual_reward exp_reward \n')  
            for k in range(self.time_int):
                f.write('{0} {1} {2} '.format(self.states[in_state][0], self.states[in_state][1], self.states[in_state][2]))
                next_state = self.get_next_state(in_state)
                rewards.append(float(next_state[2]))
                action.append(next_state[1])
                f.write('{0} {1} {2} \n'.format(next_state[1], next_state[2], next_state[3]))
                in_state = next_state[0]
        
if __name__ == '__main__':
    sample_reward = []
    actual_reward = []
    exp_reward = []
    
    #main_path = roslib.packages.get_pkg_dir('battery_scheduler')
    #######################SPECIFY LOCATION ######################
    main_path = '/home/milan/workspace/strands_ws/src/battery_scheduler'
    path_rew = main_path + '/data/sample_rewards'
    path_mod = main_path + '/models/'
    path_data = main_path + '/data/'
    
    with open(path_rew,'r') as f:
        for line in f:
            actual_reward.append(float(line.split(' ')[2]))
            exp_reward.append(float(line.split(' ')[3]))        
    
    pp = parse_model(['rrhcpre1.adv', 'rrhc.sta', 'rrhc.lab'], actual_reward, exp_reward )
    pp.simulate('real_dec16_det')


    action = []
    exp_reward = []
    actual_reward = []
    battery = []
    time = []
    data_path = '/home/milan/workspace/strands_ws/src/battery_scheduler/data/'
    # for without RHC - do not activate both at same time
    # t = 0;
    # for i in range(no_days):
    #     with open(data_path + 'wrhc_sep_pbr'+ str(i), 'r') as f:
    #         for line in f.readlines():
    #             if 'time' not in line:
    #                 s = line.split(' ')[:-1]
    #                 time.append(t)
    #                 t = t+1
    #                 battery.append(int(s[1]))
    #                 matched_reward.append(float(s[4]))
    #                 exp_reward.append(float(s[6]))
    #                 actual_reward.append(float(s[5]))
    #                 action.append(s[3])


    # ####with RHC - do not activate both at same time
    # with open(data_path+'det_sep13', 'r') as f:
    #     for line in f.readlines():
    #         if 'time' not in line:
    #             s = line.split(' ')[:-1]
    #             time.append(int(s[0]))
    #             battery.append(int(s[1]))
    #             exp_reward.append(float(s[5]))
    #             actual_reward.append(float(s[4]))
    #             action.append(s[3])
                
        
        
 #    color1 = []
 #    for a in action:
 #        if a == 'gather_reward':
 #            color1.append('rgba(27,117,20,1)')
 #        elif a == 'go_charge':
 #            color1.append('rgba(222,16,16,1)')
 #        elif a == 'stay_charging':
 #            color1.append('rgba(236, 153, 28,1)')
            
            
    
 #    plotly.tools.set_credentials_file(username='MilanMariyaTomy', api_key='wSfqCQnChPdSCqIMGPdp')#username='ThanwiraSiraj', api_key= 'y9AlaR5JI6kYeCml1NG4') 
 #    data = [go.Bar( x= time, y = actual_reward, marker=dict(color=color1)), go.Scatter(x=time, y= battery), go.Scatter( x= time, y = exp_reward)]
    
 #    fig = go.Figure(data = data)
 #    py.plot(fig, filename='det_sep13')
 # 