#!/usr/bin/env python

import numpy as np
import sys
import os
import roslib
 

###### States are probabilistic, picking them randomly 

class parse_model:

    def __init__(self, filenames, current time, cl_id, actual_reward, sample_reward, clusters):
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
                            self.states.update({state[0] : (state[1].split(',')[1], state[1].split(',')[2], state[1].split(',')[3], state[1].split(',')[4])})
    
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
        self.cl_id = cl_id
        self.sample_reward = sample_reward
        self.actual_reward = actual_reward
        self.current_t = current_time
        self.clusters = clusters
        
        
    def get_initial_state(self):
        for element in self.labels:
            if int(element[1]) == 0:
                initial_state = element[0] 
        return initial_state
        
    
    # when no observed state
    def get_next_state(self, current_state):
        possible_states = self.policy[current_state]
        matched_reward = self.sample_reward[self.current_t]  # the reward determined by the cluster (not much of use)
        exp_reward = self.exp_reward[self.current_t]         # expected probability
        act_reward = self.actual_reward[self.current_t]      # actual reward on completing task
        t_next = self.current_t+1
        next_cl_id = 0
        for ns_p_a in possible_states:
            cl_no = 0
            for i in range(self.current_t):
                cl_no = cl_no + len(self.clusters[i])
            req_id = int(self.cl_id[t_next])
  
            if (cl_no+int(self.states[ns_p_a[0]][3])) == req_id:
                next_state = [self.states[ns_p_a[0]], next_cl_id, ns_p_a[2], act_reward, matched_reward, exp_reward]   # state_id, cluster id (set for initial cluster), action to get to this state
                return next_state
            
            next_cl_id = next_cl_id + 1
            
    