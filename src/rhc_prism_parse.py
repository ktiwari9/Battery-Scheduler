#!/usr/bin/env python

import numpy as np
import sys
import os
import roslib
 

class parse_model:

    def __init__(self, filenames, current_time, sample_reward, clusters):
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
        self.sample_reward = sample_reward
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
        t_next = self.current_t+1

        if t_next < len(self.sample_reward):
            matched_reward = self.sample_reward[t_next]  # the reward determined by the cluster for next time int

            cl_no = len(self.clusters[0])
            for i in range(len(self.clusters[1])):
                #print matched_reward, float(self.clusters[1][i])
                #print type(matched_reward), type(self.clusters[1][i])
                if round(matched_reward) == round(self.clusters[1][i]):
                    req_id = cl_no + i
                    next_id = i
                    break
            
            next_state_list = dict()
            for ns_p_a in possible_states:
                next_cl_id = int(self.states[ns_p_a[0]][3])
                #print next_cl_id, req_id
                if next_cl_id == req_id:
                    next_state = [self.states[ns_p_a[0]], next_id, ns_p_a[2]]   # state[batter, charging, time, cluster], action to get to this state
                    next_state_list.update({ ns_p_a[1] : next_state})           # rewards that could be achieved in this state
            
            max_prob = max(next_state_list.keys())
            return next_state_list[max_prob]
                
        else:
            next_id = max(self.states[ns_p_a[0]][3] for ns_p_a in possible_states )
            next_state_list = dict()
            for ns_p_a in possible_states:
                next_state = [self.states[ns_p_a[0]], next_id, ns_p_a[2]]
                next_state_list.update({ ns_p_a[1] : next_state})
                
            max_prob = max(next_state_list.keys())
            return next_state_list[max_prob]
            
            
            
    