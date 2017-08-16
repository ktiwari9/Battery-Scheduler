#!/usr/bin/env python

import numpy as np
import sys
import os
 

class parse_model:

    def __init__(self, filenames, cl_id, sample_reward):
        for name in filenames:
            if name[-4:] == '.lab':
                with open("/localhome/strands/milan_ws/prism/"+name, 'r') as label_file:
                    self.labels =[]
                    for line in label_file.readlines():
                        if 'init' not in line:
                            self.labels.append(line[:-1].split(': '))
                        
            elif name[-4:] == '.sta':
                with open("/localhome/strands/milan_ws/prism/"+name, 'r') as state_file:
                    self.states = dict()
                    for line in state_file.readlines():
                    
                        if '_da' not in line and 'battery' not in line:
                            state = line[:-2].split(':(')
                            self.states.update({state[0] : (state[1].split(',')[1], state[1].split(',')[2], state[1].split(',')[3], state[1].split(',')[4], state[1].split(',')[5])})
    
            elif name[-4:] == '.adv':
                with open("/localhome/strands/milan_ws/prism/"+name, 'r') as adv_file: 
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
            
        self.cl_id = cl_id    
        self.sample_reward = sample_reward
        self.initial_state = self.get_initial_state()
       
                    
    def get_initial_state(self):
        for element in self.labels:
            if int(element[1]) == 0:
                initial_state = element[0] 
        return initial_state
        
    ### only possible when observed followed by an action with outcome of probability 1 and starting with an observed 0
    def get_next_state(self, current_state):
        ns = self.policy[current_state]
        if len(ns) > 1:
            for z in range(len(ns)):
                if self.cl_id == int(self.states[ns[z][0]][3]):
                    index  = z  
                    break 
                             
            reward = self.sample_reward
            next_state_id = self.policy[ns[index][0]][0][0]
            action_to_next_state = self.policy[ns[index][0]][0][2]
        
        else:
            if ns[0][2] != 'o':
                next_state_id = ns[0][0]    
                action_to_next_state = ns[0][2]
            else:
                reward = self.sample_reward
                next_state_id = self.policy[ns[index][0]][0][0]
                action_to_next_state = self.policy[ns[index][0]][0][2]
        
        next_state = [next_state_id, action_to_next_state, self.states[next_state_id], reward]
        return next_state
    
        
    
 