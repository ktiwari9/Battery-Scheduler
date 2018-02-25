#!/usr/bin/env python

import numpy as np
import sys
import os
import roslib
 

class parse_model:

    def __init__(self, filenames):
        # path = '/home/milan/workspace/strands_ws/src/battery_scheduler/models/'
        path = roslib.packages.get_pkg_dir('battery_scheduler_sim') + '/models/'
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
       
    def get_initial_state(self):
        for element in self.labels:
            if int(element[1]) == 0:
                initial_state = element[0] 
        return initial_state
        
    
    # when no observed state
    def get_next_state(self, current_state):
        possible_states = self.policy[current_state]
        next_state = possible_states[0][2]
        return next_state
       
            
            
            
    