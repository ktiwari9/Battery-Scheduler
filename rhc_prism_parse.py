#!/usr/bin/env python

import numpy as np
import sys
import os
import rewards_uncertain1 

###### States are probabilistic, picking them randomly 

class parse_model:

    def __init__(self, filenames, clusters, prob):
        for name in filenames:
            if name[-4:] == '.lab':
                with open("/home/milan/catkin_ws/prism/"+name, 'r') as label_file:
                    self.labels =[]
                    for line in label_file.readlines():
                        if 'init' not in line:
                            self.labels.append(line[:-1].split(': '))
                        
            elif name[-4:] == '.sta':
                with open("/home/milan/catkin_ws/prism/"+name, 'r') as state_file:
                    self.states = dict()
                    for line in state_file.readlines():
                        if '_da' not in line or 'battery' not in line:
                            state = line[:-2].split(':(')
                            self.states.update({state[0] : (state[1].split(',')[1], state[1].split(',')[2], state[1].split(',')[3], state[1].split(',')[4], state[1].split(',')[5])})
    
            elif name[-4:] == '.adv':
                with open("/home/milan/catkin_ws/prism/"+name, 'r') as adv_file: 
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
            
        self.clusters = clusters    
        self.prob = prob
        self.initial_state = self.get_initial_state()
                 
    def get_initial_state(self):
        for element in self.labels:
            if int(element[1]) == 0:
                initial_state = element[0] 
        return initial_state
        
    def get_next_state(self, current_state):
        ns = self.policy[current_state]
        t = self.states[ns[0][0]][1]
        if len(ns) > 1:           #under action o
            ind = np.random.choice([0,1,2,3,4], p=[self.prob[int(t)][0],self.prob[int(t)][1],self.prob[int(t)][2],self.prob[int(t)][3],self.prob[int(t)][4]])
            next_i = ns[ind]
            reward = self.clusters[int(t)][ind]
            next = self.policy[next_i[0]][0]
        else:
            next = ns[0]
            
        if next[2] != 'gather_reward': #state_id, action, state values, reward, avilable reward 
            next_state = [next[0], next[2], self.states[next[0]], reward, self.clusters[int(t)]]
        else:
            next_state = [next[0], next[2], self.states[next[0]], reward, self.clusters[int(t)]]
             # action taken to reach the next[0] state.          
        return next_state 
        
    
 
