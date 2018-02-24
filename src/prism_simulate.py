#!/usr/bin/env python

import numpy as np
import sys
import os
import roslib
 

class parse_model:

    def __init__(self, filenames, cl_id, actual_reward, sample_reward, exp_reward, day, clusters, probs):
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
        self.cl_id = cl_id
        self.exp_reward = exp_reward
        self.sample_reward = sample_reward
        self.actual_reward = actual_reward
        self.day = day
        self.clusters = clusters
        self.probs = probs
        self.time_int = 48
        
    def get_initial_state(self):
        for element in self.labels:
            if int(element[1]) == 0:
                initial_state = element[0] 
        return initial_state
        
    
    # when no observed state
    def get_next_state(self, current_state):
        possible_states = self.policy[current_state]
        t_current = int(self.states[current_state][2])
        matched_reward = self.sample_reward[self.day*self.time_int+t_current]  # the reward determined by the cluster (not much of use)
        exp_reward = self.exp_reward[self.day*self.time_int+t_current]         # expected probability
        act_reward = self.actual_reward[self.day*self.time_int+t_current]      # actual reward on completing task
        cluster_group = self.clusters[(self.day*self.time_int+t_current)%self.time_int]
        prob_group = self.probs[(self.day*self.time_int+t_current)%self.time_int]

        i = 0
        next_state_list = dict()
        for ns_p_a in possible_states:
            t_next = int(self.states[ns_p_a[0]][2])
            #print t_next, 't_next'

            if self.day*self.time_int+t_next >= ((self.day+1)*self.time_int): #return the only possibility for the last case
                next_state = [ns_p_a[0], ns_p_a[2], act_reward, matched_reward, exp_reward, prob_group, cluster_group]   # state_id, action to get to this state
                next_state_list.update({ float(ns_p_a[1]) : next_state})

            else:
                req_id = int(self.cl_id[self.day*self.time_int+t_next]) 
                next_cl_prob = round(self.probs[self.day*self.time_int+t_next][i%len(self.probs[self.day*self.time_int+t_next])], 7)
                # print req_id, int(self.states[ns_p_a[0]][3])
            
                if int(self.states[ns_p_a[0]][3]) == req_id :#and round(self.sample_reward[self.day*self.time_int+t_next]) == round(self.clusters[self.day*self.time_int+t_next][i%len(self.clusters[self.day*self.time_int+t_next])]):
                    next_state = [ns_p_a[0], ns_p_a[2], act_reward, matched_reward, exp_reward, prob_group, cluster_group]   # state_id, action to get to this state
                    next_state_list.update({round(float(ns_p_a[1])/next_cl_prob, 7) : next_state})
        
            i = i+1

        prob = map(lambda x: x/sum(next_state_list.keys()), next_state_list.keys())
        req_prob = np.random.choice(np.array(next_state_list.keys()), p=prob)
        return next_state_list[req_prob]
            
    def simulate(self, day, name):
        in_state = self.get_initial_state()
        rewards = []
        states_plan = []
        action = []
        path = '/home/milan/workspace/strands_ws/src/battery_scheduler/data/'
        #path = roslib.packages.get_pkg_dir('battery_scheduler') + '/data/'
        with open(path + name + str(day), 'w') as f:
            f.write('time battery charging  action  match_reward actual_reward exp_reward clusters probs\n')  
            for k in range(self.time_int):
                f.write('{0} {1} {2} '.format(self.states[in_state][2], self.states[in_state][0], self.states[in_state][1]))
                next_state = self.get_next_state(in_state)
                rewards.append(float(next_state[2]))
                action.append(next_state[1])
                f.write('{0} {1} {2} {3} '.format(next_state[1], next_state[3], next_state[2], next_state[4]))
                for c in next_state[6]:
                    f.write('{0} '.format(c))
                for p in next_state[5]:
                    f.write('{0} '.format(p))
                f.write('\n')
                in_state = next_state[0]
        final_state = self.states[in_state]
        return rewards, action, final_state
 
