#!/usr/bin/env python

import roslib

class PolicyError(Exception):
    pass


class ParseAdversary:
    def __init__(self, filenames):
        main_path = roslib.packages.get_pkg_dir('battery_scheduler')
        path = main_path+'/models/'
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
                        if 't' not in line and 'battery' not in line:
                            state = line[:-2].split(':(')
                            s = state[1].split(',')
                            self.states.update({state[0] : [el.strip() for el in s]})
            elif name[-4:] == '.adv':
                with open(path+name, 'r') as adv_file: 
                    self.policy = dict()
                    for line in adv_file:
                        array = line[:-1].split(' ')
                        if len(array) > 2:
                            if array[0] not in self.policy:
                                self.policy.update({array[0] : [array[1:]]})              
                            else:
                                self.policy[array[0]].append(array[1:])      
        self.initial_state = self.get_initial_state()

    def get_state(self, current_state):
        d = self.states[current_state]
        return d[0], d[1], d[2], d[3], d[4], d[5], d[6]
     
    def get_initial_state(self):
        for element in self.labels:
            if int(element[1]) == 0:
                return element[0]

    def get_possible_next_states(self,state):
        next_states = [l[0] for l in self.policy[state]]
        trans_prob = [l[1] for l in self.policy[state]]
        actions = [l[2] for l in self.policy[state]]
        return next_states, trans_prob, actions