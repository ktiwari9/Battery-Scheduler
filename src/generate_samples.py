#!/usr/bin/env python

import rewards_dbscan
import roslib
import numpy as np

class sample_generator:
    
    def __init__(self, validation, sampling_type=None):  ## sampling_type can be 'prob' or 'max' 
        ur = rewards_dbscan.uncertain_rewards(validation)
        self.clusters, self.prob = ur.get_rewards()
        self.len_test_rewards = 1 # no. of days to be tested
        self.time_int = 5
        if validation == True:
            self.test_rewards = ur.test_rewards
            self.len_test_rewards = len(self.test_rewards)

        self.rewards, self.cl_ids, self.act_rewards, self.exp_rewards = self.get_samples(validation,sampling_type) 
  
    def get_samples(self, validation, sampling_type):
        rewards = (self.time_int*self.len_test_rewards)*[0]
        exp_rewards = (self.time_int*self.len_test_rewards)*[0]
        act_rewards = (self.time_int*self.len_test_rewards)*[0]
        cl_ids = (self.time_int*self.len_test_rewards)*[0]
        
        if validation == False:
            for i in range(self.time_int*self.len_test_rewards):
                cl_len = len(self.clusters[i%self.time_int])
                c = cl_len*[0]
                p = cl_len*[0]
                for j in range(cl_len):
                    c[j] = self.clusters[i%self.time_int][j]
                    p[j] = self.prob[i%self.time_int][j]
                ###### Sampling according to distribution
                if sampling_type == 'prob':
                    indices = []
                    for i in range(len(p)):
                        indices.append(i)
                    index = np.random.choice(indices,p=p)
                    
                ###### Sampling the highest value available
                elif sampling_type ==  'max':
                    max_cl = 0
                    for k in range(cl_len):
                        if c[k] > max_cl:
                            max_cl = c[k]
                            index = k     
                
                rewards[i] = c[index]
                cl_ids[i] = (i%self.time_int)*5+index 
       
        elif validation == True:
            print 'Days available: ', len(self.test_rewards)
            total_reward = 0
            for day in self.test_rewards:
                print day
                print self.test_rewards[day]
            #    total_reward = total_reward + np.sum(self.test_rewards[day][16:42])
            #print 'ORIGINAL REWARDS ::', total_reward   
            ## Testing for 3 days 
            for i in range(0,self.len_test_rewards):
                cl_no = 0
                for j in range(self.time_int):
                    #print i, '-> ', j 
                    reward = self.test_rewards.values()[i][j]
                    #print 'actual_reward', reward
                    #print self.clusters[j]
                    cl_reward, index = self.closest_cluster(reward, self.clusters[j])
                    for k in range(len(self.clusters[j])):
                        exp_rewards[i*self.time_int+j] = exp_rewards[i*self.time_int+j] + self.clusters[j][k]*self.prob[j][k]
                        
       
                    act_rewards[i*self.time_int+j] = reward
                    rewards[i*self.time_int+j] = cl_reward
                    cl_ids[i*self.time_int+j] = cl_no +index
                    cl_no = cl_no+len(self.clusters[j])
        return rewards, cl_ids, act_rewards, exp_rewards

    def closest_cluster(self, reward, clusters):
        minimum = float('inf')
        for i in range(len(clusters)):
            if abs(reward - clusters[i]) < minimum:
                cl_id = i
                cl_reward = clusters[i]
                minimum = abs(reward-clusters[i])
        return cl_reward, cl_id
        
          
if __name__ == '__main__':
    sg = sample_generator(True)     
    rewards = sg.rewards
    cl_id = sg.cl_ids
    act_rewards = sg.act_rewards
    exp_rewards = sg.exp_rewards
    #path = roslib.packages.get_pkg_dir('battery_scheduler') + '/data/sample_rewards'
    path = '/home/milan/workspace/strands_ws/src/battery_scheduler/data/sample_rewards'
    with open(path,'w') as f:
        for r, c, a_r, e_r in zip(rewards, cl_id, act_rewards, exp_rewards):
            f.write('{0} {1} {2} {3} '.format(c, r, a_r, e_r))
            f.write('\n')
