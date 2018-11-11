#!/usr/bin/env python
from datetime import datetime, date, time, timedelta
import probabilistic_rewards
import pandas as pd
import numpy as np

class sample_generator:
    
    def __init__(self, validation, sampling_type=None):  ## sampling_type can be 'prob' or 'max' 
        ur = probabilistic_rewards.uncertain_rewards()
        self.clusters, self.prob = ur.get_probabilistic_reward_model()
        self.no_days = 1 # no. of days to be tested
        self.time_int = 48
        self.int_duration = 1440/self.time_int
        if validation == True:
            self.test_days = ur.test_days
            self.test_tasks = ur.tasks[ur.tasks['start_days'].isin(test_days)]
            self.no_days = len(test_days)

            ### expected data
            # self.test_rewards = []
            # with open('/home/milan/workspace/strands_ws/src/battery_scheduler/data/exp_rewards', 'r') as f:
            #     for rew in f:
            #         self.test_rewards.append(float(rew[:-1]))

        self.rewards, self.cl_ids, self.act_rewards, self.exp_rewards = self.get_samples(validation,sampling_type) 
  
    def get_samples(self, validation, sampling_type):
        rewards = (self.time_int*self.no_days)*[0]
        exp_rewards = (self.time_int*self.no_days)*[0]
        act_rewards = (self.time_int*self.no_days)*[0]
        cl_ids = (self.time_int*self.no_days)*[0]

        #####expected
        # rewards = len(self.test_rewards)*[0]
        # exp_rewards = len(self.test_rewards)*[0]
        # act_rewards = len(self.test_rewards)*[0]
        # cl_ids = len(self.test_rewards)*[0]
        
        if validation == False:
            cl_len = len(self.clusters)
            total_prob = self.prob
            for i in range(self.no_days-1):
               total_prob =  np.vtack(total_prob,self.prob)
            column_indices = [i for in range(cl_len)]
            for i in range(self.time_int*self.no_days):                
                ###### Sampling according to distribution
                if sampling_type == 'prob':
                    index = np.random.choice(column_indices,p=total_prob[i])
                    
                ###### Sampling the highest value available
                elif sampling_type ==  'max':
                    index = list(total_prob[i]).index(np.max(total_prob[i]))
 
                rewards[i] = self.clusters[index]
                cl_ids[i] = index
       
        elif validation == True:
            print 'Days available: ', len(self.no_days)
            test_rewards = np.zeros((self.no_days*self.time_int))
            for i in range(len(test_rewards)):
                day_i = i/self.time_int
                print day_i
                interval = i%self.time_int
                if interval == 0:
                    start_int = datetime.combine(self.test_days[day_i],time(0,0))
                else:
                    start_int = end_int
                end_int = start_int + timedelta(minutes=self.int_duration)

                sum_rew = self.test_tasks[self.test_tasks['start_time']<end_int & self.test_tasks['end_time']]








            total_reward = 0
            for day in self.test_rewards:
                print day
                print self.test_rewards[day]
            #    total_reward = total_reward + np.sum(self.test_rewards[day][16:42])
            #print 'ORIGINAL REWARDS ::', total_reward   
            ## Testing for 3 days 
            for i in range(0,self.no_days):
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


            ####exp rew
            # cl_no = 0 
            # for i in range(len(self.test_rewards)):
            #     if i == 48 or i==96:
            #         cl_no = 0
            #     cl_reward, index = self.closest_cluster(self.test_rewards[i], self.clusters[i%self.time_int])
            #     for k in range(len(self.clusters[i%self.time_int])):
            #         exp_rewards[i] = exp_rewards[i] + self.clusters[i%self.time_int][k]*self.prob[i%self.time_int][k]           
       
            #     act_rewards[i] = self.test_rewards[i]
            #     rewards[i] = cl_reward
            #     cl_ids[i] = cl_no +index
            #     cl_no = cl_no+len(self.clusters[i%self.time_int])

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
