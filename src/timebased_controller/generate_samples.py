#!/usr/bin/env python
from datetime import datetime, date, time, timedelta
import probabilistic_rewards
import pandas as pd
import numpy as np

class sample_generator:
    
    def __init__(self, validation, test_days, sampling_type=None):  ## sampling_type can be 'prob' or 'max' 
        ur = probabilistic_rewards.uncertain_rewards(test_days)
        task_prob, self.prob, self.clusters = ur.get_probabilistic_reward_model()
        self.time_int = ur.no_int
        self.int_duration = ur.time_int
        if validation == True:
            self.test_days = ur.test_days
            self.test_tasks = ur.test_tasks
            print (self.test_tasks)
            self.no_days = len(self.test_days)

            ### expected data
            # self.test_rewards = []
            # with open('/home/milan/workspace/strands_ws/src/battery_scheduler/data/exp_rewards', 'r') as f:
            #     for rew in f:
            #         self.test_rewards.append(float(rew[:-1]))

        else:
            self.no_days = 1 # no. of days to be tested, if not using validation

        self.rewards, self.cl_ids, self.act_rewards = self.get_samples(validation,sampling_type) 
  
    def get_samples(self, validation, sampling_type):
        print 'Generating samples...'
        rewards =  np.zeros((self.no_days*self.time_int))
        # exp_rewards =  np.zeros((self.no_days*self.time_int))
        act_rewards =  np.zeros((self.no_days*self.time_int))
        cl_ids =  np.zeros((self.no_days*self.time_int))

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
            column_indices = [i for i in range(cl_len)]
            for i in range(self.time_int*self.no_days):                
                ###### Sampling according to distribution
                if sampling_type == 'prob':
                    index = np.random.choice(column_indices,p=total_prob[i])
                    
                ###### Sampling the highest value available
                elif sampling_type ==  'max':
                    index = list(total_prob[i]).index(np.max(total_prob[i]))
 
                rewards[i] = self.clusters[index]  ## In this case actual rewards = rewards (matched)
                cl_ids[i] = index
       
        elif validation == True:
            print 'Days available: ', self.no_days
            for i in range(len(rewards)):
                day_i = i/self.time_int
                interval = i%self.time_int
                if interval == 0:
                    start_int = datetime.combine(self.test_days[day_i],time(0,0))
                else:
                    start_int = end_int
                end_int = start_int + timedelta(minutes=self.int_duration)

                act_rewards[i] = self.test_tasks[(self.test_tasks['start_time']<end_int) & (self.test_tasks['end_time']>start_int)]['priority'].sum()
                rewards[i], cl_ids[i] = self.closest_cluster(act_rewards[i])

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

        return rewards, cl_ids, act_rewards #, exp_rewards

    def closest_cluster(self, reward):
        if reward != 0:
            minimum = float('inf')
            for e,cl in enumerate(self.clusters):
                diff = abs(reward - cl) 
                if diff  < minimum:
                    cl_id = e
                    cl_reward = cl
                    minimum = diff
                
            return cl_reward, int(cl_id)

        else:
            return 0, None
        

if __name__ == '__main__':
    sg = sample_generator(True, [datetime(2017, 12, 1), datetime(2017, 12, 2), datetime(2017, 12, 3)])     
    rewards = sg.rewards
    print (rewards)
    cl_id = sg.cl_ids
    act_rewards = sg.act_rewards
    for e, rew in enumerate(act_rewards):
        print e+1, rew
    # exp_rewards = sg.exp_rewards
    #path = roslib.packages.get_pkg_dir('battery_scheduler') + '/data/sample_rewards'
    # path = '/home/milan/workspace/strands_ws/src/battery_scheduler/data/sample_rewards'
    # with open(path,'w') as f:
    #     for r, c, a_r in zip(rewards, cl_id, act_rewards):#, exp_rewards):
    #         f.write('{0} {1} {2} '.format(c, r, a_r))
    #         f.write('\n')