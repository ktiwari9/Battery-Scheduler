#!/usr/bin/env python

import rospy
import read_tasks
import pymongo
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans 
from sklearn.mixture import BayesianGaussianMixture
from datetime import time, timedelta, datetime, date


def timing_wrapper(func):
    def wrapper(*args,**kwargs):
        t = datetime.now()
        result = func(*args,**kwargs)
        t1 = datetime.now()
        print func, ' took time:', t1-t
        return result
    return wrapper

def read_all_tasks(test=[]): 
    ### read new data -  random adder tasks nov 2019
    # client = pymongo.MongoClient(rospy.get_param("mongodb_host", "localhost"),rospy.get_param("mongodb_port", 62345))
    # tasks = client.message_store.random_adder_tasks_nov2019.find(None)
    # start = []
    # end = []
    # priority = []
    # for task in tasks:
    #     start.append(datetime.fromtimestamp(task['start_after']['secs']))
    #     end.append(datetime.fromtimestamp(task['end_before']['secs']))
    #     priority.append(task['priority'])
    
    # tasks_df = pd.DataFrame(data = zip(start, end, priority), columns=['start_time', 'end_time', 'priority'])

    ### read old data
    tasks_processor = read_tasks.getTasks()
    tasks_df = tasks_processor.tasks_df

    tasks_df['start_day'] = tasks_df['start_time'].apply(lambda x: x.date())
    tasks_df['end_day'] = tasks_df['end_time'].apply(lambda x: x.date())

    test_days = [day.date() for day in test]
    if test_days:
        test_tasks = tasks_df[tasks_df['start_day'].isin(test_days)]
        tasks_df = tasks_df[~tasks_df['start_day'].isin(test_days)]
    else:
        test_tasks = None
    
    return tasks_df, test_tasks

class uncertain_rewards:
    @timing_wrapper
    def __init__(self, test_days, no_clusters='auto'):
        print 'Reading Tasks...'
        self.tasks, self.test_tasks = read_all_tasks(test_days)
        self.time_int = 30 #minutes
        self.no_int = 1440/self.time_int
        self.rewards_day  = dict()
        self.test_days = test_days
        self.no_clusters = no_clusters

    def __cluster_rewards(self):
        X = np.array([rew for day,rew in self.rewards_day.items()]).reshape(-1,1)
        X_wz = np.array([x for x in X if x[0] != 0])
        if self.no_clusters == 'auto':
            self.dpgmm = BayesianGaussianMixture(n_components=10,max_iter= 700,covariance_type='spherical', random_state=0).fit(X_wz)
        else:
            self.dpgmm = KMeans(n_clusters=self.no_clusters).fit(X)
        
        rews_as_states = self.dpgmm.predict(X) 
        for e,el in enumerate(list(X)):
            if el == [0]:
                if self.no_clusters == 'auto':
                    rews_as_states[e] = len(self.dpgmm.means_)
                else:
                    rews_as_states[e] = len(self.dpgmm.cluster_centers_)


        mean_dict = dict()
        for e,s in enumerate(rews_as_states):
            if s not in mean_dict:
                mean_dict.update({s : [X[e][0]]})
            else:
                mean_dict[s].append(X[e][0])
        
        if self.no_clusters == 'auto': 
            reward_states = np.zeros((self.dpgmm.means_.shape[0]+1))
        else:
            reward_states = np.zeros((self.dpgmm.cluster_centers_.shape[0]+1)) 
        for state, vals in mean_dict.items():
            reward_states[int(state)] = np.mean(vals)

        # print reward_states, 'manual mean'
        # print self.dpgmm.means_, 'gmm means'
 
        rews_as_states = rews_as_states.reshape(len(self.rewards_day),self.no_int)
        return rews_as_states, reward_states

    def __update_rewards_day(self, tasks, day): 
        max_end_day = tasks['end_day'].max()
        delta_days = (max_end_day - day).days
        for i in range (delta_days+1):
            rew_sum_day = np.zeros((self.no_int))
            start_int = datetime.combine(day,time(0,0))
            for i in range(self.no_int):
                end_int = start_int + timedelta(minutes=self.time_int)
                rew_sum_day[i] = tasks[(tasks['start_time'] < end_int) & (tasks['end_time'] > start_int)]['priority'].sum()
                start_int = end_int
            if day not in self.rewards_day:
                self.rewards_day.update({ day : rew_sum_day})
            else:
                self.rewards_day[day] += rew_sum_day
            day = day + timedelta(days=1)

    @timing_wrapper
    def get_probabilistic_reward_model(self):
        print 'Obtaining Rewards Model...'
        days = self.tasks['start_day'].unique().tolist()
        for day in days:
            current_tasks = self.tasks[self.tasks['start_day']==day]
            self.__update_rewards_day(current_tasks, day)

        # to_be_removed = []
        # for day in self.rewards_day:
        #     u_el, u_c = np.unique(self.rewards_day[day], return_counts=True)
        #     if u_el[0] == 0 and u_c[0] > 20:
        #         to_be_removed.append(day)

        # for day in to_be_removed:
        #     del self.rewards_day[day]

        rewards, reward_states = self.__cluster_rewards()
        states = list(np.unique(rewards))
        z_l = np.max(states)

        prob_m = np.zeros((self.no_int, len(states)-1))
        task_prob = np.zeros((self.no_int, 2))
        rewardst = rewards.T
        for i in range(rewardst.shape[0]):
            st, st_c = np.unique(rewardst[i], return_counts=True)
            st = list(st)
            if z_l in st:
                z_ind = st.index(z_l)
                total_c = np.sum(st_c) - st_c[z_ind]
                
                task_prob[i][0] = float(st_c[z_ind])/np.sum(st_c)
                task_prob[i][1] = float(total_c)/np.sum(st_c)
            else:
                total_c = np.sum(st_c)
                task_prob[i][1] = 1
            
            st_c = st_c/float(total_c)
            for j, s in enumerate(st):
                if s != z_l:
                    prob_m[i][states.index(s)] = st_c[j]

        state_means = [round(reward_states[int(s)]) for s in states if s != z_l]
        return task_prob, prob_m, state_means


if __name__ == '__main__':
    ur = uncertain_rewards([])
    task_prob, prob_m, state_means = ur.get_probabilistic_reward_model()
    print task_prob
    print prob_m
    print state_means

    # expected_rew = []
    # for row in prob_m:
    #     expected_rew.append(sum(row*state_means))

    # x = np.arange(48)
    # for day in ur.rewards_day:
    #     # if day == date(2017, 9, 28) or day == date(2017, 8, 8) or day == date(2017, 10, 28) or date(2017, 9,30) == day or date(2017, 8, 23) == day or day == date(2017, 10, 26):
    #     # if day == date(2017, 10, 1) or day == date(2017, 9, 24) or day == date(2017, 9, 25) or day == date(2017, 9, 26) or day == date(2017, 10, 27) or day == date(2017, 10, 29) or day == date(2017, 10, 31) or day == date(2017, 10, 2) or day == date(2017, 10, 3) or day == date(2017, 11, 12) or day == date(2017, 8, 31) or day == date(2017, 8, 14) or day == date(2017, 8, 15) or day == date(2017, 9, 4) or day == date(2017, 11, 10) or day == date(2017, 10, 19) or day == date(2017, 9, 28) or day == date(2017, 10, 21) or day == date(2017, 10, 22) or day == date(2017, 10, 23) or day == date(2017, 10, 10) or day == date(2017, 11, 29) or day == date(2017, 12, 13) or day == date(2017, 8, 8) or day == date(2017, 8, 10) or day == date(2017, 11, 24) or day == date(2017, 12, 19) or day == date(2017, 10, 18) or day == date(2017, 9, 20): 
    
    #     print day
    #     plt.bar(x, ur.rewards_day[day])
    #     plt.plot(x, expected_rew)
    #     plt.show()