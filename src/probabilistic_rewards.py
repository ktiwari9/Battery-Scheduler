#!/usr/bin/env python

import read_tasks
import numpy as np
import pandas as pd
import read_task_wo_priorities
import matplotlib.pyplot as plt
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


class uncertain_rewards:
    @timing_wrapper
    def __init__(self):
        print 'Reading Tasks...'
        tasks_processor = read_tasks.getTasks()
        # t = read_task_wo_priorities.getTasks()
        self.tasks = tasks_processor.tasks_df
        self.time_int = 30 #minutes
        self.no_int = 1440/self.time_int
        self.rewards_day  = dict()
        self.tasks['start_day'] = self.tasks['start_time'].apply(lambda x: x.date())
        self.tasks['end_day'] = self.tasks['end_time'].apply(lambda x: x.date())
        self.test_days = [date(2017, 8, 11), date(2017,8,18)]
        self.test_tasks =  self.tasks[self.tasks['start_day'].isin(self.test_days)]
        self.tasks = self.tasks[~self.tasks['start_day'].isin(self.test_days)]
        
    def __cluster_rewards(self):
        X = np.array([rew for day,rew in self.rewards_day.items()]).reshape(-1,1)
        dpgmm = BayesianGaussianMixture(n_components=10,max_iter= 500,covariance_type='spherical', random_state=0).fit(X)
        rews_as_states = dpgmm.predict(X) 
       
        for e,el in enumerate(list(X)):
            if el == [0]:
                rews_as_states[e] = len(dpgmm.means_)

        mean_dict = dict()
        for e,s in enumerate(rews_as_states):
            if s not in mean_dict:
                mean_dict.update({s : [X[e][0]]})
            else:
                mean_dict[s].append(X[e][0])

        reward_states = np.zeros((dpgmm.means_.shape[0]+1))
        for state, vals in mean_dict.items():
            reward_states[int(state)] = np.mean(vals)
 
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
                rew_sum_day[i] = self.tasks[(self.tasks['start_time'] < end_int) & (self.tasks['end_time'] > start_int)]['priority'].sum()
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

        rewards, reward_states = self.__cluster_rewards()
        states = list(np.unique(rewards))
        z_l = np.max(states)

        prob_m = np.zeros((self.no_int, len(states)-1))
        rewardst = rewards.T
        for i in range(rewardst.shape[0]):
            st, st_c = np.unique(rewardst[i], return_counts=True)
            st = list(st)
            if z_l in st:
                z_ind = st.index(z_l)
                total_c = np.sum(st_c) - st_c[z_ind]
            else:
                total_c = np.sum(st_c)
            
            st_c = st_c/float(total_c)
            for j, s in enumerate(st):
                if s != z_l:
                    prob_m[i][states.index(s)] = st_c[j]

        state_means = [round(reward_states[int(s)]) for s in states if s != z_l]
        return prob_m, state_means


if __name__ == '__main__':
    ur = uncertain_rewards()
    prob_m, state_means = ur.get_probabilistic_reward_model()
    print prob_m
    print state_means