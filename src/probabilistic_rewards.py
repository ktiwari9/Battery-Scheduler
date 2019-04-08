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
    def __init__(self, test_days):
        print 'Reading Tasks...'
        tasks_processor = read_tasks.getTasks()
        # tasks_processor = read_task_wo_priorities.getTasks()
        self.tasks = tasks_processor.tasks_df
        self.time_int = 30 #minutes
        self.no_int = 1440/self.time_int
        self.rewards_day  = dict()
        self.tasks['start_day'] = self.tasks['start_time'].apply(lambda x: x.date())
        self.tasks['end_day'] = self.tasks['end_time'].apply(lambda x: x.date())
        # all_days = self.tasks['start_day'].sort_values()
        # print all_days.unique()
        
        ## for reward models 
        remove_days = [date(2017, 10, 1), date(2017, 10, 2), date(2017, 10, 3)]

        if test_days:
            self.test_days = test_days #[date(2017, 10, 1), date(2017, 10, 2), date(2017, 10, 3)]
            self.test_tasks =  self.tasks[self.tasks['start_day'].isin(self.test_days)]
            self.tasks = self.tasks[~self.tasks['start_day'].isin(self.test_days)]
            for day in test_days:
                if day in remove_days or day.month == 9 or day.month == 8:
                    self.tasks = self.tasks[(self.tasks['start_day'].apply(lambda x:x.month) == 9) | (self.tasks['start_day'].apply(lambda x:x.month) == 8)]# | (self.tasks['start_day'].isin(remove_days))]

                elif day not in remove_days or day.month == 11 or day.month == 12:
                    self.tasks = self.tasks[~self.tasks['start_day'].isin(remove_days)]
                    self.tasks = self.tasks[(self.tasks['start_day'].apply(lambda x:x.month) == 10) | (self.tasks['start_day'].apply(lambda x:x.month) == 11) | (self.tasks['start_day'].apply(lambda x:x.month) == 12)]

                break
        else:
            self.tasks = self.tasks[(self.tasks['start_day'].apply(lambda x:x.month) == 9) | (self.tasks['start_day'].apply(lambda x:x.month) == 8) ]#| (self.tasks['start_day'].isin(remove_days))]


    def __cluster_rewards(self):
        X = np.array([rew for day,rew in self.rewards_day.items()]).reshape(-1,1)
        X_wz = np.array([x for x in X if x[0] != 0])
        self.dpgmm = BayesianGaussianMixture(n_components=10,max_iter= 700,covariance_type='spherical', random_state=0).fit(X_wz)
        
        rews_as_states = self.dpgmm.predict(X) 
        for e,el in enumerate(list(X)):
            if el == [0]:
                rews_as_states[e] = len(self.dpgmm.means_)

        mean_dict = dict()
        for e,s in enumerate(rews_as_states):
            if s not in mean_dict:
                mean_dict.update({s : [X[e][0]]})
            else:
                mean_dict[s].append(X[e][0])

        reward_states = np.zeros((self.dpgmm.means_.shape[0]+1))
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

    expected_rew = []
    for row in prob_m:
        expected_rew.append(sum(row*state_means))

    x = np.arange(48)
    for day in ur.rewards_day:
        if day == date(2017, 8, 14) or day == date(2017, 8, 15) or day == date(2017, 8, 16) or date(2017, 8,20) == day or date(2017, 8, 22) == day or day == date(2017, 8, 23):
            print day
            plt.bar(x, ur.rewards_day[day])
            plt.plot(x, expected_rew)
            plt.show()