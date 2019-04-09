#!/usr/bin/env python

import numpy as np
import pandas as pd
import read_task_wo_priorities
import matplotlib.pyplot as plt
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
        tasks_processor = read_task_wo_priorities.getTasks()
        self.tasks = tasks_processor.tasks_df
        self.time_int = 30 #minutes
        self.no_int = 1440/self.time_int
        self.rewards_day  = dict()
        self.tasks['start_day'] = self.tasks['start_time'].apply(lambda x: x.date())
        self.tasks['end_day'] = self.tasks['end_time'].apply(lambda x: x.date())
        # all_days = self.tasks['start_day'].sort_values()
        # print all_days.unique()
        
        ## for task models
        if test_days:
            self.test_days = test_days
            self.test_tasks =  self.tasks[self.tasks['start_day'].isin(self.test_days)]
            self.tasks = self.tasks[~self.tasks['start_day'].isin(self.test_days)]
            self.tasks = self.tasks[(self.tasks['start_day'].apply(lambda x:x.year) == 2017)]
            # self.tasks = self.tasks[self.tasks['start_day'].isin(self.test_days)] - for sanity check
        else:
            self.tasks = self.tasks[(self.tasks['start_day'].apply(lambda x:x.year) == 2017)]

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

        X = np.array([rew for day,rew in self.rewards_day.items()]).T
        states = list(np.unique(X))

        prob_m = np.zeros((self.no_int, len(states)-1))
        task_prob = np.zeros((self.no_int, 2))
        for i in range(X.shape[0]):
            st, st_c = np.unique(X[i], return_counts=True)
            st = list(st)
            if 0 in st:
                z_ind = st.index(0)
                total_c = np.sum(st_c) - st_c[z_ind]               
                task_prob[i][0] = float(st_c[z_ind])/np.sum(st_c)
                task_prob[i][1] = float(total_c)/np.sum(st_c)
            else:
                total_c = np.sum(st_c)
                task_prob[i][1] = 1
            
            st_c = st_c/float(total_c)
            for j, s in enumerate(st):
                if s != 0:
                    prob_m[i][states.index(s)-1] = st_c[j]

        state_means = [int(s) for s in states if s != 0]
        return task_prob, prob_m, state_means


if __name__ == '__main__':
    ur = uncertain_rewards([])
    task_prob, prob_m, state_means = ur.get_probabilistic_reward_model()
    # print task_prob
    # print prob_m
    # print state_means

    # expected_rew = []
    # for row in prob_m:
    #     expected_rew.append(sum(row*state_means))

    # x = np.arange(48)
    # for day in ur.rewards_day:
    #     # if day == date(2017, 10, 1) or day == date(2017, 11, 24): 
    #     print day
    #     plt.bar(x, ur.rewards_day[day])
    #     plt.plot(x, expected_rew)
    #     plt.show()