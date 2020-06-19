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
    #     priority.append(1)
    
    # tasks_df = pd.DataFrame(data = zip(start, end, priority), columns=['start_time', 'end_time', 'priority'])

    ### read old data
    tasks_processor = read_task_wo_priorities.getTasks()
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
    def __init__(self, test_days):
        print 'Reading Tasks...'
        self.tasks, self.test_tasks = read_all_tasks(test_days)
        self.time_int = 30 #minutes
        self.no_int = 1440/self.time_int
        self.rewards_day  = dict()
        self.test_days = test_days
    
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