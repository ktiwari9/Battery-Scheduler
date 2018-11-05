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
        print func, ' took time:', t1-t, 's'
        return result
    return wrapper

class uncertain_rewards:
    @timing_wrapper
    def __init__(self):
        tasks_processor = read_tasks.getTasks()
        # t = read_task_wo_priorities.getTasks()
        self.tasks = tasks_processor.tasks_df
        self.time_int = 30 #minutes
        self.priorities_as_states, self.state_means = self.__cluster_rewards()
        self.tasks['reward_states']=pd.Series(self.priorities_as_states)
        adjusted_start = self.tasks['start_time'].apply(lambda x: datetime.combine(date.today(), x.time()))
        adjusted_end = self.tasks['end_time_flag'].apply(lambda x: datetime.combine(date.today(), x[0].time()) if x[1] == False else datetime.combine((datetime.today()+timedelta(days=1)).date(), x[0].time()) )
        self.tasks['adjusted_start'] = adjusted_start
        self.tasks['adjusted_end'] = adjusted_end
        
    def __cluster_rewards(self):
        X = np.array(self.tasks['priority']).reshape(-1,1)
        dpgmm = BayesianGaussianMixture(n_components=10,max_iter= 500,covariance_type='spherical', random_state=0).fit(X)
        priorities_as_states = dpgmm.predict(X)
        reward_states = [m[0] for m in dpgmm.means_]  ## Means indxed by state

        # states = np.unique(priorities_as_states)
        # print ('Number of states: ', len(np.unique(priorities_as_states)), states)
        # print (dpgmm.means_)
        # color=['navy', 'c', 'cornflowerblue', 'gold','darkorange', 'r', 'g', 'm', 'y', 'k']
        # color_labels = []
        # for label in priorities_as_states:
        #     color_labels.append(color[int(label)])
        # print ([ color[s] for s in states])
        # plt.scatter(range(len(X)), X, color= color_labels)
        # plt.show()
        return priorities_as_states, reward_states

    def get_probabilistic_reward_model(self):
        no_int = 1440/self.time_int   # 1440 is total minutes in a day
        start_int = datetime.combine(date.today(),time(0,0)) 
        reward_prob = np.zeros((no_int,len(self.state_means)))   
        for i in range(no_int):
            end_int = start_int + timedelta(minutes=self.time_int)
            df_t = self.tasks[(self.tasks['adjusted_start'] < end_int) & (self.tasks['adjusted_end'] > start_int)]
            r_unique, r_count = np.unique(df_t['reward_states'], return_counts=True) 
            r_prob = r_count/float(np.sum(r_count))
            for r, p in zip(r_unique,r_prob):
                reward_prob[i,r] = p
            start_int = end_int
        return reward_prob, self.state_means

if __name__ == '__main__':
    ur = uncertain_rewards()
