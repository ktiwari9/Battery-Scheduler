#! /usr/bin/env python

from sklearn.mixture import BayesianGaussianMixture
from sklearn.cluster import KMeans
from datetime import datetime, timedelta, time
import pandas as pd
import numpy as np
import read_tasks 
import pymongo
import rospy

def read_tasks(train=[]): 
    ### read training data (old dataset)
    train_ts = [(int((td-datetime(1970,1,1)).total_seconds()), int((td+timedelta(days=1)-datetime(1970,1,1)).total_seconds())) for td in train]
    
    client = pymongo.MongoClient(rospy.get_param("mongodb_host", "localhost"),rospy.get_param("mongodb_port", 62345))
        
    unique_tasks = dict()
        
    for start_t, end_t in train_ts:
        tasks = client.message_store.task_events.find({"task.start_after.secs": {'$gte':start_t, '$lt':end_t}})

        for task in tasks:
            event = task['task'] 
            task_id = task['task']['task_id']
            start = datetime.utcfromtimestamp(event['start_after']['secs'])
            end = datetime.utcfromtimestamp(event['end_before']['secs'])
            priority = event['priority']
            # priority = 1
            
            action = str(task['task']['action'])
            if ('(F' in action) and ('&' in action) and ('|' not in action):
                action = 'edge_exploration'
            elif ('(F' in action) and ('&' in action) and ('|' in action):
                action = 'delivery_action'
            elif action == '':
                action = 'un-named_action'
            elif action[0] == '/':
                action = action[1:]

            node = task['task']['start_node_id']
            
            if node != 'ChargingPoint1' and action != 'cpm_action'and action != 'un-named_action' and priority != 0:
                unique_tasks.update({task_id:(start, end, priority)})              
    tasks_array = [val for val in unique_tasks.values()]
    tasks_array.sort(key=lambda x: x[0])    
    tasks_df = pd.DataFrame(data=tasks_array, columns=['start_time', 'end_time', 'priority'])
    
    min_priority = tasks_df['priority'].min()
    max_priority = tasks_df['priority'].max()
    tasks_df['priority'] = tasks_df['priority'].apply(lambda x: (float(x - min_priority)/(max_priority - min_priority))*500+10)

    tasks_df['start_day'] = tasks_df['start_time'].apply(lambda x: x.date())
    tasks_df['end_day'] = tasks_df['end_time'].apply(lambda x: x.date())
    
    return tasks_df

class ProbabilisticRewards:

    def __init__(self, train_days=[], no_clusters='auto'):   ## should be in datetime objects
        self.train_days = train_days
        self.tasks_df = read_tasks(train=train_days)
        self.int_duration = 30 ## minutes 
        self.no_int = 1440/self.int_duration
        self.no_clusters = no_clusters

    def __cluster_rewards(self):
        X = np.array([rew for day,rew in self.rewards_day.items()]).reshape(-1,1)
        X_wz = np.array([x for x in X if x[0] != 0])
        
        if len(X_wz) > 1:
            if len(X_wz) >=10:
                n_comps = 10
            elif len(X_wz) >= 2 and len(X_wz) < 10:  ## value of n_components
                n_comps = len(X_wz)

            if self.no_clusters == 'auto':
                self.dpgmm = BayesianGaussianMixture(n_components=n_comps,max_iter= 700,covariance_type='spherical', random_state=0).fit(X_wz)
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

        else:
            if len(X_wz) == 1:
                rews_as_states = []
                for el in list(X):
                    if el != 0:
                        rews_as_states.append(0)
                    else:
                        rews_as_states.append(1)
                return rews_as_states, X_wz
            else:
                rews_as_states = np.zeros((len(X)))
                return rews_as_states, [0]

    def __update_rewards_day(self, tasks, model_start): 
        max_end_day = tasks['end_day'].max()
        max_start_day = tasks['start_day'].max()
        delta_days = (max_end_day - max_start_day).days
        start_int = model_start
        for i in range (delta_days+1):
            rew_sum_day = np.zeros((self.no_int))
            start_day = start_int.date()
            for i in range(self.no_int):
                end_int = start_int + timedelta(minutes=self.int_duration)
                rew_sum_day[i] = tasks[(tasks['start_time'] < end_int) & (tasks['end_time'] > start_int)]['priority'].sum()
                start_int = end_int
            if start_day not in self.rewards_day:
                self.rewards_day.update({ start_day : rew_sum_day})
            else:
                self.rewards_day[start_day] += rew_sum_day

    def get_rewards_model_at(self, timestamp): #timestamp should be datetime
        print 'Obtaining Rewards Model...'
        ts = timestamp.time()

        if self.train_days:
            self.rewards_day = dict()
            days = self.tasks_df['start_day'].unique().tolist()
            for day in days:
                model_start = datetime.combine(day, ts)
                model_end = model_start + timedelta(days=1)
                current_tasks = self.tasks_df[(self.tasks_df['start_time']>=model_start) & (self.tasks_df['start_time']< model_end)]
                if not current_tasks.empty:
                    self.__update_rewards_day(current_tasks, model_start)

            rewards, reward_states = self.__cluster_rewards()
            states = list(np.unique(rewards))
            
            if len(states) !=0:
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
                
            else:
                prob_m = np.ones((self.no_int,1))
                state_means = [0]
                task_prob = np.array(self.no_int*[[1,0]])

        else:
            ## expect tasks of uniform rewards (500) from 8.00 am to 10.00pm
            prob_m = np.ones((self.no_int,1))
            state_means = [500]
            task_prob = np.zeros((self.no_int, 2))
            for i in range(self.no_int):
                if ((timestamp + timedelta(minutes=self.int_duration)).time() > time(9,0)) and (timestamp.time() < time(22,0)):
                    task_prob[i][1] = 1
                else:
                    task_prob[i][0] = 1
                timestamp = timestamp + timedelta(minutes=self.int_duration)
         
        print state_means   
        return task_prob, prob_m, state_means
        
        
if __name__ == "__main__":
    train = [datetime(2017,10,3)]
    # train = []
    pr = ProbabilisticRewards(train_days=train)
    a, b, c = pr.get_rewards_model_at(datetime(2017,10,3,11,30))
    print a
    print b
    print c
