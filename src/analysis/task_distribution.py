#! /usr/bin/env python

import rospy
import pymongo
import plotly
import pandas as pd 
import numpy as np
import read_tasks
import plotly.plotly as py
import plotly.graph_objs as go
import matplotlib.pyplot as plt
from datetime import datetime, timedelta, time

fname = 'reward distribution pattternless taskevents'
no_int = 48
time_int = 30

def read_all_tasks( dataset_type, test=[]): 
    ### read new data -  random adder tasks nov 2019
    if dataset_type == 'synthetic':
        client = pymongo.MongoClient(rospy.get_param("mongodb_host", "localhost"),rospy.get_param("mongodb_port", 62345))
        tasks = client.message_store.random_adder_tasks_nov2019.find(None)
        start = []
        end = []
        priority = []
        for task in tasks:
            start.append(datetime.utcfromtimestamp(task['start_after']['secs'])+timedelta(hours=5, minutes=30))
            end.append(datetime.utcfromtimestamp(task['end_before']['secs'])+timedelta(hours=5, minutes=30))
            priority.append(task['priority'])
            # priority.append(1)
        
        tasks_df = pd.DataFrame(data = zip(start, end, priority), columns=['start_time', 'end_time', 'priority'])

    elif dataset_type == 'deployment1' or dataset_type == 'deployment2':
        client = pymongo.MongoClient(rospy.get_param("mongodb_host", "localhost"),rospy.get_param("mongodb_port", 62345))
        tasks = client.message_store.task_events.find(None)
        unique_tasks = get_unique_tasks(tasks, dataset_type)
        sample_array = [val for val in unique_tasks.values()]
        tasks_df = pd.DataFrame(data=sample_array, columns=['priority', 'start_time', 'end_time'])
        # min_priority = tasks_df['priority'].min()
        # max_priority = tasks_df['priority'].max()
        # tasks_df['priority'] = tasks_df['priority'].apply(lambda x: (float(x - min_priority)/(max_priority - min_priority))*500+10)

    else:
        print 'Not a valid datatset type!!! Check valid types'
        return

    tasks_df['start_day'] = tasks_df['start_time'].apply(lambda x: x.date())
    tasks_df['end_day'] = tasks_df['end_time'].apply(lambda x: x.date())

    test_days = [day.date() for day in test]
    if test_days:
        tasks_df = tasks_df[~tasks_df['start_day'].isin(test_days)]
    
    return tasks_df

def get_unique_tasks(tasks, dataset_type):
    unique_tasks = dict()
    if dataset_type == 'deployment1':
        ## Dataset 2
        start_t = datetime(2017,8,23)
        end_t = datetime(2017,10,4)
    elif dataset_type == 'deployment2':
        ## Dataset 3
        start_t = datetime(2017,10,4)
        end_t = datetime(2017,12,31)
    
    for task in tasks:
        task_id = str(task['task']['task_id']) 
        s = rospy.Time(task['task']['start_after']['secs'],task['task']['start_after']['nsecs'])  
        start_datetime = datetime.utcfromtimestamp(rospy.Time.to_sec(s))
        e = rospy.Time(task['task']['end_before']['secs'],task['task']['end_before']['nsecs'])
        end_datetime = datetime.utcfromtimestamp(rospy.Time.to_sec(e))
        
        if start_datetime >= start_t and start_datetime < end_t :          
            priority = int(task['task']['priority'])
                    
            if 'action' in task['task'].keys():
                action = str(task['task']['action'])
                if ('(F' in action) and ('&' in action) and ('|' not in action):
                    action = 'edge_exploration'
                elif ('(F' in action) and ('&' in action) and ('|' in action):
                    action = 'delivery_action'
                elif action == '':
                    action = 'un-named_action'
                elif action[0] == '/':
                    action = action[1:]

            if 'start_node_id' in task['task'].keys():
                node = task['task']['start_node_id']
                    
            if node != 'ChargingPoint1' and action != 'cpm_action'and action != 'un-named_action' and priority != 0:
                unique_tasks.update({task_id:(1, start_datetime, end_datetime)})
    return unique_tasks

def update_rewards_day(tasks, day, rewards_day, all_tasks): 
    max_end_day = tasks['end_day'].max()
    delta_days = (max_end_day - day).days
    for i in range (delta_days+1):
        rew_sum_day = np.zeros((no_int))
        start_int = datetime.combine(day,time(0,0))
        for i in range(no_int):
            end_int = start_int + timedelta(minutes=time_int)
            rew_sum_day[i] = all_tasks[(all_tasks['start_time'] < end_int) & (all_tasks['end_time'] > start_int)]['priority'].sum()
            start_int = end_int
        if day not in rewards_day:
            rewards_day.update({ day : rew_sum_day})
        else:
            rewards_day[day] += rew_sum_day
        day = day + timedelta(days=1)

def get_organised_rewards(all_tasks):
    print 'Obtaining Rewards By Day...'
    rewards_day = dict()
    days = all_tasks['start_day'].unique().tolist()
    for day in days:
        current_tasks = all_tasks[all_tasks['start_day']==day]
        update_rewards_day(current_tasks, day, rewards_day, all_tasks)
    return rewards_day


if __name__ == "__main__":

    ### For plotting scatter plots on plotly 

    # plotly.tools.set_credentials_file(username='MilanMariyaTomy', api_key= '8HntwF4rtsUwPvjW3Sl4') 

    # tasks = read_all_tasks('deployment1')
    # rewards_dist = get_organised_rewards(tasks)
    # y_rew = []
    # x_time = []
    # x_datetime = []

    # for day, rews in rewards_dist.items():
    #     ts = datetime.combine(day,time(0,0))
    #     for i in range(no_int):
    #         y_rew.append(rews[i])
    #         x_time.append(ts.time())
    #         x_datetime.append(ts)
    #         ts += timedelta(minutes=time_int)

    # data = [go.Scatter(x=x_time, y= y_rew, name='Reward Distribution', mode='markers') ]
    # fig = go.Figure(data = data)
    # py.plot(fig, filename=fname)

    # data = [go.Scatter(x=x_datetime, y= y_rew, name='Day Reward Distribution', mode='markers') ]
    # fig = go.Figure(data = data)
    # py.plot(fig, filename='day wise '+fname)


    ### for plotting interval wise boxplots for each dataset
    dataset_names = ['Deployment Dataset 1', 'Deployment Dataset 2', 'Synthetic Dataset']
    dataset_types = ['deployment1', 'deployment2', 'synthetic']
    reward_dist_datsets  = []

    # ### grouped plots
    # fig1, axs = plt.subplots(1,3) 
    # xn = np.arange(0.5,24.5,0.5)
    # xlabel = []
    # for e,n in enumerate(xn):
    #     if e%2 == 0:
    #         xlabel.append(str(n))
    #     else:
    #         xlabel.append(str(int(n)))


    # for e,dn in enumerate(dataset_names):
    #     tasks = read_all_tasks(dataset_types[e])
    #     print dn, tasks.shape[0]
    #     reward_dist_dict = get_organised_rewards(tasks)
    #     print (len(reward_dist_dict))
    #     reward_dist_matrix = np.array(reward_dist_dict.values())
    #     reward_dist_datsets.append(reward_dist_matrix.flatten())
    #     print np.std(reward_dist_matrix.flatten())
    #     axs[e].set_title(dn)
    #     axs[e].boxplot(reward_dist_matrix)
    #     axs[e].grid(axis='both', linestyle=':')
    #     axs[e].set_xlabel('Time (Hours)')
    #     axs[e].set_xticklabels(xlabel)
    #     for k, label in enumerate(axs[e].xaxis.get_ticklabels()):
    #         if (k+1)%4 != 0: 
    #             label.set_visible(False)
    #     if e == 0:
    #         axs[e].set_ylabel('Total Rewards Available')
    
    # plt.show()


    ### individual plot
    fig1, ax = plt.subplots() 
    xn = np.arange(0.5,24.5,0.5)
    xlabel = []
    for e,n in enumerate(xn):
        if e%2 == 0:
            xlabel.append(str(n))
        else:
            xlabel.append(str(int(n)))
    for e,dn in enumerate(dataset_names):
        tasks = read_all_tasks(dataset_types[e])
        print dn, tasks.shape[0]
        reward_dist_dict = get_organised_rewards(tasks)
        print (len(reward_dist_dict))
        reward_dist_matrix = np.array(reward_dist_dict.values())
        reward_dist_datsets.append(reward_dist_matrix.flatten())
        print np.std(reward_dist_matrix.flatten())
        ax.set_title(dn)
        ax.boxplot(reward_dist_matrix)
        ax.grid(axis='both', linestyle=':')
        ax.set_xlabel('Time (Hours)')
        ax.set_xticklabels(xlabel)
        ax.set_ylabel('Total Rewards Available')

        for k, label in enumerate(ax.xaxis.get_ticklabels()):
            if (k+1)%2 != 0: 
                label.set_visible(False)      
    
        plt.show()

    # ### for plotting reward distribution of all 3 data sets 
    # fig2,ax2 = plt.subplots()
    # ax2.set_title('Reward Distribution of Tasks for Different Datasets')
    # ax2.boxplot(reward_dist_datsets, labels=dataset_names)
    # ax2.grid(axis='both', linestyle=':')
    # ax2.set_xlabel('Datasets')
    # ax2.set_ylabel('Total Rewards Available')
    # plt.show()
    

    

        