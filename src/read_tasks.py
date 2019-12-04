#!/usr/bin/env python
import pandas as pd 
import numpy as np
from datetime import datetime
import pymongo
import random
import rospy
import time


#### reading in only the unique tasks, ignoring the task events

def timing_wrapper(func):
    def wrapper(*args,**kwargs):
        t = datetime.now()
        result = func(*args,**kwargs)
        t1 = datetime.now()
        print func, ' took time:', t1-t
        return result
    return wrapper

class getTasks:
    @timing_wrapper
    def __init__(self):
        ############### SPECIFY ACCESS ##############################
        client = pymongo.MongoClient(rospy.get_param("mongodb_host", "localhost"),rospy.get_param("mongodb_port", 62345))
        start_t = (datetime(2017,8,23)-datetime(1970,1,1)).total_seconds()
        end_t = (datetime(2017,10,4)-datetime(1970,1,1)).total_seconds()
        self.tasks = client.message_store.task_events.find({"task.start_after.secs": {'$gte':start_t}, "task.end_before.secs":{'$lt':end_t}})
        self.unique_tasks = []
        self.sample_array = []
        self._get_unique_tasks()
        self.sample_array.sort(key= lambda x:x[1])
        self.tasks_df = pd.DataFrame(data=self.sample_array, columns=['priority', 'start_time', 'end_time'])
        min_priority = self.tasks_df['priority'].min()
        max_priority = self.tasks_df['priority'].max()
        self.tasks_df['priority'] = self.tasks_df['priority'].apply(lambda x: (float(x - min_priority)/(max_priority - min_priority))*500+10)
        
    def _get_unique_tasks(self):
        for task in self.tasks:
            if task['task']['task_id'] not in self.unique_tasks:
                s = rospy.Time(task['task']['start_after']['secs'],task['task']['start_after']['nsecs'])  
                start_datetime = datetime.fromtimestamp(rospy.Time.to_sec(s))
                        
                e = rospy.Time(task['task']['end_before']['secs'],task['task']['end_before']['nsecs'])
                end_datetime = datetime.fromtimestamp(rospy.Time.to_sec(e))
                            
                task_id = str(task['task']['task_id']) 
                
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
                    self.unique_tasks.append(task_id)
                    self.sample_array.append((priority, start_datetime, end_datetime))
                
    
if __name__ == '__main__':
    gt = getTasks()
    # print (gt.unique_tasks)
    # print np.mean(gt.durations)
    # print min(gt.durations)
    # print max(gt.durations)