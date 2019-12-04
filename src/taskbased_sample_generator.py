#! /usr/bin/env python

from datetime import datetime, timedelta
import pandas as pd 
import pymongo
import roslib
import rospy

class SampleGenerator:
    # test days date time objects
    def __init__(self, test_days):
        ############################# OLD DATA 
        self.test_ts = [(int((td-datetime(1970,1,1)).total_seconds()), int((td+timedelta(days=1)-datetime(1970,1,1)).total_seconds())) for td in test_days]

        client = pymongo.MongoClient(rospy.get_param("mongodb_host", "localhost"),rospy.get_param("mongodb_port", 62345))
        
        sample_array = []
        unique_tasks = []
        
        for start_t, end_t in self.test_ts:
            tasks = client.message_store.task_events.find({"task.start_after.secs": {'$gte':start_t}, "task.end_before.secs":{'$lt':end_t}})

            prev_start = datetime(1970,1,1)  
            for task in tasks:
                event = task['task'] 
                task_id = task['task']['task_id']
                if task_id not in unique_tasks: 
                    start = datetime.fromtimestamp(event['start_after']['secs'])
                    end = datetime.fromtimestamp(event['end_before']['secs'])
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
                        sample_array.append((start, end, priority))
                        unique_tasks.append(task_id)              

        sample_array.sort(key=lambda x: x[0])    
        self.samples = pd.DataFrame(data=sample_array, columns=['start', 'end', 'priority'])
        
        min_priority = self.samples['priority'].min()
        max_priority = self.samples['priority'].max()
        self.samples['priority'] = self.samples['priority'].apply(lambda x: (float(x - min_priority)/(max_priority - min_priority))*500+10)


        # ################## NEW DATA 

        # self.test_ts = [(int((td-datetime(1970,1,1)).total_seconds()), int((td+timedelta(days=1)-datetime(1970,1,1)).total_seconds())) for td in test_days]

        # client = pymongo.MongoClient(rospy.get_param("mongodb_host", "localhost"),rospy.get_param("mongodb_port", 62345))

        # sample_start = []
        # sample_end = []
        # sample_priority = []
        # unique_tasks = []

        # for start_t, end_t in self.test_ts:
        #     tasks = client.message_store.random_adder_tasks_nov2019.find({"start_after.secs": {'$gte':start_t}, "end_before.secs":{'$lt':end_t}}).sort("start_after.secs") 

        #     for event in tasks: 
        #         start = datetime.fromtimestamp(event['start_after']['secs'])
        #         end = datetime.fromtimestamp(event['end_before']['secs'])
        #         priority = event['priority']
        #         # priority = 1
        #         sample_start.append(start)
        #         sample_end.append(end)
        #         sample_priority.append(priority)
        #         unique_tasks.append(task_id)          
            
        # self.samples = pd.DataFrame(data=zip(sample_start,sample_end,sample_priority), columns=['start', 'end', 'priority'])
        

if __name__ == "__main__":
    test_days = [datetime(2017, 9, 24), datetime(2017,9,25), datetime(2017, 9, 26)]
    SampleGenerator (test_days)
    