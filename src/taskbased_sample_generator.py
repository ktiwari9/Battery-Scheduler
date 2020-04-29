#! /usr/bin/env python

from datetime import datetime, timedelta
import pandas as pd 
import pymongo
import roslib
import rospy

class SampleGenerator:
    # test days date time objects
    def __init__(self, test_days):
        ############################ OLD DATA 
        # self.test_ts = [(int((td-datetime(1970,1,1)).total_seconds()), int((td+timedelta(days=1)-datetime(1970,1,1)).total_seconds())) for td in test_days]

        # client = pymongo.MongoClient(rospy.get_param("mongodb_host", "localhost"),rospy.get_param("mongodb_port", 62345))
        
        # unique_tasks = dict()
        
        # for start_t, end_t in self.test_ts:
        #     tasks = client.message_store.task_events.find({"task.start_after.secs": {'$gte':start_t, '$lt':end_t}})

        #     for task in tasks:
        #         event = task['task'] 
        #         task_id = task['task']['task_id']
        #         start = datetime.utcfromtimestamp(event['start_after']['secs'])
        #         end = datetime.utcfromtimestamp(event['end_before']['secs'])
        #         priority = event['priority']
        #         # priority = 1
            
        #         action = str(task['task']['action'])
        #         if ('(F' in action) and ('&' in action) and ('|' not in action):
        #             action = 'edge_exploration'
        #         elif ('(F' in action) and ('&' in action) and ('|' in action):
        #             action = 'delivery_action'
        #         elif action == '':
        #             action = 'un-named_action'
        #         elif action[0] == '/':
        #             action = action[1:]

        #         node = task['task']['start_node_id']
                
        #         if node != 'ChargingPoint1' and action != 'cpm_action'and action != 'un-named_action' and priority != 0:
        #             unique_tasks.update({task_id:(start, end, priority)})              
        # sample_array = [val for val in unique_tasks.values()]
        # sample_array.sort(key=lambda x: x[0])    
        # self.samples = pd.DataFrame(data=sample_array, columns=['start', 'end', 'priority'])
        
        # min_priority = self.samples['priority'].min()
        # max_priority = self.samples['priority'].max()
        # self.samples['priority'] = self.samples['priority'].apply(lambda x: (float(x - min_priority)/(max_priority - min_priority))*500+10)


        # ################## NEW DATA 

        self.test_ts = [(int((td-datetime(1970,1,1)).total_seconds()), int((td+timedelta(days=1)-datetime(1970,1,1)).total_seconds())) for td in test_days]

        client = pymongo.MongoClient(rospy.get_param("mongodb_host", "localhost"),rospy.get_param("mongodb_port", 62345))

        sample_start = []
        sample_end = []
        sample_priority = []
        unique_tasks = []

        for start_t, end_t in self.test_ts:
            tasks = client.message_store.random_adder_tasks_nov2019.find({"start_after.secs": {'$gte':start_t, '$lt':end_t}}).sort("start_after.secs") 

            for event in tasks: 
                start = datetime.utcfromtimestamp(event['start_after']['secs']) + timedelta(hours=5,minutes=30)
                end = datetime.utcfromtimestamp(event['end_before']['secs']) + + timedelta(hours=5,minutes=30)
                priority = event['priority']
                # priority = 1
                sample_start.append(start)
                sample_end.append(end)
                sample_priority.append(priority)
            
        self.samples = pd.DataFrame(data=zip(sample_start,sample_end,sample_priority), columns=['start', 'end', 'priority'])
        

if __name__ == "__main__":
    # test_days = [datetime(2017, 10, 1), datetime(2017, 10, 2), datetime(2017,10,3)]
    test_days = [datetime(2019, 11, 10), datetime(2019, 11, 11), datetime(2019,11,12)]
    sg = SampleGenerator (test_days)
    print sg.samples
    