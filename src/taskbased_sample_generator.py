#! /usr/bin/env python

from datetime import datetime, timedelta
import pandas as pd 
import pymongo
import roslib
import rospy

class SampleGenerator:
    # test days date time objects
    def __init__(self, test_days):
        self.test_ts = [(int((td-datetime(1970,1,1)).total_seconds()), int((td+timedelta(days=1)-datetime(1970,1,1)).total_seconds())) for td in test_days]

        client = pymongo.MongoClient(rospy.get_param("mongodb_host", "localhost"),rospy.get_param("mongodb_port", 62345))
        
        sample_start = []
        sample_end = []
        sample_priority = []
        unique_tasks = []
        
        for start_t, end_t in self.test_ts:
            # tasks = client.message_store.random_adder_tasks_nov2019.find({"start_after.secs": {'$gte':start_t}, "end_before.secs":{'$lt':end_t}}).sort("start_after.secs") ## new data

            tasks = client.message_store.task_events.find({"task.start_after.secs": {'$gte':start_t}, "task.end_before.secs":{'$lt':end_t}}).sort("task.start_after.secs")  ## old data

            # for event in tasks:  ##new data 
            for task in tasks:
                event = task['task']  ### old data 
                task_id = task['task']['task_id']
                if task_id not in unique_tasks: ## remove if for new data
                    start = datetime.fromtimestamp(event['start_after']['secs'])
                    end = datetime.fromtimestamp(event['end_before']['secs'])
                    priority = event['priority']
                    # priority = 1
                    sample_start.append(start)
                    sample_end.append(end)
                    sample_priority.append(priority)
                    unique_tasks.append(task_id)
            
        self.samples = pd.DataFrame(data=zip(sample_start,sample_end,sample_priority), columns=['start', 'end', 'priority'])

   

if __name__ == "__main__":
    test_days = [datetime(2019,11,7), datetime(2019,11,8), datetime(2019,11,9)]
    SampleGenerator (test_days)
    