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
        
        for start_t, end_t in self.test_ts:
            tasks = client.message_store.random_adder_tasks_nov2019.find({"start_after.secs": {'$gte':start_t}, "end_before.secs":{'$lt':end_t}}).sort("start_after.secs")

            for event in tasks:
                start = datetime.fromtimestamp(event['start_after']['secs'])
                end = datetime.fromtimestamp(event['end_before']['secs'])
                priority = event['priority']
                sample_start.append(start)
                sample_end.append(end)
                sample_priority.append(priority)
            
        self.samples = pd.DataFrame(data=zip(sample_start,sample_end,sample_priority), columns=['start', 'end', 'priority'])

   

if __name__ == "__main__":
    test_days = [datetime(2019,11,7), datetime(2019,11,8), datetime(2019,11,9)]
    SampleGenerator (test_days)
    