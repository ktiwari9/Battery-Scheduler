#!/usr/bin/env python
import pandas as pd 
import datetime
import pymongo
import random
import rospy
import time


#### reading in only the unique tasks, ignoring the task events

class getTasks:
    
    def __init__(self):
        ############### SPECIFY ACCESS ##############################
        client = pymongo.MongoClient(rospy.get_param("mongodb_host", "localhost"),rospy.get_param("mongodb_port", 62345))
        rospy.loginfo("Connecting to mongodB")        
        self.tasks = client.message_store.task_events.find(None)
        no_tasks = client.message_store.task_events.find(None).count() 
        # self.tasks = client.betty.task_events_unique.find(None)
        # no_tasks = client.betty.task_events_unique.find(None).count() 
        rospy.loginfo("Connection established, %d tasks being analysed" %no_tasks)
        self.unique_tasks = dict()
        self._get_unique_tasks()
        #self.tasks = client.bob.task_events.find(None)
        #no_tasks = client.bob.task_events.find(None).count() 
        #rospy.loginfo("Connection established, %d tasks being analysed" %no_tasks)
        #self._get_unique_tasks('b')
        rospy.loginfo("%d unique tasks found" %len(self.unique_tasks))
        task_ids = [task_id for task_id in self.unique_tasks.keys()]
        priorities = [self.unique_tasks[task_id][0] for task_id in task_ids]
        start_times = [self.unique_tasks[task_id][1] for task_id in task_ids]
        end_times = [self.unique_tasks[task_id][2] for task_id in task_ids]
        self.tasks_df = pd.DataFrame(data=zip(task_ids, priorities, start_times, end_times), columns=['task_id', 'priority', 'start_time', 'end_time'])
        
    def _get_unique_tasks(self, collection_indicator=None):
        for task in self.tasks:
            
            for task_details in task['task'].keys():
                
                if task_details == 'start_after':
                    s = rospy.Time(task['task'][task_details]['secs'],task['task'][task_details]['nsecs'])  
                    start_datetime = datetime.datetime.fromtimestamp(rospy.Time.to_sec(s))
                    
                elif task_details == 'end_before':
                    e = rospy.Time(task['task'][task_details]['secs'],task['task'][task_details]['nsecs'])
                    end_datetime = datetime.datetime.fromtimestamp(rospy.Time.to_sec(e))
                    
                elif task_details == 'task_id':
                    task_id = str(task['task'][task_details]) #+ collection_indicator
                    
                elif task_details == 'priority':
                    priority = task['task'][task_details]
                    
                elif task_details == 'action':
                    action = str(task['task'][task_details])
                    if ('(F' in action) and ('&' in action) and ('|' not in action):
                        action = 'edge_exploration'
                    elif ('(F' in action) and ('&' in action) and ('|' in action):
                        action = 'delivery_action'
                    elif action == '':
                        action = 'un-named_action'
                    elif action[0] == '/':
                        action = action[1:]

                elif task_details == 'start_node_id':
                    node = task['task'][task_details]
                    
            if node != 'ChargingPoint1' and action != 'cpm_action'and action != 'un-named_action' and priority != 0:
                # if priority > 50 and priority < 5000:
                #     new_priority = priority%11
                # elif priority >= 5000 and priority < 50000:
                #     new_priority = priority%101
                # elif priority >= 50000:
                #     new_priority = priority%1001
                # else:
                #     new_priority = priority

                self.unique_tasks.update({task_id : (priority, start_datetime, end_datetime)})
    
if __name__ == '__main__':
    gt = getTasks()
    # print (gt.unique_tasks)