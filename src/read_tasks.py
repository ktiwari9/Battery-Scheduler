#!/usr/bin/env python

import random 
import pymongo
import rospy
import time


#### reading in only the unique tasks, ignoring the task events

class getTasks:
    
    def __init__(self):
        client = pymongo.MongoClient(rospy.get_param("mongodb_host", "localhost"),rospy.get_param("mongodb_port", 62345))
        rospy.loginfo("Connecting to mongodB")        
        self.tasks = client.message_store.task_events.find(None)
        no_tasks = client.message_store.task_events.find(None).count() 
        #self.tasks = client.dump.task_events_unique.find(None)
        #no_tasks = client.dump.task_events_unique.find(None).count() 
        rospy.loginfo("Connection established, %d tasks being analysed" %no_tasks)
        self.unique_tasks = dict()
        self._get_unique_tasks()
        #self.tasks = client.bob.task_events.find(None)
        #no_tasks = client.bob.task_events.find(None).count() 
        #rospy.loginfo("Connection established, %d tasks being analysed" %no_tasks)
        #self._get_unique_tasks('b')
        rospy.loginfo("%d unique tasks found" %len(self.unique_tasks))
        
    def _get_unique_tasks(self, collection_indicator=None):
        for task in self.tasks:
            
            for task_details in task['task'].keys():
                
                if task_details == 'start_after':
                    s = rospy.Time(task['task'][task_details]['secs'],task['task'][task_details]['nsecs'])  
                    start = time.localtime(rospy.Time.to_sec(s))

                elif task_details == 'end_before':
                    e = rospy.Time(task['task'][task_details]['secs'],task['task'][task_details]['nsecs'])
                    end = time.localtime(rospy.Time.to_sec(e))

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
                    
            if action != 'cpm_action':
                if priority < 5000:
                    new_priority = random.randint(0,10)
                elif priority >= 5000 and < 50000:
                    new_priority = random.randint(0,100)
                elif priority >= 50000:
                    new_priority = random.randint(0,1000)

                self.unique_tasks.update({task_id : (new_priority, start, end)})
    

 
