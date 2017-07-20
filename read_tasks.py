#!/usr/bin/env python

import pymongo
import rospy
import time


#### reading in only the unique tasks, ignoring the task events

class getTasks:
    
    def __init__(self):
        client = pymongo.MongoClient(rospy.get_param("mongodb_host", "localhost"),
            rospy.get_param("mongodb_port", 62345))
        rospy.loginfo("Connecting to mongodB")        
        self.tasks = client.dump.task_events_unique.find(None)
        no_tasks = client.dump.task_events_unique.find(None).count()
        rospy.loginfo("Connection established, %d tasks being analysed" %no_tasks)
        self.unique_task_events = self._get_unique_tasks()

    def _get_unique_tasks(self):
        unique_tasks = dict()
               
        for task in self.tasks:
            
            for task_details in task['task'].keys():
                
                if task_details == 'start_after':
                    s = rospy.Time(task['task'][task_details]['secs'],task['task'][task_details]['nsecs'])  
                    start = time.localtime(rospy.Time.to_sec(s))

                elif task_details == 'end_before':
                    e = rospy.Time(task['task'][task_details]['secs'],task['task'][task_details]['nsecs'])
                    end = time.localtime(rospy.Time.to_sec(e))

                elif task_details == 'task_id':
                    task_id = task['task'][task_details]
                    
                elif task_details == 'priority':
                    priority = task['task'][task_details]
                    
            unique_tasks.update({task_id : (priority, start, end)})
                
           
                   
        rospy.loginfo("%d unique tasks found" %len(unique_tasks))         
        return unique_tasks
    
  


   
