#! /usr/bin/env python

import pymongo
import rospy
import time
import numpy as np

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
                    
                elif task_details == 'action':
                    action = task['task'][task_details]
                    
            unique_tasks.update({task_id : (action, priority, start, end)})
                
           
                   
        rospy.loginfo("%d unique tasks found" %len(unique_tasks))         
        return unique_tasks
        
if __name__ == '__main__':
    
    t = getTasks()
    tasks = t.unique_task_events
    action_plist = dict()
    
    for task in tasks:
        action = tasks[task][0]
        if action not in action_plist:
            p_l = []
      
        else:
            p_l = action_plist[action]
        
        if tasks[task][1] not in p_l:
            p_l.append(tasks[task][1])
            
        action_plist.update({action : p_l})
    count = 0    
    for action in action_plist:
        p = np.array(action_plist[action])
        action_plist.update({ action : np.mean(p)})
        
        if action_plist[action] != 0:
            print action, ' : ', action_plist[action]    
            count = count+1
    print 'Total number of actions with priority values: ' , count
    
