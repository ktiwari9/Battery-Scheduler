#!/usr/bin/env python

import pymongo
import rospy
import time
from datetime import datetime
import read_tasks
import numpy as np


# Find average over all values.
class avgRewards2:

    def __init__(self):
        t = read_tasks.getTasks()
        self.tasks = t.unique_task_events
        self.avg_rewards = self.get_avg_rewards()
        
    def get_avg_rewards(self):
         interval = rospy.Time.to_sec(rospy.Time(1800,0))
         dt = -1

         for task in self.tasks:
             start_t = self.tasks[task][1]
             end_t = self.tasks[task][2]
             start = rospy.Time.to_sec(rospy.Time(0))
             dt = dt+1
             p = 48*[0]
                           
             for i in range(48):
                 end = start + interval
                 if self._belongs_to_interval(start, end, start_t, end_t):
                     p[i] = self.tasks[task][0]       
                 start = start + interval 
             
             if dt == 0:
                 p_t = np.array([p])
             else:
                 p_t = np.concatenate((p_t, np.array([p])), axis = 0)     
             
         return np.mean(p_t, axis=0) 
        
    def _belongs_to_interval(self,start_i, end_i, start_t, end_t):
         if start_t[3] < end_t[3]:
             s = time.localtime(start_i)
             e = time.localtime(end_i)
             start_i =  (start_t[0], start_t[1], start_t[2], s[3], s[4], s[5], start_t[6], start_t[7], start_t[8])
             end_i = (start_t[0], start_t[1], start_t[2], e[3], e[4], e[5], start_t[6], start_t[7], start_t[8]) 
             
         else:
             s = time.localtime(start_i)
             e = time.localtime(end_i)  
             start_i =  (end_t[0], end_t[1], end_t[2], s[3], s[4], s[5], end_t[6], end_t[7], end_t[8])
             end_i = (end_t[0], end_t[1], end_t[2], e[3], e[4], e[5], end_t[6], end_t[7], end_t[8]) 
             
         if start_t < end_i and end_t > start_i:
             return True
             
         else:
             return False
        

    

