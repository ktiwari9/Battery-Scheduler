#!/usr/bin/env python

import read_tasks
import time
import numpy as np

#Aggregate and maximum, fixing previous error while adding tasks with diffrent start and end date.

class avg_rewards:

    def __init__(self):
        t = read_tasks.getTasks()
        self.tasks = t.unique_task_events
        self.rewards_day = self.get_rewards_by_day()
        self.avg_rewards = self.get_avg_rewards()
        
    def get_rewards_by_day(self):
        dated_tasks = dict()
        for task in self.tasks:
            start_t = self.tasks[task][1]
            end_t = self.tasks[task][2]
            
            if (start_t[0], start_t[1], start_t[2]) != (end_t[0], end_t[1], end_t[2]):
                if (start_t[0], start_t[1], start_t[2]) not in dated_tasks:
                    p_l = 48*[0]
                else:
                    p_l = dated_tasks[(start_t[0], start_t[1], start_t[2])]
                
                end_t_new = (start_t[0], start_t[1], start_t[2], 23, 59, 59, start_t[6], start_t[7], start_t[8])    
                p_l = self._update_prioritylist(p_l,start_t, end_t_new, self.tasks[task][0])    
                dated_tasks.update({ (start_t[0], start_t[1], start_t[2]) : p_l})
                
                if (end_t[0], end_t[1], end_t[2]) not in dated_tasks:
                    p_l = 48*[0]
                else:
                    p_l = dated_tasks[(end_t[0], end_t[1], end_t[2])]    
                    
                start_t_new = (end_t[0], end_t[1], end_t[2], 0, 0, 0, end_t[6], end_t[7], end_t[8])
                p_l = self._update_prioritylist(p_l, start_t_new, end_t, self.tasks[task][0])
                dated_tasks.update({(end_t[0], end_t[1], end_t[2]) :p_l})
                
            else:
                if (start_t[0], start_t[1], start_t[2]) not in dated_tasks:
                    p_l = 48*[0]
                else:
                    p_l = dated_tasks[(start_t[0], start_t[1], start_t[2])]
                    
                p_l = self._update_prioritylist(p_l,start_t, end_t, self.tasks[task][0])    
                dated_tasks.update({ (start_t[0], start_t[1], start_t[2]) : p_l})
                
        return dated_tasks 
        
    def get_avg_rewards(self):
        i = 0
        f = len(self.rewards_day)*[0]
        for day in self.rewards_day:
            f[i] = np.array(self.rewards_day[day])
            i= i+1
        return np.mean(np.array(f), axis=0)
            
                
                        
    def _update_prioritylist(self,p_list, start_time, end_time, priority):
        for i in range(48):
            s_int = i*1800
            e_int = i*1800 +1800
            
            if i == 47:
                e_int = i*1800 +1800 - 1
            if self._belongs_to_interval( s_int, e_int, start_time, end_time):
                p_list[i] = p_list[i] + priority    
                
        return p_list      
            
            
    def _belongs_to_interval(self,start_i, end_i, start_t, end_t):
         
         s = time.gmtime(start_i)
         e = time.gmtime(end_i)  
         start_i =  (start_t[0], start_t[1], start_t[2], s[3], s[4], s[5], start_t[6], start_t[7], start_t[8])
         end_i = (end_t[0], end_t[1], end_t[2], e[3], e[4], e[5], end_t[6], end_t[7], end_t[8]) 
             
         if time.mktime(start_t) < time.mktime(end_i) and time.mktime(end_t) > time.mktime(start_i):
             return True
             
         else:
             return False        

