#!/usr/bin/env python

import numpy as np
import read_tasks
import time
from sklearn.cluster import KMeans

# Zero rewards considered separately

class uncertain_rewards:
    
    def __init__(self):
        t = read_tasks.getTasks()
        self.tasks = t.unique_task_events
        self.rewards_day = self.get_rewards_by_day()
        
    def get_rewards_by_day(self):
        dated_tasks = dict()
       
        for task in self.tasks:
            start_t = self.tasks[task][1]
            end_t = self.tasks[task][2]
            
            if (start_t[0], start_t[1], start_t[2]) != (end_t[0], end_t[1], end_t[2]):
                p_l = self._get_prioritylist(dated_tasks, (start_t[0], start_t[1], start_t[2]))                
                end_t_new = (start_t[0], start_t[1], start_t[2], 23, 59, 59, start_t[6], start_t[7], start_t[8])    
                p_l = self._update_prioritylist(p_l,start_t, end_t_new, self.tasks[task][0])  
                if not all( z == 0 for z in p_l):
                    dated_tasks.update({ (start_t[0], start_t[1], start_t[2]) : p_l})
                
                p_l = self._get_prioritylist(dated_tasks,(end_t[0], end_t[1], end_t[2])) 
                start_t_new = (end_t[0], end_t[1], end_t[2], 0, 0, 0, end_t[6], end_t[7], end_t[8])
                p_l = self._update_prioritylist(p_l, start_t_new, end_t, self.tasks[task][0])
                if not all( z == 0 for z in p_l):
                    dated_tasks.update({(end_t[0], end_t[1], end_t[2]) :p_l})
                              
            else:
                p_l = self._get_prioritylist(dated_tasks, (start_t[0], start_t[1], start_t[2]))
                p_l = self._update_prioritylist(p_l,start_t, end_t, self.tasks[task][0])    
                if not all( z == 0 for z in p_l):
                    dated_tasks.update({ (start_t[0], start_t[1], start_t[2]) : p_l})
        
        return dated_tasks 
        
    def get_rewards(self):
        i = 0
        f = len(self.rewards_day)*[0]
        for day in self.rewards_day:
            f[i] = np.array(self.rewards_day[day])
            i= i+1
        f_t = np.array(f).T 
        clusters = 48*[0]
        prob = []
       
        for j in range(len(f_t)):
            p_cluster = dict()
            in_km = []   
            zero_count = 0 
            # Converting to suitable input for kmeans                             
            for k in range(len(f_t[j])):
                if f_t[j][k] != 0:
                    in_km.append([f_t[j][k]])
                else:
                    zero_count = zero_count +1 
            
            if not (all(element == 0 for element in f_t[j])): 
                kmeans = KMeans(n_clusters=4, random_state=0).fit(np.array(in_km))
                clusters[j] = [[0], kmeans.cluster_centers_]
                            
                for z in range(len(in_km)):
                    label = int(kmeans.labels_[z])+1
                    if label not in p_cluster:
                        val = 1
                    else:
                        val = p_cluster[label] + 1
                    p_cluster.update({ label : float(val)})    
                
                total = 0
                for count in p_cluster.values():
                    total = total + count
                # Taking Dirichlet hyper-parameter as alpha = 1 for a flat distribution_____ASK   
                for p in p_cluster:
                    prob_val = float(p_cluster[p]+1)/float(total+5+zero_count)
                    p_cluster.update({ p : round(prob_val,4)})
                p_cluster.update({ 0 : round(float(zero_count+1)/float(total+5+zero_count), 4)})  
                    
            else:
                p_cluster.update({ 0 : float(zero_count+1)/float(5+zero_count)})   
            prob.append(p_cluster)
                
        return clusters, prob
                    
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
            
   
        
if __name__ == '__main__':

    ur = uncertain_rewards()
    clusters, probability = ur.get_rewards()
    for i in range(len(probability)):
        print clusters[i], '---->' 
        for p in probability[i]:
            print p , ' : ', probability[i][p]
        
    
                      
    
