#!/usr/bin/env python

import time
import read_tasks
import numpy as np
from scipy.cluster.vq import kmeans2 
from scipy.cluster.hierarchy import fcluster, linkage
import matplotlib.pyplot as plt

class uncertain_rewards:
    
    def __init__(self, validation):
        t = read_tasks.getTasks()
        self.tasks = t.unique_tasks
        self.min_clst = 5 # required - 1
        
        if validation == False:
            rewards_by_day = self.get_rewards_by_day()
            self.rewards_day = dict()
            for day in rewards_by_day:
                if day[1] == 11 or day[1] == 12 or day[0] == 2017:
                    self.rewards_day.update({day: rewards_by_day[day]})
            #for day in self.rewards_day:
            #    print day, '---------------------'
            #    print self.rewards_day[day]
            #print 'Days Available: ', len(self.rewards_day)
        elif validation == True: # for separating test rewards and validation rewards.
            rewards_by_day = self.get_rewards_by_day()
            self.test_rewards = dict()
            self.rewards_day = dict()
            no_test_start = 4
            no_test_end = 7
            num = 0
            for day in rewards_by_day:
                if day[1] == 12:# and day[0] == 2017:
                    if num >= no_test_start and num < no_test_end:
                        self.test_rewards.update({ day : rewards_by_day[day]})
                    num = num+1 
                elif day[1] == 11 or day[1] == 12 or day[0] == 2017:
                    self.rewards_day.update({ day : rewards_by_day[day]})
                   
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
        
        clusters = []
        prob = []
        for j in range(len(f_t)):
            zero_count = 0
            in_km = []
            # Converting to suitable input for kmeans    #change if you want zeros separately                         
            for k in range(len(f_t[j])):
                if f_t[j][k] < 1000000: #and f_t[j][k] != 0:
                    in_km.append([f_t[j][k]])
                #elif f_t[j][k] == 0:
                #    zero_count = zero_count + 1
                    
            cl_centre, p_cluster, current_dist = self._form_clusters(in_km, 100)# zero_count) #change if you want zeros separately
            print len(cl_centre)
            print cl_centre
            print p_cluster
            
            expected_reward = 0
            for x in range(len(cl_centre)):
                expected_reward = expected_reward + p_cluster[x]*cl_centre[x]
            print expected_reward    
            
            prob.append(p_cluster)
            clusters.append(cl_centre)

	return clusters , prob
        
        
    def _form_clusters(self, in_km, current_dist, zero_count=None): #change if you want zeros separately
        z = linkage(in_km,metric='euclidean', method='centroid')   
        clstred_data = fcluster(z, current_dist, criterion='distance')
        unique_id = np.unique(clstred_data)         
        clusters = dict()
        for i in range(len(clstred_data)):
            cl_id = clstred_data[i]-1
            if cl_id not in clusters:
                clusters.update({ cl_id : [in_km[i][0]] })
            else:
                data_list = clusters[cl_id]
                data_list.append(in_km[i][0])
                clusters.update({ cl_id : data_list})
         
        cl_centre = len(clusters.keys())*[0] 
        for cl_id, data in clusters.items():
            centre = np.mean(np.array(data))
            cl_centre[cl_id] = centre
        #cl_centre.append(0)  #change if you want zeros separately
        
        p_cluster = {}
        for z in range(len(in_km)):
            label = clstred_data[z]-1
            if label not in p_cluster:
                val = 1
            else:
                val = p_cluster[label] + 1
            p_cluster.update({ label : float(val)})
        #p_cluster.update({ len(cl_centre) - 1 : zero_count})   #change if you want zeros separately
            
        total = np.sum(np.array(p_cluster.values()))
        for p in p_cluster:
            prob_val = float(p_cluster[p])/float(total)
            p_cluster.update({ p : prob_val})  
        
        if current_dist < 1000 and len(unique_id) >= 4:
            cl_centre, p_cluster, current_dist = self._form_clusters(in_km, 2*current_dist, zero_count)
        elif current_dist >= 1000 and current_dist < 10000 and len(unique_id) >= 6:
           cl_centre, p_cluster, current_dist = self._form_clusters(in_km, 2*current_dist, zero_count)
        elif current_dist >= 10000 and len(unique_id) > 9:
            cl_centre, p_cluster, current_dist = self._form_clusters(in_km, 2*current_dist, zero_count)
        
        return cl_centre, p_cluster, current_dist
    
    def _get_prioritylist(self, dictionary, date):
        if date not in dictionary:
            p_list = 48*[0]
        else:
            p_list = dictionary[date]
        return p_list
                    
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
    ur = uncertain_rewards(False)
#    plotly.tools.set_credentials_file(username='MilanMariyaTomy', api_key='aoeKNl0QTEWLTb1jlFcB')
#    rewards_day = ur.get_rewards_by_day()
#    i = 0
#    f = len(rewards_day)*[0]
#   for day in rewards_day:
#        f[i] = np.array(rewards_day[day])
#        i= i+1
#    f_t = np.array(f).T 
#    traces = []
#    for j in range(len(f_t)):
#        trace = go.Scatter(x=len(f_t[j])*[j], y=f_t[j], mode='markers')
#        traces.append(trace)
#    layout = go.Layout(title='Rewards across Days per Time Interval',xaxis=dict(title='Time Interval'), yaxis=dict(title='Total rewards'))
#    fig1 = go.Figure(data=traces, layout=layout)
#    plot_url = py.plot(fig1, filename='action_priorities')
    clusters, prob = ur.get_rewards()       

