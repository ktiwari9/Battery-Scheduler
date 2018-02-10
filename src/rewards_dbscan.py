#!/usr/bin/env python

import time
import read_tasks
import numpy as np
from collections import Counter
from scipy.cluster.vq import kmeans2 
from scipy.cluster.hierarchy import fcluster, linkage
from sklearn.cluster import DBSCAN
class uncertain_rewards:
    
    def __init__(self, validation):
        t = read_tasks.getTasks()
        self.tasks = t.unique_tasks
        self.time_int = 8
        rewards_by_day = self.get_rewards_by_day()
        self.rewards_day = dict()
        
        if validation == False:
            for day in rewards_by_day:
                if day[1] == 11 or day[1] == 12 or day[0] == 2017:
                    self.rewards_day.update({day: rewards_by_day[day]})
            #for day in self.rewards_day:
            #    print day, '---------------------'
            #    print self.rewards_day[day]
            #print 'Days Available: ', len(self.rewards_day)
        elif validation == True: # for separating test rewards and validation rewards.
            self.test_rewards = dict()
            no_test_start = 0
            no_test_end = 1
            num = 0
            for day in rewards_by_day:
                if day[1] == 8 and day[0] == 2017 and num >= no_test_start and num < no_test_end:
                    print num
                    print day 
                    self.test_rewards.update({ day : rewards_by_day[day]})    
                elif day[1] == 11 or day[1] == 12 or day[0] == 2017:
                    self.rewards_day.update({ day : rewards_by_day[day]})
                if day[1] == 8 and day[0] == 2017:
                    num = num+1

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
        len_clusters=[] 
        clusters = []
        prob = []
        for j in range(len(f_t)):
            zero_count = 0
            in_km = []
            # Converting to suitable input for clustering                            
            for k in range(len(f_t[j])):
                if f_t[j][k] == 0:
                    zero_count = zero_count +1 
                elif f_t[j][k] < 1000000:
                    in_km.append([f_t[j][k]])
      
            cl_centre, p_cluster = self._form_clusters(in_km, 1000, 3, zero_count)

            print len(cl_centre)
            print cl_centre
            print p_cluster
            len_clusters.append(len(cl_centre))

            expected_reward = 0
            for x in range(len(cl_centre)):
                expected_reward = expected_reward + p_cluster[x]*cl_centre[x]
            print expected_reward    
            
            prob.append(p_cluster)
            clusters.append(cl_centre)

        print Counter(len_clusters), 'cluster_info'
        return clusters , prob

    def _form_clusters(self, in_km, epsilon, samples, zero_count):
        db = DBSCAN(eps=epsilon, min_samples=samples, metric='euclidean').fit(in_km)
        core_sample_indices = db.core_sample_indices_
        labels = db.labels_
        cores_dict = dict()

        ## considering only core samples for cluster centres
        #for index in core_sample_indices:
        #    label = labels[index]
        #    if label not in cores_dict:
        #        core_list = [in_km[index][0]]
        #    else:
        #        core_list = cores_dict[label]
        #        core_list.append(in_km[index][0])

        #    cores_dict.update({label:core_list})

        ## considering all samples in a cluster for cluster centres
        for i in range(len(in_km)):
            label = labels[i]
            if label != -1:
                if label not in cores_dict:
                    sample_list = [in_km[i][0]]
                else:
                    sample_list = cores_dict[labels[i]]
                    sample_list.append(in_km[i][0])
                cores_dict.update({labels[i]:sample_list})

        cl_centres = len(cores_dict.keys())*[0]
        for label, core_list in cores_dict.items():
            cl_centres[label] = np.average(np.array(core_list))
        cl_centres.append(0)
            
        prob = (len(cl_centres)-1)*[0]
        count_id = Counter(labels)
        for item, count in count_id.items():
            if item != -1:
                prob[item] = count
        prob.append(zero_count)

        total_count = float(np.sum(np.array(prob)))

        for i in range(len(prob)):
            prob[i] = float(prob[i])/total_count


        return cl_centres, prob

    def _get_prioritylist(self, dictionary, date):
        if date not in dictionary:
            p_list = (self.time_int)*[0]
        else:
            p_list = dictionary[date]
        return p_list
                    
    def _update_prioritylist(self,p_list, start_time, end_time, priority):
        for i in range(self.time_int):
            s_int = i*1800
            e_int = i*1800 +1800
            
            if i == self.time_int-1:
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

