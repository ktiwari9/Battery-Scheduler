#! /usr/bin/env python

from datetime import datetime, timedelta
import pandas as pd 
import numpy as np 
import roslib
import csv

### measuring rewards obtained, time spent working, time with battery under 40.

def get_kpi(fname):
    df = pd.read_csv(fname, sep=' ', index_col=False, dtype={'actual_reward':np.float64, 'obtained_reward':np.float64, 'battery':np.int32})
       
    total_reward = df['actual_reward'].sum()
    reward_obtained = df['obtained_reward'].sum()
    rew_percent = (float(reward_obtained)/total_reward)*100

    available_work = df[df['actual_reward'] != 0.0].shape[0]
    work_time = df[df['obtained_reward'] != 0.0].shape[0]
    work_percent = (float(work_time)/available_work)*100
    
    under_40 = df[df['battery'] < 40].shape[0]
    total_time = df.shape[0]    
    under40_percent = (float(under_40)/total_time)*100 

    return rew_percent, work_percent, under40_percent
       
if __name__ == "__main__":

    path = roslib.packages.get_pkg_dir('battery_scheduler') + '/data/'
    fnames = [['p0rhc_711811911_', 'p0rhc_101111111211_', 'p0rhc_131114111511_']]#,['p6rhc_711811911_', 'p6rhc_101111111211_', 'p6rhc_131114111511_'], ['p4rhc_711811911_', 'p4rhc_101111111211_', 'p4rhc_131114111511_'], ['p5rhc_711811911_', 'p5rhc_101111111211_', 'p5rhc_131114111511_'], ['p1rhc_711811911_', 'p1rhc_101111111211_', 'p1rhc_131114111511_'], ['p3rhc_711811911_', 'p3rhc_101111111211_', 'p3rhc_131114111511_'], ['rbc1_711811911_', 'rbc1_101111111211_', 'rbc1_131114111511_'], ['rbc2_711811911_', 'rbc2_101111111211_', 'rbc2_131114111511_'], ['rbc3_711811911_', 'rbc3_101111111211_', 'rbc3_131114111511_']]

    overall_rew = []
    overall_work = []
    overall_40 = []

    for file_set in fnames:
        rew_model = []
        work_model = []
        under40_model = []
        for f in file_set:
            for i in range(1,4):
                rew, worktime, under40 = get_kpi((path+f+str(i)))
                rew_model.append(rew)
                work_model.append(worktime)
                under40_model.append(under40)
        df = pd.DataFrame(zip(rew_model, work_model, under40_model), columns =['rewards', 'active_time', 'under40'])
        model_name = file_set[0].split('_')[0]
        df.to_csv('/home/milan/workspace/strands_ws/src/battery_scheduler/data/csv_files/'+model_name+'.csv', header=True, index=False)
    
        overall_rew.append(np.mean(rew_model))
        overall_work.append(np.mean(work_model))
        overall_40.append(np.mean(under40_model))

    ov_df =  pd.DataFrame(zip(overall_rew, overall_work, overall_40), columns =['rewards', 'active_time', 'under40'])
    ov_df.to_csv('/home/milan/workspace/strands_ws/src/battery_scheduler/data/csv_files/timebased_overall_models.csv', header=True, index=False)