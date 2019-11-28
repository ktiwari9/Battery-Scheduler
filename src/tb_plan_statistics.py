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
    df_tasks = df[df['action'] == 'gather_reward']
    reward_obtained = df_tasks['actual_reward'].sum()
    rew_percent = (float(reward_obtained)/total_reward)*100

    work_time = 0
    available_work = 0 
    under_40 = 0 
    for i in range(1,df.shape[0]):
        row = df.iloc[i]
        prev_row = df.iloc[i-1]
        ts = datetime.strptime(row['day']+' '+row['time'], '%Y-%m-%d %H:%M:%S')
        prev_ts = datetime.strptime(prev_row['day']+' '+prev_row['time'], '%Y-%m-%d %H:%M:%S')
        
        if prev_row['action'] == 'gather_reward':
            work_time += (ts - prev_ts).total_seconds()
                
        if float(prev_row['actual_reward']) != 0.0:
            available_work += (ts - prev_ts).total_seconds()

        if i == df.shape[0]-1 and row['action'] == 'gather_reward':
            work_time += 20*60
            available_work += 20*60

        if int(prev_row['battery']) < 40:
            if i != df.shape[0]-1:
                if int(row['battery']) < 40:
                    under_40 += (ts - prev_ts).total_seconds()
                else:
                    battery_change = int(row['battery'])-int(prev_row['battery'])
                    time_taken = (ts - prev_ts).total_seconds()
                    unit_battery_change = time_taken/battery_change
                    time_till40 = unit_battery_change*(40-int(prev_row['battery']))
                    under_40 += time_till40
            else:
                if prev_row['action'] == 'gather_reward':
                    under_40 += 20*60
                else:
                    if int(prev_row['battery']) < 37:
                        under_40 += 20
                    else:
                        under_40 += 10
    
    work_percent = (work_time/available_work)*100
    under40_percent = (under_40/(3*24*60*60))*100  ## for 3 days 

    return rew_percent, work_percent, under40_percent
       
if __name__ == "__main__":

    path = roslib.packages.get_pkg_dir('battery_scheduler') + '/data/'
    fnames = [['p0tbrhc_711811911_', 'p0tbrhc_101111111211_', 'p0tbrhc_131114111511_'], ['p6tbrhc_711811911_', 'p6tbrhc_101111111211_', 'p6tbrhc_131114111511_'], ['p4tbrhc_711811911_', 'p4tbrhc_101111111211_', 'p4tbrhc_131114111511_'], ['p5tbrhc_711811911_', 'p5tbrhc_101111111211_', 'p5tbrhc_131114111511_'], ['p1tbrhc_711811911_', 'p1tbrhc_101111111211_', 'p1tbrhc_131114111511_'], ['p3tbrhc_711811911_', 'p3tbrhc_101111111211_', 'p3tbrhc_131114111511_'], ['tbrbc1_711811911_', 'tbrbc1_101111111211_', 'tbrbc1_131114111511_'], ['tbrbc2_711811911_', 'tbrbc2_101111111211_', 'tbrbc2_131114111511_'], ['tbrbc3_711811911_', 'tbrbc3_101111111211_', 'tbrbc3_131114111511_']]

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
    ov_df.to_csv('/home/milan/workspace/strands_ws/src/battery_scheduler/data/csv_files/taskbased_overall_models.csv', header=True, index=False)