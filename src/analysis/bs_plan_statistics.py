#! /usr/bin/env python

from datetime import datetime, timedelta
import pandas as pd 
import numpy as np 
import roslib
import csv

###### ONLY FOR TIME BASED MODELS 
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
    dataset = 'D3'
    path = roslib.packages.get_pkg_dir('battery_scheduler') + '/data/'
   
    # fnames = [['p0rhct_30831819_', 'p0rhct_394959_', 'p0rhct_169179189_', 'p0rhct_209219229_', 'p0rhct_249259269_', 'p0rhct_110210310_'],['p0rhct_c4_30831819_', 'p0rhct_c4_394959_', 'p0rhct_c4_169179189_', 'p0rhct_c4_209219229_', 'p0rhct_c4_249259269_', 'p0rhct_c4_110210310_'], ['p0rhct_c5_30831819_', 'p0rhct_c5_394959_', 'p0rhct_c5_169179189_', 'p0rhct_c5_209219229_', 'p0rhct_c5_249259269_', 'p0rhct_c5_110210310_']]

    # fnames = [['p0rhct_30831819_', 'p0rhct_394959_', 'p0rhct_169179189_', 'p0rhct_209219229_', 'p0rhct_249259269_', 'p0rhct_110210310_'],['p6rhct_30831819_', 'p6rhct_394959_', 'p6rhct_169179189_', 'p6rhct_209219229_', 'p6rhct_249259269_', 'p6rhct_110210310_'], ['p4rhct_30831819_', 'p4rhct_394959_', 'p4rhct_169179189_', 'p4rhct_209219229_', 'p4rhct_249259269_', 'p4rhct_110210310_'], ['p5rhct_30831819_', 'p5rhct_394959_', 'p5rhct_169179189_', 'p5rhct_209219229_', 'p5rhct_249259269_', 'p5rhct_110210310_'], ['p1rhct_30831819_', 'p1rhct_394959_', 'p1rhct_169179189_', 'p1rhct_209219229_', 'p1rhct_249259269_', 'p1rhct_110210310_'], ['p3rhct_30831819_', 'p3rhct_394959_', 'p3rhct_169179189_', 'p3rhct_209219229_', 'p3rhct_249259269_', 'p3rhct_110210310_']]

    fnames = [['p0rhc_101011101210_', 'p0rhc_191020102110_', 'p0rhc_291030103110_','p0rhc_121113111411_'],['p6rhc_101011101210_', 'p6rhc_191020102110_', 'p6rhc_291030103110_', 'p6rhc_121113111411_'], ['p4rhc_101011101210_', 'p4rhc_191020102110_', 'p4rhc_291030103110_', 'p4rhc_121113111411_'], ['p5rhc_101011101210_', 'p5rhc_191020102110_', 'p5rhc_291030103110_', 'p5rhc_121113111411_'], ['p1rhc_101011101210_', 'p1rhc_191020102110_', 'p1rhc_291030103110_',  'p1rhc_121113111411_'], ['p3rhc_101011101210_', 'p3rhc_191020102110_', 'p3rhc_291030103110_', 'p3rhc_121113111411_']]

    # fnames = [['p0rhc_101011101210_', 'p0rhc_191020102110_', 'p0rhc_291030103110_', 'p0rhc_411511611_', 'p0rhc_121113111411_', 'p0rhc_112212312_'],['p6rhc_101011101210_', 'p6rhc_191020102110_', 'p6rhc_291030103110_', 'p6rhc_411511611_', 'p6rhc_121113111411_', 'p6rhc_112212312_'], ['p4rhc_101011101210_', 'p4rhc_191020102110_', 'p4rhc_291030103110_', 'p4rhc_411511611_', 'p4rhc_121113111411_', 'p4rhc_112212312_'], ['p5rhc_101011101210_', 'p5rhc_191020102110_', 'p5rhc_291030103110_', 'p5rhc_411511611_', 'p5rhc_121113111411_', 'p5rhc_112212312_'], ['p1rhc_101011101210_', 'p1rhc_191020102110_', 'p1rhc_291030103110_', 'p1rhc_411511611_', 'p1rhc_121113111411_', 'p1rhc_112212312_'], ['p3rhc_101011101210_', 'p3rhc_191020102110_', 'p3rhc_291030103110_', 'p3rhc_411511611_', 'p3rhc_121113111411_', 'p3rhc_112212312_']]
    
    overall_rew = []
    overall_work = []
    overall_40 = []

    for file_set in fnames:
        rew_model = []
        work_model = []
        under40_model = []
        for f in file_set:
            for i in range(1,4):
                print(path+f+str(i))
                rew, worktime, under40 = get_kpi((path+f+str(i)))
                rew_model.append(rew)
                work_model.append(worktime)
                under40_model.append(under40)
        df = pd.DataFrame(zip(rew_model, work_model, under40_model), columns =['rewards', 'active_time', 'under40'])
        model_name = file_set[0].split('_')[0] + '_' + dataset 
        df.to_csv('/home/milan/workspace/strands_ws/src/battery_scheduler/data/csv_files/'+model_name+'.csv', header=True, index=False)
    
        overall_rew.append(np.mean(rew_model))
        overall_work.append(np.mean(work_model))
        overall_40.append(np.mean(under40_model))

    ov_df =  pd.DataFrame(zip(overall_rew, overall_work, overall_40), columns =['rewards', 'active_time', 'under40'])
    ov_df.to_csv('/home/milan/workspace/strands_ws/src/battery_scheduler/data/csv_files/timebased_overall_'+dataset+'_task_models.csv', header=True, index=False)