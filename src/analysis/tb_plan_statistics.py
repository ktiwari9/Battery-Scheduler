#! /usr/bin/env python

from datetime import datetime, timedelta
import matplotlib.pyplot as plt
from matplotlib import cm
from scipy import stats
import pandas as pd 
import numpy as np 
import roslib
import csv

### measuring rewards obtained, time spent working, time with battery under 40.

def get_kpi(fname):  ## overall across 3 days
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

        if i == df.shape[0]-1 and row['action'] == 'gather_reward': ## assuming last task is 20 minutes
            work_time += 20*60
            available_work += 20*60

        if int(prev_row['battery']) < 40:
            if int(row['battery']) < 40:
                under_40 += (ts - prev_ts).total_seconds()
            else:
                battery_change = int(row['battery'])-int(prev_row['battery'])
                time_taken = (ts - prev_ts).total_seconds()
                unit_battery_change = time_taken/battery_change
                time_till40 = unit_battery_change*(40-int(prev_row['battery']))
                under_40 += time_till40

        if i == df.shape[0]-1 and row['battery'] < 40:
            if row['action'] == 'gather_reward':  ## assuming last task extends for 20 mins
                under_40 += 20*60
            else:
                if int(row['battery']) < 37:  ## because too lazy to cal actual value.
                    under_40 += 20
                else:
                    under_40 += 10
    
    work_percent = (work_time/available_work)*100
    under40_percent = (under_40/(3*24*60*60))*100  ## for 3 days 

    return rew_percent, work_percent, under40_percent

def get_daywise_kpi(fname):
    df_alldays = pd.read_csv(fname, sep=' ', index_col=False, dtype={'actual_reward':np.float64, 'obtained_reward':np.float64, 'battery':np.int32})

    rew_percent_days = []
    work_percent_days = []
    under40_percent_days = []

    unique_days = df_alldays['day'].unique()
    for day in unique_days:
        df = df_alldays[df_alldays['day'] == day]
       
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

            if i == df.shape[0]-1 and row['action'] == 'gather_reward': # assuming last task duration is 20 mins
                work_time += 20*60
                available_work += 20*60

            if int(prev_row['battery']) < 40:
                if int(row['battery']) < 40:
                    under_40 += (ts - prev_ts).total_seconds()
                else:
                    battery_change = int(row['battery'])-int(prev_row['battery'])
                    time_taken = (ts - prev_ts).total_seconds()
                    unit_battery_change = time_taken/battery_change
                    time_till40 = unit_battery_change*(40-int(prev_row['battery']))
                    under_40 += time_till40

            if i == df.shape[0]-1 and row['battery'] < 40:
                if row['action'] == 'gather_reward':  ## assuming last task extends for 20 mins
                    under_40 += 20*60
                else:
                    if int(row['battery']) < 37:  ## because too lazy to cal actual value.
                        under_40 += 20
                    else:
                        under_40 += 10

        if available_work != 0.0:    
            rew_percent_days.append(rew_percent)
            work_percent = (work_time/available_work)*100
            work_percent_days.append(work_percent)
            under40_percent = (under_40/(24*60*60))*100  ## for 1 day
            under40_percent_days.append(under40_percent) 

    return rew_percent_days, work_percent_days, under40_percent_days
       
if __name__ == "__main__":
    dataset = 'deployment1'
    tag1 = 'b'
    tag2 = 'set' ## daily or set 
    
    path = '/home/milan/workspace/strands_ws/src/battery_scheduler/data/'
    # path = roslib.packages.get_pkg_dir('battery_scheduler') + '/data/'
    # path = '/home/milan/workspace/strands_ws/src/battery_scheduler/data/dec2019_results(jounal)/'

    
    ## general
    # fnames = [['p0tbrhc_711811911_', 'p0tbrhc_101111111211_', 'p0tbrhc_131114111511_', 'p0tbrhc_181119112011_', 'p0tbrhc_241125112611_', 'p0tbrhc_281129113011_', 'p0tbrhc_16111711_', 'p0tbrhc_211122112311_', 'p0tbrhc_2711112_'],['p6tbrhc_711811911_', 'p6tbrhc_101111111211_', 'p6tbrhc_131114111511_', 'p6tbrhc_181119112011_', 'p6tbrhc_241125112611_', 'p6tbrhc_281129113011_', 'p6tbrhc_16111711_', 'p6tbrhc_211122112311_', 'p6tbrhc_2711112_'], ['p4tbrhc_711811911_', 'p4tbrhc_101111111211_', 'p4tbrhc_131114111511_', 'p4tbrhc_181119112011_', 'p4tbrhc_241125112611_', 'p4tbrhc_281129113011_', 'p4tbrhc_16111711_', 'p4tbrhc_211122112311_', 'p4tbrhc_2711112_'],  ['p3tbrhc_711811911_', 'p3tbrhc_101111111211_', 'p3tbrhc_131114111511_', 'p3tbrhc_181119112011_', 'p3tbrhc_241125112611_', 'p3tbrhc_281129113011_', 'p3tbrhc_16111711_', 'p3tbrhc_211122112311_', 'p3tbrhc_2711112_'], ['tbrbc1_711811911_', 'tbrbc1_101111111211_', 'tbrbc1_131114111511_', 'tbrbc1_181119112011_', 'tbrbc1_241125112611_', 'tbrbc1_281129113011_', 'tbrbc1_16111711_', 'tbrbc1_211122112311_', 'tbrbc1_2711112_'], ['tbrbc2_711811911_', 'tbrbc2_101111111211_', 'tbrbc2_131114111511_', 'tbrbc2_181119112011_', 'tbrbc2_241125112611_', 'tbrbc2_281129113011_', 'tbrbc2_16111711_', 'tbrbc2_211122112311_', 'tbrbc2_2711112_'], ['tbrbc3_711811911_', 'tbrbc3_101111111211_', 'tbrbc3_131114111511_', 'tbrbc3_181119112011_', 'tbrbc3_241125112611_', 'tbrbc3_281129113011_', 'tbrbc3_16111711_', 'tbrbc3_211122112311_', 'tbrbc3_2711112_']]

    ## cluster 
    # fnames = [['p0tbrhc_c1_711811911_', 'p0tbrhc_c1_101111111211_', 'p0tbrhc_c1_131114111511_', 'p0tbrhc_c1_181119112011_', 'p0tbrhc_c1_241125112611_', 'p0tbrhc_c1_281129113011_'],    ['p0tbrhc_c2_711811911_', 'p0tbrhc_c2_101111111211_', 'p0tbrhc_c2_131114111511_', 'p0tbrhc_c2_181119112011_', 'p0tbrhc_c2_241125112611_', 'p0tbrhc_c2_281129113011_'],    ['p0tbrhc_c3_711811911_', 'p0tbrhc_c3_101111111211_', 'p0tbrhc_c3_131114111511_', 'p0tbrhc_c3_181119112011_', 'p0tbrhc_c3_241125112611_', 'p0tbrhc_c3_281129113011_'],    ['p0tbrhc_c4_711811911_', 'p0tbrhc_c4_101111111211_', 'p0tbrhc_c4_131114111511_', 'p0tbrhc_c4_181119112011_', 'p0tbrhc_c4_241125112611_', 'p0tbrhc_c4_281129113011_'],['p0tbrhc_c5_711811911_', 'p0tbrhc_c5_101111111211_', 'p0tbrhc_c5_131114111511_', 'p0tbrhc_c5_181119112011_', 'p0tbrhc_c5_241125112611_', 'p0tbrhc_c5_281129113011_'],['p0tbrhc_c7_711811911_', 'p0tbrhc_c7_101111111211_', 'p0tbrhc_c7_131114111511_', 'p0tbrhc_c7_181119112011_', 'p0tbrhc_c7_241125112611_', 'p0tbrhc_c7_281129113011_'], ['p0tbrhc_c9_711811911_', 'p0tbrhc_c9_101111111211_', 'p0tbrhc_c9_131114111511_', 'p0tbrhc_c9_181119112011_', 'p0tbrhc_c9_241125112611_', 'p0tbrhc_c9_281129113011_']]

    ## horizon
    # fnames = [['p0tbrhc_h12_711811911_', 'p0tbrhc_h12_101111111211_', 'p0tbrhc_h12_131114111511_', 'p0tbrhc_h12_181119112011_', 'p0tbrhc_h12_241125112611_', 'p0tbrhc_h12_281129113011_'],['p0tbrhc_h24_711811911_', 'p0tbrhc_h24_101111111211_', 'p0tbrhc_h24_131114111511_', 'p0tbrhc_h24_181119112011_', 'p0tbrhc_h24_241125112611_', 'p0tbrhc_h24_281129113011_'], ['p0tbrhc_h36_711811911_', 'p0tbrhc_h36_101111111211_', 'p0tbrhc_h36_131114111511_', 'p0tbrhc_h36_181119112011_', 'p0tbrhc_h36_241125112611_', 'p0tbrhc_h36_281129113011_'], ['p0tbrhc_c2_711811911_', 'p0tbrhc_c2_101111111211_', 'p0tbrhc_c2_131114111511_', 'p0tbrhc_c2_181119112011_', 'p0tbrhc_c2_241125112611_', 'p0tbrhc_c2_281129113011_']]

    ## Battery models
    fnames = fnames = [['p0tb3rhc_711811911_', 'p0tb3rhc_101111111211_', 'p0tb3rhc_131114111511_', 'p0tb3rhc_181119112011_', 'p0tb3rhc_241125112611_', 'p0tb3rhc_281129113011_', 'p0tb3rhc_16111711_', 'p0tb3rhc_211122112311_', 'p0tb3rhc_2711112_'],['p6tb3rhc_711811911_', 'p6tb3rhc_101111111211_', 'p6tb3rhc_131114111511_', 'p6tb3rhc_181119112011_', 'p6tb3rhc_241125112611_', 'p6tb3rhc_281129113011_', 'p6tb3rhc_16111711_', 'p6tb3rhc_211122112311_', 'p6tb3rhc_2711112_'], ['p4tb3rhc_711811911_', 'p4tb3rhc_101111111211_', 'p4tb3rhc_131114111511_', 'p4tb3rhc_181119112011_', 'p4tb3rhc_241125112611_', 'p4tb3rhc_281129113011_', 'p4tb3rhc_16111711_', 'p4tb3rhc_211122112311_', 'p4tb3rhc_2711112_'],  ['p3tb3rhc_711811911_', 'p3tb3rhc_101111111211_', 'p3tb3rhc_131114111511_', 'p3tb3rhc_181119112011_', 'p3tb3rhc_241125112611_', 'p3tb3rhc_281129113011_', 'p3tb3rhc_16111711_', 'p3tb3rhc_211122112311_', 'p3tb3rhc_2711112_'], ['tb3rbc1_711811911_', 'tb3rbc1_101111111211_', 'tb3rbc1_131114111511_', 'tb3rbc1_181119112011_', 'tb3rbc1_241125112611_', 'tb3rbc1_281129113011_', 'tb3rbc1_16111711_', 'tb3rbc1_211122112311_', 'tb3rbc1_2711112_'], ['tb3rbc2_711811911_', 'tb3rbc2_101111111211_', 'tb3rbc2_131114111511_', 'tb3rbc2_181119112011_', 'tb3rbc2_241125112611_', 'tb3rbc2_281129113011_', 'tb3rbc2_16111711_', 'tb3rbc2_211122112311_', 'tb3rbc2_2711112_'], ['tb3rbc3_711811911_', 'tb3rbc3_101111111211_', 'tb3rbc3_131114111511_', 'tb3rbc3_181119112011_', 'tb3rbc3_241125112611_', 'tb3rbc3_281129113011_', 'tb3rbc3_16111711_', 'tb3rbc3_211122112311_', 'tb3rbc3_2711112_']]

    ### Training data effect 
    # fnames = [['p0t25brhc_711811911_', 'p0t25brhc_101111111211_', 'p0t25brhc_131114111511_', 'p0t25brhc_181119112011_', 'p0t25brhc_241125112611_', 'p0t25brhc_281129113011_'], ['p0t50brhc_711811911_', 'p0t50brhc_101111111211_', 'p0t50brhc_131114111511_', 'p0t50brhc_181119112011_', 'p0t50brhc_241125112611_', 'p0t50brhc_281129113011_'], ['p0t75brhc_711811911_', 'p0t75brhc_101111111211_', 'p0t75brhc_131114111511_', 'p0t75brhc_181119112011_', 'p0t75brhc_241125112611_', 'p0t75brhc_281129113011_'], ['p0tbrhc_c2_711811911_', 'p0tbrhc_c2_101111111211_', 'p0tbrhc_c2_131114111511_', 'p0tbrhc_c2_181119112011_', 'p0tbrhc_c2_241125112611_', 'p0tbrhc_c2_281129113011_']]

    # xlabel = [6, 12, 18, 24]
    # xlabel = [2,3,4,5,6,8,10]
    xlabel = ['RHC1', 'RHC2', 'RHC3', 'RHC4', 'RBC1-40','RBC1-25', 'RBC2']
    # xlabel = [25, 50, 75, 100]
    xn = [i for i in range(len(xlabel))]

    ###### Overall KPI DFs

    overall_rew_mean = []
    overall_work_mean = []
    overall_40_mean = []
    overall_rew = []
    overall_work = []
    overall_40 = []

    for file_set in fnames:
        rew_model = []
        work_model = []
        under40_model = []
        for f in file_set:
            rew_s = []
            wrk_s = []
            u40_s = []
            for i in range(1,4):
                print path+f+str(i)
                if tag2 == 'set':
                    rew, worktime, under40 = get_kpi((path+f+str(i)))
                elif tag2 =='daily':
                    rew, worktime, under40 = get_daywise_kpi((path+f+str(i)))
                rew_s.append(rew)
                wrk_s.append(worktime)
                u40_s.append(under40)

            if tag2 == 'set':                          
                rew_model.append(np.mean(rew_s))
                work_model.append(np.mean(wrk_s))
                under40_model.append(np.mean(u40_s))
            elif tag2 == 'daily':
                rew_model.extend(np.mean(rew_s,axis=0))
                work_model.extend(np.mean(wrk_s,axis=0))
                under40_model.extend(np.mean(u40_s,axis=0))

        
        df = pd.DataFrame(zip(rew_model, work_model, under40_model), columns =['rewards', 'active_time', 'under40'])
        
        model_name = file_set[0].split('_')[0] + '_' + file_set[0].split('_')[1] + '_'+ dataset + '_' + tag2  ## only for horizon and cluster 
        
        model_name = file_set[0].split('_')[0] + '_' + dataset + '_' + tag2
        
        df.to_csv('/home/milan/workspace/strands_ws/src/battery_scheduler/data/csv_files/'+model_name+'.csv', header=True, index=False)
    
        overall_rew_mean.append(np.mean(rew_model))
        overall_work_mean.append(np.mean(work_model))
        overall_40_mean.append(np.mean(under40_model))
        overall_rew.append(rew_model)
        overall_work.append(work_model)
        overall_40.append(under40_model)


    ov_df =  pd.DataFrame(zip(overall_rew_mean, overall_work_mean, overall_40_mean), columns =['rewards', 'active_time', 'under40'])
    ov_rew_df = pd.DataFrame((np.array(overall_rew)).T, columns=xlabel)
    ov_work_df = pd.DataFrame((np.array(overall_work)).T, columns = xlabel)
    ov_u40_df = pd.DataFrame((np.array(overall_40)).T, columns = xlabel)

    # tag = tag1+'_'+tag2
    # ov_df.to_csv('/home/milan/workspace/strands_ws/src/battery_scheduler/data/csv_files/eb_meanKPI_'+dataset+'_'+tag+'.csv', header=True, index=False) 
    # ov_rew_df.to_csv('/home/milan/workspace/strands_ws/src/battery_scheduler/data/csv_files/eb_rew_'+dataset+'_'+tag+'.csv', header=True, index=False)
    # ov_work_df.to_csv('/home/milan/workspace/strands_ws/src/battery_scheduler/data/csv_files/eb_work_'+dataset+'_'+tag+'.csv', header=True, index=False)  
    # ov_u40_df.to_csv('/home/milan/workspace/strands_ws/src/battery_scheduler/data/csv_files/eb_u40_'+dataset+'_'+tag+'.csv', header=True, index=False)


    ####### Hypothesis Tests 
   
    # #### multiple comparisons (horizon, cluster, training data) - more accurate with daily KPI 
    # tstat_rew, pvalue_rew = stats.friedmanchisquare(ov_rew_df[25], ov_rew_df[50], ov_rew_df[75], ov_rew_df[100])
    # tstat_wrk, pvalue_wrk = stats.friedmanchisquare(ov_work_df[25], ov_work_df[50], ov_work_df[75], ov_work_df[100])
    # print "Test Statitic Rew: ", tstat_rew
    # print "PValue Rew: ", pvalue_rew
    # print "Test Statistic Work: ", tstat_wrk
    # print "Pvalue Work: ", pvalue_wrk


    # ######## Paired t tests for RHC vs RBC
    imp_rew = np.zeros((3,4))
    t_rew = np.zeros((3,4))  # RBC1 vs RHC1, RHC2, RHC3, RHC4
    p_rew = np.zeros((3,4))
    ci_rew = np.zeros((3,4,2))
    imp_wrk = np.zeros((3,4))
    t_wrk = np.zeros((3,4)) 
    p_wrk = np.zeros((3,4))
    ci_wrk = np.zeros((3,4,2))
    imp_u40 = np.zeros((3,4))
    t_u40 = np.zeros((3,4)) 
    p_u40 = np.zeros((3,4))
    ci_u40 = np.zeros((3,4,2))

    l_critical, r_critical = stats.t.interval(0.95, ov_rew_df.shape[0-1])
    for e, rbc in enumerate(['RBC1-40', 'RBC1-25', 'RBC2']):
        for n, rhc in enumerate(['RHC1', 'RHC2', 'RHC3', 'RHC4']):
            rew_diff = ov_rew_df[rhc] - ov_rew_df[rbc]
            t_rew[e][n], p_rew[e][n] = stats.ttest_rel(ov_rew_df[rhc], ov_rew_df[rbc]) 
            imp_rew[e][n] = rew_diff.mean()           
            ci_rew[e][n][0] = rew_diff.mean() - r_critical*(rew_diff.std()/np.sqrt(rew_diff.shape[0]))
            ci_rew[e][n][1] = rew_diff.mean() + r_critical*(rew_diff.std()/np.sqrt(rew_diff.shape[0]))
            
            wrk_diff = ov_work_df[rhc] - ov_work_df[rbc]
            t_wrk[e][n], p_wrk[e][n] = stats.ttest_rel(ov_work_df[rhc], ov_work_df[rbc])
            imp_wrk[e][n] = wrk_diff.mean()
            ci_wrk[e][n][0] = wrk_diff.mean() - r_critical*(wrk_diff.std()/np.sqrt(wrk_diff.shape[0]))
            ci_wrk[e][n][1] = wrk_diff.mean() + r_critical*(wrk_diff.std()/np.sqrt(wrk_diff.shape[0]))
            
            u40_diff = ov_u40_df[rhc] - ov_u40_df[rbc]
            t_u40[e][n], p_u40[e][n] = stats.ttest_rel(ov_u40_df[rhc], ov_u40_df[rbc])
            imp_u40[e][n] = u40_diff.mean()
            ci_u40[e][n][0] = u40_diff.mean() - r_critical*(u40_diff.std()/np.sqrt(u40_diff.shape[0]))
            ci_u40[e][n][1] = u40_diff.mean() + r_critical*(u40_diff.std()/np.sqrt(u40_diff.shape[0]))

    print 'Reward P-----------------'
    print p_rew
    print 'Work P-------------------'
    print p_wrk
    print 'U40-----------------------'
    print p_u40

    ###### Error Bars

    fig, axs = plt.subplots(1,3)
    x_error_bars = ['RHC1', 'RHC2', 'RHC3', 'RHC4']
    x = [h+1 for h in range(len(x_error_bars))]
    for i,subtitle in enumerate(['RBC1-40', 'RBC1-25', 'RBC2']):
        ax = axs[i]
        ax.set_title(subtitle, size=12)
        ax.set_xlabel('Battery Scheduler Controllers')
        ax.set_ylabel('Percentage %')
        ax.grid(axis='both', linestyle=':')
        
        rew_l = imp_rew[i] - ci_rew[i,:,0]
        rew_u = ci_rew[i,:,1] - imp_rew[i]
        i_rew = ax.errorbar(x, imp_rew[i], yerr=[rew_l, rew_u], fmt='og', elinewidth=2, capsize=7, label='Rewards Achieved (%)')
        wrk_l = imp_wrk[i] - ci_wrk[i,:,0]
        wrk_u = ci_wrk[i,:,1] - imp_wrk[i]
        i_wrk = ax.errorbar(x, imp_wrk[i], yerr=[wrk_l, wrk_u], fmt='ob', elinewidth=2, capsize=7, label='Time Worked (%)')
        u40_l = imp_u40[i] - ci_u40[i,:,0]
        u40_u = ci_u40[i,:,1] - imp_u40[i]
        i_u40 = ax.errorbar(x, imp_u40[i], yerr=[u40_l, u40_u], fmt='or', elinewidth=2, capsize=7, label='Time with Battery < 40 (%)')

        ax.set_xlim(0,len(x_error_bars)+1)
        ax.set_xticks([k+1 for k in range(len(x_error_bars))])
        ax.set_xticklabels(x_error_bars)
        ax.set_yticks([k for k in range(-50,50,5)])
        ax.set_yticklabels([k for k in range(-50,50,5)])

        if i == 0:
            ax.legend(loc='lower center')
    
    fig.suptitle('Average Improvement of RHC - 95% Confidence Interval',size=13) 
    # plt.tight_layout(pad=0.5)
    plt.show()


    # ####### Graphs

    fig1, ax1 = plt.subplots()

    ax1.set_title("Performance Analysis\n Scitos Battery Model (Charge:11.5 hrs, Discharge:16.5 hrs)\n Synthetic Dataset")
    ax1.set_xlabel("Controllers")
    ax1.set_ylabel("Percentage %")    

    # ax1.set_title("Horizon Analysis - Deployment Dataset 2")
    # ax1.set_xlabel("Horizon Length (Hours)")
    # ax1.set_ylabel("Percentage %")
    
    # ax1.set_title("Cluster Analysis - Deployment Dataset 2")
    # ax1.set_xlabel("No. of Clusters")
    # ax1.set_ylabel("Percentage %")

    # ax1.set_title("Effect of Size of Training Data\n Deployment Dataset 1")
    # ax1.set_xlabel("Percentage of Available Data")
    # ax1.set_ylabel("Percentage %")
    
    ax1.grid(axis='both', linestyle=':')  
    bp = ax1.boxplot(overall_rew, positions=xn)#, notch=True, bootstrap=10000)
    rew_line,  = ax1.plot(xn, overall_rew_mean, marker='^', color='green', linestyle='--' )
    work_line,  = ax1.plot(xn, overall_work_mean, marker='^', color='orange', linestyle='--' )
    u40_line,  = ax1.plot(xn, overall_40_mean, marker='^', color='red', linestyle='--' )
    ax1.legend((bp['boxes'][0], rew_line, work_line, u40_line), ("Distribution of Rewards Achieved", "Average Rewards Achieved", "Average Time Worked", "Average Time With Battery < 40"), loc='best')
    # ax1.legend((bp['boxes'][0], rew_line, work_line), ("Distribution of Rewards Achieved", "Average Rewards Achieved", "Average Time Worked"), loc='best')
    ax1.set_xticks(xn)
    ax1.set_xticklabels(xlabel)
    ax1.set_yticks([i for i in range(0,101,5)])
    ax1.set_yticklabels([i for i in range(0,101,5)])
    ax1.set_ylim(0,101)
    # plt.show()
      
    # ##### if including computation
    # comptime = np.array(pd.read_csv(path+'/csv_files/hc_comptime_d2.csv', header=None))
    # ax2 = ax1.twinx()
    # comp_line, = ax2.plot(xn, comptime[3,:7], marker='.', color='magenta', linestyle='--')
    # # comp_line, = ax2.plot(xn, comptime[:,5], marker='.', color='magenta', linestyle='--')
    # ax2.set_ylabel('Computation Time (secs)')
    # ax2.set_yticks([i for i in range(0,201,10)])
    # ax2.set_yticklabels([i for i in range(0,201,10)])
    # ax2.set_ylim(0,201)
    # ax1.legend((bp['boxes'][0], rew_line, work_line, comp_line), ("Distribution of Rewards Achieved", "Average Rewards Achieved", "Average Time Worked", "Average Computation Time"), loc='best')  
    # ax1.set_xlim(-0.5,len(xlabel)-0.5) 
    # plt.show()


    ### for plotting computation time heatmap
    # xlabel = [2,3,4,5,6,8,10]
    # ylabel = [6,12,18,24]


    ### single plot
    # fig, ax = plt.subplots()
    # comptime = np.array(pd.read_csv(path+'/csv_files/hc_extracttime.csv', header=None))
    
    # im = ax.imshow(comptime)

    # ax.set_xticks(np.arange(len(xlabel)))
    # ax.set_yticks(np.arange(len(ylabel)))
    # ax.set_xticklabels(xlabel)
    # ax.set_yticklabels(ylabel)
    # ax.set_ylabel("Horizon Length")
    # ax.set_xlabel("No. of Clusters")

    # # Loop over data dimensions and create text annotations.
    # for i in range(len(ylabel)):
    #     for j in range(len(xlabel)):
    #         text = ax.text(j, i, comptime[i, j],
    #                     ha="center", va="center", color="w")
    
       
    # ax.set_title("Policy Extraction Computation Time")
    
    # cbar = fig.colorbar(im,fraction=0.019, pad=0.04 )
    # cbar.ax.set_ylabel('Computation Time (secs)', rotation=-90, va="bottom")
    
    # fig.tight_layout()
    # plt.show()

    ##### multiple plots
    # comp_path = path + 'csv_files/hc_'
    # fnames = ['formtime_d1.csv', 'solvetime_d1.csv', 'extracttime_d1.csv', 'comptime_d1.csv',]
    # titles = ['(a) Model Formation', '(b) Model Solution', '(c) Policy Extraction', '(d) Overall Computation Time']
    # fig, axs = plt.subplots(2,2)
    # ims = []
    # ax_l = []

    # f_count = 0
    # for i in range(2):
    #     for j in range(2):
    #         ax = axs[i][j]
    #         f = comp_path+fnames[f_count]
    #         comptime = np.array(pd.read_csv(f, header=None))

    #         im = ax.imshow(comptime, cmap=cm.PuBu_r)

    #         ax.set_xticks(np.arange(len(xlabel)))
    #         ax.set_yticks(np.arange(len(ylabel)))
    #         ax.set_xticklabels(xlabel)
    #         ax.set_yticklabels(ylabel)
    #         ax.set_ylabel("Horizon Length (hrs)")
    #         ax.set_xlabel("No. of Clusters")

    #         # Loop over data dimensions and create text annotations.
    #         for y in range(len(ylabel)):
    #             for x in range(len(xlabel)):
    #                 text = ax.text(x, y, comptime[y, x],
    #                             ha="center", va="center", color="k")

                
    #         ax.set_title(titles[f_count])
    #         ax_l.append(ax)
    #         ims.append(im)
    #         f_count += 1

    # for im, ax, oe in zip(ims,ax_l,[1,2,3,4]):
    #     cbar = fig.colorbar(im, ax=ax, fraction=0.03, pad=0.04)
    #     if oe%2 == 0:
    #         cbar.ax.set_ylabel('Computation Time (secs)', rotation=-90, va="bottom")
    
    # # fig.tight_layout()
    # plt.show()

    ###################

    ##### Plotting Execution Trace

    # fname = path + 'p0tbrhc_c4_181119112011_1'
    # df = pd.read_csv(fname, sep=' ', index_col=False, dtype={'actual_reward':np.float64, 'obtained_reward':np.float64, 'battery':np.int32})

    # df = df[df['day']=='2017-09-24']

    # time_x = []
    # xn = []
    # colour = []

    # for i in range(df.shape[0]):
    #     ts = df.iloc[i]['time']
    #     if df.iloc[i]['obtained_reward'] != 0:
    #         colour.append('green')
    #     else:
    #         colour.append('red')
    #     xn.append(i)
    #     time_x.append(ts)
    
    # fig, ax = plt.subplots()
    # ax.grid(axis='both', linestyle=':')
    # ax.set_title('Execution Trace')
    # rew_bar = ax.bar(xn, df['actual_reward'], color=colour)    
    # ax.set_xlim(-1,df.shape[0]+1)
    # ax.set_xticks(xn)
    # ax.set_xticklabels(time_x, rotation=90, ha='left')
    # ax.set_yticks([i for i in range(0,1201,60)])
    # ax.set_ylabel('Rewards')
    # ax.set_xlabel('Time')

    # ax1 = ax.twinx() 
    # b_line,  = ax1.plot(xn, df['battery'], marker='.')
    # ax1.set_ylabel('Battery Life')
    # ax1.set_yticks([i for i in range(0,101,5)])
    # ax.legend((rew_bar[6], rew_bar[0], b_line ), ('Rewards Achieved on Task Execution', 'Rewards Lost when Charging', 'Battery Life'))
    # plt.show()




        




