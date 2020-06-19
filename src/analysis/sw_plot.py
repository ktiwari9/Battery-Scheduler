#! /usr/bin/env python

import matplotlib.pyplot as plt
from datetime import datetime
from scipy import stats
import pandas as pd
import numpy as np

file_path = '/home/milan/workspace/strands_ws/src/battery_scheduler/data/'
no_sim = 3

def get_kpi(fname):  ## kpi for file_length days == 1

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
    under40_percent = (under_40/(24*60*60))*100  ## for 1 day 

    return rew_percent, work_percent, under40_percent


if __name__ == "__main__":

    test_days = [datetime(2017,8,30), 
    datetime(2017,8,31), 
    datetime(2017,9,1), 
    datetime(2017,9,3), 
    datetime(2017,9,4), 
    datetime(2017,9,5),
    datetime(2017,9,16), 
    datetime(2017,9,24), 
    datetime(2017,9,25), 
    datetime(2017,9,26), 
    datetime(2017,10,1), 
    datetime(2017,10,2), 
    datetime(2017,10,3),
    datetime(2017,10,10),
    datetime(2017,10,11),
    datetime(2017,10,12),
    datetime(2017,10,19),
    datetime(2017,10,20),
    datetime(2017,10,21),
    datetime(2017,10,29),
    datetime(2017,10,30),
    datetime(2017,10,31),
    datetime(2017,11,4),
    datetime(2017,11,12),
    datetime(2017,11,13),
    datetime(2017,11,14)]

    rewp_m = np.zeros((len(test_days),no_sim))
    workp_m = np.zeros((len(test_days),no_sim))
    u40p_m = np.zeros((len(test_days),no_sim))

    rewp_mr = np.zeros((len(test_days),no_sim))
    workp_mr = np.zeros((len(test_days),no_sim))
    u40p_mr = np.zeros((len(test_days),no_sim))

    for e, d in enumerate(test_days):
        for s in range(no_sim):
            fname = file_path+'p0swtbrhc_'+str(d.day)+str(d.month)+'_'+str(s+1)
            print fname
            rewp_m[e][s], workp_m[e][s], u40p_m[e][s] = get_kpi(fname)

    for e, d in enumerate(test_days):
        for s in range(no_sim):
            fname = file_path+'swtbrbc1_'+str(d.day)+str(d.month)+'_'+str(s+1)
            print fname 
            rewp_mr[e][s], workp_mr[e][s], u40p_mr[e][s] = get_kpi(fname)

    rewm_mean = np.mean(rewp_m, axis=1)
    workm_mean = np.mean(workp_m, axis=1)
    u40m_mean = np.mean(u40p_m, axis=1)

    rewmr_mean = np.mean(rewp_mr, axis=1)
    workmr_mean = np.mean(workp_mr, axis=1)
    u40mr_mean = np.mean(u40p_mr, axis=1)

    xn = [i+1 for i in range(0,len(test_days))]

    fig1, ax1 = plt.subplots()

    ax1.set_title("Comparison of Sliding Window RHC and RBC")
    ax1.set_xlabel("Days")
    ax1.set_ylabel("Percentage %")
    ax1.set_xlim(0, len(test_days)+1)
    ax1.set_ylim(0, 100)
    
    ax1.grid(axis='both', linestyle=':')  
    rew_line,  = ax1.plot(xn, rewm_mean, color='green', marker='.', linestyle='-' )
    work_line,  = ax1.plot(xn, workm_mean, color='orange', marker='.', linestyle='-' )
    u40_line,  = ax1.plot(xn, u40m_mean, color='red', marker='.', linestyle='-' )
    rewr_line,  = ax1.plot(xn, rewmr_mean, color='green', marker='.', linestyle='--' )
    workr_line,  = ax1.plot(xn, workmr_mean, color='orange', marker='.', linestyle='--' )
    u40r_line,  = ax1.plot(xn, u40mr_mean, color='red', marker='.', linestyle='--' )
    ax1.legend((rew_line, rewr_line, work_line, workr_line, u40_line, u40r_line), ("Average Rewards Achieved - RHC1", "Average Rewards Achieved - RBC1", "Average Time Worked - RHC1", "Average Time Worked - RBC1", "Average Time With Battery < 40 - RHC1", "Average Time With Battery <  40 - RBC1"), loc='best')
    ax1.set_xticks([i+1 for i in range(0,len(test_days))])
    ax1.set_xticklabels(xn)
    ax1.set_yticks([i for i in range(0,101,5)])
    ax1.set_yticklabels([i for i in range(0,101,5)])
    plt.tight_layout()
    plt.show()

    ### Hypothesis Tests T Tests Paired 

    t_rew, p_rew = stats.ttest_rel(rewm_mean, rewmr_mean)
    print 'P Value Reward: ', p_rew
    t_wrk, p_wrk = stats.ttest_rel(workm_mean, workmr_mean)
    print 'P Value Work: ', p_wrk

    imp_rew = rewm_mean - rewmr_mean
    imp_wrk = workm_mean - workmr_mean
    cl, cr = stats.t.interval(0.95, len(imp_rew))
    imp_rew_mu = np.mean(imp_rew)
    rew_val = cr*(np.std(imp_rew,ddof=1)/np.sqrt(len(imp_rew)))
    imp_wrk_mu = np.mean(imp_wrk)
    wrk_val = cr*(np.std(imp_wrk,ddof=1)/np.sqrt(len(imp_wrk)))


    print 'Improvement Reward Conf Interval 95%: ', imp_rew_mu - rew_val, imp_rew_mu, imp_rew_mu + rew_val
    print 'Improvement Work Conf Interval 95%: ', imp_wrk_mu - wrk_val, imp_wrk_mu, imp_wrk_mu + wrk_val


