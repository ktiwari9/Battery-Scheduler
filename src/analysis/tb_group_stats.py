#! /usr/bin/env python

from datetime import datetime, timedelta
import matplotlib.pyplot as plt
from scipy import stats
import pandas as pd 
import numpy as np 
import roslib
import csv

path = roslib.packages.get_pkg_dir('battery_scheduler') + '/data/'

datasets = ["Deployment Dataset 1", "Deployment Dataset 2", "Synthetic Dataset"]
date_strs = [['30831819', '394959', '169179189', '209219229', '249259269', '110210310'], ['101011101210', '191020102110', '291030103110', '411511611', '121113111411', '112212312'], ['711811911', '101111111211', '131114111511', '181119112011', '241125112611', '281129113011']]

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

    OV_REW = []
    OV_REWM = []
    OV_WRK = []
    OV_WRKM = []

    # xlabel = [6, 12, 18, 24]
    # xlabel = [2,3,4,5,6,8,10]
    # xlabel = ['RHC1', 'RHC2', 'RHC3', 'RHC4', 'RBC1-40','RBC1-25', 'RBC2']
    xlabel = [25, 50, 75, 100]
    xn = [i for i in range(len(xlabel))]

    
    for en, ds in enumerate(date_strs):
        fnames = ['p0t25brhc_', 'p0t50brhc_', 'p0t75brhc_']
        # fnames = ['_c1_', '_c2_', '_c3_', '_c4_', '_c5_', '_c7_', '_c9_']
        # fnames = ['_h12_', '_h24_', '_h36_']
        tag2 = 'daily'
        if en == 0:
            fnames.append('_c4_')
        elif en == 1:
            fnames.append('_c2_')
        else:
            fnames.append('_h48_')
            # tag2 = 'set'

    
        ###### Overall KPI DFs

        overall_rew_mean = []
        overall_work_mean = []
        overall_40_mean = []
        overall_rew = []
        overall_work = []
        overall_40 = []

        for enum, m_name in enumerate(fnames):
            rew_model = []
            work_model = []
            under40_model = []
            for f in ds:
                rew_s = []
                wrk_s = []
                u40_s = []
                for i in range(1,4):
                    if enum == 3:
                        f_path = path+'p0tbrhc'+m_name+f+'_'+str(i)
                    else:
                        f_path = path+m_name+f+'_'+str(i)

                    # f_path = path+'p0tbrhc'+m_name+f+'_'+str(i)
                    print f_path
                    if tag2 == 'set':
                        rew, worktime, under40 = get_kpi(f_path)
                    elif tag2 =='daily':
                        rew, worktime, under40 = get_daywise_kpi(f_path)
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

        OV_REW.append(overall_rew)
        OV_WRK.append(overall_work)
        OV_REWM.append(overall_rew_mean)
        OV_WRKM.append(overall_work_mean)       


        ####### Hypothesis Tests 
    
        # #### multiple comparisons (horizon, cluster, training data) - more accurate with daily KPI 
        # tstat_rew, pvalue_rew = stats.friedmanchisquare(ov_rew_df[12], ov_rew_df[18], ov_rew_df[24])
        # tstat_wrk, pvalue_wrk = stats.friedmanchisquare(ov_work_df[12], ov_work_df[18], ov_work_df[24])
        # print "Test Statitic Rew: ", tstat_rew
        # print "PValue Rew: ", pvalue_rew
        # print "Test Statistic Work: ", tstat_wrk
        # print "Pvalue Work: ", pvalue_wrk

    

    ####### Graphs

    fig, axs = plt.subplots(1,3)

    for e, ax1 in enumerate(axs):

        # ax1.set_title("Performance Analysis\n Battery Model 3 (Charge:3 hrs, Discharge:6 hrs)\n Deployment Dataset 1")
        # ax1.set_xlabel("Controllers")
        # ax1.set_ylabel("Percentage %")    

        ax1.set_title(datasets[e], size=12)
        # ax1.set_xlabel("Horizon Length (Hours)")
        # ax1.set_xlabel("No. of Clusters")


        # ax1.set_title("Effect of Size of Training Data\n Deployment Dataset 1")
        ax1.set_xlabel("Percentage of Available Data")
        ax1.set_ylabel("Percentage %")
        
        ax1.grid(axis='both', linestyle=':')  
        bp = ax1.boxplot(OV_REW[e], positions=xn)#, notch=True, bootstrap=10000)
        rew_line,  = ax1.plot(xn, OV_REWM[e], marker='^', color='green', linestyle='--' )
        work_line,  = ax1.plot(xn, OV_WRKM[e], marker='^', color='orange', linestyle='--' )
        # u40_line,  = ax1.plot(xn, overall_40_mean, marker='^', color='red', linestyle='--' )
        # ax1.legend((bp['boxes'][0], rew_line, work_line, u40_line), ("Distribution of Rewards Achieved", "Average Rewards Achieved", "Average Time Worked", "Average Time With Battery < 40"), loc='best')
        if e == 2:
            ax1.legend((bp['boxes'][0], rew_line, work_line), ("Distribution of Rewards Achieved", "Average Rewards Achieved", "Average Time Worked"), loc='best')
        ax1.set_xticks(xn)
        ax1.set_xticklabels(xlabel)
        ax1.set_yticks([i for i in range(0,101,5)])
        ax1.set_yticklabels([i for i in range(0,101,5)])
        ax1.set_ylim(0,101)
        # plt.show()
        
        # #### if including computation
        # ax2 = ax1.twinx()

        # if e == 0:
        #     ax1.set_ylabel("Percentage %")
        #     comptime = np.array(pd.read_csv(path+'/csv_files/hc_comptime_d1.csv', header=None))
        #     # comp_line, = ax2.plot(xn, comptime[:,2], marker='.', color='magenta', linestyle='--')
        #     comp_line, = ax2.plot(xn, comptime[1,:7], marker='.', color='magenta', linestyle='--')
        # elif e == 1:
        #     comptime = np.array(pd.read_csv(path+'/csv_files/hc_comptime_d2.csv', header=None))
        #     # comp_line, = ax2.plot(xn, comptime[:,5], marker='.', color='magenta', linestyle='--')
        #     comp_line, = ax2.plot(xn, comptime[1,:7], marker='.', color='magenta', linestyle='--')
        # else:
        #     comptime = np.array(pd.read_csv(path+'/csv_files/hc_comptime_syn.csv', header=None))
        #     # comp_line, = ax2.plot(xn, comptime[:,2], marker='.', color='magenta', linestyle='--')
        #     comp_line, = ax2.plot(xn, comptime[1,:7], marker='.', color='magenta', linestyle='--')
        #     ax2.set_ylabel('Computation Time (secs)')
        #     ax1.legend((bp['boxes'][0], rew_line, work_line, comp_line), ("Distribution of Rewards Achieved", "Average Rewards Achieved", "Average Time Worked", "Average Computation Time"), loc='center right')

    
            
        # ax2.set_yticks([i for i in range(0,101,5)])
        # ax2.set_yticklabels([i for i in range(0,101,5)])
        # ax2.set_ylim(0,101) 
        # ax1.set_xlim(-0.5,len(xlabel)-0.5) 
    
    # fig.suptitle('Horizon Analysis',size=13)
    plt.show()
    