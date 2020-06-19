#! /usr/bin/env python3

import plotly.graph_objs as go
import plotly.plotly as py 
import pandas as pd 
import numpy as np 
import plotly
import re
import os

## comp time stored in the following order: learn rew model, obtain current rew/current system, form prism model, solve prism model, read prism policy, extract action

data_path = '/home/milan/workspace/strands_ws/src/battery_scheduler/data/'
clusters = [1,2,3,4,5,7,9]
horizons = [6,12,18,24]
cnames = ['lr', 'cs', 'fp', 'sp', 'rp', 'ea']

if __name__ == "__main__":
    hc_total = []  # horizon(y) vs cluster(x) grid
    hc_lrcsfp = []
    hc_sp = []
    hc_rpea = []
    for h in horizons:
        c_total = []
        c_lrcsfp = []
        c_sp = []
        c_rpea = []
        for c in clusters:
            df = pd.read_csv(data_path+"scalability/"+str(c)+"c"+str(h)+"h_ctd1", header=None, names=cnames, converters={'lr':pd.to_timedelta, 'cs':pd.to_timedelta, 'fp':pd.to_timedelta, 'sp':pd.to_timedelta, 'rp':pd.to_timedelta, 'ea':pd.to_timedelta})
            mean_df = df.mean()
            c_total.append(round(mean_df.sum().total_seconds(),2))
            c_lrcsfp.append(round((mean_df['lr']+mean_df['cs']+mean_df['fp']).total_seconds(),2))
            c_sp.append(round(mean_df['sp'].total_seconds(), 2))
            c_rpea.append(round((mean_df['rp']+mean_df['ea']).total_seconds(), 2))
            
        hc_total.append(c_total)
        hc_lrcsfp.append(c_lrcsfp)
        hc_sp.append(c_sp)
        hc_rpea.append(c_rpea)

    df1 = pd.DataFrame(data=hc_total)
    df2 = pd.DataFrame(data=hc_lrcsfp)
    df3 = pd.DataFrame(data=hc_sp)
    df4 = pd.DataFrame(data=hc_rpea)

    df1.to_csv(data_path+'csv_files/hc_comptime_d1.csv', header=False, index=False)
    df2.to_csv(data_path+'csv_files/hc_formtime_d1.csv', header=False, index=False)
    df3.to_csv(data_path+'csv_files/hc_solvetime_d1.csv', header=False, index=False)
    df4.to_csv(data_path+'csv_files/hc_extracttime_d1.csv', header=False, index=False)
    