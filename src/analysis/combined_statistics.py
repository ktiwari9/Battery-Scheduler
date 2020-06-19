#! /usr/bin/env python

import pandas as pd 

if __name__ == '__main__':

    dataset = 'D3'

    path = '/home/milan/workspace/strands_ws/src/battery_scheduler/data/csv_files/'

    eb_f = path+'taskbased_overall_'+dataset+'_models.csv'
    tb_f = path+'timebased_overall_'+dataset+'_models.csv'
    c_f = path+'combined_overall_'+dataset+'_models.csv'

    eb = pd.read_csv(eb_f)
    tb = pd.read_csv(tb_f)

    rew = []
    u40 = []
    time = []

    for i in range(6):
        rew.append(tb['rewards'][i])
        time.append(tb['active_time'][i])
        u40.append(tb['under40'][i])
        rew.append(eb['rewards'][i])
        time.append(eb['active_time'][i])
        u40.append(eb['under40'][i])

    for i in range(6,9):
        rew.append(eb['rewards'][i])
        time.append(eb['active_time'][i])
        u40.append(eb['under40'][i])

    df = pd.DataFrame(data=zip(rew, time, u40), columns =['rewards', 'active_time', 'under40'])
    df.to_csv(c_f, header=True, index=False)

