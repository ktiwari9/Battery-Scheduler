#!/usr/bin/env python

import plotly
import plotly
import plotly.plotly as py
import plotly.graph_objs as go

if __name__ == '__main__':
    action = []
    matched_reward = []
    exp_reward = []
    actual_reward = []
    battery = []
    time = []
    no_days = 1
    data_path = '/home/milan/workspace/strands_ws/src/battery_scheduler/data/'
    ###for without RHC - do not activate both at same time
    # t = 0;
    # for i in range(no_days):
    #     with open(data_path + 'sep13_prob'+ str(i), 'r') as f:
    #         for line in f.readlines():
    #             if 'time' not in line:
    #                 s = line.split(' ')[:-1]
    #                 time.append(t)
    #                 t = t+1
    #                 battery.append(int(s[1]))
    #                 # matched_reward.append(float(s[4]))
    #                 # exp_reward.append(float(s[6]))
    #                 actual_reward.append(float(s[5]))
    #                 action.append(s[3])


    # ####with RHC - do not activate both at same time
    with open(data_path+'det_sep13', 'r') as f:
        for line in f.readlines():
            if 'time' not in line:
                s = line.split(' ')[:-1]
                time.append(int(s[0]))
                battery.append(int(s[1])) #2
                # matched_reward.append(float(s[4]))
                # exp_reward.append(float(s[6]))
                actual_reward.append(float(s[4])) #5
                action.append(s[3])
                
    max_rew = max(actual_reward)

    r_g = len(action)*[None]
    r_s = len(action)*[None]
    c_indicator = len(action)*[None]
    for i in range(len(action)):
        if action[i] == 'gather_reward':
            r_g[i] = actual_reward[i]
        elif action[i] == 'stay_charging' or action[i] == 'go_charge':
            r_s[i] = actual_reward[i]
            c_indicator[i] = max_rew+5
           # c_indicator[i+1] = max_rew+5

            
            
    
    plotly.tools.set_credentials_file(username='MilanMariyaTomy', api_key='wSfqCQnChPdSCqIMGPdp')#username='ThanwiraSiraj', api_key= 'y9AlaR5JI6kYeCml1NG4') 
    data = [go.Bar( x= time, y = r_g, name='Rewards Achieved',marker=dict(color='rgba(27,117,20,1)')), 
    go.Bar( x= time, y = r_s, name='Rewards Lost',marker=dict(color='rgba(27,117,20,0.4)')), 
    go.Scatter(x=time, y= battery, name='Battery', line=dict(color=('rgba(22,96,167,1')))]
    # go.Scatter( x= time, y = c_indicator, name = 'Charging Indicator', mode='marker', marker=dict(color=('rgba(0,0,0,1)'), symbol='circle', size=8)) ]
    
    fig = go.Figure(data = data)
    py.plot(fig, filename='sep13_det')
 