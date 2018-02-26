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
    no_days = 3
    data_path = '/home/milan/workspace/strands_ws/src/battery_scheduler/data/'
    # for without RHC - do not activate both at same time
    # t = 0;
    # for i in range(no_days):
    #     with open(data_path + 'f_un_aug_pbr_wrhc'+ str(i), 'r') as f:
    #         for line in f.readlines():
    #             if 'time' not in line:
    #                 s = line.split(' ')[:-1]
    #                 time.append(t)
    #                 t = t+1
    #                 battery.append(int(s[1]))
    #                 matched_reward.append(float(s[4]))
    #                 exp_reward.append(float(s[6]))
    #                 actual_reward.append(float(s[5]))
    #                 action.append(s[3])


    # with RHC - do not activate both at same time
    with open(data_path+'rrhc_aug_pbr', 'r') as f:
        for line in f.readlines():
            if 'time' not in line:
                s = line.split(' ')[:-1]
                time.append(int(s[0]))
                battery.append(int(s[2]))
                matched_reward.append(float(s[4]))
                exp_reward.append(float(s[6]))
                actual_reward.append(float(s[5]))
                action.append(s[3])
                
        
        
    color1 = []
    for a in action:
        if a == 'gather_reward':
            color1.append('rgba(27,117,20,1)')
        elif a == 'go_charge':
            color1.append('rgba(222,16,16,1)')
        elif a == 'stay_charging':
            color1.append('rgba(236, 153, 28,1)')
            
            
    color2 = []
    for a in action:
        if a == 'gather_reward':
            color2.append('rgba(27,117,20,0.5)')
        elif a == 'go_charge':
            color2.append('rgba(222,16,16,0.5)')
        elif a == 'stay_charging':
            color2.append('rgba(236, 153, 28,0.5)')
    
    plotly.tools.set_credentials_file(username='MilanMariyaTomy', api_key='wSfqCQnChPdSCqIMGPdp')#username='ThanwiraSiraj', api_key= 'y9AlaR5JI6kYeCml1NG4') 
    data = [go.Bar( x= time, y = actual_reward, marker=dict(color=color1)), go.Bar( x= time, y = matched_reward, marker=dict(color=color2)), go.Scatter(x=time, y= battery), go.Scatter( x= time, y = exp_reward)]
    
    fig = go.Figure(data = data)
    py.plot(fig, filename='rrhc_aug_pbr')
 