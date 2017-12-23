#!/usr/bin/env python

import plotly
import plotly
import plotly.plotly as py
import plotly.graph_objs as go

if __name__ == '__main__':
    action = []
    exp_reward = []
    matched_reward = []
    actual_reward = []
    battery = []
    time = []
    prob = []
    with open('/home/milan/workspace/strands_ws/src/battery_scheduler/data/un_dec150', 'r') as f:
        for line in f.readlines():
            if 'time' not in line:
                s = line.split(' ')
                print s
                time.append(s[0])
                battery.append(int(s[2]))
                matched_reward.append(float(s[4]))
                exp_reward.append(float(s[6]))
                actual_reward.append(float(s[5]))
                action.append(s[3])
                prob.append(s[7][:-1])
                
        
        
    color1 = []
    for a, p in zip(action, prob):
        if a == 'gather_reward':
            color1.append('rgba(27,117,20,'+p+')')
        elif a == 'go_charge':
            color1.append('rgba(222,16,16,'+p+')')
        elif a == 'stay_charging':
            color1.append('rgba(236, 153, 28,'+p+')')
            
            
    color2 = []
    for a in action:
        if a == 'gather_reward':
            color2.append('rgba(27,117,20,0.5)')
        elif a == 'go_charge':
            color2.append('rgba(222,16,16,0.5)')
        elif a == 'stay_charging':
            color2.append('rgba(236, 153, 28,0.5)')
    
    plotly.tools.set_credentials_file(username='RagulDeep', api_key= 'Ryk98QVNYFGBEtaZMKPS')
    data = [go.Bar( x= time, y = actual_reward, marker=dict(color=color1)), go.Bar( x= time, y = matched_reward, marker=dict(color=color2)), go.Scatter(x=time, y= battery), go.Scatter( x= time, y = exp_reward)]
    
    fig = go.Figure(data = data)
    py.plot(fig, filename='un_dec15_prob')
 
