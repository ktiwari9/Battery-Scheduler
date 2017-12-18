#!/usr/bin/env python

import plotly
import plotly
import plotly.plotly as py
import plotly.graph_objs as go

if __name__ == '__main__':
    action = []
    exp1_reward = []
    exp_reward = []
    actual_reward = []
    battery = []
    time = []
    with open('/home/milan/catkin_ws/prism/test_rhc_april', 'r') as f:
        for line in f.readlines():
            if 'time' not in line:
                s = line.split(' ')
                time.append(s[0])
                battery.append(int(s[3]))
                exp_reward.append(float(s[5]))
                exp1_reward.append(float(s[7][:-1]))
                actual_reward.append(float(s[6]))
                action.append(s[4])
                
        
        
    color1 = []
    for a in action:
        print a
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
    
    plotly.tools.set_credentials_file(username='MilanMariyaTomy', api_key= 'aoeKNl0QTEWLTb1jlFcB')
    data = [go.Bar( x= time, y = actual_reward, marker=dict(color=color1)), go.Bar( x= time, y = exp_reward, marker=dict(color=color2)), go.Scatter(x=time, y= battery), go.Scatter( x= time, y = exp1_reward)]
    
    fig = go.Figure(data = data)
    py.plot(fig, filename='test_rhc_april_test')
 
