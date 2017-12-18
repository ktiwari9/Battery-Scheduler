#!/usr/bin/env python

import plotly
import plotly
import plotly.plotly as py
import plotly.graph_objs as go
import numpy as np

if __name__ == '__main__':
    action = []
    exp_reward = []
    curr_batt = []
    exp_batt = [91]
    time = []
    with open('/home/milan/catkin_ws/aug20_4pmexp_state.txt', 'r') as f:
        for line in f.readlines():
            if 'decision' in line:
                s = line.split(': ')
                action.append(s[1][:-1])
            elif 'reward' in line:
                s = line.split(': ')
                exp_reward.append(float(s[1][:-1]))
            elif 'current_battery' in line:
                s = line.split(': ')
                curr_batt.append(int(s[1][:-1]))
            elif 'battery_life' in line:
                s = line.split(': ')
                exp_batt.append(int(s[1][:-1]))
                
    for i in range(len(action)):
        time.append(str(i))
        
    actual_reward = []  # remove first element from other 
    with open('/home/milan/catkin_ws/aug20_4pm_rew.txt', 'r') as f:
        for line in f.readlines():
                if 'reward' in line:
                    s = line.split(': ')
                    actual_reward.append(float(s[1][:-1]))
                
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
            
    plotly.tools.set_credentials_file(username='MilanMariyaTomy', api_key='aoeKNl0QTEWLTb1jlFcB')        
            
    trace0 = go.Bar(x=time,y=actual_reward, marker=dict(color=color1))
    trace1 = go.Bar(x=time,y=exp_reward, marker=dict(color=color2))
    trace2 = go.Scatter(x=time, y=curr_batt)
    trace3= go.Scatter(x=time, y=exp_batt[:-1])
    plotly.tools.set_credentials_file(username='MilanMariyaTomy', api_key= 'aoeKNl0QTEWLTb1jlFcB')
    data = [trace0,trace1, trace2, trace3]
    
    fig = go.Figure(data = data)
    py.plot(fig, filename='battery_scheduler')
    print 'TOTAL REWARD ::::::', np.sum(actual_reward)
    
#    data1 = [trace0]
#    layout = go.Layout(title='Battery Scheduler', yaxis=dict(title='Battery'))
#    fig1 = go.Figure(data=data1, layout=layout)
#    plot_url = py.plot(fig1, filename='battery_sch')
#   data2 = [trace1, trace2]
#    layout = go.Layout(title='Battery Scheduler', yaxis=dict(title='Rewards'))
#    fig2 = fig1 = go.Figure(data=data2, layout=layout)
#    plot_url = py.plot(fig2, filename='battery_sch_rewards')

 with open('/home/milan/catkin_ws/prism/un_4_april1', 'r') as f:
        for line in f.readlines():
            if 'time' not in line:
                s = line.split(' ')
                time.append(str(48+int(s[0])))
                battery.append(int(s[3]))
                exp_reward.append(float(s[5]))
                exp1_reward.append(float(s[7][:-1]))
                actual_reward.append(float(s[6]))
                action.append(s[4])
                
    with open('/home/milan/catkin_ws/prism/un_4_april2', 'r') as f:
        for line in f.readlines():
            if 'time' not in line:
                s = line.split(' ')
                time.append(str(96+int(s[0])))
                battery.append(int(s[3]))
                exp_reward.append(float(s[5]))
                exp1_reward.append(float(s[7][:-1]))
                actual_reward.append(float(s[6]))
                action.append(s[4])   
                
    with open('/home/milan/catkin_ws/prism/un_4_april3', 'r') as f:
        for line in f.readlines():
            if 'time' not in line:
                s = line.split(' ')
                time.append(str(144+int(s[0])))
                battery.append(int(s[3]))
                exp_reward.append(float(s[5]))
                exp1_reward.append(float(s[7][:-1]))
                actual_reward.append(float(s[6]))
                action.append(s[4])    
