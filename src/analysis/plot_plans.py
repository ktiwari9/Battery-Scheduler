#!/usr/bin/env python

import plotly
import plotly.plotly as py
import plotly.graph_objs as go

if __name__ == '__main__':
    action = []
    exp_reward = []
    actual_reward = []
    obtained_reward = []
    matched_reward = []
    battery = []
    time = []
    no_days = 1
    data_path = '/home/milan/workspace/strands_ws/src/battery_scheduler/data/'
    fname = 'rbc40test_210_1'
    
    with open(data_path+fname, 'r') as f:
        for line in f.readlines():
            if 'time' not in line:
                s = line.split(' ')
                # print s
                time.append(int(s[0].strip()))
                battery.append(int(s[1].strip())) 
                # exp_reward.append(float(s[7].strip()))
                obtained_reward.append(float(s[4].strip()))
                # matched_reward.append(float(s[5].strip()))
                actual_reward.append(float(s[5].strip())) 
                action.append(s[3].strip())
                        
    reward_percent = (float(sum(obtained_reward))/sum(actual_reward))*100
    print ("Reward Percent: " , reward_percent )
            
    
    # plotly.tools.set_credentials_file(username='MilanMariyaTomy', api_key= '8HntwF4rtsUwPvjW3Sl4')
    # data = [go.Bar( x= time, y = obtained_reward, name='Reward Obtained',marker=dict(color='rgba(27,117,20,1)')), go.Bar( x= time, y = matched_reward, name='Matched Reward',marker=dict(color='rgba(27,117,20,0.5)')), go.Scatter( x= time, y = actual_reward, name='Actual Reward', line=dict(color=('rgba(255,69,0,1)'))), 
    # go.Scatter(x=time, y= battery, name='Battery', line=dict(color=('rgba(22,96,167,1)'))), go.Scatter(x=time, y= exp_reward, name='Expected Reward') ]
    
    # fig = go.Figure(data = data)
    # py.plot(fig, filename=fname)
 