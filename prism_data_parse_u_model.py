#!/usr/bin/env python

import sys
import os
import plotly 
import plotly.plotly as py
import plotly.graph_objs as go
import rewards

###### States are probabilistic, picking them randomly 
def get_data(filenames):
    label_file = open("/home/milan/catkin_ws/prism/"+filenames[0], 'r')
    labels =[]
    for line in label_file.readlines():
        if 'init' not in line:
            labels.append(line[:-1].split(': '))
            
            
    state_file = open("/home/milan/catkin_ws/prism/"+filenames[1], 'r')
    states = dict()
    for line in state_file.readlines():
        if '_da' not in line or 'battery' not in line:
            state = line[:-2].split(':(')
            states.update({int(state[0]) : (state[1].split(',')[1], state[1].split(',')[2], state[1].split(',')[3], state[1].split(',')[4], state[1].split(',')[5])})
           
            
    adv_file = open("/home/milan/catkin_ws/prism/"+filenames[2], 'r')   
    policy = dict()
    for line in adv_file:
        array = line[:-1].split(' ')
        if len(array) > 2:
            if array[0] not in policy:
                l = []                
            else:
                l = policy[array[0]]
                
            l.append(array[1:])
            policy.update({int(array[0]) : l})

    return labels, states, policy
        
        
if __name__ == '__main__':
    # just the names of the labels, states and adv files in this order
    labels, states, policy = get_data(sys.argv[1:])  
    
    for element in labels:
        if int(element[1]) == 0:
            initial_state = int(element[0])
    
    #Plotting all probabilities
    plan = []
    q_n = initial_state
    for i in range(48):
        if len(policy[q_n]) == 1:
            plan.append([states[q_n],policy[q_n][0][2]])
            q_n1 = int(policy[q_n][0][0])
            q_n = q_n1
    
    rew = rewards.avgRewards1()
    avg_reward = rew.avg_rewards
    reward = []
    for r in avg_reward:
        reward.append(round(r))
    
    battery = []
    time = []
    actions = []
    for x in plan:
        print x 
        battery.append(int(x[0][2]))
        time.append(int(x[0][1]))
        actions.append(x[1])  
    
    color1 = []
    for a in actions:
        if a == 'gather_reward':
            color1.append('rgba(27,117,20,1)')
        elif a == 'go_charge':
            color1.append('rgba(222,16,16,1)')
        elif a == 'stay_charging':
            color1.append('rgba(236, 153, 28,1)')
    
#    plotly.tools.set_credentials_file(username='MilanMariyaTomy', api_key= 'cT38KdeHJxqmDc0Svqji')
#    data = [go.Bar( x= time, y = battery, marker=dict(color=color1))]
#    
#    layout = go.Layout(annotations=[ dict(x=t, y=b, text=str(r), xanchor='center', yanchor='bottom', showarrow=False) for t,b,r in zip#(time, battery,reward)])
#    fig = go.Figure(data = data, layout = layout)
#    py.plot(fig, filename='model-1')
 
        
