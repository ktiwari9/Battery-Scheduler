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
    clusters = []
    probs = []
    with open('/home/milan/workspace/strands_ws/src/battery_scheduler/data/un_apr140', 'r') as f:
        for line in f.readlines():
            if 'time' not in line:
                s = line.split(' ')[:-1]
                time.append(int(s[0]))
                battery.append(int(s[2]))
                matched_reward.append(float(s[4]))
                exp_reward.append(float(s[6]))
                actual_reward.append(float(s[5]))
                action.append(s[3])
                l = len(s[7:])
                cluster = []
                for c in s[7:7+l/2]:
                    cluster.append(float(c))
                prob = []
                for p in s[7+l/2:]:
                    prob.append(float(p))

                clusters.append(cluster)
                probs.append(prob)

    traces = []
    for i in range(len(time)):
        trace = go.Scatter(x= len(clusters[i])*[time[i]], y= clusters[i], mode = 'markers', marker = dict(size='12', color = probs[i], colorscale = 'Viridis'))
        traces.append(trace)


    plotly.tools.set_credentials_file(username='RagulDeep', api_key= 'Ryk98QVNYFGBEtaZMKPS')
    data = traces
    fig = go.Figure(data = data)
    py.plot(fig, filename='un_apr14_prob_vals')



        
        
   
