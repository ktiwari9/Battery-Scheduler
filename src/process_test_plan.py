#!/usr/bin/env python
import subprocess
import prism_script_test
import form_prism_script
import battery_model
import plotly
import plotly
import plotly.plotly as py
import plotly.graph_objs as go
import numpy as np


class StateProcessor:

    def __init__(self, clusters, charge_model, discharge_model):
        self.t = 0
        self.b = 0
        self.c = 0
        self.cl_no = 0 
        #self.cl_id = 0
        self.clusters = clusters
        self.charge_model = charge_model
        self.discharge_model = discharge_model
        
    def current_state(self, current_time, current_battery, current_charging):
        self.t = current_time
        self.b = current_battery
        self.c = current_charging
        self.cl_no = 0 

        for i in range(self.t+1):
            self.cl_no = self.cl_no + len(self.clusters[i])

    ## for actions charge and work 
    def next_state(self, action):
        #cl_id
        cl_id_next = []

        if self.t+1 < len(self.clusters):
            for cl in self.clusters[self.t+1]:
                cl_id_next.append(self.cl_no)
                self.cl_no = self.cl_no+1

        #time
        t_next = self.t + 1

        if action == 'charge':
            # charge
            c_next = 1
            #battery
            b_next = self._get_b_next('charge')

        elif action == 'work':
            # charge
            c_next = 0
            #battery
            b_next = self._get_b_next('discharge')

        return t_next, b_next, c_next, cl_id_next

    # either charge or discharge
    def _get_b_next(self, model):

        if model == 'charge':
            total_count = np.sum(np.array(self.charge_model[self.b].values()))
            b_items = self.charge_model[self.b].items()
            nb = list(map(lambda x: [x[0],x[1]/total_count], b_items))

            if self.c == 0 and self.b != 100:
                nb = list(map(lambda x: [x[0]-1,x[1]/total_count], b_items)) 
                

        elif model == 'discharge':
            total_count = np.sum(np.array(self.discharge_model[self.b].values()))
            b_items = self.discharge_model[self.b].items()
            nb = list(map(lambda x: [x[0],x[1]/total_count], b_items))
                
        return nb


if __name__ == '__main__':
    action = []
    exp_reward = []
    matched_reward = []
    actual_reward = []
    battery = []
    time = []
    clusters = []
    probs = []
    charging = []
    time_int = 48
    charge_model, discharge_model = battery_model.get_battery_model('/media/milan/DATA/battery_logs')
    #main_path = roslib.packages.get_pkg_dir('battery_scheduler')
    #path_rew = main_path +'/data/sample_rewards'
    main_path = '/home/milan/workspace/strands_ws/src/battery_scheduler'
    path_result= main_path + '/data/test_result'
    path_model = main_path + '/models/'
 
    with open('/home/milan/workspace/strands_ws/src/battery_scheduler/data/un_aug11_pb40', 'r') as f:
        for line in f.readlines():
            if 'time' not in line:
                s = line.split(' ')[:-1]
                time.append(int(s[0]))
                charging.append(int(s[2]))
                battery.append(int(s[1]))
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
    print clusters
    print probs  
    return_charge = []
    return_work = []
    cases = ['charge', 'work']
    sp = StateProcessor(clusters, charge_model, discharge_model)
    for t,c,b, cluster in zip(time, charging, battery, clusters):
        for case in cases:
            sp.current_state(t,b,c)
            #print t, case
            if t != time_int-1:
                t_next, b_next, c_next, cl_id_next = sp.next_state(case)
                #print cl_id_next
                return_cluster = []
                for i in range(len(cl_id_next)):
                    for j in range(len(b_next)):
                        pm = form_prism_script.make_model('test_plan_process.prism', t_next, b_next[j][0], c_next, cl_id_next[i], clusters, probs, charge_model, discharge_model)
                        subprocess.call('./prism '+ path_model + 'test_plan_process.prism '+ path_model +'test_prop.props -exportresults '+path_result,cwd='/home/milan/prism-svn/prism/bin',shell=True)
                        with open(path_result, 'r') as f:
                            for line in f.readlines():
                                if 'Result' not in line:
                                    tuples = line[2:-3].split('), (')
                                    prob_pareto = []
                                    reward_pareto = []
                                    for element in tuples:
                                        prob_rew = element.split(', ')
                                        prob_pareto.append(float(prob_rew[0]))
                                        reward_pareto.append(float(prob_rew[1])) 
                        req_prob = min(prob_pareto, key=lambda x:abs(1-x))
                        req_indx = prob_pareto.index(req_prob)
                        total_reward = reward_pareto[req_indx]

                        # gives return from different possible current states to one particular next state
                        return_cl = []
                        for cl in cluster:
                            if case == 'charge':
                                return_cl.append(total_reward*b_next[j][1]*probs[t+1][i])
                            elif case == 'work':
                                return_cl.append((total_reward+cl)* b_next[j][1]*probs[t+1][i])  

                        return_cluster.append(return_cl)
                    
                if case == 'charge':
                    return_charge.append(return_cluster)
                elif case == 'work':
                    return_work.append(return_cluster)  

            else:
                if case == 'charge':
                    return_charge.append([len(cluster)*[0]])
                elif case == 'work':
                    return_work.append([cluster])


    exp_return_list = []
    for array in [return_charge, return_work]:
        exp_return = []
        for i in range(len(array)):
            exp_rew = 0
            #print array[i][0]
            for j in range(len(array[i][0])):
                rew = 0 
                for k in range(len(array[i])):
                    if i != time_int-1:
                        rew = rew + array[i][k][j]
                    else:
                        rew = rew + array[i][k][j]
                #print rew, 'rew'
                #print probs[i][j], 'probs'

                ## for calculating exp_return over all possible states in a particular time interval
                # exp_rew = exp_rew + probs[i][j]*rew

                ## for calculating exp return from one particular state in a particular time interval
                if matched_reward[i] == clusters[i][j]:
                    exp_rew = rew
            exp_return.append(exp_rew)
        exp_return_list.append(exp_return)




        # exp_return_charge = []
        # for i in range(len(return_charge)):
        #     rew = 0 
        #     for j in range(len(return_charge[i])):
        #         if i != time_int-1:
        #             rew = rew + probs[i+1][j]*return_charge[i][j]
        #         else:
        #             rew = rew + 1*return_charge[i][j]
        #     exp_return_charge.append(rew)

        # exp_return_work = []
        # for i in range(len(return_work)):
        #     rew = 0 
        #     for j in range(len(return_work[i])):
        #         if i != time_int-1:
        #             rew = rew + probs[i+1][j]*return_work[i][j]
        #         else:
        #             rew = rew + 1*return_work[i][j]
        #     exp_return_work.append(rew)
        
    i = 0
    for t,b,c in zip(time, battery, charging):
        sp.current_state(t,b,c)
        for case in cases:
            x, b_next, xx, xxx = sp.next_state(case)
            if b_next < 31:
                exp_return_list[1][i] = exp_return_list[0][i]
        i = i+1


    color1 = []
    for a in action:
        if a == 'gather_reward':
            color1.append('rgba(27,117,20,1)')
        elif a == 'go_charge':
            color1.append('rgba(222,16,16,1)')
        elif a == 'stay_charging':
            color1.append('rgba(236,153,28,1)')
            
            
    color2 = []
    for a in action:
        if a == 'gather_reward':
            color2.append('rgba(27,117,20,0.5)')
        elif a == 'go_charge':
            color2.append('rgba(222,16,16,0.5)')
        elif a == 'stay_charging':
            color2.append('rgba(236, 153, 28,0.5)')
    
    plotly.tools.set_credentials_file(username='MilanMariyaTomy', api_key= 'wSfqCQnChPdSCqIMGPdp')
    data = [go.Bar( x= time, y = actual_reward, marker=dict(color=color1)), go.Bar( x= time, y = matched_reward, marker=dict(color=color2)), go.Scatter(x=time, y= battery), go.Scatter( x= time, y = exp_reward), go.Scatter(x=time, y=exp_return_list[1]), go.Scatter(x=time, y=exp_return_list[0])]
    
    fig = go.Figure(data = data)
    py.plot(fig, filename='un_aug11_exp_pb4')