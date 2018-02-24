#! /usr/bin/env python
import rewards_uncertain_hk
import subprocess
import form_prism_script
import prism_script_test
import prism_simulate
import rewards_dbscan
import battery_data
import numpy as np
import roslib
import sys
import battery_model

if __name__ == '__main__':
    ur = rewards_uncertain_hk.uncertain_rewards(True)
    clusters, prob = ur.get_rewards()
    path_to_directory = '/media/milan/DATA/battery_logs' 
    charge_model, discharge_model = battery_model.get_battery_model(path_to_directory)
    cl_id = []
    sample_reward = []
    actual_reward = []
    exp_reward = []
    
    #main_path = roslib.packages.get_pkg_dir('battery_scheduler')
    main_path = '/home/milan/workspace/strands_ws/src/battery_scheduler'
    path_rew = main_path + '/data/sample_rewards'
    path_mod = main_path + '/models/'
    path_data = main_path + '/data/'
    
    with open(path_rew,'r') as f:
        for line in f:
            cl_id.append(int(line.split(' ')[0]))
            sample_reward.append(float(line.split(' ')[1]))
            actual_reward.append(float(line.split(' ')[2]))
            exp_reward.append(float(line.split(' ')[3]))        
    
    no_days = 1
    avg_totalreward = no_days*[0]
    init_battery = 70
    init_charging = 1
    init_cluster = cl_id[0]
    init_time= 0
    no_simulations = 1

    for k in range(no_days):
        policy_file = None
        
        pm = form_prism_script.make_model('model_t.prism', init_time, init_battery, init_charging, init_cluster, clusters, prob, charge_model, discharge_model)
        
        # running prism and saving output from prism
        with open(path_data+'result_wrhc', 'w') as file:
            process = subprocess.Popen('./prism '+ path_mod + 'model_t.prism '+ path_mod +'model_prop.props -exportadv '+ path_mod+ 'model_t.adv -exportprodstates ' + path_mod +'model_t.sta -exporttarget '+path_mod+'model_t.lab',cwd='/home/milan/prism-svn/prism/bin', shell=True, stdout=subprocess.PIPE)
            for c in iter(lambda: process.stdout.read(1), ''):
                sys.stdout.write(c)
                file.write(c)
        ##reading output from prism to find policy file
        with open(path_data+'result_wrhc', 'r') as f:
            line_list = f.readlines()
            for i in range(len(line_list)):
                if 'Optimal value for weights [1.000000,0.000000] from initial state:' in line_list[i]:
                    if 'pre' not in line_list[i+2]:
                        start_p = len('Adversary written to file "'+path_mod)
                        policy_file = line_list[i+2][start_p:-3]
        
        if policy_file != None:
            pp = prism_simulate.parse_model([str(policy_file),'model_t.sta','model_t.lab'], cl_id, actual_reward, sample_reward, exp_reward,k, pm.clusters, pm.prob)
        else:
            pp = prism_simulate.parse_model(['model_tpre1.adv','model_t.sta','model_t.lab'], cl_id, actual_reward, sample_reward, exp_reward,k, pm.clusters, pm.prob)
        
        battery = no_simulations*[0]
        charging = no_simulations*[0]
        init_cluster= no_simulations*[0]
        tr_day = no_simulations*[0]
        for i in range(no_simulations):
            rewards, action, final_state = pp.simulate(k,'un_aug11_pb')  
            battery[i] = int(final_state[0])
            charging[i] = int(final_state[1])
            init_cluster[i] = int(final_state[3])
            tr_day[i] = 0
            for r in range(len(rewards)):
                if action[r] == 'gather_reward':
                    tr_day[i] = rewards[r] + tr_day[i]

        init_battery = int(np.average(np.array(battery)))
        init_charging = int(np.average(np.array(charging)))
        init_cluster = int(np.average(np.array(init_cluster)))
        avg_totalreward[k] = np.average(np.array(tr_day))
        print init_battery, ' end battery'
        print init_charging, ' end charging'
        print init_cluster, ' end cluster'
    
    for k in range(len(avg_totalreward)):
        print avg_totalreward[k], ' total_reward, day',k+1
    print np.sum(avg_totalreward), ' : Reward for Aug'
