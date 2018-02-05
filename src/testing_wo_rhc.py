#! /usr/bin/env python

import subprocess
import form_prism_script
import prism_script_test
import prism_simulate
import rewards_dbscan
import battery_data
import numpy as np
import roslib

if __name__ == '__main__':
    ur = rewards_dbscan.uncertain_rewards(True)
    clusters, prob = ur.get_rewards()
    #path_to_directory = ##TO BE ADDED 
    #charge_model, discharge_model = battery_data.get_battery_model(path_to_directory)
    charge_model = 'ab'
    discharge_model = 'cd'
    cl_id = []
    sample_reward = []
    actual_reward = []
    exp_reward = []
    #main_path = roslib.packages.get_pkg_dir('battery_scheduler')
    #path_rew = main_path +'/data/sample_rewards'
    main_path = '/home/milan/workspace/strands_ws/src/battery_scheduler'
    path_rew = main_path + '/data/sample_rewards'
    with open(path_rew,'r') as f:
        for line in f:
            cl_id.append(int(line.split(' ')[0]))
            sample_reward.append(float(line.split(' ')[1]))
            actual_reward.append(float(line.split(' ')[2]))
            exp_reward.append(float(line.split(' ')[3]))        
    no_days = 1
    avg_totalreward = no_days*[0]
    init_battery = 100
    init_charging = 1
    init_cluster = cl_id[0]
    init_time= 0
    no_simulations = 1
    path_mod = main_path+ '/models/'
    for k in range(no_days):
        #pm = form_prism_script.make_model('model_t.prism', init_time, init_battery, init_charging, init_cluster, clusters, prob, charge_model, discharge_model)
        pm = prism_script_test.make_model('model_test.prism', init_time, init_battery, init_charging, init_cluster, clusters, prob, charge_model, discharge_model)
        #subprocess.call('./prism '+ path_mod + 'model_t.prism '+ path_mod +'model_prop.props -exportadv '+ path_mod+ 'model_t.adv -exportprodstates ' + path_mod +'model_t.sta -exporttarget '+path_mod+'model_t.lab',cwd='/home/milan/prism-svn/prism/bin',shell=True)
        subprocess.call('./prism '+ path_mod + 'model_test.prism '+ path_mod +'test_prop.props -exportadv '+ path_mod+ 'model_test.adv -exportprodstates ' + path_mod +'model_test.sta -exporttarget '+path_mod+'model_test.lab',cwd='/home/milan/prism-svn/prism/bin',shell=True)
        pp = prism_simulate.parse_model(['model_testpre1.adv','model_test.sta','model_test.lab'], cl_id, actual_reward, sample_reward, exp_reward,k, pm.clusters, pm.prob)
        battery = no_simulations*[0]
        charging = no_simulations*[0]
        init_cluster= no_simulations*[0]
        tr_day = no_simulations*[0]
        for i in range(no_simulations):
            rewards, action, final_state = pp.simulate(k,'un_aug18_10')  
            battery[i] = int(final_state[1])
            charging[i] = int(final_state[0])
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
    print np.sum(avg_totalreward), ' : Reward for Dec'
