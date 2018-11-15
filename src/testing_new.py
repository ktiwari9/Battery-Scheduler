#! /usr/bin/env python
import probabilistic_rewards
import read_adversary
import prism_model
import numpy as np
import subprocess
import roslib
import yaml
import sys
import os

def get_battery_model():
    ################ SPECIFY PATHS OF MODELS #######################
    path = '/home/milan/workspace/strands_ws/src/battery_scheduler'
    if os.path.isfile(path+'/models/battery_charge_model.yaml') and os.path.isfile(path+'/models/battery_discharge_model.yaml'):
        with open (path+'/models/battery_charge_model.yaml', 'r') as f_charge:
            charge_model = yaml.load(f_charge)
        with open (path+'/models/battery_discharge_model.yaml', 'r') as f_discharge:
            discharge_model = yaml.load(f_discharge)
        print ('Battery Models Found at: ' +path+'/models/battery_discharge_model.yaml'+', '+ path+'/models/battery_charge_model.yaml' )
        return charge_model, discharge_model
    else:
        raise ValueError('No models found. First create battery model with probabilistic_battery_model.py')
 

if __name__ == '__main__':
    ur = probabilistic_rewards.uncertain_rewards()
    prob, clusters = ur.get_probabilistic_reward_model()
    charge_model, discharge_model = get_battery_model()
    cl_id = []
    sample_reward = []
    actual_reward = []
    exp_reward = []
    
    #main_path = roslib.packages.get_pkg_dir('battery_scheduler')
    #######################SPECIFY LOCATION ######################
    main_path = '/home/milan/workspace/strands_ws/src/battery_scheduler'
    path_rew = main_path + '/data/sample_rewards'
    path_mod = main_path + '/models/'
    path_data = main_path + '/data/'
    
    if not os.path.isfile(path_rew):
        raise ValueError('Sample Rewards Not Generated. Generate rewards with generate_samples.py')

    with open(path_rew,'r') as f:
        for line in f:
            x = line.split(' ')
            cl_id.append(int(float(x[0].strip())))
            sample_reward.append(float(x[1].strip()))
            actual_reward.append(float(x[2].strip()))
            # exp_reward.append(float(line.split(' ')[3]))        
    
    no_days = len(ur.test_days)
    avg_totalreward = no_days*[0]
    init_battery = 70
    init_charging = 1
    init_time = 0
    no_simulations = 1

    for k in range(no_days):
                
        pm = prism_model.PrismModel('model_t.prism', init_time, init_battery, init_charging, clusters, prob, charge_model, discharge_model)
        #######################SPECIFY LOCATION ######################
        # running prism and saving output from prism
        with open(path_data+'result_wrhc', 'w') as file:
            process = subprocess.Popen('./prism '+ path_mod + 'model_t.prism '+ path_mod +'model_prop.props -exportadv '+ path_mod+ 'model_t.adv -exportprodstates ' + path_mod +'model_t.sta -exporttarget '+path_mod+'model_t.lab',cwd='/home/milan/prism-svn/prism/bin', shell=True, stdout=subprocess.PIPE)
            for c in iter(lambda: process.stdout.read(1), ''):
                sys.stdout.write(c)
                file.write(c)
        ##reading output from prism to find policy file
        policy_file = []
        with open(path_data+'result_wrhc', 'r') as f:
            line_list = f.readlines()
            for i in range(len(line_list)):
                if 'Computed point: ' in line_list[i]:
                    el = line_list[i].split(' ')
                    req_point = el[2][1:-1]
                    if abs(1.0-float(req_point)) < 0.001:
                        start_p = len('Adversary written to file "'+path_mod)
                        file_name = line_list[i-1][start_p:-3]
                        policy_file.append((req_point, file_name))

                if 'Result:' in line_list[i]:
                    s_policy_file = sorted(policy_file, key= lambda x: abs(1-float(x[0])))
                    
                    if len(s_policy_file) == 1:
                        policy_file_name = s_policy_file[0][1]
                    elif '('+s_policy_file[0][0]+',' in line_list[i]:
                        policy_file_name = s_policy_file[0][1]
                    else:
                        policy_file_name = s_policy_file[1][1]

        #######################SPECIFY LOCATION AS BEFORE ######################
        print 'Reading from ', policy_file_name
        pp = read_adversary.ParseAdversary([policy_file_name, 'model_t.sta', 'model_t.lab'])

    #     pp = prism_simulate.parse_model([policy_file_name,'model_t.sta','model_t.lab'], cl_id, actual_reward, sample_reward, exp_reward,k, pm.clusters, pm.prob, charge_model, discharge_model)
        
        
    #     battery = no_simulations*[0]
    #     charging = no_simulations*[0]
    #     init_cluster= no_simulations*[0]
    #     tr_day = no_simulations*[0]
    #     for i in range(no_simulations):
    #         rewards, action, final_state = pp.simulate(k,'real_dec16_wrhc')  #######################SPECIFY LOCATION ######################
    #         battery[i] = int(final_state[0])
    #         print battery[i]
    #         charging[i] = int(final_state[1])
    #         print charging[i]
    #         init_cluster[i] = int(final_state[3])
    #         print init_cluster[i]
    #         tr_day[i] = 0
    #         for r in range(len(rewards)):
    #             if action[r] == 'gather_reward':
    #                 tr_day[i] = rewards[r] + tr_day[i]

    #     init_battery = int(np.average(np.array(battery)))
    #     init_charging = int(np.average(np.array(charging)))
    #     init_cluster = int(np.average(np.array(init_cluster)))
    #     avg_totalreward[k] = np.average(np.array(tr_day))
    #     print init_battery, ' end battery'
    #     print init_charging, ' end charging'
    #     print init_cluster, ' end cluster'
    
    # for k in range(len(avg_totalreward)):
    #     print avg_totalreward[k], ' total_reward, day',k+1
    # print np.sum(avg_totalreward), ' : Reward for Aug'
