#! /usr/bin/env python

import subprocess
import rhc_prism_script
#import rhc_mod_parse
import rhc_prism_parse
#import rewards_dbscan
import rewards_uncertain_hk
import time
import numpy as np

if __name__ == '__main__':
    ur = rewards_uncertain_hk.uncertain_rewards(True)
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

    
    init_battery = 70
    init_charging = 1
    init_cluster = cl_id[0]

    reward = []
    time_v = []
    battery = []
    charging = []
    action = []
    battery.append(init_battery)
    charging.append(init_charging)
    t1 = time.time()
    output_path = main_path + '/data/rhc_aug1'
    model_path = main_path + '/models/'
    with open(output_path, 'w') as fw:
        fw.write('time charging battery action matched_reward actual_reward exp_reward  cluster_vals prob_vals\n')
        for t in range(48*3):     # 48*3 for 3 days
            rhc_pm = rhc_prism_script.make_model('model_rhc.prism', t, init_battery, init_charging, init_cluster, clusters, prob, charge_model, discharge_model)
            fw.write('{0} {1} {2} '.format(t, init_charging, init_battery))
            # path to prism and prism files 
            subprocess.call('./prism '+model_path+'model_rhc.prism '+model_path+ 'model_prop.props -exportadv '+model_path+'model_rhc.adv -exportprodstates '+model_path+'model_rhc.sta -exporttarget '+model_path+'model_rhc.lab',cwd='/home/milan/prism-svn/prism/bin',shell=True)
            rhc_pp = rhc_prism_parse.parse_model(['model_rhcpre1.adv', 'model_rhc.sta', 'model_rhc.lab'], t,  sample_reward, rhc_pm.clusters)
            next_state = rhc_pp.get_next_state(rhc_pp.initial_state) ## next state, action, 

            ### Only appplicable for real robot.
            # if next_state[0][0] == 0:
            #     bn_dict = discharge_model[init_battery]
            # elif next_state[0][0] == 1:
            #     bn_dict = charge_model[init_battery]
            # if next_state[0][1] not in bn_dict:
            #     count_val = 1
            # else:
            #     count_val = bn_dict[next_state[0][1]] + 1
            # bn_dict.update({next_state[0][1] : count_val})
            # if next_state[0][0] == 0:
            #     discharge_model.update({init_battery : bn_dict})
            # elif next_state[0][0] == 1:
            #     charge_model.update({init_battery})

            action.append(next_state[2])
            battery.append(next_state[0][0])
            charging.append(next_state[0][1])
            time_v.append(t)
            if next_state[2] == 'gather_reward':
                reward.append(actual_reward[t])
            else:
                reward.append(0)
            fw.write('{0} {1} {2} {3}'.format(next_state[2], sample_reward[t], actual_reward[t], exp_reward[t])) 
            for cl in rhc_pm.clusters[0]:
                fw.write(' {0}'.format(cl))
            for p in rhc_pm.prob[0]:
                fw.write(' {0}'.format(p))
            fw.write('\n')
            init_battery = int(next_state[0][0])
            init_charging = int(next_state[0][1])
            init_cluster = int(next_state[1])

    print 'Time taken: ', (time.time()-t1)
    print 'Reward Day 1: ', np.sum(np.array(reward[:48]))
    print 'Reward Day 2: ', np.sum(np.array(reward[48:96]))
    print 'Reward Day 3: ', np.sum(np.array(reward[96:144]))
    print 'Reward Day 4: ', np.sum(np.array(reward[144:]))
    print 'Total: ', np.sum(np.array(reward))
    
