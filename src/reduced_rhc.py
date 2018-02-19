#! /usr/bin/env python
import battery_model
import subprocess
import reduced_rhc_script
#import rhc_mod_parse
import rrhc_prism_parse
import rewards_uncertain_hk
import time
import numpy as np

if __name__ == '__main__':
    ur = rewards_uncertain_hk.uncertain_rewards(True)
    clusters, prob = ur.get_rewards()
    path_to_directory = '/media/milan/DATA/battery_logs'
    charge_model, discharge_model = battery_model.get_battery_model(path_to_directory)
    cl_id =[]
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
    output_path = main_path + '/data/rrhc_aug3'
    model_path = main_path + '/models/'
    
    t1 = time.time()
    with open(output_path, 'w') as fw:
        fw.write('time charging battery action matched_reward actual_reward exp_reward  cluster_vals prob_vals\n')
        for t in range(48*3):     # 48*3 for 3 days
            rhc_pm = reduced_rhc_script.make_model('rrhc.prism', t, init_battery, init_charging, init_cluster, clusters, prob, charge_model, discharge_model)
            fw.write('{0} {1} {2} '.format(t, init_charging, init_battery))
            subprocess.call('./prism '+model_path+'rrhc.prism '+model_path+ 'model_prop.props -exportadv '+model_path+'rrhc.adv -exportprodstates '+model_path+'rrhc.sta -exporttarget '+model_path+'rrhc.lab',cwd='/home/milan/prism-svn/prism/bin',shell=True)
            rhc_pp = rrhc_prism_parse.parse_model(['rrhcpre1.adv', 'rrhc.sta', 'rrhc.lab'], t,  sample_reward, rhc_pm.clusters, rhc_pm.no_cluster)
            next_state = rhc_pp.get_next_state(rhc_pp.initial_state)
            
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
    
