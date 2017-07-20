#! /usr/bin/env python

import subprocess
import rhc_prism_script
import rhc_prism_parse
import rewards_uncertain1

if __name__ == '__main__':
    ur = rewards_uncertain1.uncertain_rewards(False)
    clusters, prob = ur.get_rewards()
    init_battery = 50
    init_charging = 1
    init_observed = 0
    reward = []
    time = []
    battery = []
    charging = []
    observed = []
    action = []
    battery.append(init_battery)
    charging.append(init_charging)
    observed.append(init_observed)
    with open('/home/milan/catkin_ws/prism/plan_rhc1', 'w') as fw:
        fw.write('time charging observed battery action reward available_rewards\n')
        for t in range(48*3+1):     # 48*3 for 3 days
            rhc_pm = rhc_prism_script.make_model(t, init_battery, init_charging, init_observed, clusters, prob)
            fw.write('{0} {1} {2} {3} '.format(t, init_charging, init_observed, init_battery))
            # path to prism and prism files 
            subprocess.call('./prism /home/milan/catkin_ws/prism/u_model_t.prism /home/milan/catkin_ws/prism/model1.prop -exportadv /home/milan/catkin_ws/prism/u_model_t.adv -exportprodstates /home/milan/catkin_ws/prism/u_model_t.sta -exporttarget /home/milan/catkin_ws/prism/u_model_t.lab',cwd='/home/milan/prism-svn/prism/bin',shell=True)
            rhc_pp = rhc_prism_parse.parse_model(['u_model_tpre1.adv', 'u_model_t.sta', 'u_model_t.lab'], rhc_pm.clusters, rhc_pm.prob)
            next_state = rhc_pp.get_next_state(rhc_pp.initial_state)
            action.append(next_state[1])
            battery.append(next_state[2][2])
            charging.append(next_state[2][0])
            observed.append(next_state[2][4])
            time.append(t)
            reward.append(next_state[3])
            fw.write('{0} {1} {2},{3},{4},{5},{6}\n'.format(next_state[1], next_state[3], next_state[4][0], next_state[4][1], next_state[4][2], next_state[4][3], next_state[4][4])) 
            init_battery = int(next_state[2][2])
            init_charging = int(next_state[2][0])
            init_observed = int(next_state[2][4])
            

