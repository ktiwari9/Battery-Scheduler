#! /usr/bin/env python

import subprocess
import form_prism_script
import prism_simulate
import rewards_uncertain1
import numpy as np

if __name__ == '__main__':
    ur = rewards_uncertain1.uncertain_rewards(False)
    clusters, prob = ur.get_rewards()
    avg_totalreward = 3*[0]
    init_battery = 50
    init_charging = 1
    init_observed = 0
    for k in range(3):
        pm = form_prism_script.make_model(init_battery, init_charging, init_observed, clusters, prob)
        subprocess.call('./prism /home/milan/catkin_ws/prism/model_t.prism /home/milan/catkin_ws/prism/model1.prop -exportadv /home/milan/catkin_ws/prism/model_t.adv -exportprodstates /home/milan/catkin_ws/prism/model_t.sta -exporttarget /home/milan/catkin_ws/prism/model_t.lab',cwd='/home/milan/prism-svn/prism/bin',shell=True)
        pp = prism_simulate.parse_model(['model_tpre1.adv','model_t.sta','model_t.lab'], clusters,prob)
        battery = 200000*[0]
        charging = 200000*[0]
        observed = 200000*[0]
        tr_day = 200000*[0]
        for i in range(200000):
            rewards, plan, action, final_state = pp.simulate()  
            battery[i] = int(final_state[2])
            charging[i] = int(final_state[0])
            observed[i] = int(final_state[4])
            tr_day[i] = np.sum(rewards)

        init_battery = int(round(np.average(np.array(battery))))
        init_charging = int(round(np.average(np.array(charging))))
        init_observed = int(round(np.average(np.array(observed))))
        avg_totalreward[k] = round(np.average(np.array(tr_day)))
        print init_battery, ' end battery'
        print init_charging, ' end charging'
        print init_observed, ' end observed'
        print avg_totalreward[k], ' total_reward, day',k+1
    print np.sum(avg_totalreward), ' : Reward for three days'
