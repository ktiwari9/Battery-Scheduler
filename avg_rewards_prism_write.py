#! /usr/bin/env python

import avgRewards
import os

if __name__ == '__main__':
    
    r = avgRewards.avg_rewards()
    a = r.avg_rewards
    with open(os.path.join('/home/milan/catkin_ws/prism','det_model_t.prism'), 'w') as f:
        f.write('mdp\n\n')
        f.write('module battery_model\n\n')
        f.write('charging:[0..1] init 1;\n')
        f.write('t:[0..48] init 0;\n')
        f.write('battery:[0..100] init 100;\n\n')
        f.write("[gather_reward] (battery>89) & (battery<=100) & (t<48) -> (charging'=0) & (battery'=battery-4) & (t'=t+1);\n")
        f.write("[gather_reward] (battery>24) & (battery<90) & (t<48) -> (charging'=0) & (battery'=battery-5) & (t'=t+1);\n")
        f.write("[gather_reward] (battery>4) & (battery<25) & (t<48) -> (charging'=0) & (battery'=battery-4) & (t'=t+1);\n")
        f.write("[gather_reward] (battery>2) & (battery<5) & (t<48) -> (charging'=0) & (battery'=battery-3) & (t'=t+1);\n")
        f.write("[gather_reward] (battery=2) & (t<48) -> (charging'=0) & (battery'=battery-2) & (t'=t+1);\n")
        f.write("[gather_reward] (battery=1) & (t<48) -> (charging'=0) & (battery'=battery-1) & (t'=t+1);\n")
        f.write("[go_charge] (charging=0) & (battery>0) & (battery<5) & (t<48) -> (charging'=1) & (battery'=battery+6) & (t'=t+1);\n")
        f.write("[go_charge] (charging=0) & (battery>4) & (battery<9) & (t<48) -> (charging'=1) & (battery'=battery+5) & (t'=t+1);\n")
        f.write("[go_charge] (charging=0) & (battery>8) & (battery<15) & (t<48) -> (charging'=1) & (battery'=battery+4) & (t'=t+1);\n")
        f.write("[go_charge] (charging=0) & (battery>14) & (battery<29) & (t<48) -> (charging'=1) & (battery'=battery+3) & (t'=t+1);\n")
        f.write("[go_charge] (charging=0) & (battery>28) & (battery<45) & (t<48) -> (charging'=1) & (battery'=battery+2) & (t'=t+1);\n")
        f.write("[go_charge] (charging=0) & (battery>44) & (battery<65) & (t<48) -> (charging'=1) & (battery'=battery+1) & (t'=t+1);\n")
        f.write("[go_charge] (charging=0) & (battery>64) & (battery<98) & (t<48) -> (charging'=1) & (battery'=battery+2) & (t'=t+1);\n")
        f.write("[go_charge] (charging=0) & (battery=98) & (t<48) -> (charging'=1) & (battery'=battery+1) & (t'=t+1);\n")
        f.write("[go_charge] (charging=0) & (battery=99) & (t<48) -> (charging'=1) & (t'=t+1);\n")
        f.write("[go_charge] (charging=0) & (battery=100) & (t<48) -> (charging'=1) & (t'=t+1);\n")
        f.write("[stay_charging] (charging=1) & (battery>0) & (battery<5) & (t<48) -> (charging'=1) & (battery'=battery+7) & (t'=t+1);\n")
        f.write("[stay_charging] (charging=1) & (battery>4) & (battery<9) & (t<48) -> (charging'=1) & (battery'=battery+6) & (t'=t+1);\n")
        f.write("[stay_charging] (charging=1) & (battery>8) & (battery<15) & (t<48) -> (charging'=1) & (battery'=battery+5) & (t'=t+1);\n")
        f.write("[stay_charging] (charging=1) & (battery>14) & (battery<29) & (t<48) -> (charging'=1) & (battery'=battery+4) & (t'=t+1);\n")
        f.write("[stay_charging] (charging=1) & (battery>28) & (battery<45) & (t<48) -> (charging'=1) & (battery'=battery+3) & (t'=t+1);\n")
        f.write("[stay_charging] (charging=1) & (battery>44) & (battery<65) & (t<48) -> (charging'=1) & (battery'=battery+2) & (t'=t+1);\n")
        f.write("[stay_charging] (charging=1) & (battery>64) & (battery<98) & (t<48) -> (charging'=1) & (battery'=battery+3) & (t'=t+1);\n")
        f.write("[stay_charging] (charging=1) & (battery=98) & (t<48) -> (charging'=1) & (battery'=battery+2) & (t'=t+1);\n")
        f.write("[stay_charging] (charging=1) & (battery=99) & (t<48) -> (charging'=1) & (battery'=battery+1) & (t'=t+1);\n")
        f.write("[stay_charging] (charging=1) & (battery=100) & (t<48) -> (charging'=1) & (t'=t+1);\n")
        f.write("[tick] (battery=0) & (t<48) -> (t'=t+1);\n\n")
        f.write('endmodule\n\n\n')
        
        
        f.write('rewards "rew"\n\n')
        # For 48 points of time
        for j in range(48):
            f.write('[gather_reward] (t={0}):{1};\n'.format(j, round(a[j])))
        f.write('\nendrewards\n\n')
            
        
