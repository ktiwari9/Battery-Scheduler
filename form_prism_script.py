#! /usr/bin/env python

import rewards_uncertain1
import os

class make_model:
    
    def __init__(self,init_b, init_ch, init_ob, clusters, prob):
        # initial values that make model. from_t set to zero
        self.clusters = clusters
        self.prob = prob
        self.write_prism_file(init_b, init_ch, init_ob)    
        
    def write_prism_file(self, init_b, init_ch, init_ob):
        # path to prism files   
        with open(os.path.join('/home/milan/catkin_ws/prism','model_t.prism'), 'w') as f:
            f.write('mdp\n\n')
            f.write('module battery_model\n\n')
            f.write('charging:[0..1] init {0};\n'.format(init_ch))
            f.write('t:[0..48] init 0;\n')
            f.write('battery:[0..100] init {0};\n\n'.format(init_b))
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
        
            f.write('module cluster_evol\n\n')
            f.write('cl:[0..239];\n\n')
            #For 47 time steps
            for i in range(48):
                f.write("[o] (t={0}) -> {1}:(cl'={2}) + {3}:(cl'={4}) + {5}:(cl'={6}) + {7}:(cl'={8}) + {9}:(cl'={10});\n".format(i, self.prob[i][0], i*5, self.prob[i][1], i*5+1, self.prob[i][2], 5*i+2, self.prob[i][3], 5*i+3, self.prob[i][4], 5*i+4))
            f.write('\nendmodule\n\n\n')
        
            f.write('module observation\n\n')
            f.write('observed:[0..1] init {0};\n\n'.format(init_ob))
            f.write("[o] (observed=0) -> (observed'=1);\n")
            f.write("[gather_reward] (observed=1) -> (observed'=0);\n")
            f.write("[stay_charging] (observed=1) -> (observed'=0);\n")
            f.write("[go_charge] (observed=1) -> (observed'=0);\n\n")
            f.write('endmodule\n\n\n')
        
            f.write('rewards "rew"\n\n')
            # For 240 clusters in total
            for j in range(240):
                f.write('[gather_reward](cl={0}):{1};\n'.format(j, self.clusters[j//5][j%5]))
            f.write('\nendrewards\n\n')
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
