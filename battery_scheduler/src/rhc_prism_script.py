#! /usr/bin/env python

import os

class make_model:
    
    def __init__(self, from_t,clusters, prob):
        # initial values that make model. from_t set to zero
        self.horizon = 48 # for a horizon of 48 time steps
        self.clusters = self.horizon*[0]
        self.prob = self.horizon*[0]
        self.total_cl = 0
        for k in range(self.horizon):  
            self.prob[k] = prob[(from_t+k)%48]
            self.clusters[k] = clusters[(from_t+k)%48]
            self.total_cl = self.total_cl+len(self.clusters[k])   
        
    def write_prism_file(self, filename,init_b, init_ch, init_ob):
        # path to prism files   
        with open(os.path.join('/localhome/strands/milan_ws/prism',filename), 'w') as f: ## change path
            f.write('mdp\n\n')
            f.write('module battery_model\n\n')
            f.write('charging:[0..1] init {0};\n'.format(init_ch))
            f.write('t:[0..{0}] init 0;\n'.format(self.horizon))
            f.write('battery:[0..100] init {0};\n\n'.format(init_b))
            f.write("[gather_reward] (battery>89) & (battery<=100) & (t<{0}) -> (charging'=0) & (battery'=battery-4) & (t'=t+1);\n".format(self.horizon))
            f.write("[gather_reward] (battery>30) & (battery<90) & (t<{0}) -> (charging'=0) & (battery'=battery-5) & (t'=t+1);\n".format(self.horizon))
          #  f.write("[gather_reward] (battery>4) & (battery<25) & (t<{0}) -> (charging'=0) & (battery'=battery-4) & (t'=t+1);\n".format(self.horizon))
          #  f.write("[gather_reward] (battery>2) & (battery<5) & (t<{0}) -> (charging'=0) & (battery'=battery-3) & (t'=t+1);\n".format(self.horizon))
          #  f.write("[gather_reward] (battery=2) & (t<{0}) -> (charging'=0) & (battery'=battery-2) & (t'=t+1);\n".format(self.horizon))
          #  f.write("[gather_reward] (battery=1) & (t<{0}) -> (charging'=0) & (battery'=battery-1) & (t'=t+1);\n".format(self.horizon))
            f.write("[go_charge] (charging=0) & (battery>0) & (battery<5) & (t<{0}) -> (charging'=1) & (battery'=battery+6) & (t'=t+1);\n".format(self.horizon))
            f.write("[go_charge] (charging=0) & (battery>4) & (battery<9) & (t<{0}) -> (charging'=1) & (battery'=battery+5) & (t'=t+1);\n".format(self.horizon))
            f.write("[go_charge] (charging=0) & (battery>8) & (battery<15) & (t<{0}) -> (charging'=1) & (battery'=battery+4) & (t'=t+1);\n".format(self.horizon))
            f.write("[go_charge] (charging=0) & (battery>14) & (battery<29) & (t<{0}) -> (charging'=1) & (battery'=battery+3) & (t'=t+1);\n".format(self.horizon))
            f.write("[go_charge] (charging=0) & (battery>28) & (battery<45) & (t<{0}) -> (charging'=1) & (battery'=battery+2) & (t'=t+1);\n".format(self.horizon))
            f.write("[go_charge] (charging=0) & (battery>44) & (battery<65) & (t<{0}) -> (charging'=1) & (battery'=battery+1) & (t'=t+1);\n".format(self.horizon))
            f.write("[go_charge] (charging=0) & (battery>64) & (battery<98) & (t<{0}) -> (charging'=1) & (battery'=battery+2) & (t'=t+1);\n".format(self.horizon))
            f.write("[go_charge] (charging=0) & (battery=98) & (t<{0}) -> (charging'=1) & (battery'=battery+1) & (t'=t+1);\n".format(self.horizon))
            f.write("[go_charge] (charging=0) & (battery=99) & (t<{0}) -> (charging'=1) & (t'=t+1);\n".format(self.horizon))
            f.write("[go_charge] (charging=0) & (battery=100) & (t<{0}) -> (charging'=1) & (t'=t+1);\n".format(self.horizon))
            f.write("[stay_charging] (charging=1) & (battery>0) & (battery<5) & (t<{0}) -> (charging'=1) & (battery'=battery+7) & (t'=t+1);\n".format(self.horizon))
            f.write("[stay_charging] (charging=1) & (battery>4) & (battery<9) & (t<{0}) -> (charging'=1) & (battery'=battery+6) & (t'=t+1);\n".format(self.horizon))
            f.write("[stay_charging] (charging=1) & (battery>8) & (battery<15) & (t<{0}) -> (charging'=1) & (battery'=battery+5) & (t'=t+1);\n".format(self.horizon))
            f.write("[stay_charging] (charging=1) & (battery>14) & (battery<29) & (t<{0}) -> (charging'=1) & (battery'=battery+4) & (t'=t+1);\n".format(self.horizon))
            f.write("[stay_charging] (charging=1) & (battery>28) & (battery<45) & (t<{0}) -> (charging'=1) & (battery'=battery+3) & (t'=t+1);\n".format(self.horizon))
            f.write("[stay_charging] (charging=1) & (battery>44) & (battery<65) & (t<{0}) -> (charging'=1) & (battery'=battery+2) & (t'=t+1);\n".format(self.horizon))
            f.write("[stay_charging] (charging=1) & (battery>64) & (battery<98) & (t<{0}) -> (charging'=1) & (battery'=battery+3) & (t'=t+1);\n".format(self.horizon))
            f.write("[stay_charging] (charging=1) & (battery=98) & (t<{0}) -> (charging'=1) & (battery'=battery+2) & (t'=t+1);\n".format(self.horizon))
            f.write("[stay_charging] (charging=1) & (battery=99) & (t<{0}) -> (charging'=1) & (battery'=battery+1) & (t'=t+1);\n".format(self.horizon))
            f.write("[stay_charging] (charging=1) & (battery=100) & (t<{0}) -> (charging'=1) & (t'=t+1);\n".format(self.horizon))
            f.write("[tick] (battery=0) & (t<{0}) -> (t'=t+1);\n\n".format(self.horizon))
            f.write('endmodule\n\n\n')
        
            f.write('module cluster_evol\n\n')
            f.write('cl:[0..{0}];\n\n'.format((self.total_cl-1)))
            #For horizon time steps
            cl_no = 0
            for i in range(self.horizon):
                f.write('[o] (t={0}) -> '.format(i))
                for j in range(len(self.prob[i])):
                    if j == (len(self.prob[i])-1):
                        f.write("{0}:(cl'={1});\n".format(self.prob[i][j], cl_no))
                    else:
                        f.write("{0}:(cl'={1}) + ".format(self.prob[i][j], cl_no))
                    cl_no = cl_no + 1
            f.write('\nendmodule\n\n\n')
        
            f.write('module observation\n\n')
            f.write('observed:[0..1] init {0};\n\n'.format(init_ob))
            f.write("[o] (observed=0) -> (observed'=1);\n")
            f.write("[gather_reward] (observed=1) -> (observed'=0);\n")
            f.write("[stay_charging] (observed=1) -> (observed'=0);\n")
            f.write("[go_charge] (observed=1) -> (observed'=0);\n\n")
            f.write('endmodule\n\n\n')
        
            f.write('rewards "rew"\n\n')
            # For n clusters in total in h horizon
            cl_no = 0
            for i in range(self.horizon):
                for j in range(len(self.clusters[i])):
                    f.write('[gather_reward](cl={0}):{1};\n'.format(cl_no, self.clusters[i][j]))
                    cl_no = cl_no+1
            f.write('\nendrewards\n\n')
            

