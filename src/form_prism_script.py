#! /usr/bin/env python

import rewards_dbscan
import roslib

class make_model:
    
    def __init__(self,filename, init_b, init_ch, init_cluster, clusters, prob):
        # initial values that make model. from_t set to zero
        self.clusters = clusters
        self.total_cl = 0
        for i in range(len(clusters)):
            self.total_cl = self.total_cl + len(self.clusters[i])
        self.prob = prob
        self.actions = ['gather_reward', 'go_charge', 'stay_charging', 'tick']
        self.write_prism_file(filename, init_b, init_ch, init_cluster)    
        
    # file for one day - 48 time steps
    def write_prism_file(self, filename, init_b, init_ch, init_cluster):
        # path to prism files 
        #path = roslib.packages.get_pkg_dir('battery_scheduler') + '/models/' +filname) 
        path = '/home/milan/workspace/strands_ws/src/battery_scheduler/models/' + filename 
        with open(path, 'w') as f:
            f.write('mdp\n\n')
            f.write('module battery_model\n\n')
            f.write('charging:[0..1] init {0};\n'.format(init_ch))
            f.write('battery:[0..100] init {0};\n\n'.format(init_b))
            f.write("[gather_reward] (battery>89) & (battery<=100) -> (charging'=0) & (battery'=battery-4);\n")
            f.write("[gather_reward] (battery>30) & (battery<90) -> (charging'=0) & (battery'=battery-5);\n")
            f.write("[gather_reward] (battery>4) & (battery<25) -> (charging'=0) & (battery'=battery-4);\n")
            f.write("[gather_reward] (battery>2) & (battery<5) -> (charging'=0) & (battery'=battery-3);\n")
            f.write("[gather_reward] (battery=2) -> (charging'=0) & (battery'=battery-2) ;\n")
            f.write("[gather_reward] (battery=1) -> (charging'=0) & (battery'=battery-1) ;\n")
            f.write("[go_charge] (charging=0) & (battery>0) & (battery<5) -> (charging'=1) & (battery'=battery+6) ;\n")
            f.write("[go_charge] (charging=0) & (battery>4) & (battery<9) -> (charging'=1) & (battery'=battery+5) ;\n")
            f.write("[go_charge] (charging=0) & (battery>8) & (battery<15)-> (charging'=1) & (battery'=battery+4) ;\n")
            f.write("[go_charge] (charging=0) & (battery>14) & (battery<29)-> (charging'=1) & (battery'=battery+3) ;\n")
            f.write("[go_charge] (charging=0) & (battery>28) & (battery<45)-> (charging'=1) & (battery'=battery+2) ;\n")
            f.write("[go_charge] (charging=0) & (battery>44) & (battery<65) -> (charging'=1) & (battery'=battery+1) ;\n")
            f.write("[go_charge] (charging=0) & (battery>64) & (battery<98) -> (charging'=1) & (battery'=battery+2) ;\n")
            f.write("[go_charge] (charging=0) & (battery=98) -> (charging'=1) & (battery'=battery+1) ;\n")
            f.write("[go_charge] (charging=0) & (battery=99) -> (charging'=1) ;\n")
            f.write("[go_charge] (charging=0) & (battery=100)  -> (charging'=1) ;\n")
            f.write("[stay_charging] (charging=1) & (battery=0) -> (battery'=battery+7) & (charging'=1);\n")
            f.write("[stay_charging] (charging=1) & (battery>0) & (battery<5) -> (charging'=1) & (battery'=battery+7) ;\n")
            f.write("[stay_charging] (charging=1) & (battery>4) & (battery<9)  -> (charging'=1) & (battery'=battery+6) ;\n")
            f.write("[stay_charging] (charging=1) & (battery>8) & (battery<15) -> (charging'=1) & (battery'=battery+5) ;\n")
            f.write("[stay_charging] (charging=1) & (battery>14) & (battery<29)  -> (charging'=1) & (battery'=battery+4) ;\n")
            f.write("[stay_charging] (charging=1) & (battery>28) & (battery<45)  -> (charging'=1) & (battery'=battery+3) ;\n")
            f.write("[stay_charging] (charging=1) & (battery>44) & (battery<65) -> (charging'=1) & (battery'=battery+2) ;\n")
            f.write("[stay_charging] (charging=1) & (battery>64) & (battery<98)  -> (charging'=1) & (battery'=battery+3) ;\n")
            f.write("[stay_charging] (charging=1) & (battery=98)  -> (charging'=1) & (battery'=battery+2) ;\n")
            f.write("[stay_charging] (charging=1) & (battery=99)  -> (charging'=1) & (battery'=battery+1) ;\n")
            f.write("[stay_charging] (charging=1) & (battery=100) -> (charging'=1);\n")
            f.write("[tick] (battery=0) -> (battery' = battery);\n\n")
            f.write('endmodule\n\n\n')

            f.write('module time_model\n\n')
            f.write('t:[0..48] init 0;\n')
            for action in self.actions:
                f.write("[{0}] (t<48) -> (t'=t+1);\n".format(action))
            f.write('\n')
            f.write('endmodule\n\n\n')

            f.write('module cluster_evolution\n\n')
            f.write('cl:[0..{0}] init {1};\n\n'.format((self.total_cl), init_cluster))
            cl_no = 0
            for i in range(49): #change to 48, if 48
                if i == 48:
                    for action in self.actions:
                        f.write('[{0}] (t={1}) -> '.format(action,i-1))
                        f.write("1:(cl'={0});\n".format(cl_no))

                else:
                    for action in self.actions:
                        if cl_no >= len(self.prob[0]):
                            f.write('[{0}] (t={1}) -> '.format(action,i-1))
                        for j in range(len(self.prob[i])):
                            if j == len(self.prob[i])-1 and cl_no >= len(self.prob[0]):
                                f.write("{0}:(cl'={1});\n".format(self.prob[i][j], cl_no))
                            elif cl_no >= len(self.prob[0]):
                                f.write("{0}:(cl'={1}) + ".format(self.prob[i][j], cl_no))
                            cl_no = cl_no + 1

                        if action != self.actions[-1]:
                            cl_no = cl_no - len(self.prob[i])

            f.write('\n\n')
            f.write('endmodule\n\n\n')

            f.write('rewards "rew"\n\n')
            # For n clusters in total in 49 time steps
            cl_no = 0
            for i in range(48):
                for j in range(len(self.clusters[i])):
                    f.write('[gather_reward](cl={0}):{1};\n'.format(cl_no, self.clusters[i][j]))
                    cl_no = cl_no+1
            f.write('[gather_reward](cl={0}):0;\n'.format(cl_no))
            f.write('\nendrewards\n\n')
               
if __name__ == '__main__':
    ur = rewards_dbscan.uncertain_rewards(False)
    clusters, prob = ur.get_rewards()
    mm = make_model('model_t.prism', 35, 1, 0, clusters, prob)
        
        
        
        
        
                
