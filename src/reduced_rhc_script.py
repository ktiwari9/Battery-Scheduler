#! /usr/bin/env python
import os
import rewards_uncertain_hk
import roslib
import numpy as np
from scipy.cluster.vq import kmeans2 
from collections import Counter

class make_model:
    
    def __init__(self, filename, from_t, init_b, init_ch, init_cluster, clusters, prob, charge_model, discharge_model):
        # initial values that make model. from_t set to zero
        self.time_int = 48
        self.horizon = 48 # for a horizon of 48 time steps
        self.no_cluster = 1
        self.clusters = self.horizon*[0]
        self.prob = self.horizon*[0]
        for k in range(self.horizon):  
            self.prob[k] = prob[(from_t+k)%self.time_int]
            self.clusters[k] = clusters[(from_t+k)%self.time_int]
        
        self.total_cl = self.time_int-self.no_cluster  ## 47 -> time steps for which clusters are not present
        for j in range(self.no_cluster):
            self.total_cl = self.total_cl + len(self.clusters[j])

        self.charge_model = self.reduce_battery_model(charge_model)
        self.discharge_model = self.reduce_battery_model(discharge_model)
        self.actions = ['gather_reward', 'go_charge', 'stay_charging', 'tick']
        self.write_prism_file(filename,init_b, init_ch, init_cluster)

    def reduce_battery_model(self, model):
        for b in model:
            if len(model[b].keys()) > 10:
                in_arr = []
                for nb in model[b]:
                    in_arr.append([float(nb)])
                centroid, labels = kmeans2(np.array(in_arr), 5, minit='points')
                #print centroid
                count_dict = Counter(labels)
                b_model = dict()
                for i in range(len(centroid)):
                    b_model.update({int(centroid[i][0]) : count_dict[i]})
                print b
                print b_model
                print '####'
                model.update({b : b_model})
        return model    
        
    def write_prism_file(self, filename,init_b, init_ch, init_cluster):
        # path to prism files   
        #path = roslib.packages.get_pkg_dir('battery_scheduler') + '/models/' +filname) 
        path = '/home/milan/workspace/strands_ws/src/battery_scheduler/models/' + filename 
        with open(path, 'w') as f:
            f.write('mdp\n\n')
            f.write('module battery_model\n\n')
            f.write('battery:[0..100] init {0};\n\n'.format(init_b))

            ### Probabilistic Model            
            for b in self.discharge_model:
                if b != 0:
                    bnext_dict = self.discharge_model[b]
                    total = np.sum(np.array(bnext_dict.values()))
                    f.write("[gather_reward] (battery={0}) -> ".format(b))
                    plus = 0
                    for bnext, val in bnext_dict.items():
                        if plus != 0:
                            f.write(' + ')
                        f.write("{0}:(battery'={1})".format(float(val)/float(total), bnext))
                        plus = plus +1
                    f.write(';\n')

                    
            for b in self.charge_model:
                if b != 0:
                    if b == 100:
                        bnext_dict = self.charge_model[b]
                        total = np.sum(np.array(bnext_dict.values()))
                        f.write("[go_charge] (battery={0}) -> ".format(b))
                        plus = 0
                        for bnext, val in bnext_dict.items():
                            if plus != 0:
                                f.write(' + ')
                            f.write("{0}:(battery'={1})".format(float(val)/float(total), bnext))
                            plus = plus +1
                        f.write(';\n')
                    else:
                        bnext_dict = self.charge_model[b]
                        total = np.sum(np.array(bnext_dict.values()))
                        f.write("[go_charge] (battery={0}) -> ".format(b))
                        plus = 0
                        for bnext, val in bnext_dict.items():
                            if plus != 0:
                                f.write(' + ')
                            f.write("{0}:(battery'={1})".format(float(val)/float(total), bnext-1))
                            plus = plus +1
                        f.write(';\n')

            for b in self.charge_model:
                bnext_dict = self.charge_model[b]
                total = np.sum(np.array(bnext_dict.values()))
                f.write("[stay_charging] (battery={0}) -> ".format(b))
                plus = 0
                for bnext, val in bnext_dict.items():
                    if plus != 0:
                        f.write(' + ')
                    f.write("{0}:(battery'={1})".format(float(val)/float(total), bnext))
                    plus = plus +1
                f.write(';\n')

            ## Deterministic Model
            # for b in self.discharge_model:
            #     if b != 0:
            #         bnext_dict = self.discharge_model[b]
            #         total = np.sum(np.array(bnext_dict.values()))
            #         avg = 0.0
            #         for bnext, val in bnext_dict.items():
            #             avg = avg + bnext*val
            #         f.write("[gather_reward] (battery={0}) -> 1:(battery'={1});\n".format(b, int(round(avg/total))))

            # for b in self.charge_model:
            #     bnext_dict = self.charge_model[b]
            #     total = np.sum(np.array(bnext_dict.values()))
            #     avg = 0.0
            #     for bnext, val in bnext_dict.items():
            #         avg = avg + bnext*val
            #     if b!= 0:
            #         if b == 100:
            #             f.write("[go_charge] (battery={0}) -> 1:(battery'={1});\n".format(b, int(round(avg/total))))
            #         f.write("[go_charge] (battery={0}) -> 1:(battery'={1});\n".format(b, int(round(avg/total)-1)))

            # for b in self.charge_model:
            #     bnext_dict = self.charge_model[b]
            #     total = np.sum(np.array(bnext_dict.values()))
            #     avg = 0.0
            #     for bnext, val in bnext_dict.items():
            #         avg = avg + bnext*val
            #     f.write("[stay_charging] (battery={0}) -> 1:(battery'={1});\n".format(b, int(round(avg/total))))

            f.write("[tick] (battery=0) -> (battery' = battery);\n\n")
            f.write('endmodule\n\n\n')

            f.write('module charging_state\n\n')
            f.write('charging:[0..1] init {0};\n'.format(init_ch))
            f.write("[gather_reward] (charging=0) | (charging=1) -> (charging'=0);\n")
            f.write("[stay_charging] (charging=1) -> (charging'=1);\n")
            f.write("[go_charge] (charging=0) -> (charging'=1);\n")
            f.write("[tick] (charging=0) -> (charging'=0);\n\n")
            f.write('endmodule\n\n\n')

            f.write('module time_model\n\n')
            f.write('t:[0..{0}] init 0;\n'.format(self.horizon))
            for action in self.actions:
                f.write("[{0}] (t<{1}) -> (t'=t+1);\n".format(action, self.horizon))
            f.write('\n')
            f.write('endmodule\n\n\n')

            f.write('module cluster_evolution\n\n')
            f.write('cl:[0..{0}] init {1};\n\n'.format(self.total_cl, init_cluster))
            cl_no = 0
            for i in range(self.horizon+1): #change to 49, if 48
                if i == self.horizon:
                    for action in self.actions:
                        f.write('[{0}] (t={1}) -> '.format(action,i-1))
                        f.write("1:(cl'={0});\n".format(cl_no))

                elif i < self.no_cluster:
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

                else:
                    for action in self.actions:
                        f.write("[{0}] (t={1}) -> (cl'={2});\n".format(action, i-1, cl_no))
                        cl_no = cl_no+1

                        if action != self.actions[-1]:
                            cl_no = cl_no - 1

            f.write('\n\n')
            f.write('endmodule\n\n\n')
        
             
            f.write('rewards "rew"\n\n')
            # For n clusters in total in h horizon
            cl_no = 0
            for i in range(self.horizon):
                if i < self.no_cluster: ## no.of time steps for which clusters are considered
                    test_sum = 0
                    for j in range(len(self.clusters[i])):
                        f.write('[gather_reward](cl={0}):{1};\n'.format(cl_no, self.clusters[i][j]))
                        print self.clusters[i][j]
                        cl_no = cl_no+1
                        test_sum = test_sum +  self.clusters[i][j]*self.prob[i][j]
                    print test_sum 
                    print '-----'
                else:
                    reward = 0
                    for j in range(len(self.clusters[i])): 
                        reward = reward + self.clusters[i][j]*self.prob[i][j]
                    f.write('[gather_reward](cl={0}):{1};\n'.format(cl_no, reward))
                    cl_no = cl_no+1
                    print reward
                    print '----------'
            f.write('\nendrewards\n\n')
            

