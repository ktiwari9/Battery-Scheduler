#! /usr/bin/env python

import probabilistic_rewards
import numpy as np
import subprocess
import yaml
import os

class PrismModel:
    
    def __init__(self,filename, init_b, init_ch, task_prob, clusters, prob, charge_model, discharge_model):
        # initial values that make model. from_t set to zero
        self.task_prob = task_prob
        self.clusters = clusters
        self.prob = prob
        self.actions = ['gather_reward', 'go_charge', 'stay_charging']
        self.charge_model = charge_model
        self.discharge_model = discharge_model
        self.time_int = self.prob.shape[0]
        self.write_prism_file(filename, init_b, init_ch) 
        
    def write_prism_file(self, filename, init_b, init_ch):
        #######################SPECIFY LOCATION ######################       
        path = '/home/milan/workspace/strands_ws/src/battery_scheduler/models/' + filename 
        with open(path, 'w') as f:
            f.write('mdp\n\n')
            f.write('module time_model\n')
            f.write('t:[0..{0}] init 0;\n'.format(self.time_int))
            f.write('task_present:[0..1] init 0;\n')
            f.write('o:[0..1] init 0;\n')
            f.write('e:[0..1] init 0;\n')
            for i in range(self.time_int):
                f.write("[observe] (t={0}) & (o=0) & (e=0) -> {1}:(task_present'=1) & (o'=1) + {2}:(task_present'=0) & (o'=1);\n".format(i, self.task_prob[i][1], self.task_prob[i][0]))
            f.write("[evaluate] (t<{0}) & (o=1) & (task_present=1) & (e=0) -> (e'=1);\n".format(self.time_int))
            for action in self.actions:
                f.write("[{0}] (t<{1}) & (o=1) & (task_present=1) & (e=1) -> (t'=t+1) & (o'=0) & (e'=0) ;\n".format(action, self.time_int))
            for action in self.actions[1:]:
                f.write("[{0}] (t<{1}) & (o=1) & (task_present=0) -> (t'=t+1) & (o'=0);\n".format(action, self.time_int))
            f.write("[dead] (t<{0}) -> (t'=t+1) & (o'=0);\n".format(self.time_int))
            f.write('endmodule\n\n')

            f.write('module battery_model\n')
            f.write('battery:[0..100] init {0};\n'.format(init_b))
            ##### PROBABILISTIC BATTERY
            for b in self.discharge_model:
                if b != 0:
                    bnext_dict = self.discharge_model[b]
                    total = np.sum(np.array(bnext_dict.values()))
                    f.write("[gather_reward] (battery={0}) & (t>-1) -> ".format(b))
                    plus = 0
                    for bnext, val in bnext_dict.items():
                        if plus != 0:
                            f.write(' + ')
                        f.write("{0}:(battery'={1})".format(float(val)/float(total), int(bnext)))
                        plus = plus +1
                    f.write(';\n')

            for b in self.charge_model:
                if b != 0:
                    if b == 100 or b == 99:
                        bnext_dict = self.charge_model[b]
                        total = np.sum(np.array(bnext_dict.values()))
                        f.write("[go_charge] (battery={0}) & (t>-1) -> ".format(b))
                        plus = 0
                        for bnext, val in bnext_dict.items():
                            if plus != 0:
                                f.write(' + ')
                            f.write("{0}:(battery'={1})".format(float(val)/float(total), int(bnext)))
                            plus = plus +1
                        f.write(';\n')
                    else:
                        bnext_dict = self.charge_model[b]
                        total = np.sum(np.array(bnext_dict.values()))
                        f.write("[go_charge] (battery={0}) & (t>-1) -> ".format(b))
                        plus = 0
                        for bnext, val in bnext_dict.items():
                            if plus != 0:
                                f.write(' + ')
                            f.write("{0}:(battery'={1})".format(float(val)/float(total), int(bnext-1)))
                            plus = plus +1
                        f.write(';\n')

            for b in self.charge_model:
                bnext_dict = self.charge_model[b]
                total = np.sum(np.array(bnext_dict.values()))
                f.write("[stay_charging] (battery={0}) & (t>-1) -> ".format(b))
                plus = 0
                for bnext, val in bnext_dict.items():
                    if plus != 0:
                        f.write(' + ')
                    f.write("{0}:(battery'={1})".format(float(val)/float(total), int(bnext)))
                    plus = plus +1
                f.write(';\n')

            f.write("[evaluate] (battery>0) -> (battery'=battery);\n")
            f.write("[observe] (battery>0) -> (battery'=battery);\n")
            f.write("[dead] (battery=0) & (t>-1) -> (battery' = battery);\n")
            f.write('endmodule\n\n')

            f.write('module charging_state\n')
            f.write('charging:[0..1] init {0};\n'.format(init_ch))
            f.write("[gather_reward] (charging=0) | (charging=1) -> (charging'=0);\n")
            f.write("[stay_charging] (charging=1) -> (charging'=1);\n")
            f.write("[go_charge] (charging=0) -> (charging'=1);\n")
            f.write("[dead] (charging=0) -> (charging'=0);\n")
            f.write('endmodule\n\n')


            f.write('module cluster_evolution\n\n')
            f.write('cl:[0..{0}] init 0;\n'.format(len(self.clusters)))
            for t in range(self.time_int):
                f.write('[evaluate] (task_present=1) & (t={0}) -> '.format(t))
                plus = 0
                for cl in range(len(self.clusters)):
                    p = self.prob[t][cl]
                    if p != 0:
                        if plus != 0:
                            f.write(' + ')
                        f.write("{0}:(cl'={1})".format(p, cl))
                        plus+=1
                f.write(';\n')
            f.write('endmodule\n\n')

            f.write('rewards "rew"\n')
            for e,cl in enumerate(self.clusters):
                f.write('[gather_reward] (cl={0}):{1};\n'.format(e,cl))
            f.write('\nendrewards\n\n')

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
    task_prob, prob, clusters = ur.get_probabilistic_reward_model()
    charge_model, discharge_model = get_battery_model()
    mm = PrismModel('model_test.prism', 70, 1, task_prob, clusters, prob, charge_model, discharge_model)