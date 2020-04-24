#! /usr/bin/env python

import probabilistic_rewards
import numpy as np
import subprocess
import roslib
import copy
import yaml
import os

def get_gocharge_model(charge_model):
    gocharge_model = dict ()
    for b, bdict in charge_model.items():
        g_bdict = dict()
        if b == 99 or b == 100:
            g_bdict = copy.deepcopy(bdict)
        else:
            for bn, count in bdict.items():
                gbn = round(0.99*bn)
                if gbn > b:
                    if gbn in g_bdict:
                        g_bdict[gbn] += count
                    else:
                        g_bdict.update({gbn : count})
                else:
                    if bn in g_bdict:
                        g_bdict[bn] += count
                    else:
                        g_bdict.update({bn : count})
        gocharge_model.update({b:g_bdict})
    return gocharge_model


class PrismModel:
    
    def __init__(self,filename, horizon, init_b, init_ch, init_clid, task_prob, clusters, prob, charge_model, discharge_model):
        # initial values that make model. from_t set to zero
        self.task_prob = task_prob
        self.clusters = clusters
        self.prob = prob
        self.actions = ['gather_reward', 'go_charge', 'stay_charging']
        self.charge_model = charge_model
        self.discharge_model = discharge_model
        self.gocharge_model = get_gocharge_model(charge_model)
        self.time_int = horizon  ## horizon in number of intervals

        self.write_prism_file(filename, init_b, init_ch, init_clid) 
        
    def write_prism_file(self, filename, init_b, init_ch, init_clid):
        path = roslib.packages.get_pkg_dir('battery_scheduler')+'/models/' + filename 
        with open(path, 'w') as f:
            f.write('mdp\n\n')
            f.write('module time_model\n')
            f.write('t:[0..{0}] init 0;\n'.format(self.time_int))
            f.write('task_present:[0..1] init {0};\n'.format(1 if init_clid != None else 0))
            f.write('o:[0..1] init 1;\n')
            f.write('e:[0..1] init {0};\n'.format(1 if init_clid != None else 0))
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
                if b > 25:
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

            for b in self.gocharge_model:
                bnext_dict = self.gocharge_model[b]
                total = np.sum(np.array(bnext_dict.values()))
                f.write("[go_charge] (battery={0}) & (t>-1) -> ".format(b))
                plus = 0
                for bnext, val in bnext_dict.items():
                    if plus != 0:
                        f.write(' + ')
                    f.write("{0}:(battery'={1})".format(float(val)/float(total), int(bnext)))
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
            f.write('cl:[0..{0}] init {1};\n'.format(len(self.clusters), init_clid if init_clid != None else 0))
            for t in range(self.time_int):
                if not all([p == 0 for p in self.prob[t]]):
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

            f.write('rewards "batterycost"\n')
            f.write('[gather_reward] (battery<40):1;\n')
            f.write('[stay_charging] (battery<40):1;\n')
            f.write('[go_charge] (battery<40):1;\n')
            f.write('\nendrewards\n\n')

def get_battery_model():
    path = roslib.packages.get_pkg_dir('battery_scheduler')
    if os.path.isfile(path+'/models/battery_charge_model.yaml') and os.path.isfile(path+'/models/battery_discharge_model.yaml'):
        with open (path+'/models/battery_charge_model.yaml', 'r') as f_charge:
            charge_model = yaml.load(f_charge)
        with open (path+'/models/battery_discharge_model.yaml', 'r') as f_discharge:
            discharge_model = yaml.load(f_discharge)
        print ('Battery Models Found at: ' +path+'/models/battery_discharge_model.yaml'+', '+ path+'/models/battery_charge_model.yaml' )
        return charge_model, discharge_model
    else:
        raise ValueError('No models found. First create battery model with probabilistic_battery_modelm.py')
               
if __name__ == '__main__':
    ur = probabilistic_rewards.uncertain_rewards([])
    task_prob, prob, clusters = ur.get_probabilistic_reward_model()
    charge_model, discharge_model = get_battery_model()
    mm = PrismModel('bcth_model.prism', 48, 70, 1, 1, task_prob, clusters, prob, charge_model, discharge_model)

    # mm = PrismModel('bcth12_model.prism', 12, 70, 1, 1, task_prob, clusters, prob, charge_model, discharge_model)

    # mm = PrismModel('bcth24_model.prism', 24, 70, 1, 1, task_prob, clusters, prob, charge_model, discharge_model)

    # mm = PrismModel('bcth36_model.prism', 36, 70, 1, 1, task_prob, clusters, prob, charge_model, discharge_model)