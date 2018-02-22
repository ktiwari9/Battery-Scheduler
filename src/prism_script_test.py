#! /usr/bin/env python
import rewards_uncertain_hk
import rewards_dbscan
import roslib
import numpy as np

class make_model:
    
    def __init__(self,filename, init_t, init_b, init_ch, init_cluster, clusters, prob, charge_model, discharge_model):
        # initial values that make model. from_t set to zero
        self.clusters = clusters
        self.total_cl = 0
        for i in range(len(clusters)):
            self.total_cl = self.total_cl + len(self.clusters[i])
        self.prob = prob
        self.actions = ['gather_reward', 'go_charge', 'stay_charging', 'tick']
        self.charge_model = charge_model
        self.discharge_model = discharge_model   
        self.time_int = 8 # file for one day - 48 time steps ## multiply each battery by 5
        self.write_prism_file(filename, init_t, init_b, init_ch, init_cluster) 
       
        
    def write_prism_file(self, filename, init_t, init_b, init_ch, init_cluster):
        # path to prism files 
        #path = roslib.packages.get_pkg_dir('battery_scheduler') + '/models/' +filname) 
        path = '/home/milan/workspace/strands_ws/src/battery_scheduler/models/' + filename 
        with open(path, 'w') as f:
            f.write('mdp\n\n')
            f.write('module battery_model\n\n')
            f.write('battery:[0..100] init {0};\n\n'.format(init_b))

            ## 48 inetrval
            # f.write("[gather_reward] (battery>89) & (battery<=100) -> (battery'=battery-4);\n")
            # f.write("[gather_reward] (battery>24) & (battery<90) -> (battery'=battery-5);\n")
            # f.write("[gather_reward] (battery>4) & (battery<25) -> (battery'=battery-4);\n")
            # f.write("[gather_reward] (battery>2) & (battery<5) -> (battery'=battery-3);\n")
            # f.write("[gather_reward] (battery=2) ->  (battery'=battery-2) ;\n")
            # f.write("[gather_reward] (battery=1) -> (battery'=battery-1) ;\n")
            # f.write("[go_charge]  (battery>0) & (battery<5) -> (battery'=battery+6) ;\n")
            # f.write("[go_charge]  (battery>4) & (battery<9) -> (battery'=battery+5) ;\n")
            # f.write("[go_charge]  (battery>8) & (battery<15)-> (battery'=battery+4) ;\n")
            # f.write("[go_charge]  (battery>14) & (battery<29)-> (battery'=battery+3) ;\n")
            # f.write("[go_charge]  (battery>28) & (battery<45)-> (battery'=battery+2) ;\n")
            # f.write("[go_charge]  (battery>44) & (battery<65) -> (battery'=battery+1) ;\n")
            # f.write("[go_charge]  (battery>64) & (battery<98) ->  (battery'=battery+2) ;\n")
            # f.write("[go_charge]  (battery=98) -> (battery'=battery+1) ;\n")
            # f.write("[go_charge]  (battery=99) -> (battery'=battery) ;\n")
            # f.write("[go_charge]  (battery=100)  ->  (battery'=battery) ;\n")
            # f.write("[stay_charging] (battery=0) -> (battery'=battery+7) ;\n")
            # f.write("[stay_charging] (battery>0) & (battery<5) ->  (battery'=battery+7) ;\n")
            # f.write("[stay_charging] (battery>4) & (battery<9)  ->  (battery'=battery+6) ;\n")
            # f.write("[stay_charging] (battery>8) & (battery<15) -> (battery'=battery+5) ;\n")
            # f.write("[stay_charging] (battery>14) & (battery<29)  ->  (battery'=battery+4) ;\n")
            # f.write("[stay_charging] (battery>28) & (battery<45)  -> (battery'=battery+3) ;\n")
            # f.write("[stay_charging] (battery>44) & (battery<65) -> (battery'=battery+2) ;\n")
            # f.write("[stay_charging] (battery>64) & (battery<98)  ->(battery'=battery+3) ;\n")
            # f.write("[stay_charging] (battery=98)  -> (battery'=battery+2) ;\n")
            # f.write("[stay_charging] (battery=99)  ->  (battery'=battery+1) ;\n")
            # f.write("[stay_charging] (battery=100) ->  (battery'=battery) ;\n")

            # ## 16 inetrval
            # f.write("[gather_reward] (battery>89) & (battery<=100) -> (battery'=battery-12);\n")
            # f.write("[gather_reward] (battery>24) & (battery<90) -> (battery'=battery-15);\n")
            # f.write("[gather_reward] (battery>11) & (battery<25) -> (battery'=battery-12);\n")
            # f.write("[gather_reward] (battery>0) & (battery<12) ->  (battery'=0) ;\n")
            # f.write("[go_charge]  (battery>0) & (battery<5) -> (battery'=battery+20) ;\n")
            # f.write("[go_charge]  (battery>4) & (battery<9) -> (battery'=battery+17) ;\n")
            # f.write("[go_charge]  (battery>8) & (battery<15)-> (battery'=battery+14) ;\n")
            # f.write("[go_charge]  (battery>14) & (battery<29)-> (battery'=battery+11) ;\n")
            # f.write("[go_charge]  (battery>28) & (battery<45)-> (battery'=battery+8) ;\n")
            # f.write("[go_charge]  (battery>44) & (battery<65) -> (battery'=battery+5) ;\n")
            # f.write("[go_charge]  (battery>64) & (battery<93) ->  (battery'=battery+8) ;\n")
            # f.write("[go_charge]  (battery>92) & (battery<=100) -> (battery'=100) ;\n")
            # f.write("[stay_charging] (battery=0) -> (battery'=battery+21) ;\n")
            # f.write("[stay_charging] (battery>0) & (battery<5) ->  (battery'=battery+21) ;\n")
            # f.write("[stay_charging] (battery>4) & (battery<9)  ->  (battery'=battery+18) ;\n")
            # f.write("[stay_charging] (battery>8) & (battery<15) -> (battery'=battery+15) ;\n")
            # f.write("[stay_charging] (battery>14) & (battery<29)  ->  (battery'=battery+12) ;\n")
            # f.write("[stay_charging] (battery>28) & (battery<45)  -> (battery'=battery+9) ;\n")
            # f.write("[stay_charging] (battery>44) & (battery<65) -> (battery'=battery+6) ;\n")
            # f.write("[stay_charging] (battery>64) & (battery<92)  ->(battery'=battery+9) ;\n")
            # f.write("[stay_charging] (battery>91) & (battery<=100) -> (battery'=100) ;\n")

            ## 24 inetrval
            # f.write("[gather_reward] (battery>89) & (battery<=100) -> (battery'=battery-8);\n")
            # f.write("[gather_reward] (battery>24) & (battery<90) -> (battery'=battery-10);\n")
            # f.write("[gather_reward] (battery>7) & (battery<25) -> (battery'=battery-8);\n")
            # f.write("[gather_reward] (battery>0) & (battery<8) -> (battery'=0);\n")
            # f.write("[go_charge]  (battery>0) & (battery<5) -> (battery'=battery+13) ;\n")
            # f.write("[go_charge]  (battery>4) & (battery<9) -> (battery'=battery+11) ;\n")
            # f.write("[go_charge]  (battery>8) & (battery<15)-> (battery'=battery+9) ;\n")
            # f.write("[go_charge]  (battery>14) & (battery<29)-> (battery'=battery+7) ;\n")
            # f.write("[go_charge]  (battery>28) & (battery<45)-> (battery'=battery+5) ;\n")
            # f.write("[go_charge]  (battery>44) & (battery<65) -> (battery'=battery+3) ;\n")
            # f.write("[go_charge]  (battery>64) & (battery<96) ->  (battery'=battery+5) ;\n")
            # f.write("[go_charge]  (battery>95) & (battery<=100) -> (battery'=100) ;\n")
            # f.write("[stay_charging] (battery=0) -> (battery'=battery+14) ;\n")
            # f.write("[stay_charging] (battery>0) & (battery<5) ->  (battery'=battery+14) ;\n")
            # f.write("[stay_charging] (battery>4) & (battery<9)  ->  (battery'=battery+12) ;\n")
            # f.write("[stay_charging] (battery>8) & (battery<15) -> (battery'=battery+10) ;\n")
            # f.write("[stay_charging] (battery>14) & (battery<29)  ->  (battery'=battery+8) ;\n")
            # f.write("[stay_charging] (battery>28) & (battery<45)  -> (battery'=battery+6) ;\n")
            # f.write("[stay_charging] (battery>44) & (battery<65) -> (battery'=battery+4) ;\n")
            # f.write("[stay_charging] (battery>64) & (battery<95)  ->(battery'=battery+6) ;\n")
            # f.write("[stay_charging] (battery>94) & (battery<=100)  -> (battery'=100) ;\n")
            
            ## 5 interval
            # f.write("[gather_reward] (battery>89) & (battery<=100) -> (battery'=battery-40);\n")
            # f.write("[gather_reward] (battery>49) & (battery<90) -> (battery'=battery-50);\n")
            # f.write("[gather_reward] (battery>0) & (battery<50) -> (battery'=0);\n")
            # f.write("[go_charge]  (battery>0) & (battery<5) -> (battery'=battery+69) ;\n")
            # f.write("[go_charge]  (battery>4) & (battery<9) -> (battery'=battery+59) ;\n")
            # f.write("[go_charge]  (battery>8) & (battery<15)-> (battery'=battery+49) ;\n")
            # f.write("[go_charge]  (battery>14) & (battery<29)-> (battery'=battery+39) ;\n")
            # f.write("[go_charge]  (battery>28) & (battery<45)-> (battery'=battery+29) ;\n")
            # f.write("[go_charge]  (battery>44) & (battery<65) -> (battery'=battery+19) ;\n")
            # f.write("[go_charge]  (battery>64) & (battery<72) ->  (battery'=battery+29) ;\n")
            # f.write("[go_charge]  (battery>71) & (battery<=100)-> (battery'=100) ;\n")
            # f.write("[stay_charging] (battery=0) -> (battery'=battery+70) ;\n")
            # f.write("[stay_charging] (battery>0) & (battery<5) ->  (battery'=battery+70) ;\n")
            # f.write("[stay_charging] (battery>4) & (battery<9)  ->  (battery'=battery+60) ;\n")
            # f.write("[stay_charging] (battery>8) & (battery<15) -> (battery'=battery+50) ;\n")
            # f.write("[stay_charging] (battery>14) & (battery<29)  ->  (battery'=battery+40) ;\n")
            # f.write("[stay_charging] (battery>28) & (battery<45)  -> (battery'=battery+30) ;\n")
            # f.write("[stay_charging] (battery>44) & (battery<65) -> (battery'=battery+20) ;\n")
            # f.write("[stay_charging] (battery>64) & (battery<71)  ->(battery'=battery+30) ;\n")
            # f.write("[stay_charging] (battery>70) & (battery<=100) -> (battery'=100) ;\n")
            
            ##10 interval
            # f.write("[gather_reward] (battery>89) & (battery<=100) -> (battery'=battery-20);\n")
            # f.write("[gather_reward] (battery>24) & (battery<90) -> (battery'=battery-25);\n")
            # f.write("[gather_reward] (battery>19) & (battery<25) -> (battery'=battery-20);\n")
            # f.write("[gather_reward] (battery>0) & (battery<20) -> (battery'=0);\n")
            # f.write("[go_charge]  (battery>0) & (battery<5) -> (battery'=battery+34) ;\n")
            # f.write("[go_charge]  (battery>4) & (battery<9) -> (battery'=battery+29) ;\n")
            # f.write("[go_charge]  (battery>8) & (battery<15)-> (battery'=battery+24) ;\n")
            # f.write("[go_charge]  (battery>14) & (battery<29)-> (battery'=battery+19) ;\n")
            # f.write("[go_charge]  (battery>28) & (battery<45)-> (battery'=battery+14) ;\n")
            # f.write("[go_charge]  (battery>44) & (battery<65) -> (battery'=battery+9) ;\n")
            # f.write("[go_charge]  (battery>64) & (battery<87) ->  (battery'=battery+14) ;\n")
            # f.write("[go_charge]  (battery>86) & (battery<=100)-> (battery'=100) ;\n")
            # f.write("[stay_charging] (battery=0) -> (battery'=battery+35) ;\n")
            # f.write("[stay_charging] (battery>0) & (battery<5) ->  (battery'=battery+35) ;\n")
            # f.write("[stay_charging] (battery>4) & (battery<9)  ->  (battery'=battery+30) ;\n")
            # f.write("[stay_charging] (battery>8) & (battery<15) -> (battery'=battery+25) ;\n")
            # f.write("[stay_charging] (battery>14) & (battery<29)  ->  (battery'=battery+20) ;\n")
            # f.write("[stay_charging] (battery>28) & (battery<45)  -> (battery'=battery+15) ;\n")
            # f.write("[stay_charging] (battery>44) & (battery<65) -> (battery'=battery+10) ;\n")
            # f.write("[stay_charging] (battery>64) & (battery<86)  ->(battery'=battery+15) ;\n")
            # f.write("[stay_charging] (battery>85) & (battery<=100)  -> (battery'=100) ;\n")
            
            
            
            # for b in self.discharge_model:
            #     bnext_dict = self.discharge_model[b]
            #     total = np.sum(np.array(bnext_dict.values()))
            #     f.write("[gather_reward] (battery={0}) -> ".format(b))
            #     plus = 0
            #     for bnext, val in bnext_dict.items():
            #         if plus != 0:
            #             f.write(' + ')
            #         f.write("{0}:(battery'={1})".format(bnext, float(val)/float(total)))
            #         plus = plus +1
            #     f.write(';\n')

            # for b in self.charge_model:
            #     bnext_dict = self.charge_model[b]
            #     total = np.sum(np.array(bnext_dict.values()))
            #     f.write("[go_charge] (battery={0}) -> ".format(b))
            #     plus = 0
            #     for bnext, val in bnext_dict.items():
            #         if plus != 0:
            #             f.write(' + ')
            #         f.write("{0}:(battery'={1})".format(bnext-1, float(val)/float(total)))
            #         plus = plus +1
            #     f.write(';\n')

            # for b in self.charge_model:
            #     bnext_dict = self.charge_model[b]
            #     total = np.sum(np.array(bnext_dict.values()))
            #     f.write("[stay_charging] (battery={0}) -> ".format(b))
            #     plus = 0
            #     for bnext, val in bnext_dict.items():
            #         if plus != 0:
            #             f.write(' + ')
            #         f.write("{0}:(battery'={1})".format(bnext, float(val)/float(total)))
            #         plus = plus +1
            #     f.write(';\n')

            f.write("[tick] (battery=0) -> (battery' = battery);\n\n")
            f.write('endmodule\n\n\n')

            f.write('module charging_state\n\n')
            f.write('charging:[0..1] init {0};\n'.format(init_ch))
            f.write("[gather_reward] (charging=0) | (charging=1) -> (charging'=0);\n")
            f.write("[stay_charging] (charging=1) -> (charging'=1);\n")
            f.write("[go_charge] (charging=0) -> (charging'=1);\n\n")
            f.write('endmodule\n\n\n')

            f.write('module time_model\n\n')
            f.write('t:[0..{0}] init {1};\n'.format(self.time_int,init_t))
            for action in self.actions:
                f.write("[{0}] (t<{1}) -> (t'=t+1);\n".format(action,self.time_int))
            f.write('\n')
            f.write('endmodule\n\n\n')

            f.write('module cluster_evolution\n\n')
            f.write('cl:[0..{0}] init {1};\n\n'.format((self.total_cl), init_cluster))
            cl_no = 0
            for i in range(self.time_int+1): #change to 48, if 48
                if i == self.time_int:
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
            for i in range(self.time_int):
                for j in range(len(self.clusters[i])):
                    f.write('[gather_reward](cl={0}):{1};\n'.format(cl_no, self.clusters[i][j]))
                    cl_no = cl_no+1
            f.write('[gather_reward](cl={0}):0;\n'.format(cl_no))
            f.write('\nendrewards\n\n')
               
if __name__ == '__main__':
    ur = rewards_dbscan.uncertain_rewards(True)
    clusters, prob = ur.get_rewards()
    mm = make_model('model_test.prism', 0, 70, 1, 2, clusters, prob, 'a', 'b')