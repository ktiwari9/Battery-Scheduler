#!/usr/bin/env python

import sys
import os
import time
import numpy as np
import math
import yaml
import roslib

class BatteryData:

    #parsing for latest data
    def __init__(self, battery_str):
        arr =  battery_str[:-1].split(',')
        if len(arr) != 10:
            raise ValueError("battery string given was invalid: {0}".format(battery_str))
        self.epoch = int(int(arr[0])/1000000000)
        self.life = int(arr[6]) if arr[6] else 0
        self.is_charging = arr[8] == '1'
        
    
def extract_data(path_to_files):
    charging_data = []
    discharging_data = []
    for d_file in os.listdir(path_to_files):
        if d_file.endswith('.txt'):
            f = open(path_to_files+'/'+d_file, 'r') 
            print 'reading file' 
            for line in f.readlines():
                    if line[0] != '%' or line[0] != '#':
                        try:
                            battery_state = BatteryData(line)
                        
                            if battery_state.is_charging == True:
                                if len(charging_data) == 0:
                                    charging_data.append([battery_state])
                        
                                else:
                                    if charging_data[-1][-1].epoch != battery_state.epoch:
                                        #print battery_state.life, battery_state.epoch
                                        #print abs(charging_data[-1][-1].epoch - battery_state.epoch)
                                        if abs(charging_data[-1][-1].epoch - battery_state.epoch) == 60:
                                            if charging_data[-1][-1].life <= battery_state.life:
                                        #        print battery_state.life, battery_state.epoch
                                                charging_data[-1].append(battery_state)
                                            else:
                                                charging_data.append([battery_state])

                                        elif abs(charging_data[-1][-1].epoch - battery_state.epoch) % 60 == 0:
                                            charging_data.append([battery_state])
                            
                            elif battery_state.is_charging == False:
                                if len(discharging_data) == 0:
                                    discharging_data.append([battery_state])
                                    #print battery_state.life, battery_state.epoch
                                else:
                                    if discharging_data[-1][-1].epoch != battery_state.epoch:
                                        #print battery_state.life, battery_state.epoch
                                        #print abs(discharging_data[-1][-1].epoch - battery_state.epoch)
                                        if abs(discharging_data[-1][-1].epoch - battery_state.epoch) == 60:
                                            if discharging_data[-1][-1].life >= battery_state.life:
                                        #        print battery_state.life, battery_state.epoch
                                                discharging_data[-1].append(battery_state)
                                            else:
                                                discharging_data.append([battery_state])
                                        elif abs(discharging_data[-1][-1].epoch - battery_state.epoch) % 60 == 0:
                                            discharging_data.append([battery_state])

                        except ValueError:
                            continue
            
    return charging_data, discharging_data
    
def check_variation_values(data):
    unique = []
    for d in data:
        if d.life not in unique:
             unique.append(d.life)
    if len(unique) > 3:
        return True
    else:
        return False

# input array of Battery
def get_battery_model(path_to_directory):
    path = '/home/milan/workspace/strands_ws/src/battery_scheduler'
    #path = roslib.packages.get_pkg_dir('battery_scheduler')
    if os.path.isfile(path+'/models/battery_charge_model.yaml') and os.path.isfile(path+'/models/battery_discharge_model.yaml'):
        with open (path+'/models/battery_charge_model.yaml', 'r') as f_charge:
            charge_model = yaml.load(f_charge)
        with open (path+'/models/battery_discharge_model.yaml', 'r') as f_discharge:
            discharge_model = yaml.load(f_discharge)

    else:
        charge_model = dict()
        discharge_model = dict()
        for model in [charge_model, discharge_model]:
            for i in range (101):
                model.update({ i : dict()})
   
        charging_data, discharging_data = extract_data(path_to_directory) ## specify path to battery files
        for data_set in [charging_data, discharging_data]:
            for data in data_set:
                for i in range(len(data)):
                    if len(data) >= 30 and check_variation_values(data):
                        current_bs = data[i]
                        if i < len(data)-30:
                            next_bs = data[i+30]
                            if current_bs.is_charging == True:
                                bl_dict = charge_model[current_bs.life]
                                if next_bs.life not in bl_dict:
                                    count = 1
                                else:
                                    count = bl_dict[next_bs.life] + 1
                                bl_dict.update({next_bs.life : count})
                                charge_model.update({current_bs.life : bl_dict})

                            elif current_bs.is_charging == False:
                                bl_dict = discharge_model[current_bs.life]
                                if next_bs.life not in bl_dict:
                                    count = 1
                                else:
                                    count = bl_dict[next_bs.life] + 1
                                bl_dict.update({next_bs.life : count})
                                discharge_model.update({current_bs.life : bl_dict})

        for battery_val in discharge_model:
            if len(discharge_model[battery_val].keys()) == 0:
                if battery_val == 100:
                    nb_dict = discharge_model[99]
                else:
                    nb_dict = dict({0 : 1})
                discharge_model.update({battery_val : nb_dict})

        for battery_val in charge_model:
            if len(charge_model[battery_val].keys()) == 0:
                nb_dict = dict({100 : 1})
                charge_model.update({battery_val : nb_dict})



        f_discharge = file(path+'/models/battery_discharge_model.yaml', 'w')
        yaml.dump(discharge_model, f_discharge)
        f_charge =file(path+ '/models/battery_charge_model.yaml', 'w')
        yaml.dump(charge_model, f_charge)

    return charge_model, discharge_model



if __name__ == '__main__':
    
    file_path = '/media/milan/DATA/battery_logs'
    #ch_mod, dch_mod = get_battery_model(file_path)
    ch_mod, dch_mod = extract_data(file_path)
    for mod in [ch_mod, dch_mod]:
        print '############'
        for data in mod:
            print '%%%%%%%%%%%%%%%%'
            battery_l = list(map(lambda x: x.life, data))
            print battery_l



