#!/usr/bin/env python3

import sys
import os
import time
import numpy as np
import math

class BatteryData:

    def __init__(self, battery_str):
        arr =  battery_str[:-1].split(', ')
        if len(arr) != 6:
            raise ValueError("battery string given was invalid: {0}".format(battery_str))
        self.epoch = arr[0]
        self.cur_node = arr[1]
        self.life = int(arr[2]) if arr[2] else 0
        self.voltage = float(arr[3]) if arr[3] else 0
        self.is_charging = arr[4] == '1'
        self.current = float(arr[5]) if arr[5] else 0
        self.day = (time.localtime(float(self.epoch))[0], time.localtime(float(self.epoch))[1], time.localtime(float(self.epoch))[2])
        
        
def get_files(directories):
    to_process = []
    for directory in directories:
        if not os.path.isdir(directory):
            print("Path {0} is not a directory.".format(directory))
            return

        print("Processing directory {0}".format(directory))

        for root, dirs, files in os.walk(directory):
            if files:
                # reverse to ensure correct alphanumeric order
                to_process.extend(reversed([os.path.abspath(os.path.join(root, f)) for f in files]))

        print("Total {0} files".format(len(to_process)))
    return to_process
    
def extract_data(files):
    data_objects = []
    for d_file in sorted(files):
        f = open(d_file, encoding = "ISO-8859-1") 
        for line in f.readlines():
                if line[0] != '#':
                    try:
                        data_objects.append(BatteryData(line))
                    except ValueError:
                        continue
    print (len(data_objects), 'Original amount of data')
    dated_data = dict()
    print ("Extracting data according to date")
    for data in data_objects:
        if data.day not in dated_data:
            data_l = []
            data_l.append(data)
            
        else:
            data_l = dated_data[data.day]
            if data_l[-1].life != data.life:
                data_l.append(data)
            
        dated_data.update({ data.day : data_l})
        
    return dated_data
    
    
    
def check_variation_values(data):
    unique = []
    for d in data:
        if d.life not in unique:
             unique.append(d.life)
    if len(unique) > 3:
        return True
    else:
        return False
        
           
def get_charging_data(dated_data, boolean):
    print ("Extracting data relevant for charging and discharging")
    cdated_data = dict()
    for day in dated_data:
        c_data = []
        for data in dated_data[day]:
            if data.is_charging == boolean:
                c_data.append(data)
        #if len(c_data) > 10 and check_variation_values(c_data):        
        cdated_data.update({day : c_data})  
           
    return cdated_data
        


