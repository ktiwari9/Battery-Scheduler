#!/usr/bin/env python3

from datetime import datetime, time, date 
import pandas as pd
from io import open
import sys
import os
import time
import numpy as np
import math
import yaml

class BatteryData:
    # parsing for old data
    def __init__(self, battery_str):
        arr =  battery_str[:-1].split(',')
        if len(arr) == 6:
            if ' ' != arr[0] and ' ' != arr[2] and ' ' != arr[4]:
                self.epoch = round(float(arr[0]))
                self.life = int(arr[2]) if arr[2] else 0
                self.is_charging = arr[4] == '1'
                self.date_time = datetime.fromtimestamp(self.epoch)
       
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
                to_process.extend(reversed([os.path.abspath(os.path.join(root, f)) for f in files if f.endswith('.txt')]))

        print("Total {0} files".format(len(to_process)))
    return to_process
    
def extract_data(files):
    print ('Extracting data....')
    data_battery = []
    data_timestamp = []
    data_ischarging = []
    for d_file in sorted(files):
        f = open(d_file, mode='r', encoding = "ISO-8859-1") 
        for line in f.readlines():
           if '%' not in line and '#' not in line:
                try:
                    data_battery.append(BatteryData(line).life)
                    data_timestamp.append(BatteryData(line).date_time)
                    data_ischarging.append(BatteryData(line).is_charging)
                except AttributeError:
                    continue
        battery_data = pd.DataFrame(data=list(zip(data_timestamp, data_battery, data_ischarging)), columns= ['time','life', 'is_charging'])
        battery_charge = battery_data[battery_data['is_charging'] == True]
        battery_discharge = battery_data[battery_data['is_charging'] == False]
       
        break
        ## resample for everyhalf hour 
        ## update the probabilistic model
    return battery_data
    


if __name__ == '__main__':
    paths = ['/media/milan/DATA/data_project/battery_data/betty','/media/milan/DATA/data_project/battery_data/bob' ]
    files = get_files(paths)
    dated_data = extract_data(files)
