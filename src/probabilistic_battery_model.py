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
                self.is_charging = arr[4].strip() == '1'
                self.date_time = datetime.fromtimestamp(self.epoch)
        elif len(arr) == 10:
            if ' ' != arr[0] and ' ' != arr[6] and ' ' != arr[8]:   
                self.epoch = int(int(arr[0])/1000000000)
                self.life = int(arr[6]) if arr[6] else 0
                self.is_charging = arr[8] == '1'
                self.date_time = datetime.fromtimestamp(self.epoch)
       
def get_files(directories):
    to_process = []
    for directory in directories:
        if not os.path.isdir(directory):
            print("Path {0} is not a directory.".format(directory))
            return

        print("Processing directory {0}".format(directory))

        for root, dirs, files in os.walk(directory):
            day_files = []
            if files:
                # reverse to ensure correct alphanumeric order
                day_files.extend(reversed([os.path.abspath(os.path.join(root, f)) for f in files if f.endswith('.txt')]))
                to_process.append(day_files)

    print("Total {0} days".format(len(to_process)))
    return to_process
    
def extract_data(files):
    print ('Extracting data....')
    for file_set in files:
        data_battery = []
        data_timestamp = []
        data_ischarging = []
        data_day = []
        data_time = []
        for d_file in file_set:
            print (d_file)
            if file_set == files[-1]:
                f = open(d_file, 'r')
            else:
                f = open(d_file, mode='r', encoding = "ISO-8859-1") 
            for line in f.readlines():
                if '%' not in line and '#' not in line:
                    bd = BatteryData(line)
                    if hasattr(bd, 'life'):
                        data_battery.append(bd.life)
                        data_timestamp.append(bd.date_time)
                        data_ischarging.append(bd.is_charging)
                        data_day.append(bd.date_time.date())
                        data_time.append(bd.date_time.time())

        battery_data = pd.DataFrame(data=list(zip(data_battery, data_ischarging, data_time, data_day)), index=data_timestamp, columns= ['life', 'is_charging', 'time', 'day'])
        battery_data.sort_index(inplace=True)
        battery_data.drop_duplicates(subset='time', keep='last', inplace=True)
        battery_data.set_index(pd.DatetimeIndex(battery_data.index), drop=False, inplace=True, verify_integrity=True)
        battery_data = battery_data.resample('30T').pad()
        battery_charge = battery_data[battery_data['is_charging'] == True]
        battery_discharge = battery_data[battery_data['is_charging'] == False]
        # if not battery_discharge.empty:
        #     print (battery_discharge)
        # if not battery_charge.empty:
        #     print (battery_charge) 

        ## update the probabilistic model
    return battery_data
    
def get_battery_model
    
if __name__ == '__main__':
    paths = ['/media/milan/DATA/data_project/battery_data/betty','/media/milan/DATA/data_project/battery_data/bob', '/media/milan/DATA/battery_logs/real_battery']
    files = get_files(paths)
    dated_data = extract_data(files)
