#!/usr/bin/env python3

from datetime import datetime, time, date 
import pandas as pd
from io import open
import codecs
import math
import yaml
import sys
import csv
import os

def timing_wrapper(func):
    def wrapper(*args,**kwargs):
        t = datetime.now()
        result = func(*args,**kwargs)
        t1 = datetime.now()
        print (func, ' took time:', t1-t)
        return result
    return wrapper


class BatteryModel:
    
    def __init__(self, time_interval, is_charging):
        directory_paths = ['/media/milan/DATA1/data_project/battery_data/betty', '/media/milan/DATA1/battery_logs/real_battery']
        self.time_interval = time_interval ## minutes
        self.is_charging = is_charging
        self.get_battery_model(directory_paths)

    def get_files(self, directories):
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
                    if day_files:
                        to_process.append(day_files)
        print("Total {0} days".format(len(to_process)))
        return to_process


    @timing_wrapper 
    def extract_data(self, files):
        print ('Extracting data and forming model....')
        for e, file_set in enumerate(files):
            for d_file in file_set:
                if e == len(files)-1:  ## because last file being read is in different format
                    df = pd.read_csv(d_file, header=None, names=['epoch', 'seq', 'stamp', 'frame_id', 'voltage', 'current', 'life', 'lifetime', 'charging', 'powersupply'], skipinitialspace=True, skiprows=[0], usecols=['epoch', 'life', 'charging'], na_values=[''], converters={'epoch': lambda x: datetime.fromtimestamp(float(x)/1000000000)})
                else:
                    df = pd.read_csv(d_file, header=None, names=['epoch', 'node', 'life', 'voltage', 'charging', 'current'], skipinitialspace=True, skiprows=[0], usecols=['epoch', 'life', 'charging'], na_values=[''], converters={'epoch': lambda x: datetime.fromtimestamp(float(x))})
                
                df.dropna(inplace=True)
                df.set_index(pd.DatetimeIndex(df['epoch']), drop=False, inplace=True)
                df['life'] = df['life'].apply(lambda x: int(x))
                df['charging'] = df['charging'].apply(lambda x: int(x))
                df.sort_index(inplace=True)
                df.drop_duplicates(subset='epoch', keep='last', inplace=True)
                df = df.resample('1T').mean()
               
                if self.is_charging:
                    battery_charge = df[df['charging'] == True]
                    if not battery_charge.empty:
                        self.__update_battery_model(battery_charge['life'],charging=True)
                else:
                    battery_discharge = df[df['charging'] == False]
                    if not battery_discharge.empty:
                        self.__update_battery_model(battery_discharge['life'], charging=False)

    @timing_wrapper 
    def get_battery_model(self, directories):
        path ='/home/milan/workspace/strands_ws/src/battery_scheduler'
        self.charge_model = dict()
        self.discharge_model = dict()

        if self.time_interval > 600: ## If time > 10 hrs default to max, min battery 
            if self.is_charging:
                for i in range(101):
                    self.charge_model.update({i: {100 : 1}})
            else:
                for i in range(101):
                    self.discharge_model.update({i: {0 : 1}})

        else:
            for model in [self.charge_model, self.discharge_model]:
                for i in range (101):
                    model.update({ i : dict()})
            self.discharge_model[100] = self.discharge_model[99]
            self.charge_model[0]  = self.charge_model[1]

            filesets = self.get_files(directories)
            self.extract_data(filesets)
        
        if self.is_charging:
            with open(path+ '/models/'+str(self.time_interval)+'battery_charge_model.yaml', 'w') as f_charge:
                yaml.dump(self.charge_model, f_charge)
        else:
            with open(path+'/models/'+str(self.time_interval)+'battery_discharge_model.yaml', 'w') as f_discharge:
                yaml.dump(self.discharge_model, f_discharge)
        
        print ('Battery Models Created at: '+path+'/models/'+str(self.time_interval)+'battery_discharge_model.yaml'+ ', '+ path+'/models/'+str(self.time_interval)+'battery_charge_model.yaml')

    # @timing_wrapper       
    def __update_battery_model(self,df,charging=True):
        if charging:
            model = self.charge_model
        else:
            model = self.discharge_model
        for i in range(0,df.shape[0]-self.time_interval-1):
            if (df.index[i+self.time_interval] - df.index[i]) == pd.Timedelta(minutes=self.time_interval) and not math.isnan(df[i]) and not math.isnan(df[i+self.time_interval]):
                current_b = int(round(df[i]))
                next_b = int(round(df[i+self.time_interval]))
                if charging and next_b < current_b:
                    next_b = current_b
                if not charging and next_b > current_b:
                    next_b = current_b
                if next_b > 100:
                    next_b = 100
                if next_b not in model[current_b]:
                    model[current_b].update({next_b:1})
                else:
                    model[current_b][next_b] += 1
    
if __name__ == '__main__':
    time_int = int(sys.argv[1])
    charging = bool(int(sys.argv[2]))
    bm = BatteryModel(time_int, charging)
  