#!/usr/bin/env python3

from datetime import datetime, time, date 
import pandas as pd
from io import open
import yaml
import os

class BatteryData:
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
       

class BatteryModel:
    def __init__(self, directory_paths):
        self.charge_model = None
        self.discharge_model = None 
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
                    to_process.append(day_files)
        print("Total {0} days".format(len(to_process)))
        return to_process
    
    def extract_data(self, files):
        print ('Extracting data and forming model....')
        for file_set in files:
            data_battery = []
            data_timestamp = []
            data_ischarging = []
            data_day = []
            data_time = []
            for d_file in file_set:
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

            battery_data = pd.DataFrame(data=list(zip(data_battery, data_ischarging, data_timestamp)), index=data_timestamp, columns= ['life', 'is_charging', 'time'])
            battery_data.sort_index(inplace=True)
            battery_data.drop_duplicates(subset='time', keep='last', inplace=True)
            battery_data.set_index(pd.DatetimeIndex(battery_data.index), drop=False, inplace=True, verify_integrity=True)
            battery_data = battery_data.resample('1T').pad()
            battery_charge = battery_data[battery_data['is_charging'] == True]
            battery_discharge = battery_data[battery_data['is_charging'] == False]
            if not battery_discharge.empty:
                self.__update_battery_model(battery_discharge['life'], charging=False)
            if not battery_charge.empty:
                self.__update_battery_model(battery_charge['life'],charging=True)
           
    def __update_battery_model(self,df,charging=True):
        if charging:
            model = self.charge_model
        else:
            model = self.discharge_model
        for i in range(0,df.shape[0]-31):
            if (df.index[i+30] - df.index[i]) == pd.Timedelta(minutes=30):
                if df[i+30] not in model[df[i]]:
                    model[df[i]].update({df[i+30]:1})
                else:
                   model[df[i]][df[i+30]] += 1
    
    def get_battery_model(self, paths):
        ################ SPECIFY PATHS OF MODELS #######################
        path = '/home/milan/workspace/strands_ws/src/battery_scheduler'
        if os.path.isfile(path+'/models/battery_charge_model.yaml') and os.path.isfile(path+'/models/battery_discharge_model.yaml'):
            with open (path+'/models/battery_charge_model.yaml', 'r') as f_charge:
                self.charge_model = yaml.load(f_charge)
            with open (path+'/models/battery_discharge_model.yaml', 'r') as f_discharge:
                self.discharge_model = yaml.load(f_discharge)
            print ('Battery Models Found at: ' +path+'/models/battery_discharge_model.yaml'+', '+ path+'/models/battery_charge_model.yaml' )

        else:
            self.charge_model = dict()
            self.discharge_model = dict()
            for model in [self.charge_model, self.discharge_model]:
                for i in range (101):
                    model.update({ i : dict()})

            files = self.get_files(paths)
            self.extract_data(files)

            with open(path+'/models/battery_discharge_model.yaml', 'w') as f_discharge:
                yaml.dump(self.discharge_model, f_discharge)
            with open(path+ '/models/battery_charge_model.yaml', 'w') as f_charge:
                yaml.dump(self.charge_model, f_charge)
            print ('Battery Models Created at: '+path+'/models/battery_discharge_model.yaml'+ ', '+ path+'/models/battery_charge_model.yaml')


if __name__ == '__main__':
    ################ SPECIFY PATHS OF DATA #######################
    paths = ['/media/milan/DATA/data_project/battery_data/betty','/media/milan/DATA/data_project/battery_data/bob', '/media/milan/DATA/battery_logs/real_battery']
    bm = BatteryModel(paths)
    # print (bm.charge_model)
    # print (bm.discharge_model)