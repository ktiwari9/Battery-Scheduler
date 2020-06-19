#! /usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np 
import subprocess
import roslib
import yaml
import os

def get_simbattery_model(time_passed, charging):  # time_int in minutes
    path = roslib.packages.get_pkg_dir('battery_scheduler')
    if bool(charging):
        # time_passed = int(2.5*time_passed)
        if os.path.isfile(path+'/models/'+str(time_passed)+'battery_charge_model.yaml'):
            with open (path+'/models/'+str(time_passed)+'battery_charge_model.yaml', 'r') as f_charge:
                model = yaml.load(f_charge)
        else:
            subprocess.call('./probabilistic_simbattery_model.py '+ str(time_passed)+' '+str(charging),shell=True, cwd=path+'/src')
            with open (path+'/models/'+str(time_passed)+'battery_charge_model.yaml', 'r') as f_charge:
                model = yaml.load(f_charge)
    else:
        # time_passed = int(3*time_passed)
        if os.path.isfile(path+'/models/'+str(time_passed)+'battery_discharge_model.yaml'):
            with open (path+'/models/'+str(time_passed)+'battery_discharge_model.yaml', 'r') as f_discharge:
                model = yaml.load(f_discharge)
        else:
            subprocess.call('./probabilistic_simbattery_model.py '+ str(time_passed)+' '+str(charging),shell=True, cwd=path+'/src')
            with open (path+'/models/'+str(time_passed)+'battery_discharge_model.yaml', 'r') as f_discharge:
                model = yaml.load(f_discharge)
    
    for b in model:
        bnext_dict = model[b]
        total = np.sum(np.array(bnext_dict.values()))
        for bn in bnext_dict:
            bnext_dict[bn] = float(bnext_dict[bn])/total
    
    return model


def get_current_battery(prev_battery, prev_charging):

    time_passed = 30
    model = get_simbattery_model(time_passed, prev_charging)
    
    if model[prev_battery]: 
        predict_b = []
        for j in range(1):
            nb = []
            prob = []
            for b,p in model[prev_battery].items():
                nb.append(b)
                prob.append(p)
            predict_b.append(np.random.choice(nb, p=prob))
        return int(np.mean(predict_b))
    else:
        if bool(prev_charging):
            return 100
        else:
            return 0


if __name__ == "__main__":
    start_bs = [0, 100]
    end_bs = [100, 0]
    charging_vals = [1, 0]

    b_model = []
    t_model = []
    
    for m in range(len(charging_vals)):
        start_b = start_bs[m]
        end_b = end_bs[m]
        charging = charging_vals[m]
        battery_runs = [] 

        ### Plotting Battery Model
        for k in range(100):
            print k, '----'
            battery = []
            current_b = start_b
            battery.append(current_b)
            while (current_b!=end_b):
                current_b = get_current_battery(current_b, charging) ## gets next battery after time interval T as in prob battery model
                battery.append(current_b)

            battery_runs.append(battery)

        max_len = 0
        for battery in battery_runs:
            if len(battery) > max_len:
                max_len = len(battery)

        ## adjusting array lengths to max_length
        for battery in battery_runs:
            b_len = len(battery)
            if b_len < max_len:            
                last_b_el = battery[-1]
                battery.extend([last_b_el for i in range(1,(max_len-b_len)+1)])
            

        battery_mean = np.mean(np.array(battery_runs),axis=0)
        b_model.append(battery_mean)
        time = [i for i in range(max_len)]
        t_model.append(time)



    fig1, ax1 = plt.subplots()
    ax1.set_title("Scitos Battery Model")
    ax1.set_xlabel("Time\n No of. Intervals (Each interval = 30 minutes)")
    ax1.set_ylabel("Battery Life %")
    ax1.grid(axis='both', linestyle=':')  
    ch_line,  = ax1.plot(t_model[0], b_model[0], 'g-')
    dch_line,  = ax1.plot(t_model[1], b_model[1], color='orange' )
    ax1.legend((ch_line, dch_line), ("Charging", "Discharging"), loc='best')

    max_len = 0
    for x in t_model:
        if len(x) > max_len:
            max_len = len(x)

    ax1.set_xticks([i for i in range(max_len)])
    ax1.set_xticklabels([i for i in range(max_len)])
    ax1.set_yticks([i for i in range(0,101,5)])
    ax1.set_yticklabels([i for i in range(0,101,5)])
    plt.show()




    


    