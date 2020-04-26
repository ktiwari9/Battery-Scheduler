#! /usr/bin/env python

import taskbased_sample_generator
from datetime import datetime, timedelta
import numpy as np
import subprocess
import roslib
import yaml
import os


def timing_wrapper(func):
    def wrapper(*args,**kwargs):
        t = datetime.now()
        result = func(*args,**kwargs)
        t1 = datetime.now()
        print func, ' took time:', t1-t
        return result
    return wrapper


def get_simbattery_model(time_passed, charging, action):  # time_int in minutes
    path = roslib.packages.get_pkg_dir('battery_scheduler')
    # time_passed = 3*time_passed
    if bool(charging):
        if os.path.isfile(path+'/models/'+str(time_passed)+'battery_charge_model.yaml'):
            with open (path+'/models/'+str(time_passed)+'battery_charge_model.yaml', 'r') as f_charge:
                model = yaml.load(f_charge)
        else:
            subprocess.call('./probabilistic_simbattery_model.py '+ str(time_passed)+' '+str(charging),shell=True, cwd=path+'/src')
            with open (path+'/models/'+str(time_passed)+'battery_charge_model.yaml', 'r') as f_charge:
                model = yaml.load(f_charge)
    else:
        if os.path.isfile(path+'/models/'+str(time_passed)+'battery_discharge_model.yaml'):
            with open (path+'/models/'+str(time_passed)+'battery_discharge_model.yaml', 'r') as f_discharge:
                model = yaml.load(f_discharge)
        else:
            subprocess.call('./probabilistic_simbattery_model.py '+ str(time_passed)+' '+str(charging),shell=True, cwd=path+'/src')
            with open (path+'/models/'+str(time_passed)+'battery_discharge_model.yaml', 'r') as f_discharge:
                model = yaml.load(f_discharge)

    if action == 'go_charge':
        gocharge_model = dict ()
        for b, bdict in model.items():
            g_bdict = dict()
            if b == 99 or b == 100:
                g_bdict = copy.deepcopy(bdict)
            else:
                for bn, count in bdict.items():
                    gbn = int(round(0.99*bn))
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
        model = copy.deepcopy(gocharge_model)

    for b in model:
        bnext_dict = model[b]
        total = np.sum(np.array(bnext_dict.values()))
        for bn in bnext_dict:
            bnext_dict[bn] = float(bnext_dict[bn])/total

    return model


class TaskBasedRBC3:  # Rule - charge only when available rewards exceed a threshold
    def __init__(self, init_battery, init_charging, test_days):
        
        self.samples = taskbased_sample_generator.SampleGenerator(test_days).samples

        self.threshold = self.samples['priority'].mean()

        self.main_path = roslib.packages.get_pkg_dir('battery_scheduler')
        self.path_data = self.main_path + '/data/'

        self.battery = []
        self.charging = []
        self.actions = []
        self.time = []
        self.obtained_rewards = []
        self.actual_rewards = []

        self.simulate(init_battery, init_charging)


    def get_current_rew(self, ts):
        tasks = self.samples[self.samples['start']<=ts]
        current_tasks = tasks[tasks['end']>=ts]
        if not current_tasks.empty:
            total_rew = current_tasks['priority'].sum()
            return total_rew, current_tasks        
        else:
            return 0, current_tasks
	

    def get_current_battery(self, prev_battery, prev_charging, current_ts, charging_started, discharging_started, action):
        if bool(prev_charging):
            time_passed = int(round((current_ts - charging_started).total_seconds()/60))
        else:
            time_passed = int(round((current_ts - discharging_started).total_seconds()/60))
            
        model = get_simbattery_model(time_passed, prev_charging, action)

        predict_b = []
        for j in range(3):
            nb = []
            prob = []
            for b,p in model[prev_battery].items():
                nb.append(b)
                prob.append(p)
            predict_b.append(np.random.choice(nb, p=prob))

        return int(np.mean(predict_b))


    def get_obtained_rew(self, ts, discharging_from, charging):
        if bool(charging):
            obtained_rew = 0
        else:
            completed_tasks = self.samples[(self.samples['end'] >= discharging_from) & (self.samples['end'] < ts)]
            if completed_tasks.empty:
                obtained_rew = 0
            else:
                obtained_rew = completed_tasks['priority'].sum()
        return obtained_rew

    @timing_wrapper
    def simulate(self, init_battery, init_charging):
        print 'Simulating...'
        all_ts = (self.samples['start'].unique()).astype(datetime)/1000000000
        all_ts = [datetime.utcfromtimestamp(t) for t in all_ts]
        unique_ts = []
        for e, t in enumerate(all_ts):
            if e == 0:
                unique_ts.append(t)
            else:
                if t - all_ts[e-1] > timedelta(minutes=5): ## New task once every 5 mins - to reduce data points
                    unique_ts.append(t)
        charging_from = discharging_from = unique_ts[0]
        battery = init_battery
        charging = init_charging 
        initial_tasks =  self.samples[self.samples['start'] == unique_ts[0]]
        task_end = initial_tasks['end'].max()
        replenish = False
        for e, ts in enumerate(unique_ts):
            print "Task ", e+1, "/", len(unique_ts), "......."
            current_rew, current_tasks = self.get_current_rew(ts)
            if e!= 0:
                if charging == 0 and task_end + timedelta(minutes=5) < ts:
                    obtained_rew = self.get_obtained_rew(task_end + timedelta(minutes=5), discharging_from, charging)
                    self.obtained_rewards.append(obtained_rew)
                    
                    battery = self.get_current_battery(battery, charging, task_end + timedelta(minutes=5), charging_from, discharging_from, action)
                    
                    self.time.append(task_end + timedelta(minutes=5))
                    self.battery.append(battery)
                    self.charging.append(charging)
                    action = 'go_charge'
                    self.actions.append(action)
                    self.actual_rewards.append(0)
                    charging = 1
                    charging_from = task_end + timedelta(minutes=5)
                    
                    self.obtained_rewards.append(0)
                    
                    battery = self.get_current_battery(battery, charging, ts, charging_from, discharging_from, action)
                else:
                    obtained_rew = self.get_obtained_rew(ts, discharging_from, charging)
                    self.obtained_rewards.append(obtained_rew)                
                    
                    battery = self.get_current_battery(battery, charging, ts, charging_from, discharging_from, action)

            if battery >= 70:
                replenish = False

            if battery <= 25 or (current_rew <= self.threshold and not bool(charging)):
                action = 'go_charge' 
                if battery <= 25:
                    replenish = True
            elif (bool(charging) and current_rew <= self.threshold)  or (replenish and battery <70):
                action = 'stay_charging'
            else:
                action = 'gather_reward'


            self.time.append(ts)
            self.battery.append(battery)
            self.charging.append(charging)
            self.actions.append(action)

            if action == 'stay_charging' or action == 'go_charge':
                charging = 1
                charging_from = ts
            elif action == 'gather_reward':
                charging = 0
                discharging_from = ts
            
            self.actual_rewards.append(current_rew)

            task_end =  current_tasks['end'].max()

            if e == len(unique_ts)-1:
                if bool(charging):
                    self.obtained_rewards.append(0)
                else:
                    self.obtained_rewards.append(current_tasks['priority'].sum())


    def get_plan(self, fname):
        plan_path = self.path_data + fname
        print 'Writing plan to ', plan_path, ' ...'
        with open(plan_path, 'w') as f:
            f.write('day time battery charging action obtained_reward actual_reward\n')
            for t, b, ch, a, obr, ar in zip(self.time, self.battery, self.charging, self.actions, self.obtained_rewards, self.actual_rewards):
                f.write('{0} {1} {2} {3} {4} {5}\n'.format(t, b, ch, a, obr, ar))

if __name__ == '__main__':

    stbrc3 = TaskBasedRBC3(70, 1, [datetime(2017,8,30), datetime(2017,8,31), datetime(2017,9,1)])
    stbrc3.get_plan('stbrc3_30831819_1')

    # np.random.seed(1)
    # stbrc3 = TaskBasedRBC3(70, 1, [datetime(2017,8,30), datetime(2017,8,31), datetime(2017,9,1)])
    # stbrc3.get_plan('stbrc3_30831819_2')

    # np.random.seed(2)
    # stbrc3 = TaskBasedRBC3(70, 1, [datetime(2017,8,30), datetime(2017,8,31), datetime(2017,9,1)])
    # stbrc3.get_plan('stbrc3_30831819_3')

    # stbrc3 = TaskBasedRBC3(70, 1, [datetime(2017,9,3), datetime(2017,9,4), datetime(2017,9,5)])
    # stbrc3.get_plan('stbrc3_394959_1')

    # np.random.seed(1)
    # stbrc3 = TaskBasedRBC3(70, 1, [datetime(2017,9,3), datetime(2017,9,4), datetime(2017,9,5)])
    # stbrc3.get_plan('stbrc3_394959_2')

    # np.random.seed(2)
    # stbrc3 = TaskBasedRBC3(70, 1, [datetime(2017,9,3), datetime(2017,9,4), datetime(2017,9,5)])
    # stbrc3.get_plan('stbrc3_394959_3')

    stbrc3 = TaskBasedRBC3(70, 1, [datetime(2017,9,16), datetime(2017,9,17), datetime(2017,9,18)])
    stbrc3.get_plan('stbrc3_169179189_1')

    # np.random.seed(1)
    # stbrc3 = TaskBasedRBC3(70, 1, [datetime(2017,9,16), datetime(2017,9,17), datetime(2017,9,18)])
    # stbrc3.get_plan('stbrc3_169179189_2')

    # np.random.seed(2)
    # stbrc3 = TaskBasedRBC3(70, 1, [datetime(2017,9,16), datetime(2017,9,17), datetime(2017,9,18)])
    # stbrc3.get_plan('stbrc3_169179189_3')

    stbrc3 = TaskBasedRBC3(70, 1, [datetime(2017,9,20), datetime(2017,9,21), datetime(2017,9,22)])
    stbrc3.get_plan('stbrc3_209219229_1')

    # np.random.seed(1)
    # stbrc3 = TaskBasedRBC3(70, 1, [datetime(2017,9,20), datetime(2017,9,21), datetime(2017,9,22)])
    # stbrc3.get_plan('stbrc3_209219229_2')

    # np.random.seed(2)
    # stbrc3 = TaskBasedRBC3(70, 1, [datetime(2017,9,20), datetime(2017,9,21), datetime(2017,9,22)])
    # stbrc3.get_plan('stbrc3_209219229_3')

    stbrc3 = TaskBasedRBC3(70, 1, [datetime(2017,9,24), datetime(2017,9,25), datetime(2017,9,26)])
    stbrc3.get_plan('stbrc3_249259269_1')

    # np.random.seed(1)
    # stbrc3 = TaskBasedRBC3(70, 1, [datetime(2017,9,24), datetime(2017,9,25), datetime(2017,9,26)])
    # stbrc3.get_plan('stbrc3_249259269_2')

    # np.random.seed(2)
    # stbrc3 = TaskBasedRBC3(70, 1, [datetime(2017,9,24), datetime(2017,9,25), datetime(2017,9,26)])
    # stbrc3.get_plan('stbrc3_249259269_3')

    stbrc3 = TaskBasedRBC3(70, 1, [datetime(2017,10,1), datetime(2017,10,2), datetime(2017,10,3)])
    stbrc3.get_plan('stbrc3_110210310_1')

    # np.random.seed(1)
    # stbrc3 = TaskBasedRBC3(70, 1, [datetime(2017,10,1), datetime(2017,10,2), datetime(2017,10,3)])
    # stbrc3.get_plan('stbrc3_110210310_2')

    # np.random.seed(2)
    # stbrc3 = TaskBasedRBC3(70, 1, [datetime(2017,10,1), datetime(2017,10,2), datetime(2017,10,3)])
    # stbrc3.get_plan('stbrc3_110210310_3')