#!/usr/bin/env python

import plotly
import plotly.plotly as py
import plotly.graph_objs as go
import sys
# from scipy.interpolate import InterpolatedUnivariateSpline, interp1d
import os
import time
import numpy as np
import math
import yaml
import battery_data as bd

class BatteryData:

    def __init__(self, battery_str):
        arr =  battery_str[:-1].split(', ')
        if len(arr) != 6:
            raise ValueError("battery string given was invalid: {0}".format(battery_str))
        self.epoch = round(float(arr[0]))
        self.cur_node = arr[1]
        self.life = int(arr[2]) if arr[2] else 0
        self.voltage = float(arr[3]) if arr[3] else 0
        self.is_charging = arr[4] == '1'
        self.current = float(arr[5]) if arr[5] else 0
        # self.time = dates.date2num(datetime.datetime.fromtimestamp(float(self.epoch)))

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
    charging_data = []
    discharging_data = []
    for d_file in sorted(files):
        with open(d_file, 'r') as f_in:
            for line in f_in:
                if line[0] != '#' or line[0] != '%':
                    try:
                        battery_state = BatteryData(line)

                        if battery_state.is_charging == True:
                            if len(charging_data) == 0:
                                charging_data.append([battery_state])
                        
                            else:
                                if charging_data[-1][-1].epoch != battery_state.epoch:
                                    #print battery_state.life, battery_state.epoch
                                    #print abs(charging_data[-1][-1].epoch - battery_state.epoch)
                                    if abs(charging_data[-1][-1].epoch - battery_state.epoch) == 64.0:
                                        if charging_data[-1][-1].life <= battery_state.life:
                                           print battery_state.life, battery_state.epoch
                                            charging_data[-1].append(battery_state)
                                        else:
                                            charging_data.append([battery_state])

                                    elif abs(charging_data[-1][-1].epoch - battery_state.epoch) % 64 == 0.0:
                                        charging_data.append([battery_state])
                            
                        elif battery_state.is_charging == False:
                            if len(discharging_data) == 0:
                                discharging_data.append([battery_state])
                                # print battery_state.life, battery_state.epoch
                            else:
                                if discharging_data[-1][-1].epoch != battery_state.epoch:
                                    print battery_state.life, battery_state.epoch
                                    #print abs(discharging_data[-1][-1].epoch - battery_state.epoch)
                                    if abs(discharging_data[-1][-1].epoch - battery_state.epoch) == 64.0:
                                        if discharging_data[-1][-1].life >= battery_state.life:
                                            print battery_state.life, battery_state.epoch
                                            discharging_data[-1].append(battery_state)
                                        else:
                                            discharging_data.append([battery_state])
                                    elif abs(discharging_data[-1][-1].epoch - battery_state.epoch) % 64.0 == 0:
                                        discharging_data.append([battery_state])

                    except ValueError:
                       continue
            
    return charging_data, discharging_data


def get_battery_model(path_to_directory):
    ############### SPECIFY PATHS #######################
    path = '/home/milan/workspace/strands_ws/src/battery_scheduler'
    #path = roslib.packages.get_pkg_dir('battery_scheduler')
    if os.path.isfile(path+'/models/battery_charge_model2.yaml') and os.path.isfile(path+'/models/battery_discharge_model2.yaml'):
        with open (path+'/models/battery_charge_model2.yaml', 'r') as f_charge:
            charge_model = yaml.load(f_charge)
        with open (path+'/models/battery_discharge_model2.yaml', 'r') as f_discharge:
            discharge_model = yaml.load(f_discharge)

    else:
        charge_model = dict()
        discharge_model = dict()
        for model in [charge_model, discharge_model]:
            for i in range (101):
                model.update({ i : dict()})
   
        charging_data, discharging_data = extract_data(get_files(path_to_directory)) ## specify path to battery files
        for data_set in [charging_data, discharging_data]:
            for data in data_set:
                print '#####################'
                for i in range(len(data)):
                    if len(data) >= 30 and check_variation_values(data):
                        current_bs = data[i]
                        print current_bs.life
                        if i < len(data)-30:
                            next_bs = data[i+30]
                            # if abs(next_bs.life - current_bs.life) <= 10: #for fake battery model
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
                if battery_val == 0:
                    nb_dict = charge_model[1]
                else:
                    nb_dict = dict({100 : 1})
                charge_model.update({battery_val : nb_dict})



        # f_discharge = file(path+'/models/battery_discharge_model.yaml', 'w')
        # yaml.dump(discharge_model, f_discharge)
        # f_charge =file(path+ '/models/battery_charge_model.yaml', 'w')
        # yaml.dump(charge_model, f_charge)

    return charge_model, discharge_model

def main():
    data1, data2 = extract_data(get_files(sys.argv[1:]))
    # data1, data2 =get_battery_model(sys.argv[1:])
    # print data
    #plot_data(data)
    #plot_current_integral(data)
    #generate_data_file(data)

if __name__ == '__main__':
    main()

# def get_axes(data, boolean):
#      time_battery = dict()
#      k = 0    
#      for day in data:
#          info = data[day]
#          for i in range(len(info)-1):
#              (b1,b2) = (float(info[i].life), float(info[i+1].life))
#              time = round((abs(float(info[i].epoch) - float(info[i+1].epoch)))/(60), 4)
#              if not ( abs(b1-b2) != 1):
#                  if (b1 > b2 and boolean == False) or (b1 < b2 and boolean == True):
#                      if (b1,b2) not in time_battery:
#                          time_l = []
#                          time_l.append(time)
#                      else:
#                          time_l = time_battery[(b1,b2)]
#                          time_l.append(time)
             
#                      time_battery.update({(b1,b2) : time_l})
           
#      for battery in time_battery:
#          avg = np.mean(np.array(time_battery[battery]))
#          time_battery.update({ battery : avg})
         
#      if boolean == True:
#          delta_bt = sorted(time_battery.items())             
#      elif boolean == False:
#          delta_bt = sorted(time_battery.items(), reverse=True)
#      else:
#          delta_bt = None
#          print ('Input Value Should be Boolean')
         
#      time_axis = []
#      battery_axis = []
#      for j in range(len(delta_bt)):
#          battery_axis.append(delta_bt[j][0][1])
#          if j == 0:
#              time_axis.append(delta_bt[j][1])
             
#          else:
#              time_axis.append(time_axis[j-1]+delta_bt[j][1])
#      return battery_axis, time_axis
     
# def indxof(num, array):
#     for i in range(len(array)):
#         if num == 100:
#             if math.floor(array[i]) == 100:
#                 return i
#         elif num == 0:
#             if math.ceil(array[i]) == 0:
#                 return i
#         else:
#             if array[i] == num:
#                 return i
      
# def get_model(data, boolean):
#     battery_axis, time_axis = get_axes(data,boolean)    
#     f = interp1d(np.array(time_axis), np.array(battery_axis), kind='slinear', fill_value='extrapolate') 
#     xnew = np.linspace(-30, 1170, num=12000, endpoint=False)
#     y = f(xnew)
    
#     ind_100 = indxof(100, y)
#     print ('INDEX 100', ind_100, xnew[ind_100])
#     ind_0 = indxof(0, y) 
#     print ('INDEX 0', ind_0, xnew[ind_0])
    
#     x_new = len(xnew)*[0]
#     for k in range(len(xnew)):  
#         if boolean == True:
#             x_new[k] = xnew[k] - xnew[ind_0]
            
#         elif boolean == False:
#             x_new[k] = xnew[k] - xnew[ind_100]
            
#     if boolean == True:
#         x_f = x_new[ind_0:ind_100 + 1]
#         y_f = y[ind_0:ind_100 + 1]
#         trace = go.Scatter(x=time_axis, y=battery_axis)     
#         layout = go.Layout(title='Charging Battery Model',xaxis=dict(title='Time (secs)'), yaxis=dict(title='Battery Life (%)'))
#         fig1 = go.Figure(data=[trace], layout=layout)
#         plot_url = py.plot(fig1, filename='battery_model_ch')   
        
#     elif boolean == False:
#         x_f = x_new[ind_100:ind_0 + 1]
#         y_f = y[ind_100:ind_0 + 1]
#         trace = go.Scatter(x=x_f, y=y_f)     
#         layout = go.Layout(title='Discharging Battery Model',xaxis=dict(title='Time (secs)'), yaxis=dict(title='Battery Life (%)'))
#         fig1 = go.Figure(data=[trace], layout=layout)
#         plot_url = py.plot(fig1, filename='battery_model_disch') 
              
#     return x_f , y_f
    
# def get_model_from(init_charge, data, boolean):
#     f  = 48*[0]
#     model_x, model_y = get_model(data, boolean)
#     index = indxof(init_charge, model_y)
#     for i in range(0,48):
#         if (index+30*i) < len(model_y):
#             f[i] = model_y[index + 30*i]
#         else:
#             if boolean == True:
#                 f[i] = 100
#             elif boolean == False:
#                 f[i] = 0
   
#     return f    
    
# def get_prism_model(battery_axis):
#     p_dict = dict()
#     for j in range(len(battery_axis)-1):
#         if abs(battery_axis[j+1]-battery_axis[j]) not in p_dict:
#             l = []
#             l.append(battery_axis[j])
#         else:
#             l = p_dict[abs(battery_axis[j+1]-battery_axis[j])]
#             if battery_axis[j] not in l:
#                 l.append(battery_axis[j])
#         p_dict.update({ abs(battery_axis[j+1]-battery_axis[j]) : l})
#     return p_dict 

# def all_indices(num, array):
#     index = []
#     for l in range(len(array)):
#         if array[l] == num:
#             index.append(l)
#     return index
        
# def prism_model(data, boolean):
#     model_x, model_y = get_model(data, boolean)
#     p_l = []
#     for i in range(0,101):
#         index = all_indices(i, model_y)
#         diff_l = []
#         for indx in index:
#             if indx+30 >= len(model_y):
#                 if boolean == True:
#                     diff_l.append(100-model_y[indx])
#                 else:
#                     diff_l.append(0-model_y[indx])
#             else:
#                 diff_l.append(model_y[indx+30] - model_y[indx])
                
#         diff = round(np.mean(np.array(diff_l))) 
#         p_l.append(diff)
#     return p_l      
      
# if __name__ == "__main__":
    
#    plotly.tools.set_credentials_file(username='MilanMariyaTomy', api_key='aoeKNl0QTEWLTb1jlFcB')
#    data = bd.extract_data(bd.get_files(sys.argv[1:]))
#    charge_data = bd.get_charging_data(data, 1)
#    discharge_data = bd.get_charging_data(data, 0)
#    #battery_axis = get_model_from(0,charge_data, True)
#    b = prism_model(charge_data, True)
#    for j in range(len(b)):
#        print( j, ' : ', b[j])
#    print ('-------------------------------------------') 
#    #battery_axis = get_model_from(100,discharge_data, False)
#    bb = prism_model(discharge_data, False)
#    for j in range(len(bb)):
#        print( 100-j, ' : ', bb[100-j])   
#    print ('-------------------------------------------') 
   
