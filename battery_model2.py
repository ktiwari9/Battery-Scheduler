#!/usr/bin/env python3

import sys
from scipy.interpolate import InterpolatedUnivariateSpline, interp1d
import os
import time
import matplotlib.pyplot as plt
import numpy as np
import math
import battery_data as bd

def get_axes(data, boolean):
     time_battery = dict()
     k = 0    
     for day in data:
         info = data[day]
         for i in range(len(info)-1):
             (b1,b2) = (float(info[i].life), float(info[i+1].life))
             time = round((abs(float(info[i].epoch) - float(info[i+1].epoch)))/(60), 4)
             if not ( abs(b1-b2) != 1):
                 if (b1 > b2 and boolean == False) or (b1 < b2 and boolean == True):
                     if (b1,b2) not in time_battery:
                         time_l = []
                         time_l.append(time)
                     else:
                         time_l = time_battery[(b1,b2)]
                         time_l.append(time)
             
                     time_battery.update({(b1,b2) : time_l})
           
     for battery in time_battery:
         avg = np.mean(np.array(time_battery[battery]))
         time_battery.update({ battery : avg})
         
     if boolean == True:
         delta_bt = sorted(time_battery.items())             
     elif boolean == False:
         delta_bt = sorted(time_battery.items(), reverse=True)
     else:
         delta_bt = None
         print ('Input Value Should be Boolean')
         
     time_axis = []
     battery_axis = []
     for j in range(len(delta_bt)):
         battery_axis.append(delta_bt[j][0][1])
         if j == 0:
             time_axis.append(delta_bt[j][1])
             
         else:
             time_axis.append(time_axis[j-1]+delta_bt[j][1])
     return battery_axis, time_axis
     
def indxof(num, array):
    for i in range(len(array)):
        if array[i] == num:
            return i
      
def get_model(data, boolean):
    battery_axis, time_axis = get_axes(data,boolean)    
    f = interp1d(np.array(time_axis), np.array(battery_axis), kind='slinear', fill_value='extrapolate') 
    xnew = np.linspace(-30, 1170, num=1200, endpoint=False)
    y = f(xnew)
     
    for j in range(len(y)):
       y[j] = round(y[j])  
       
    ind_100 = indxof(100, y)
    ind_0 = indxof(0, y) 
    
    x_new = len(xnew)*[0]
    for k in range(len(xnew)):  
        if boolean == True:
            x_new[k] = xnew[k] - xnew[ind_0]
            
        elif boolean == False:
            x_new[k] = xnew[k] - xnew[ind_100]
            
    if boolean == True:
        x_f = x_new[ind_0:ind_100 + 1]
        y_f = y[ind_0:ind_100 + 1]
        
    elif boolean == False:
        x_f = x_new[ind_100:ind_0 + 1]
        y_f = y[ind_100:ind_0 + 1]
                      
    return x_f , y_f
    
def get_model_from(init_charge, data, boolean):
    f  = 48*[0]
    model_x, model_y = get_model(data, boolean)
    index = indxof(init_charge, model_y)
    for i in range(0,48):
        if (index+30*i) < len(model_y):
            f[i] = model_y[index + 30*i]
        else:
            if boolean == True:
                f[i] = 100
            elif boolean == False:
                f[i] = 0
   
    return f    
    
def get_prism_model(battery_axis):
    p_dict = dict()
    for j in range(len(battery_axis)-1):
        if abs(battery_axis[j+1]-battery_axis[j]) not in p_dict:
            l = []
            l.append(battery_axis[j])
        else:
            l = p_dict[abs(battery_axis[j+1]-battery_axis[j])]
            if battery_axis[j] not in l:
                l.append(battery_axis[j])
        p_dict.update({ abs(battery_axis[j+1]-battery_axis[j]) : l})
    return p_dict 

def all_indices(num, array):
    index = []
    for l in range(len(array)):
        if array[l] == num:
            index.append(l)
    return index
        
def prism_model(data, boolean):
    model_x, model_y = get_model(data, boolean)
    p_l = []
    for i in range(0,101):
        index = all_indices(i, model_y)
        diff_l = []
        for indx in index:
            if indx+30 >= len(model_y):
                if boolean == True:
                    diff_l.append(100-model_y[indx])
                else:
                    diff_l.append(0-model_y[indx])
            else:
                diff_l.append(model_y[indx+30] - model_y[indx])
                
        diff = round(np.mean(np.array(diff_l))) 
        p_l.append(diff)
    return p_l      
      
if __name__ == "__main__":

   data = bd.extract_data(bd.get_files(sys.argv[1:]))
   charge_data = bd.get_charging_data(data, 1)
   discharge_data = bd.get_charging_data(data, 0)
   #battery_axis = get_model_from(0,charge_data, True)
   b = prism_model(charge_data, True)
   for j in range(len(b)):
       print( j, ' : ', b[j])
   print ('-------------------------------------------') 
   #battery_axis = get_model_from(100,discharge_data, False)
   bb = prism_model(discharge_data, False)
   for j in range(len(bb)):
       print( 100-j, ' : ', bb[100-j])   
   print ('-------------------------------------------') 
   
