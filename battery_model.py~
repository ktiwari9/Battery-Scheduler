#!/usr/bin/env python3

import sys
from scipy.interpolate import InterpolatedUnivariateSpline, interp1d
import os
import time
import matplotlib.pyplot as plt
import numpy as np
import math


def nz_index(array):
    k = 0
    for d in array:
        if d != 0:
            return k 
        k = k+1  

def check_decreasing_array(array):
    for i in range(len(array)-1):
        if array[i] != 0: 
            if (array[i+1] > array[i] and array[i] != 0):
               return False
                 
    return True
    
def check_increasing_array(array):
    for i in range(len(array)-1):
        if array[i] != 0: 
            if (array[i+1] < array[i] and array[i+1] !=0):
                return False
                 
    return True
        
def get_ordered_f_model(f_model, boolean):
    first_val = []
    ordered_model = []
    for model in f_model:
        first_val.append(next( k for k in model if k != 0))
        
    if boolean == True:
        first_val.sort()
        for f in first_val:
            for m in f_model:
                if next(j for j in m if j !=0) == f:
                    if not check_decreasing_array(m):
                        ordered_model.append(m)        

    elif boolean == False:
        first_val = reversed(sorted(first_val))
        for f in first_val:
            for m in f_model:
                if next(j for j in m if j !=0) == f:
                    if not check_increasing_array(m):
                        ordered_model.append(m) 
    else:
        raise ValueError("{0} should be Boolean".format(boolean))
        return None
    
    
    return ordered_model

def find_closest_indices(a1, a2):
    diff = float('inf')
    val = next(k for k in a2 if k != 0)
    ind2 = nz_index(a2)
    for i in range(len(a1)):
       if abs(a1[i] - val) < diff:
           diff = abs(a1[i] - val)
           ind1 = i
    return ind1, ind2
                                
   

def obtain_charging_model(ordered_data, boolean):
    m = 0
    f_model = []
    for day in ordered_data:
        m = m+1
        life = []
        time = []
        for data in ordered_data[day]:
            life.append(data.life)
            time.append((float(data.epoch) - float(ordered_data[day][0].epoch))/(60*60))
            
        f = interp1d(time,life,kind='zero',bounds_error=False,fill_value=0)
        xnew = np.linspace(0, 24, num=48, endpoint=True)
        y = f(xnew)
        for i in range(len(y)):
            y[i] = round(y[i],1)
        #print ('----------------------->', m)
        #print ('original')
        #print (y)
        
        
        split = 0
        ind_s = nz_index(y)
        ind_e = 47 - nz_index(reversed(y))
        
        
        for i in range(ind_e):
            if boolean == True:
                if  i > ind_s:                                        
                    if all(next(z for z in reversed(y[:i]) if z!=0) > k for k in y[i:i+3]):
                    ##split list
                        split = 1
                        f1 = np.concatenate((y[:i],np.array((48-i)*[0])))
                        #print (f1)
                        f2 = np.concatenate((i*[0],y[i:]))
                        #print (f2)
                        for q in range(47 - nz_index(reversed(f2))):
                            if q > nz_index(f2):
                                if all(next(z for z in reversed(f2[:q]) if z!=0) > k for k in f2[q:q+3]):
                                    #print ('split twice')
                                    ##split list again
                                    split = 2
                                    f3 = np.concatenate((f2[:q],np.array((48-q)*[0])))
                                    #print (f3)
                                    f4 = np.concatenate((q*[0],f2[q:]))
                                    #print (f4)
                                elif not (f2[q] >= next(z for z in reversed(f2[:q]) if z != 0) and f2[q] <= f2[q+1] and f2[q+1] >= next(z for z in reversed(f2[:q]) if z!= 0)):
                                    f2[q] = 0;
                        i = ind_e
                                  
                    elif not (y[i] >= next(z for z in reversed(y[:i]) if z != 0) and y[i] <= y[i+1] and y[i+1] >= next(z for z in reversed(y[:i]) if z!= 0)):
                        y[i] = 0;
                        
            elif boolean == False:
                if  i > ind_s:
                    if all(next(z for z in reversed(y[:i]) if z!=0) < k for k in y[i:i+3]):
                    ##split list
                        split = 1
                        f1 = np.concatenate((y[:i],np.array((48-i)*[0])))
                        f2 = np.concatenate((i*[0],y[i:]))
                        for q in range(47 - nz_index(reversed(f2))):
                            if q > nz_index(f2):
                                if all(next(z for z in reversed(f2[:q]) if z!=0) < k for k in f2[q:q+3]):
                                    ##split list again
                                    split = 2
                                    f3 = np.concatenate((f2[:q],np.array((48-q)*[0])))
                                    #print(f3)
                                    f4 = np.concatenate((q*[0],f2[q:]))
                                    #print(f4)
                                elif not (f2[q] <= next(z for z in reversed(f2[:q]) if z != 0) and f2[q] >= f2[q+1] and f2[q+1] <= next(z for z in reversed(f2[:q]) if z!= 0)):
                                    f2[q] = 0;
                        i = ind_e                    
                    elif not (y[i] <= next(z for z in reversed(y[:i]) if z!=0) and y[i] >= y[i+1] and y[i+1] <= next(z for z in reversed(y[:i]) if z!=0)):
                        y[i] = 0
                        
            
            else:
                raise ValueError("{0} should be Boolean".format(boolean))
        
        print ('-----final',m)            
        if split == 1:
            if not all(k == 0.0 for k in f1):
                f_model.append(f1)
                #print (f1)
            if not all(k == 0.0 for k in f2):
                f_model.append(f2)
                #print (f2)
                
        elif split == 2:
            if not all(k == 0.0 for k in f1):
                f_model.append(f1)
                #print (f1)
            if not all(k == 0.0 for k in f3):
                f_model.append(f3)
                #print (f3)
            if not all(k == 0.0 for k in f4):
                f_model.append(f4)
                #print (f4)
            
        else:
            f_model.append(y) 
            #print (y)  
                    
    #Shift and average
    if boolean == True:
        # Arrange in ascending order according to first non-zero value 
        ordered_f_model = get_ordered_f_model(f_model, True)
        print ('--FINAL LIST:')
        for a in range(len(ordered_f_model)):
            print (ordered_f_model[a])
    
    elif boolean == False:
        # Arrange in descending order according to first non-zero value
        ordered_f_model = get_ordered_f_model(f_model, False)
        print (ordered_f_model)
    
    else:
        raise ValueError("{0} should be Boolean".format(boolean))
       
    #Shift the models accordingly
    final = []
    m1_ind = nz_index(ordered_f_model[1])
    m1 = ordered_f_model[m1_ind:] 
    final.append(m1)
    for m in range(1,len(ordered_f_model)):
        ind1, ind2 = find_closest_indices(ordered_f_model[m-1],ordered_f_model[m])
        #shifted_m = shift(ind1, ind2, ordered_f_model[m])
    return f_model






    

