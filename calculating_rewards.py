#! /usr/bin/env python

import numpy as np
import sys 
import os 

if __name__ == '__main__':

#rhc
    total_rhc = 8*[0]
    for i in range (8):
        with open('/home/milan/catkin_ws/prism/plan_rhc'+str(i), 'r') as f:
            for line in f:
                if 'battery' not in line:
                    s = line.split(' ')
                    reward = float(s[5])
                    if s[4] == 'gather_reward':
                        total_rhc[i] = total_rhc[i] + reward
        print 'Total reward plan ' ,i ,' :' , total_rhc[i]  
    print 'Average Reward for 8 days RHC : ', np.mean(np.array(total_rhc))
