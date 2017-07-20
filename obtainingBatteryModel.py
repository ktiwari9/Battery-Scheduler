#!/usr/bin/env python3

import sys
import battery_data as bd
import battery_model as bm



if __name__ == "__main__":
    try:
       
        battery_data = bd.extract_data(bd.get_files(sys.argv[1:]))
        print (len(battery_data), ' days of data available')
        total = 0
        for day in battery_data:
            total = total + len(battery_data[day])
        print ('Total points available: ', total)
    
        charge_data = bd.get_charging_data(battery_data, 1)
        print (len(charge_data), ' days of charging data available')
        print ('CHARGING DATA')
        total = 0
        for day in charge_data:
            total = total + len(charge_data[day])
        print ('Total charging data points :' , total) 
        
        charging = bm.obtain_charging_model(charge_data, True)
        #print (charging)
    
        #discharge_data = bd.get_charging_data(battery_data, 0)
        #print (len(discharge_data), ' days of discharging data available')
        #print ('DISCHARGING DATA')
        #total = 0
        #for day in discharge_data:
        #    total = total + len(discharge_data[day])
        #print ('Total discharging data points :', total)
        
         #discharging = obtain_charging_model(discharge_data,False)
         #print (discharging)
        
          
    except (ValueError): 
        pass
