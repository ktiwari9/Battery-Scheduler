#!/usr/bin/env python

import avgRewards



if __name__ == "__main__":
    try:
        rew = avgRewards.avg_rewards()
        i =0
        total = 0
        for r in rew.avg_rewards:
            print i, ' : ', round(r)
            i = i+1
            total = total + r
        print '--------------------------------------------------------------'
        print 'TOTAL : ' , total
        
        
        
        
    except (ValueError): 
        pass
