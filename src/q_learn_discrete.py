#!/usr/bin/env python
import numpy as np
import scipy.linalg as spl


class state:
    def __init__(self, charging, battery, time, rew_cl_id, index=None):
        self.charging = charging
        self.battery = battery
        self.time = time
        self.rew_cl = rew_cl_id
        self.index = index

class q_learning:
    def __init__(self, charging_values, battery_values, time_values, cl_id_values, rewards_cl):
        # possible start value and possible end value
        self.charging = charging_values
        self.battery = battery_values
        self.time = time_values
        self.rew_cl_id = cl_id_values
        self.state_list = []
        self.action_list = ['gather_reward', 'stay_charging', 'go_charge']

        self.NUM_STATES = 0
        for i in range(self.charging[0],self.charging[1]+1):
            for j in range(self.battery[0],self.battery[1]+1):
                for k in range(self.time[0],self.time[1]+1):
                    self.state_list.append(state(i,j,k,k,index=self.NUM_STATES))
                    self.NUM_STATES = self.NUM_STATES+1 

        self.NUM_ACTIONS = len(self.action_list)  
        self.alpha = 0.8
        self.gamma = 0.99
        self.epsConv = 0.0 #find a good converging conditon, other than zero it doesn't cover entire table

        self.reward_cl = rewards_cl


    def dynamics(self,state, action):
        _state = self.state_list[state]
        _action = self.action_list[action]
        ##do something with _state and _action
        ##find index of next state from state_list and return that
        t = _state.time+1 
        rew_id = t
        if _action == 'gather_reward':
            b = _state.battery - 2
            c = 0 
            if b < self.battery[0]:
                b = self.battery[0]
        else:
            if _action == 'stay_charging':
                b = _state.battery + 2
            else:
                b = _state.battery + 1
            c = 1 
            if b > self.battery[1]:
                b = self.battery[1]

        print action
        print t, 'n_time'
        print c, 'n_charge'
        print b, 'n_battery'
        print rew_id, 'rew_id'
        
        for state in self.state_list:
            if state.time == t and state.charging == c and state.battery == b and state.rew_cl == rew_id:
                return state.index 



    def learn(self):

        s0 = []
        sFnl = []
        for i in range(len(self.state_list)):
            if self.state_list[i].time == self.time[1]:
                sFnl.append(i)
            elif self.state_list[i].time == self.time[0]:
                s0.append(i)
        print s0
        print sFnl

        #states  = np.array([i for i in range(NUM_STATES)])
        rewards = np.zeros((self.NUM_STATES,self.NUM_ACTIONS))
        # rewards = np.zeros(NUM_STATES)
        # rewards[NUM_STATES-1] = 100

        for i in range(len(rewards[:][0])):
            rewards[i][0] = self.reward_cl[self.state_list[i].time]

        Qtable = np.zeros((self.NUM_STATES, self.NUM_ACTIONS))

        #sCurr =  s0
        keepRunning = True
        itr    = 0
        count  = 0
        oldItr = 0

        while(keepRunning): # run till convergence
            itr += 1

            Qold = Qtable

            # shuffledStates = np.random.permutation(states) #start at a random state
            # sCurr = shuffledStates[0]
            sCurr = np.random.choice(s0)
            print sCurr , 'init'

            while(sCurr not in sFnl): #start of an episode
                print sCurr, type(sCurr)

                # shuffledActions = np.random.permutation(actions) #choose an action randomly
                # a = shuffledActions[0]
                if self.state_list[sCurr].charging == 1:
                    a = np.random.choice([0,1])
                elif self.state_list[sCurr].charging == 0:
                    a = np.random.choice([0,2])

                #dynamics, in this case there is no transition probabilities
                sNxt = self.dynamics(state=sCurr, action=a)

                if(sNxt in sFnl):
                    break #gone out of the states or reached sFnl
                #print Qtable[sCurr, a]
                #print rewards[sCurr,a]
                Qtable[sCurr,a] = (1.0-self.alpha)*Qtable[sCurr,a] + self.alpha*(rewards[sCurr,a] + self.gamma*(Qtable[sNxt,:]-Qtable[sCurr,a]).max())

                sCurr = sNxt

            Qnew = Qtable
            #print 'itr\t{}'.format(itr)
            if np.absolute(np.sum(Qnew-Qold)) == self.epsConv: #converging condition
                if (itr-oldItr)==1: #just to see it is the very next iteration
                    count += 1
                    oldItr = itr
                else:
                    count = 0 #if it is not consecutive iteration, reset count
                if(count>1000): #the Qtable stays constant for atleat 1000 consecutive iterations
                    keepRunning = False

        print Qtable

if __name__ == '__main__':
    time = [0,5]
    battery = [0,5]
    charging = [0,1]
    cl_id = [0, 5]
    rewards_cl = np.array([20., 10., 3000., 2000., 500., 5000.])
    qlearn = q_learning(charging, battery, time, cl_id, rewards_cl)
    qlearn.learn()