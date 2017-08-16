#! /usr/bin/env python

import time
import rospy
import subprocess
import numpy as np
import rhc_prism_parse
import rhc_prism_script
import rewards_uncertain
from rospy import ROSInterruptException
from scitos_msgs.msg import BatteryState
from rospy.exceptions import ROSException
from strands_executive_msgs import task_utils
from battery_scheduler.msg import ExpectedState
from std_msgs.msg import Bool, String, Float32, Int32, Empty
from strands_executive_msgs.srv import DemandTask, SetExecutionStatus
from strands_executive_msgs.msg import Task, ExecutionStatus, TaskEvent

global int_duration
int_duration = 1800 # Half Hour intervals
global no_int
no_int = 48

class battery_scheduler:
  def __init__(self):
    self.init_time = 0
    self.init_battery = 100
    self.init_charging = 1
    self.init_observed = 0
    ur = rewards_uncertain.uncertain_rewards(False)
    self.rewards_model, self.rewards_prob = ur.get_rewards()
    #for j in range(len(self.rewards_model)):
    #  print self.rewards_model[j]
    #  print self.rewards_prob[j]
    #  print '----------------------------------------'
    self.queue = []
    self.task_dict = dict()
    self.pub_ex_st = rospy.Publisher('/battery_scheduler/expected_state', ExpectedState, queue_size = 10)
    self.index = 0


  def event_processor(self, task_event):
    ##check failure of charging action
    if task_event.task.action == 'wait_action' and task_event.task.start_node_id == 'ChargingPoint1':
      if task_event.event == 15:
        rospy.loginfo('Charging action aborted, starting over again...')
        self._demand_wait_action()
      elif task_event.event == 16 or task_event.event == 17:
        rospy.loginfo('Betty has successfully charged')

    ##forming a proper queue
    if task_event.event == 1 and task_event.task.start_node_id != 'ChargingPoint1':
      self.task_dict.update({ task_event.task.task_id : task_event.task})

    if (task_event.event == 15 or task_event.event == 12 or task_event.event == 13 or task_event.event == 16) and task_event.task.start_node_id != 'ChargingPoint1':
      if task_event.task.task_id in self.task_dict:
        self.task_dict.update({ task_event.task.task_id : 'Removed'})

    task_list = []
    for task in self.task_dict.values():
      if task != 'Removed':
          task_list.append(task)
    self.queue = task_list


  def _find_reward(self):
    rospy.loginfo('Obtaining expected reward from possible tasks..')
    print 'length of the current schedule : ', len(self.queue)
    task_list = self.queue
    for task in task_list:
      print task.task_id 
      print 'Start after: ', time.asctime(time.localtime(task.start_after.to_sec()))
      print 'End before: ', time.asctime(time.localtime(task.end_before.to_sec()))
      print '---------------------------------------------------------'
    current_time = time.time()
    total_reward = 0
    for task in task_list:
      task_end = task.end_before.to_sec()
      task_start = task.start_after.to_sec()
      if task_end > current_time and (current_time+int_duration) > task_start:
	total_reward = total_reward + task.priority
    rospy.loginfo('Reward that can be obtained in the current interval: %d', total_reward)
    return total_reward

  def _closest_cluster(self, reward, clusters):
    reward_array = np.array(len(clusters)*[reward])
    index = (np.abs(np.array(clusters)-reward_array)).argmin()
    return index, clusters[index]

  def obtain_action(self, event):
    rospy.loginfo('Obtaining action from battery scheduler...')
    self._setting_execution_status(False)
    if self.index == 0:
      rospy.sleep(rospy.Duration(60))
    ps = rhc_prism_script.make_model(self.init_time, self.rewards_model, self.rewards_prob)
    rospy.loginfo('Writing PRISM file...')
    ps.write_prism_file('rhc_model.prism',self.init_battery, self.init_charging, self.init_observed)
    subprocess.call('./prism /localhome/strands/milan_ws/prism/rhc_model.prism /localhome/strands/milan_ws/prism/rhc_model.props'+
         	    ' -exportadv /localhome/strands/milan_ws/prism/rhc_model.adv -exportprodstates /localhome/strands/milan_ws/prism/rhc_model.sta'+
		    ' -exporttarget /localhome/strands/milan_ws/prism/rhc_model.lab',cwd='/opt/prism-robots/prism/bin',shell=True) ## CHANGE PATH
    total_reward = self._find_reward()
    global no_int
    cl_id, cl_reward = self._closest_cluster(total_reward, self.rewards_model[self.init_time%no_int])
    rospy.loginfo('Reading PRISM script...')
    #t1 = time.time()
    pp = rhc_prism_parse.parse_model(['rhc_modelpre1.adv', 'rhc_model.sta', 'rhc_model.lab'], cl_id, cl_reward)
    next_state = pp.get_next_state(pp.initial_state)
    #period = time.time() - t1
    #print 'Took ', period, ' seconds to find action from the policy generated'
    rospy.loginfo('Reward expected: %s' % cl_reward)
    rospy.loginfo('Action obtained: %s' % next_state[1])
    if next_state[1] == 'go_charge' or next_state[1] == 'stay_charging':
      rospy.loginfo('Demanded charging action...')
      self._demand_wait_action()
    else:
      rospy.loginfo('Continuing with normal tasks...')
      self._setting_execution_status(True)
      
    current_life = rospy.wait_for_message('/battery_state', BatteryState)
    self.pub_ex_st.publish(ExpectedState(next_state[1], int(next_state[2][2]), total_reward, current_life.lifePercent, rospy.Time.now()))
    self.init_time = self.init_time+1
    self.init_battery = current_life.lifePercent
    #self.init_battery = int(next_state[2][2])
    self.init_charging = int(next_state[2][0])
    self.init_observed = int(next_state[2][4])
    self.index = self.index+1


  def _setting_execution_status(self,boolean):
    set_exe_stat_srv_name = '/task_executor/set_execution_status'
    rospy.loginfo("Waiting for task_executor service...")
    rospy.wait_for_service(set_exe_stat_srv_name)
    rospy.loginfo("Done")
    set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)
    set_execution_status(boolean)

  def _demand_wait_action(self):
    try:
      demand_task, set_execution_status = self.get_services()
      demanded_wait = Task(action='wait_action', max_duration=rospy.Duration(int_duration-10), start_node_id='ChargingPoint1')
      set_execution_status(True)
      print 'set execution'
      task_utils.add_time_argument(demanded_wait, rospy.Time(0))
      task_utils.add_duration_argument(demanded_wait, rospy.Duration(int_duration-10))
      resp = demand_task(demanded_wait)
      print 'demanded task as id: %s' % resp.task_id
      rospy.loginfo('Success: %s' % resp.success)
      rospy.loginfo('Wait: %s' % resp.remaining_execution_time)
      if resp.success == False:
	rospy.sleep(rospy.Duration(10))
      	rospy.loginfo('Demanding Task again due to failure')
        self._demand_wait_action()
    except ROSInterruptException:
      raise


  def get_services(self):
    demand_task_srv_name = '/task_executor/demand_task'
    set_exe_stat_srv_name = '/task_executor/set_execution_status'
    rospy.loginfo("Waiting for task_executor service...")
    rospy.wait_for_service(demand_task_srv_name)
    rospy.wait_for_service(set_exe_stat_srv_name)
    rospy.loginfo("Done")
    add_tasks_srv = rospy.ServiceProxy(demand_task_srv_name, DemandTask)
    set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)
    return add_tasks_srv, set_execution_status



if __name__ == '__main__':
  rospy.init_node('battery_scheduler')
  bs = battery_scheduler()
  sub = rospy.Subscriber('/task_executor/events', TaskEvent, bs.event_processor)
  bs._setting_execution_status(False)
  rospy.loginfo('Waiting for node to be initialised...')
  
  try:
    while True:
      current_time = time.localtime(time.time())
      if (current_time[4] == 0 and current_time[5] == 0) or (current_time[4] == 30 and current_time[5] == 0):
        break
  except ROSInterruptException:
    raise

  if current_time[4] == 0:
    current_time_int = 2*current_time[3]
  else:
    current_time_int = 2*current_time[3]+1
    
  msg = rospy.wait_for_message('/battery_state', BatteryState)
  rospy.loginfo('Obtained Initial Battery State')
  print 'Battery Life: ' , msg.lifePercent
  print 'Charging State: ', msg.charging
  bs.init_time = current_time_int
  bs.init_battery = msg.lifePercent
  bs.init_charging = 1 if msg.charging else 0
  rospy.loginfo('Battery Scheduler node initialised...')
  bs.obtain_action(Empty)
  rospy.Timer(rospy.Duration(1800), bs.obtain_action)
  rospy.spin()
