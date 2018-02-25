#! /usr/bin/env python
import sys
import time
import rospy
import roslib
import actionlib
import subprocess
import rewards_hc
import numpy as np
import battery_model
import rhc_prism_parse
import rhc_prism_script
from rospy import ROSInterruptException
from scitos_msgs.msg import BatteryState
from rospy.exceptions import ROSException
from strands_executive_msgs import task_utils
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from battery_scheduler_sim.msg import BatterySchedulerStatus
from std_msgs.msg import Bool, String, Float32, Int32, Empty
from strands_executive_msgs.srv import DemandTask, SetExecutionStatus
from strands_executive_msgs.msg import Task, ExecutionStatus, TaskEvent
from strands_executive_msgs.msg import ExecutePolicyExtendedAction, ExecutePolicyExtendedGoal

global int_duration
int_duration = 1800 # Half Hour intervals
global no_int
no_int = 48

def setting_execution_status(boolean):
  set_exe_stat_srv_name = '/task_executor/set_execution_status'
  rospy.loginfo("Waiting for task_executor service...")
  rospy.wait_for_service(set_exe_stat_srv_name)
  rospy.loginfo("Done")
  set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)
  set_execution_status(boolean)



class battery_scheduler:
  def __init__(self):
    self.init_time = 0
    self.init_battery = 100
    self.init_charging = 1
    self.possible_reward = 0

    ## reward model
    rospy.loginfo('Obtaining reward models...')
    ur = rewards_hc.uncertain_rewards(False)
    self.rewards_model, self.rewards_prob = ur.get_rewards()
    #for j in range(len(self.rewards_model)):
    #  print self.rewards_model[j]
    #  print self.rewards_prob[j]
    #  print '----------------------------------------'

    ## battery model
    rospy.loginfo('Obtaining battery models...')
    self.charge_model, self.discharge_model = battery_model.get_battery_model()

    ## docking and undocking action clients
    self.dock_as = actionlib.SimpleActionClient("docking", MoveBaseAction)
    self.undock_as = actionlib.SimpleActionClient("undocking", MoveBaseAction)
    rospy.loginfo("Waiting for docking service...")
    self.dock_as.wait_for_server()
    rospy.loginfo("Done")
    rospy.loginfo("Waiting for undocking service...")
    self.undock_as.wait_for_server()
    rospy.loginfo("Done")


    self.model_path = roslib.packages.get_pkg_dir('battery_scheduler_sim') + '/models/' 
    self.path_data =  roslib.packages.get_pkg_dir('battery_scheduler_sim') + '/data/'

    self.pub = rospy.Publisher('/battery_scheduler_sim/status', BatterySchedulerStatus, queue_size = 10)
    
    
  def task_processor(self, task_event):
    rospy.loginfo('Obtaining expected reward from possible tasks..')
    print 'length of the current schedule : ', len(task_event.execution_queue)
    current_time = time.time()
    total_reward = 0
    for task in task_event.execution_queue:
      task_end = task.end_before.to_sec()
      task_start = task.start_after.to_sec()
      if task_end > current_time and (current_time+int_duration) > task_start:
        total_reward = total_reward + task.priority
    rospy.loginfo('Reward that can be obtained in the current interval: %d', total_reward)
    self.possible_reward = total_reward
    

  def _closest_cluster(self, reward, clusters):
    reward_array = np.array(len(clusters)*[reward])
    index = (np.abs(np.array(clusters)-reward_array)).argmin()
    return index, clusters[index]

  def obtain_action(self, event):
    setting_execution_status(False)

    msg = rospy.wait_for_message('/sim/battery_state', BatteryState)
    rospy.loginfo('Obtained Initial Battery State')
    print 'Battery Life: ' , msg.lifePercent
    print 'Charging State: ', msg.charging
    self.init_time = current_time_int
    self.init_battery = msg.lifePercent
    self.init_charging = 1 if msg.charging else 0

    rospy.loginfo('Waiting for tasks in the execution list')
    exe_list = rospy.wait_for_message('/task_executor/all_tasks', ExecutionStatus)
    self.task_processor(exe_list)
    global no_int
    cl_id, cl_reward = self._closest_cluster(self.possible_reward, self.rewards_model[self.init_time%no_int])

    rospy.loginfo('Obtaining action from battery scheduler...')   
  
    rospy.loginfo('Writing PRISM file...')
    ps = rhc_prism_script.make_model('model_rhc.prism', self.init_time, self.init_battery, self.init_charging, cl_id, self.rewards_model, self.rewards_prob, self.charge_model, self.discharge_model)
  
    with open(self.path_data+'result_rhc', 'w') as file:
      process = subprocess.Popen('./prism '+self.model_path+'model_rhc.prism '+
        self.model_path+ 'model_prop.props -exportadv '+self.model_path+'model_rhc.adv -exportprodstates '+
        self.model_path+'model_rhc.sta -exporttarget '+self.model_path+'model_rhc.lab',cwd='/home/milan/prism-svn/prism/bin', shell=True, stdout=subprocess.PIPE)
      for c in iter(lambda: process.stdout.read(1), ''):
        sys.stdout.write(c)
        file.write(c)
        ##reading output from prism to find policy file
    with open(self.path_data+'result_rhc', 'r') as f:
      line_list = f.readlines()
      for i in range(len(line_list)):
        if 'Optimal value for weights [1.000000,0.000000] from initial state:' in line_list[i]:
          if 'pre' not in line_list[i+2]:
            start_p = len('Adversary written to file "'+self.model_path)
            policy_file = line_list[i+2][start_p:-3]
            
    rospy.loginfo('Reading PRISM policy...')
    if policy_file != None:
      rhc_pp = rhc_prism_parse.parse_model([str(policy_file),'model_rhc.sta','model_rhc.lab'])
    else:
      rhc_pp = rhc_prism_parse.parse_model(['model_rhcpre1.adv', 'model_rhc.sta', 'model_rhc.lab'])
            
    action = rhc_pp.get_next_state(rhc_pp.initial_state) ## next state, action, 

    rospy.loginfo('Reward expected: %s' % cl_reward)
    rospy.loginfo('Action obtained: %s' % action)
    self.pub.publish(BatterySchedulerStatus(self.init_time, self.init_battery, self.init_charging, action, cl_reward, self.possible_reward))

    goal=MoveBaseGoal()
    goal.target_pose.pose.position.x = 3.22186541557 
    goal.target_pose.pose.position.y =  2.4388449192
    if action == 'go_charge' or action == 'stay_charging':
      rospy.loginfo('Demanded charging action...')
      self.dock_as.send_goal(goal)
      setting_execution_status(False)
    else:
      rospy.loginfo('Continuing with normal tasks...')
      self.undock_as.send_goal(goal)
      setting_execution_status(True)
      
    self.init_time = self.init_time+1
    

if __name__ == '__main__':
  rospy.init_node('battery_scheduler')
  bs = battery_scheduler()  
  setting_execution_status(False)
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
    
  rospy.loginfo('Battery Scheduler node initialised...')

  bs.obtain_action(Empty)

  rospy.Timer(rospy.Duration(1800), bs.obtain_action)
  rospy.spin()
