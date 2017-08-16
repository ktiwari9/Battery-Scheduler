#!/usr/bin/env python

import rospy
from scitos_msgs.msg import BatteryState
from battery_scheduler.msg import ExpectedState, RewardsCollected
from task_executor import task_routine, task_query
from strands_executive_msgs.msg import *
from random import random
from dateutil.tz import *
from datetime import *

global pub
global next_battery
global task_dict 
task_dict = {}

def callback(state):
  reward = 0
  global next_battery
  global pub
  t = datetime.fromtimestamp(state.time.to_sec(), tzlocal())
  rospy.loginfo('%s\t %d\t %d\t %d\t %s' %(t.strftime('%d/%m/%y %H:%M:%S'), state.current_battery, next_battery, state.reward, state.decision))
  next_battery = state.battery_life
  for task in task_dict.values():
    if task != None and task != 'Failed':
      if task[0].to_sec() > (state.time.to_sec()-1800) and task[0].to_sec() < state.time.to_sec():
        reward = reward + task[1]
  rospy.loginfo('Reward obtained at the end of half hour: %d' %int(reward))
  pub.publish(RewardsCollected(reward, state.time.to_sec()))
  

def on_event(task_event):
  rostime_now = rospy.get_rostime()
  now = datetime.fromtimestamp(rostime_now.to_sec(), tzlocal())
  if task_query.task_event_string(task_event.event) == 'TASK_STARTED' or task_query.task_event_string(task_event.event) == 'TASK_SUCCEEDED':
    rospy.loginfo('task %s\t%s\t%s\t%s\t%s\t%d' % (task_event.task.task_id, task_event.task.action, task_event.task.start_node_id, task_query.task_event_string(task_event.event), now.strftime('%d/%m/%y %H:%M:%S'),task_event.task.priority))
    if task_query.task_event_string(task_event.event) == 'TASK_STARTED':
      task_dict.update({ task_event.task.task_id : None})
    elif task_query.task_event_string(task_event.event) == 'TASK_SUCCEEDED':
      task_dict.update({ task_event.task.task_id : [task_event.time, task_event.task.priority]})
    elif task_query.task_event_string(task_event.event) == 'TASK_FAILED':
      task_dict.update({ task_event.task.task_id : 'Failed'})

  

if __name__ == '__main__':
  rospy.init_node('betty_plan_printer')
  msg = rospy.wait_for_message('/battery_state', BatteryState)
  next_battery = msg.lifePercent
  rospy.loginfo('Time\t Actual B.life\t Exp. B.life\t Exp. Reward \t Decison ')
  pub = rospy.Publisher('/battery_scheduler/reward_collected', RewardsCollected, queue_size=10)
  rospy.Subscriber('/task_executor/events', TaskEvent, on_event)
  rospy.Subscriber ('/battery_scheduler/expected_state', ExpectedState, callback)
  rospy.spin()