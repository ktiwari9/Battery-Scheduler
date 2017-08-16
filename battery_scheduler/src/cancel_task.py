#!/usr/bin/env python
import sys
import rospy
from strands_executive_msgs.srv import CancelTask 


def get_services():
    # get services necessary to do cancel task
    cancel_task_srv_name = '/task_executor/cancel_task'
    rospy.loginfo("Waiting for task_executor service...")
    rospy.wait_for_service(cancel_task_srv_name)
    rospy.loginfo("Done")        
    cancel_tasks_srv = rospy.ServiceProxy(cancel_task_srv_name, CancelTask)
    return cancel_tasks_srv


if __name__ == '__main__':
    rospy.init_node("cancel_charge_task")

    task_id = int(sys.argv[1])
    # get services to call into execution framework
    cancel_task = get_services()
    resp = cancel_task(task_id)
    rospy.loginfo(' Cancelled Id: %d' % task_id)
    rospy.loginfo(' Cancelled : %s' % resp.cancelled)
 
