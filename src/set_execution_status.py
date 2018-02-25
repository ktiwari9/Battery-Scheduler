#! /usr/bin/env python

from strands_executive_msgs.srv import SetExecutionStatus
from strands_executive_msgs.msg import ExecutionStatus
import rospy


if __name__ == '__main__':
	rospy.init_node('exe_status', anonymous='True')
	set_exe_stat_srv_name = '/task_executor/set_execution_status'
	rospy.loginfo("Waiting for task_executor service...")
	rospy.wait_for_service(set_exe_stat_srv_name)
	rospy.loginfo("Done")
	set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)	
	try:
		set_execution_status(True)
	except rospy.ServiceException:
		rospy.loginfo('Service Call Failed')




