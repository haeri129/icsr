#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

global mode_str 

def keyboard():
    while 1:
	key_input= raw_input("What place do you want? (1/2/3): ")
	global mode_str
	if key_input =="1":
		mode_str ="1"
		print("Send message : you choose '1'")
		break;
	elif key_input =="2":
		mode_str ="2"
		print("Send message : you choose '2'")
		break;
        elif key_input =="3":
		mode_str ="3"
		print("Send message : you choose '3'")
		break;

def movebase_client():

    global mode_str
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    if mode_str =="1" :
	goal.target_pose.pose.position.x = -0.5
    	goal.target_pose.pose.position.y = 0
    	goal.target_pose.pose.orientation.w = 1.0
	client.send_goal(goal)
    
    
    if mode_str =="2" :
	goal.target_pose.pose.position.x = 0
    	goal.target_pose.pose.position.y = 2.0
    	goal.target_pose.pose.orientation.w = 1.0
	client.send_goal(goal)
    
    

    if mode_str =="3" :
	goal.target_pose.pose.position.x = -0.5
    	goal.target_pose.pose.position.y = 1.0
    	goal.target_pose.pose.orientation.w = 1.0
	client.send_goal(goal)
    #wait = client.wait_for_result()
    #if not wait:
        #rospy.logerr("Action server not available!")
        #rospy.signal_shutdown("Action server not available!")
    #else:
        #return client.get_result()

if __name__ == '__main__':
    try:
	while 1:
	    keyboard()
            rospy.init_node('movebase_client_py')
            result = movebase_client()
            if result:
                rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
