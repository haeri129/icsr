#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
#from std_msgs.msg import Int64

class stool:
    def __init__(self):
        rospy.Subscriber("height", String, self.callback1)
        rospy.Subscriber("book", String, self.callback2)


    def callback1(self, data, self.count):

        rospy.loginfo("I'm moving to %s", data.data)

        if data.data == "adult":
            self.count = 1
          
        elif data.data == "child":
            self.count = 2

    def scenario1(self, data2, self.count):
        stack = 0
        if data2.data == "book5" or data2.data == "book6" or data2.data == "book7" or data2.data == "book9"
            stack=stack+1
        
            
        if data2.data == "book5":   
            if result:
                rospy.loginfo("position 1 arrived!")
                result = self.movebase_client(0,0,0)
                if result:
                    rospy.loginfo("position 0 arrived!")    

        if data2.data == "book6":
           result = self.movebase_client(-1.3,1.5,0)
           if result:
                rospy.loginfo("position 1 arrived!")
                result = self.movebase_client(0,0,0)
                if result:
                    rospy.loginfo("position 0 arrived!")    

        if data2.data == "book7":
           result = self.movebase_client(-1.3,1.5,0)
           if result:
                rospy.loginfo("position 1 arrived!")
                result = self.movebase_client(0,0,0)
                if result:
                    rospy.loginfo("position 0 arrived!")    

                    

        if data2.data == "book9":
           result = self.movebase_client(-1.3,1.5,0)
           if result:
                rospy.loginfo("position 1 arrived!")
                result = self.movebase_client(0,0,0)
                if result:
                    rospy.loginfo("position 0 arrived!")    
          

    def movebase_client(self,x,y,z):

        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = 1.0

        client.send_goal(goal)

	    #return 1
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
        	return client.get_result()

if __name__ == '__main__':

        rospy.init_node('stool', anonymous=False)
        stool()
        rospy.spin()