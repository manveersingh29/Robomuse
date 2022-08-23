#!/usr/bin/env python
import rospy
import time
import os
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Vector3

class moveAPIClient():
    def __init__(self):
        self.position = rospy.Subscriber('/apiserver', Vector3, self.callback_position)
        self.vect3 = Vector3()
        self.vect3_x = 0.0
        self.vect3_y = 0.0
	self.count = 0


    def callback_position(self, pose):
        self.count += 1
	self.vect3 = pose
        self.vect3_x = pose.x
        self.vect3_y = pose.y
        Goal.movebase_client()

    def movebase_client(self):
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Recieved destination command via /apiserver. Setting goal parameters ...")
        client.wait_for_server()
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
	#rospy.loginfo("Requested Destination x : %s , y : %s", string(self.vect3_x), string(self.vect3_y))
        goal.target_pose.pose.position.x = float(self.vect3_x)
        goal.target_pose.pose.position.y = float(self.vect3_y)
        
        # No rotation of the mobile base frame w.r.t. map frame
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        
	rospy.loginfo("Sending goal to /move_base (may be checked on /move_base/goal topic)")
        # Sends the goal to the action server.
        client.send_goal(goal)
        
	# Waits for the server to finish performing the action.
	rospy.loginfo("Task running ...")
	wait = client.wait_for_result()

        # If the result doesn't arrive, assume the Server is not available
        # time.sleep(3)
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            # Result of executing the action
        else:
	    #print wait
	    #print self.count
	    rospy.loginfo("Destination Reached !!")
            return client.get_result()


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('ego_goal_test')
        Goal = moveAPIClient()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

