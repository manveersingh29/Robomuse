#!/usr/bin/env python
# license removed for brevity
import rospy
import time
import os
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Vector3
class goal1():
    def __init__(self):
        self.position = rospy.Subscriber('/apiserver', Vector3, self.callback_position)
        self.vect3 = Vector3()
        self.vect3_x = 0.0
        self.vect3_y = 0.0


    def callback_position(self, pose):
        self.vect3 = pose
        self.vect3_x = pose.x
        print pose.x
        self.vect3_y = pose.y
        Goal.movebase_client()

    def movebase_client(self):
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        print "Goal Service working"
        rate = rospy.Rate(10)
        # Waits until the action server has started up and started listening for goals.
        # Move 0.5 meters forward along the x axis of the "map" coordinate frame.
        # if wait_2:
        client.wait_for_server()
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(self.vect3_x)
        print self.vect3_x
        goal.target_pose.pose.position.y = float(self.vect3_y)
        # No rotation of the mobile base frame w.r.t. map frame
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        # Sends the goal to the action server.
        client.send_goal(goal)
        # Waits for the server to finish performing the action.
        wait_1 = client.wait_for_result()
        print "Station 1"
        # If the result doesn't arrive, assume the Server is not available
        # time.sleep(3)
        #if False:# not wait_1:
        #    rospy.logerr("Action server not available!")
        #    rospy.signal_shutdown("Action server not available!")
            # Result of executing the action
        #else:
        print "wait for client"
        return client.get_result()
            # If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('ego_goal_test')
        Goal =goal1()
        rospy.spin()
        # if result:
        #     rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

