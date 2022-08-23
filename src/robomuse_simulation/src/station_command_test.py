#!/usr/bin/env python
# license removed for brevity

import rospy
import time
import os
import sys, select
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

def getKey():
    global settings
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
def movebase_client():
    pub = rospy.Publisher('/station_flag',String, queue_size=10)
    global settings
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    print "hello"
    rate = rospy.Rate(10)
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()
    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # Move 0.5 meters forward along the x axis of the "map" coordinate frame.
    while(1):
        wait_1 = 1
        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)
        print"hey"
        key = getKey()
        if key == 'w':
            key_pressed = 'w'
            pub.publish(key_pressed)
            print 'w'
            station_1_flag = 's1'
            goal.target_pose.pose.position.x = 6.61
            goal.target_pose.pose.position.y = 1.56
            # No rotation of the mobile base frame w.r.t. map frame
            goal.target_pose.pose.orientation.z = 0.0
            goal.target_pose.pose.orientation.w = 1
            # Sends the goal to the action server.
            client.send_goal(goal)
            # Waits for the server to finish performing the action.
            wait_1 = client.wait_for_result()
            print "Station 1"
            station_1_flag = 'r1'
            pub.publish(station_1_flag)
            # If the result doesn't arrive, assume the Server is not available
            time.sleep(3)
        if key == 'x':
            key_pressed = 'x'
            pub.publish(key_pressed)
            station_2_flag = 's2'
            goal.target_pose.pose.position.x = 10.75
            goal.target_pose.pose.position.y = 1.65
            goal.target_pose.pose.orientation.z = 0.0
            goal.target_pose.pose.orientation.w = 1
            # Sends the goal to the action server.
            client.send_goal(goal)
            # Waits for the server to finish performing the action.
            wait_2 = client.wait_for_result()
            print "Station 2"
            station_2_flag = 'r2'
            pub.publish(station_2_flag)
            time.sleep(3)
            # If the result doesn't arrive, assume the Server is not available
            continue
        if not wait_1:
            rospy.loger("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            # Result of executing the action
        #else:
        #   return client.get_result()
        rate.sleep()
# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('station_test')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
