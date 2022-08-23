#! /usr/bin/env python

import rospy
import roslib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int16
from nav_msgs.msg import MapMetaData
import time
import os
import tf
from tf.msg import *
from std_msgs.msg import String
class PoseSaver:
    def __init__(self):
        self.lwheel_sub = rospy.Subscriber('/lwheel', Int16, self.callback_lwheel)
        self.rwheel_sub = rospy.Subscriber('/rwheel', Int16, self.callback_rwheel)
        self.odom_sub = rospy.Subscriber('robomuse_diff/odom', Odometry, self.callback_odom) #robomuse_diff
        self.cmd_sub = rospy.Subscriber('robomuse_diff/cmd_vel', Twist, self.callback_vel)
        self.twist = Twist()
#        self.map_sub = rospy.Subscriber('/map_metadata', MapMetaData, self.mapdatacallback)
#        self.m = MapMetaData()
        self.odom = Odometry()
        self.rate = rospy.Rate(10)


#    def mapdatacallback(self, m):
#        self.origin_x = m.origin.position.x
#        self.origin_y = m.origin.position.y
#        self.resolution = m.resolution
#        self.width = m.width
#        self.height = m.height
#        print 'origin_x : ', self.origin_x
#        print 'origin_y : ', self.origin_y
#        print 'resolution(m/pixel) : ', self.resolution
#        print 'width : ', self.width
#        print 'height : ', self.height



    def callback_lwheel(self, lwheel):
        self.ldata = lwheel.data
        print "left wheel data =", self.ldata
#        posesaver.mapdatacallback()
    def callback_rwheel(self, rwheel):
        self.rdata = rwheel.data
        print "right wheel data=", self.rdata
    def callback_odom(self, odom):
        self.odom = odom
        print(odom)
        self.position_x = odom.pose.pose.position.x
        self.position_y = odom.pose.pose.position.y
        self.orientation_z = odom.pose.pose.orientation.z
        print "Position_x = ", self.position_x
        print "Position_y = ", self.position_y
        print "Orientation_z = ", self.orientation_z
    def callback_vel(self, twist):
        self.twist = twist
        self.vel = twist.linear.x
        print "cmd_vel =", self.vel
        posesaver.copy()
        posesaver.sub()

    def copy(self):
        self.position_x1 = self.odom.pose.pose.position.x
        self.position_y1 = self.odom.pose.pose.position.y
        self.orientation_z1 = self.odom.pose.pose.orientation.z
        print "Position_x1 = ", self.position_x1
        print "Position_y1 = ", self.position_y1
        print "Orientation_z1 = ", self.orientation_z1
#        os.system('roslaunch robomuse_hardware hw_control.launch')
    #def publish_odom(self):
    def sub(self):
        if (self.ldata == 0 and self.rdata == 0 and self.twist.linear.x != 0.0):
            while (self.ldata == 0 and self.rdata == 0 and self.twist.linear.x != 0.0):
                self.x = self.position_x1
                self.y = self.position_y1
                self.z = self.orientation_z1
                self.odom_pub = rospy.Publisher('robomuse_diff/odom', Odometry, queue_size=10 )
                #self.odom_pub.publish(self.position_x1)
                #self.odom_pub.publish(self.position_y1)
                #self.odom_pub.publish(self.orientation_z1)
                msg = Odometry()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = '/odom' # i.e. '/odom'
                msg.child_frame_id = '/base_link' # i.e. '/base_footprint'
                msg.pose.pose.position = Point(self.x, self.y, self.z)
                print 'loop working'
                msg.pose.pose.orientation = Quaternion()
                pos = (msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z)
                ori = (msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
                # Publish odometry message
                self.odom_pub.publish(msg)
                # Also publish tf if necessary
                self.rate.sleep()
                self.R=0.0
                self.P=0.0
                self.Y=0.0
#            os.system('roslaunch robomuse_hardware hw_control.launch')
#            print "launch file launched"

    def talker(self):
        pub = rospy.Publisher('/initialpose', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)
        pose = geometry_msgs.msg.PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = self.position_x1
        pose.pose.pose.position.y = self.position_y1
        pose.pose.pose.position.z = 0
        pose.pose.covariance=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        pose.pose.pose.orientation.z = self.orientation_z1
        pose.pose.pose.orientation.w=0.999641971333
        rospy.loginfo(pose)
        pub.publish(pose)

if __name__ == '__main__':
    try:
        rospy.init_node('spot_recorder')
        posesaver = PoseSaver()
        rospy.spin()
        #Testing our function()
    except rospy.ROSInterruptException:
        pass
