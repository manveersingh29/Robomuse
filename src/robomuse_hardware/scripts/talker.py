#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Header
def talker():
    pub_l = rospy.Publisher('lwheel', Int16, queue_size=10)
    pub_r = rospy.Publisher('rwheel', Int16, queue_size=10)
    pub_vel_l = rospy.Publisher('rwheel_vel', Float32, queue_size=10)
    pub_vel_r = rospy.Publisher('lwheel_vel', Float32, queue_size=10)
    pub_vel_time = rospy.Publisher('vel_time',Header, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 100hz
    time_vel = Header() 
    i = 0
    while not rospy.is_shutdown():
	time_vel.frame_id = 'velocity_time'
        left_encoder=5
        right_encoder=5
        left_vel = 10
        right_vel = 10 
        #hello_str = "hello world %s" % rospy.get_time()
        #//rospy.loginfo(hello_str)
        pub_l.publish(left_encoder)
        pub_r.publish(right_encoder)
        pub_vel_l.publish(left_vel)
        pub_vel_r.publish(right_vel)
	time_vel.seq += i + 1
	time_vel.stamp = rospy.Time.now()
	pub_vel_time.publish(time_vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
