#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

def talker():
    MAX_ENCODER = 256
    pub_lwheel = rospy.Publisher('lwheel', Int16, queue_size=10)
    pub_rwheel = rospy.Publisher('rwheel', Int16, queue_size=10)
    rospy.init_node('wheel_encoders', anonymous=True)
    rospy.loginfo("Started ros node wheel_encoders")
    rate = rospy.Rate(10) # 10hz
    encoder_left = 0
    encoder_right = 128
    while not rospy.is_shutdown():
        encoder_left = encoder_left + 1
        encoder_right = encoder_right + 1
        if (encoder_left  > MAX_ENCODER):
           encoder_left = 0
        if (encoder_right  > MAX_ENCODER):
           encoder_right = 0
        log_str = "sent encoder %s" % rospy.get_time()
        rospy.loginfo(log_str)
        pub_lwheel.publish(encoder_left)
        pub_rwheel.publish(encoder_right)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
