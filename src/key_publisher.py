#!/usr/bin/python3.8

import sys, tty, termios, select
import rospy
from std_msgs.msg import String


if __name__ == "__main__":
    rospy.init_node('keyboard_driver')
    pub = rospy.Publisher('keys', String, queue_size=1)
    rate = rospy.Rate(100)
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            pub.publish(sys.stdin.read(1))
        rate.sleep()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)