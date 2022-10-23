#!/usr/bin/python3.8

from glob import glob
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


mapping = {
    'w': [1, 0],
    'x': [-1, 0],
    'a': [0, -1], 
    'd': [0, 1],
    's': [0, 0]
}

g_last_twist = None

def callback(msg, pub):
    global g_last_twist
    if(msg.data in mapping):
        speed = mapping[msg.data]
        g_last_twist.linear.x = speed[0]
        g_last_twist.angular.z = speed[1]
        pub.publish(g_last_twist)
        


if __name__ == "__main__":
    rospy.init_node('key_to_twist')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('keys', String, callback, pub)
    # rospy.spin()    
    rate = rospy.Rate(10)
    g_last_twist = Twist()
    while not rospy.is_shutdown():
        pub.publish(g_last_twist)
        rate.sleep()