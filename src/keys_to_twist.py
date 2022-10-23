#!/usr/bin/python3.8

from glob import glob
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math

mapping = {
    'w': [1, 0],
    'x': [-1, 0],
    'a': [0, -1], 
    'd': [0, 1],
    's': [0, 0]
}

g_twist_pub = None
g_target_twist = None
g_last_twist = None
g_last_send_time = None
g_sclaing_factors = [0.1, 0.1]
g_vel_ramps = [1.0, 1.0]

def callback(msg, pub):
    global g_target_twist, g_sclaing_factors
    if(msg.data in mapping):
        speed = mapping[msg.data]
        g_target_twist.linear.x = speed[0] * g_sclaing_factors[0]
        g_target_twist.angular.z = speed[1] * g_sclaing_factors[1]
        

def ramped_vel(v_prev, v_target, t_prev, t_now, ramp_rate):
    step = ramp_rate * (t_now - t_prev).to_sec()
    sign = 1.0 if v_target > v_prev else -1.0
    error = math.fabs(v_target - v_prev)
    if error < step:
        return v_target
    else:
        return v_prev + step * sign

def ramped_twist(prev, target, t_prev, t_now, ramps):
    tw = Twist()
    tw.linear.x = ramped_vel(prev.linear.x, target.linear.x, t_prev, t_now, ramps[0])
    tw.angular.z = ramped_vel(prev.angular.z, target.angular.z, t_prev, t_now, ramps[1])
    return tw


def send_twist():
    global g_twist_pub, g_target_twist, g_last_twist, g_last_send_time,\
        g_sclaing_factors, g_vel_ramps
        
    t_now = rospy.Time.now()
    g_last_twist = ramped_twist(g_last_twist, g_target_twist, 
                                g_last_send_time, t_now, g_vel_ramps)
    g_last_send_time = t_now
    g_twist_pub.publish(g_last_twist)
    
def fetch_parameter(name, default_value):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        rospy.logwarn(f"Parameter {name} not provided; using {default_value}")
        return default_value

if __name__ == "__main__":
    rospy.init_node('key_to_twist')
    g_twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('keys', String, callback, g_twist_pub)
    
    rate = rospy.Rate(20)
    
    g_last_send_time = rospy.Time.now()
    g_last_twist = Twist()
    g_target_twist = Twist()
    
    g_sclaing_factors[0] = fetch_parameter('~linear_scale', g_sclaing_factors[0])
    g_sclaing_factors[1] = fetch_parameter('~angular_scale', g_sclaing_factors[1])
    g_vel_ramps[0] = fetch_parameter('~linear_accel', g_vel_ramps[0])
    g_vel_ramps[1] = fetch_parameter('~angular_accel', g_vel_ramps[1])
    
    while not rospy.is_shutdown():
        send_twist()
        rate.sleep()